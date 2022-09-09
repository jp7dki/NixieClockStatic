#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/rtc.h"
#include "hardware/irq.h"
#include "hardware/flash.h"
#include "hardware/resets.h"
#include "hardware/clocks.h"
#include "hardware/xosc.h"
#include "pico/util/datetime.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "nixiev.h"


//-------------------------------------------------------
//- Macro Define 
//-------------------------------------------------------
// UART0(DEBUG) 
#define UART_DEBUG uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// UART1(GPS)
#define UART_GPS uart1
#define BAUD_RATE_GPS 9600

// SPI(BME280)
#define READ_BIT 0x80

//-------------------------------------------------------
//- Global Variable
//-------------------------------------------------------
const uint8_t version[6] = {0,0,1+0x10,1,0,0};                              // Firmware Version
const uint duty_comp[6] = {300,0,100,0,0,0};                                // 桁ごとの明るさ補正値
const uint duty_num_comp[12] = {0,0,50,50,-50,-50,0,-200,50,50,200,100};    // 数値ごとの明るさ補正値
uint8_t disp[6];                                                            // 表示数値
uint8_t post_disp[6];                                                       // 前回の表示の数値
const uint8_t GPS_HEADER[7] = {'$','G','P','R','M','C',','};                // NMEAのヘッダ
uint16_t rx_counter;                                                        // GPSのUART受信バイト数
uint16_t rx_sentence_counter;
uint8_t gps_time[16];                                                       // GPSから受信した時刻
uint8_t gps_date[16];                                                       // GPSから受信した日時
uint8_t gps_valid=0;                                                        // GPSの測位完了したかどうか
bool flg_time_correct=false;                                                // 時刻の補正が完了したか
bool flg_random=false;                                                      // ランダム表示のフラグ
bool flg_cloud=true;                                                        // クラウド表示のフラグ
bool flg_tick=false;                                                        // 1秒経過のフラグ
bool flg_off=false;                                                         // 表示OFFのフラグ
bool flg_on=false;                                                          // 表示ON処理開始のフラグ
uint8_t blink_counter=0;                                                    // 一時消灯（ブリンク）のフラグ
uint8_t global_counter=0;
uint8_t num;

// 動作モードの列挙
typedef enum Mode{
    WAKEUP,
    CLOCK,
    RANDOM,
    DEMO,
    SETUP,
    LIGHT_ADJ,
    SLEEP_TIME,
    WAKE_TIME
}mode;

// 動作モードの初期化
mode operation_mode = WAKEUP;

// Flashに格納するデータ(構造体)
typedef struct Nvol_contents{
    uint8_t writed;
    int16_t adj_duty[6];
    uint16_t fluc_amp;
    uint8_t switch_mode;
    uint8_t hour_demo;
    uint8_t sleep_on;
    int8_t off_hour;
    int8_t off_min;
    int8_t off_sec;
    int8_t on_hour;
    int8_t on_min;
    int8_t on_sec;
}nvol_contents;

// Flashに書き込むために共用体で配列としても扱えるようにする。
typedef union Nvol_data{
    uint8_t flash_byte[FLASH_PAGE_SIZE];
    nvol_contents flash_contents;
}nvol_data;

// Flashから読みだしたデータを格納する変数
nvol_data flash_data;

// 設定モードなどの際に使用するカーソル
uint8_t cursor;

//-------------------------------------------------------
//- Function Prototyping
//-------------------------------------------------------
void hardware_init(void);           // ハードウェアの初期化
void set_duty(uint16_t duty);       // デューティ比設定(全桁まとめて)
void set_dutys(uint16_t *duty);     // デューティ比設定(各桁ごと)

//-------------------------------------------------------
//- IRQ
//-------------------------------------------------------
//---- timer_alarm0 : 1秒ごとの割り込み-------------------
static void timer_alarm0_irq(void) {
    // Clear the irq
    hw_clear_bits(&timer_hw->intr, 1u << 0);
    uint64_t target = timer_hw->timerawl + 999999; // interval 1s
    timer_hw->alarm[0] = (uint32_t)target;

    uint8_t i;
    char buf[10];           // UART 送信用のバッファ(デバッグ用)
//    int32_t h, p, t;
//    bme280_read_raw(&h, &p, &t);

    // 時刻取得
    datetime_t time;
    rtc_get_datetime(&time);

    // 消灯時間かの判定
    if(flash_data.flash_contents.sleep_on==1){
        if((time.hour==flash_data.flash_contents.off_hour)
            &&(time.min==flash_data.flash_contents.off_min)
            &&(time.sec==flash_data.flash_contents.off_sec)){
                
            flg_off=true;
        }
    }

    // 点灯時間かの判定
    if((time.hour==flash_data.flash_contents.on_hour)
        &&(time.min==flash_data.flash_contents.on_min)
        &&(time.sec==flash_data.flash_contents.on_sec)){

        flg_on=true;
    }

    // 動作モードに応じた処理
    switch(operation_mode){

        //---- 時計モード ---------------------
        case CLOCK:
            
            if(!flg_off){
                for(i=0;i<6;i++){
                    post_disp[i]=disp[i];
                }

                // 切り替えモードに応じた処理
                switch(flash_data.flash_contents.switch_mode){
                    case 0:     // normal switch
                        disp[0] = (time.hour/10);
                        disp[1] = (time.hour%10);
                        disp[2] = (time.min/10);
                        disp[3] = (time.min%10);
                        disp[4] = (time.sec/10);
                        disp[5] = (time.sec%10);
                        multicore_fifo_push_blocking(0);
                        break;

                    case 1:     // from cloud
                        flg_tick=true;
                        global_counter=4;
                        break;
                    
                    case 2:     // patapata
                        disp[0] = (time.hour/10);
                        disp[1] = (time.hour%10);
                        disp[2] = (time.min/10);
                        disp[3] = (time.min%10);
                        disp[4] = (time.sec/10);
                        disp[5] = (time.sec%10);

                        for(i=0;i<6;i++){
                            if(disp[i]!=post_disp[i]) disp[i]+=0x20;    // 変化した桁はドット表示
                        }
                        flg_tick=true;
                        global_counter=7;
                        multicore_fifo_push_blocking(0);
                        
                        break;

                    case 3:     // dot move
                        disp[0] = (time.hour/10);
                        disp[1] = (time.hour%10);
                        disp[2] = (time.min/10);
                        disp[3] = (time.min%10);
                        disp[4] = (time.sec/10);
                        disp[5] = (time.sec%10);
                        multicore_fifo_push_blocking(0);
                        flg_tick=true;
                        global_counter=0;
                        break;

                    default:
                        disp[0] = (time.hour/10);
                        disp[1] = (time.hour%10);
                        disp[2] = (time.min/10);
                        disp[3] = (time.min%10);
                        disp[4] = (time.sec/10);
                        disp[5] = (time.sec%10);
                        multicore_fifo_push_blocking(0);
                        break;
                }
            }

            break;

        //---- セットアップモード ---------------------
        case SETUP:
            blink_counter=5;
            break;
        
        //---- 明るさ調整モード -----------------------
        case LIGHT_ADJ:
            blink_counter=5;
            break;

        default:
            break;
    }
}

//---- timer_alarm1 : 40msごとの割り込み-----------------
static void timer_alerm1_irq(void) {
    // Clear the irq
    hw_clear_bits(&timer_hw->intr, 1u << 1);
    uint64_t target = timer_hw->timerawl + 40000; // interval 40ms
    timer_hw->alarm[1] = (uint32_t)target;
    char buf[30];
    float pr;
    uint16_t duty[6];
    uint8_t i;
    datetime_t time;

    // Light Sensor Control
    uint16_t result = adc_read();

    // 1/fゆらぎ処理
    for(i=0;i<6;i++){
        int rd=random()/(RAND_MAX/10000)-5000;      // 乱数生成
        pr = pinkfilter(rd);                        // ピンクフィルタを通して1/fゆらぎに変換

        duty[i]=(result+6000)/10 + (int)(pr*flash_data.flash_contents.fluc_amp);
    }
    set_dutys(duty);

    // 数値切り替えの処理
    // 時計モードでクラウドの切り替えの場合の処理
    if((operation_mode==CLOCK) && (flash_data.flash_contents.switch_mode==1) && flg_tick){
        if(!flg_off){
            if(global_counter==0){
                flg_tick=false;
                rtc_get_datetime(&time);

                disp[0] = (time.hour/10);
                disp[1] = (time.hour%10);
                disp[2] = (time.min/10);
                disp[3] = (time.min%10);
                disp[4] = (time.sec/10);
                disp[5] = (time.sec%10); 

                multicore_fifo_push_blocking(0);

            }else{
                global_counter--;
                rtc_get_datetime(&time);

                disp[0] = (time.hour/10);
                disp[1] = (time.hour%10);
                disp[2] = (time.min/10);
                disp[3] = (time.min%10);
                disp[4] = (time.sec/10);
                disp[5] = (time.sec%10);
                
                for(i=0;i<6;i++){
                    if(disp[i]!=post_disp[i]){
                        disp[i] = (post_disp[i]+(5-global_counter))%10;     // 数値が変わったところはパタパタさせる
                    }
                }
                multicore_fifo_push_blocking(0);
            }
        }

    // 時計モードでパタパタ切り替えの場合の処理
    }else if((operation_mode==CLOCK)&&(flash_data.flash_contents.switch_mode==2)&&flg_tick){
        if(!flg_off){
            if(global_counter==0){
                // 一定時間経過したら消灯していた数値をもとに戻す。
                flg_tick=false;
                rtc_get_datetime(&time);

                disp[0] = (time.hour/10);
                disp[1] = (time.hour%10);
                disp[2] = (time.min/10);
                disp[3] = (time.min%10);
                disp[4] = (time.sec/10);
                disp[5] = (time.sec%10);

                multicore_fifo_push_blocking(0);
            }else{
                global_counter--;
            }
        }
    
    // 時計モードでドットムーブの場合の処理
    }else if((operation_mode==CLOCK)&&(flash_data.flash_contents.switch_mode==3)&&flg_tick){
        if(!flg_off){
            for(i=0;i<6;i++){
                disp[i] = (disp[i]&0x0F);   // いったんドットはすべて消灯
            }
            switch(global_counter){         // カウンタの値に応じてドットを表示。
                case 0:
                    disp[5]+=0x10;
                    break;
                case 1:
                    disp[5]+=0x20;
                    break;
                case 3:
                    disp[4]+=0x10;
                    break;
                case 4:
                    disp[4]+=0x20;
                    break;
                case 5:
                    disp[3]+=0x10;
                    break;
                case 6:
                    disp[3]+=0x20;
                    break;
                case 7:
                    disp[2]+=0x10;
                    break;
                case 8:
                    disp[2]+=0x20;
                    break;
                case 9:
                    disp[1]+=0x10;
                    break;
                case 10:
                    disp[1]+=0x20;
                    break;
                case 11:
                    disp[0]+=0x10;
                    break;
                case 12:
                    disp[0]+=0x20;
                    break;
                case 13:
                    disp[0]=disp[0];
                    break;
                case 14:
                    disp[0]+=0x10;
                    break;
                case 15:
                    disp[1]+=0x20;
                    break;
                case 16:
                    disp[1]+=0x10;
                    break;
                case 17:
                    disp[2]+=0x20;
                    break;
                case 18:
                    disp[2]+=0x10;
                    break;
                case 19:
                    disp[3]+=0x20;
                    break;
                case 20:
                    disp[3]+=0x10;
                    break;
                case 21:
                    disp[4]+=0x20;
                    break;
                case 22:
                    disp[4]+=0x10;
                    break;
                case 23:
                    disp[5]+=0x20;
                    break;
                case 24:
                    disp[5]+=0x10;
                    break;
                case 25:
                    flg_tick=false;
                    break;
            }
            multicore_fifo_push_blocking(0);
            global_counter++;
        }

    // セットアップモードの場合の処理
    }else if(operation_mode==SETUP){
        if(blink_counter==5){
            disp[cursor]=10;        // カーソルの桁を消灯
            blink_counter--;
            multicore_fifo_push_blocking(0);

        }else if(blink_counter==1){
            disp[0] = flash_data.flash_contents.switch_mode;
            disp[1] = flash_data.flash_contents.hour_demo;
            disp[2] = flash_data.flash_contents.sleep_on;
            disp[3] = (uint8_t)(flash_data.flash_contents.fluc_amp/100);
            disp[4] = (uint8_t)(flash_data.flash_contents.fluc_amp/10)%10;
            disp[5] = (uint8_t)(flash_data.flash_contents.fluc_amp%10);
            blink_counter--;
            multicore_fifo_push_blocking(0);

        }else if(blink_counter!=0){
            blink_counter--;
        }

    // 明るさ調整モードの場合の処理
    }else if(operation_mode==LIGHT_ADJ){
        if(blink_counter==5){
            disp[cursor]=10;        // カーソルの桁を消灯
            blink_counter--;
            multicore_fifo_push_blocking(0);

        }else if(blink_counter==1){
            for(i=0;i<6;i++) disp[i] = num;
            blink_counter--;
            multicore_fifo_push_blocking(0);

        }else if(blink_counter!=0){
            blink_counter--;
        }
    }

}

//---- alarm : 毎秒の割り込み --------------------
static void alarm_callback(void) {
    datetime_t t;
    uint8_t sec;
    uint8_t i;
    char buf[10];           // UART 送信用のバッファ(デバッグ用)
    

    // 時刻取得
    rtc_get_datetime(&t);

    if((t.min==0)&&(t.sec==0)){
        // 毎時にGPSの時間合わせをする
        flg_time_correct=false;
        // 毎時のデモ
        if(flash_data.flash_contents.hour_demo==1) flg_random = true;
    }

    // 毎秒の割り込みのために次の1秒でアラームを設定する。
    sec=t.sec+1;
    if(sec==60) sec=0;

    rtc_hw->irq_setup_1 = ((uint)sec)<<RTC_IRQ_SETUP_1_SEC_LSB;
    hw_set_bits(&rtc_hw->irq_setup_1, RTC_IRQ_SETUP_1_SEC_ENA_BITS);
/*
    // 消灯時間かの判定
    if(flash_data.flash_contents.sleep_on==1){
        if((t.hour==flash_data.flash_contents.off_hour)
            &&(t.min==flash_data.flash_contents.off_min)
            &&(t.sec==flash_data.flash_contents.off_sec)){
                
            flg_off=true;
        }
    }

    // 点灯時間かの判定
    if((t.hour==flash_data.flash_contents.on_hour)
        &&(t.min==flash_data.flash_contents.on_min)
        &&(t.sec==flash_data.flash_contents.on_sec)){

        flg_on=true;
    }

    // 動作モードに応じた処理
    switch(operation_mode){

        //---- 時計モード ---------------------
        case CLOCK:
            
            if(!flg_off){
                for(i=0;i<6;i++){
                    post_disp[i]=disp[i];
                }

                // 切り替えモードに応じた処理
                switch(flash_data.flash_contents.switch_mode){
                    case 0:     // normal switch
                        disp[0] = (t.hour/10);
                        disp[1] = (t.hour%10);
                        disp[2] = (t.min/10);
                        disp[3] = (t.min%10);
                        disp[4] = (t.sec/10);
                        disp[5] = (t.sec%10);
                        multicore_fifo_push_blocking(0);
                        break;

                    case 1:     // from cloud
                        flg_tick=true;
                        global_counter=4;
                        break;
                    
                    case 2:     // patapata
                        disp[0] = (t.hour/10);
                        disp[1] = (t.hour%10);
                        disp[2] = (t.min/10);
                        disp[3] = (t.min%10);
                        disp[4] = (t.sec/10);
                        disp[5] = (t.sec%10);

                        for(i=0;i<6;i++){
                            if(disp[i]!=post_disp[i]) disp[i]+=0x20;    // 変化した桁はドット表示
                        }
                        flg_tick=true;
                        global_counter=7;
                        multicore_fifo_push_blocking(0);
                        
                        break;

                    case 3:     // dot move
                        disp[0] = (t.hour/10);
                        disp[1] = (t.hour%10);
                        disp[2] = (t.min/10);
                        disp[3] = (t.min%10);
                        disp[4] = (t.sec/10);
                        disp[5] = (t.sec%10);
                        multicore_fifo_push_blocking(0);
                        flg_tick=true;
                        global_counter=0;
                        break;

                    default:
                        disp[0] = (t.hour/10);
                        disp[1] = (t.hour%10);
                        disp[2] = (t.min/10);
                        disp[3] = (t.min%10);
                        disp[4] = (t.sec/10);
                        disp[5] = (t.sec%10);
                        multicore_fifo_push_blocking(0);
                        break;
                }
            }

            break;

        //---- セットアップモード ---------------------
        case SETUP:
            blink_counter=5;
            break;
        
        //---- 明るさ調整モード -----------------------
        case LIGHT_ADJ:
            blink_counter=5;
            break;

        default:
            break;
    }*/

}

//---- uart_rx : GPSの受信割り込み --------------------
void on_uart_rx(){
    int i;
    uint8_t ch;
    uint8_t hour,min,sec;

    // GPSの受信処理。GPRMCのみ使用する。
    while(uart_is_readable(UART_GPS)){
        ch = uart_getc(UART_GPS);

        switch(rx_sentence_counter){
            case 0:
                // $GPRMC待ち
                if(GPS_HEADER[rx_counter]==ch){
                    rx_counter++;
                }else{
                    rx_counter=0;
                }

                if(rx_counter==7){
                    rx_sentence_counter++;
                    rx_counter=0;
                }
                break;
            case 1:
                // 時刻取得
                if(ch==','){
                    rx_sentence_counter++;
                    rx_counter=0;
                }else{
                    gps_time[rx_counter]=ch;
                    rx_counter++;
                }
                break;
            case 2:
                // Status取得
                if(ch==','){
                    rx_sentence_counter++;
                    rx_counter=0;
                }else{
                    gps_valid = ch;
                    rx_counter++;
                }
                break;
            case 3:
                // 緯度
                if(ch==','){
                    rx_sentence_counter++;
                    rx_counter=0;
                }else{
                    //読み捨て
                    rx_counter++;
                }
                break;
            case 4:
                // 北緯or南緯
                if(ch==','){
                    rx_sentence_counter++;
                    rx_counter=0;
                }else{
                    //読み捨て
                    rx_counter++;
                }
                break;
            case 5:
                // 経度
                if(ch==','){
                    rx_sentence_counter++;
                    rx_counter=0;
                }else{
                    //読み捨て
                    rx_counter++;
                }
                break;
            case 6:
                // 東経or西経
                if(ch==','){
                    rx_sentence_counter++;
                    rx_counter=0;
                }else{
                    //読み捨て
                    rx_counter++;
                }
                break;
            case 7:
                // 地表における移動の速度
                if(ch==','){
                    rx_sentence_counter++;
                    rx_counter=0;
                }else{
                    //読み捨て
                    rx_counter++;
                }
                break;
            case 8:
                // 地表における移動の真方位
                if(ch==','){
                    rx_sentence_counter++;
                    rx_counter=0;
                }else{
                    //読み捨て
                    rx_counter++;
                }
                break;
            case 9:
                // 地表における移動の真方位
                if(ch==','){
                    rx_sentence_counter++;
                    rx_counter=0;
                    uart_putc(UART_ID, 'C');

                    if((gps_valid=='A') && (flg_time_correct==false)){
                        hour = (gps_time[0]-48)*10+(gps_time[1]-48);
                        min = (gps_time[2]-48)*10+(gps_time[3]-48);
                        sec = (gps_time[4]-48)*10+(gps_time[5]-48);

                        // JSTへの補正
                        if(hour>14){
                            hour = hour + 9 -24;
                        }else{
                            hour = hour + 9;
                        }

                        datetime_t t = {
                            .year = 2000+(gps_date[4]-48)*10+(gps_date[5]-48),
                            .month = (gps_date[2]-48)*10+(gps_date[3]-48),
                            .day = (gps_date[0]-48)*10+(gps_date[1]-48),
                            .dotw = 1,
                            .hour = hour,
                            .min = min,
                            .sec = sec
                        };

                        // RTCをリセットしておく
                        reset_block(RESETS_RESET_RTC_BITS);
                        unreset_block_wait(RESETS_RESET_RTC_BITS);
                        rtc_init();

                        rtc_set_datetime(&t);

                        if(sec==59){
                            sec=0;
                        }else{
                            sec++;
                        }
                        // 1秒ごとに割り込みを発生させるため、1秒後に設定
                        datetime_t alarm = {
                            .year  = -1,
                            .month = -1,
                            .day   = -1,
                            .dotw  = -1,
                            .hour  = -1,
                            .min   = -1,
                            .sec   = sec
                        };
                        rtc_set_alarm(&alarm, alarm_callback);
                        flg_time_correct=true;
                    }

                    
                }else{
                    gps_date[rx_counter]=ch;
                    rx_counter++;
                }
                break;
            default:
                rx_sentence_counter=0;
                rx_counter=0;
        }

    }

}

//---------------------------------------------------------
//- core1 entry
//---------------------------------------------------------
// core1はニキシー管の表示処理のみ行う。
void core1_entry(){
    uint32_t mcore_pop;

    while(1){
        mcore_pop = multicore_fifo_pop_blocking();
        if(mcore_pop==0){
            disp_num(disp);
        }
    }
}

char uart_receive(void){
    // 受信待ち
//    while(!uart_is_readable(UART_GPS));
    return uart_getc(UART_GPS);
}

//---------------------------------------------------------
//- Main function (Core0)
//---------------------------------------------------------
int main(){
    float time_raw;
    int i,j;
    char datetime_buf[256];
    char *datetime_str = &datetime_buf[0];
    struct repeating_timer timer1,timer2;
    uint16_t sw_count;
    datetime_t t;
    bool flg_off_animation=false;
    bool flg_on_animation=true;

    // ハードウェア初期化
    hardware_init();

    // 最初は表示しない
    for(i=0;i<6;i++){
        disp[i] = 10;
    }

    // タイマのセッティング
    hw_set_bits(&timer_hw->inte, 1u<<0);        // Alarm0
    irq_set_exclusive_handler(TIMER_IRQ_0, timer_alarm0_irq);
    irq_set_enabled(TIMER_IRQ_0, true);
    uint64_t target = timer_hw->timerawl + 1000000; // interval 1s
    timer_hw->alarm[0] = (uint32_t)target;

    hw_set_bits(&timer_hw->inte, 1u<<1);        // Alarm1
    irq_set_exclusive_handler(TIMER_IRQ_1, timer_alerm1_irq);
    irq_set_enabled(TIMER_IRQ_1, true);
    target = timer_hw->timerawl + 100000;   // interval 100ms
    timer_hw->alarm[1] = (uint32_t)target;

    // Core1に処理を割り当て
    multicore_launch_core1(core1_entry);

    // BME280の初期化
    uint8_t id;
    read_registers(0xD0, &id, 1);
    read_compensation_parameters();

    write_register(0xF2, 0x01);
    write_register(0xF4, 0x27);
    write_register(0xF5, 0xA0);

    // ピンクフィルタの初期化
    init_pink();;

    // Flash data read and initialize
    flash_read(flash_data.flash_byte);

    if(flash_data.flash_contents.writed!=0xA5){
        // if not writed, flash initialized
        flash_data.flash_contents.writed = 0xA5;
        for(i=0;i<6;i++) flash_data.flash_contents.adj_duty[i] = 0;
        flash_data.flash_contents.fluc_amp=0;
        flash_data.flash_contents.switch_mode=1;
        flash_data.flash_contents.sleep_on=1;
        flash_data.flash_contents.off_hour=22;
        flash_data.flash_contents.off_min=0;
        flash_data.flash_contents.off_sec=0;
        flash_data.flash_contents.on_hour=6;
        flash_data.flash_contents.on_min=0;
        flash_data.flash_contents.on_sec=0;
        flash_data.flash_contents.hour_demo=1;
        flash_write(flash_data.flash_byte);
    }

    while(1){
        switch(operation_mode){
            //---- MODE: WAKEUP ----------------------------------
            case WAKEUP:

                // power up demonstration
                sleep_ms(500);
                for(j=0;j<10;j++){
                    for(i=0;i<6;i++){
                        disp[i]=j;
                    }
                    multicore_fifo_push_blocking(0);
                    sleep_ms(300);
                }

                // 乱数&ファームウェアバージョン表示
                rtc_get_datetime(&t);
                srand(t.month+t.day+t.hour+t.min+t.sec);
                for(j=0;j<(6*40+100);j++){
                    for(i=0;i<6;i++){
                        if(j<(i*20)){
                            disp[i] = disp[i];
                        }else if(j<((i*40)+100)){
                            disp[i] = (uint8_t)(rand()%10)+0x30;
                        }else{
                            disp[i]=version[i];
                        }
                    }
                    sleep_ms(5);
                    multicore_fifo_push_blocking(0);
                }
                sleep_ms(500);

                operation_mode=CLOCK;

                break;

            //---- MODE: CLOCK -----------------------------------
            case CLOCK:
                
                // SWITCH CONTROL : SWA
                if(!gpio_get(SWA_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWA_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し
                        operation_mode = SETUP;
                        disp[0] = flash_data.flash_contents.switch_mode;
                        disp[1] = flash_data.flash_contents.hour_demo;
                        disp[2] = flash_data.flash_contents.sleep_on;
                        disp[3] = (uint8_t)(flash_data.flash_contents.fluc_amp/100);
                        disp[4] = (uint8_t)(flash_data.flash_contents.fluc_amp/10)%10;
                        disp[5] = (uint8_t)(flash_data.flash_contents.fluc_amp%10);
                        multicore_fifo_push_blocking(0);
                        cursor=0;

                    }else if(sw_count>10){
                        // 短押し
                    }

                    while(!gpio_get(SWA_PIN));
                }

                // SWITCH CONTROL : SWB
                if(!gpio_get(SWB_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWB_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し
                        operation_mode = RANDOM;
                        for(i=0;i<6;i++){
                            disp[i]=0;
                        }       
                        multicore_fifo_push_blocking(0);

                    }else if(sw_count>10){
                        // 短押し
                    }

                    while(!gpio_get(SWB_PIN));
                }
                
                // SWITCH CONTROL : SWC
                if(!gpio_get(SWC_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWC_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し
                        operation_mode = LIGHT_ADJ;
                        num=0;
                        for(i=0;i<6;i++){
                            disp[i]=num;
                        }       
                        multicore_fifo_push_blocking(0);

                    }else if(sw_count>10){
                        // 短押し
                    }

                    while(!gpio_get(SWC_PIN));
                }

                if(flg_off && !flg_off_animation){
                    // OFFするときのアニメーション
                    for(j=0;j<(6*40+100);j++){
                        for(i=0;i<6;i++){
                            if(j<(i*20)){
                                disp[i] = disp[i];
                            }else if(j<((i*40)+100)){
                                disp[i] = (uint8_t)(rand()%10)+0x30;
                            }else{
                                disp[i]=10;
                            }
                        }
                        sleep_ms(5);
                        multicore_fifo_push_blocking(0);
                    }
                    flg_off_animation=true;
                }else if(!flg_off){
                    flg_off_animation=false;
                }

                if(flg_on && !flg_on_animation){
                    // ONするときのアニメーション
                    for(j=0;j<(6*40+100);j++){
                        rtc_get_datetime(&t);
                        for(i=0;i<6;i++){
                            if(j<(i*20)){
                                disp[i] = 10;       // 消灯
                            }else if(j<((i*40)+100)){
                                disp[i] = (uint8_t)(rand()%10)+0x30;
                            }else{
                                switch(i){
                                    case 0:
                                        disp[0] = t.hour/10;
                                        break;
                                    case 1:
                                        disp[1] = t.hour%10;
                                        break;
                                    case 2:
                                        disp[2] = t.min/10;
                                        break;
                                    case 3:
                                        disp[3] = t.min%10;
                                        break;
                                    case 4:
                                        disp[4] = t.sec/10;
                                        break;
                                    case 5:
                                        disp[5] = t.sec%10;
                                        break;
                                }
                            }
                        }
                        sleep_ms(5);
                        multicore_fifo_push_blocking(0);
                    }
                    flg_on_animation=true;
                    flg_on=false;
                    flg_off=false;
                }else if(flg_off){
                    flg_on_animation=false;
                }
                
                if(flg_random && !flg_off){
                    rtc_get_datetime(&t);
                    srand(t.month+t.day+t.hour+t.min+t.sec);
                    t.sec+=7;
                    if(t.sec>=60){
                        t.sec-=60;
                        if(++t.min>=60){
                            t.min=0;
                            if(++t.hour>=24){
                                t.hour=0;
                            }
                        }
                    }
                    for(j=0;j<780;j++){
                        for(i=0;i<6;i++){
                            if(j<((i*80)+300)){
                                disp[i] = (uint8_t)(rand()%10)+0x30;
                            }else{
                                switch(i){
                                    case 0:
                                        disp[i]=t.hour/10;
                                        break;
                                    case 1:
                                        disp[i]=t.hour%10;
                                        break;
                                    case 2:
                                        disp[i]=t.min/10;
                                        break;
                                    case 3:
                                        disp[i]=t.min%10;
                                        break;
                                    case 4:
                                        disp[i]=t.sec/10;
                                        break;
                                    case 5:
                                        disp[i]=t.sec%10;
                                        break;
                                }
                            }
                        }
                        sleep_ms(10);
                        multicore_fifo_push_blocking(0);
                    }
                    flg_random=false;
                }

                //---- GPSから時刻を取得 ----
                // GPRMCのみ使用する
                char buf[20];
                uint8_t hour,min,sec;
                if(flg_time_correct==false){
                    //while(uart_is_readable(UART_GPS)){
                    //    uart_getc(UART_GPS);
                    //}
                    if(uart_receive()=='$'){
                        if(uart_receive()=='G'){
                            if(uart_receive()=='P'){
                                if(uart_receive()=='R'){
                                    if(uart_receive()=='M'){
                                        if(uart_receive()=='C'){
                                            if(uart_receive()==','){
                                                for(i=0;i<20;i++){
                                                    buf[i]=uart_receive();
                                                }
                                                if(buf[11]=='A'){
                                                    uint64_t target = timer_hw->timerawl + 1250000; // interval 1s
                                                    // RTCをリセットしておく
//                                                    reset_block(RESETS_RESET_RTC_BITS);
                                                    hour = ((buf[0]-0x30)*10 + (buf[1]-0x30) + 9)%24;
                                                    min = (buf[2]-0x30)*10 + (buf[3]-0x30);
                                                    sec = (buf[4]-0x30)*10 + (buf[5]-0x30);

                                                    datetime_t t = {
                                                        .year = 2000,
                                                        .month = 1,
                                                        .day = 1,
                                                        .dotw = 1,
                                                        .hour = hour,
                                                        .min = min,
                                                        .sec = sec
                                                    };

//                                                    unreset_block_wait(RESETS_RESET_RTC_BITS);
//                                                    rtc_init();

                                                    rtc_set_datetime(&t);

                                                    if(sec==59){
                                                        sec=0;
                                                    }else{
                                                        sec++;
                                                    }
                                                    // 1秒ごとに割り込みを発生させるため、1秒後に設定
                                                    datetime_t alarm = {
                                                        .year  = -1,
                                                        .month = -1,
                                                        .day   = -1,
                                                        .dotw  = -1,
                                                        .hour  = -1,
                                                        .min   = -1,
                                                        .sec   = sec
                                                    };
                                                    rtc_set_alarm(&alarm, alarm_callback);
                                                    timer_hw->alarm[0] = (uint32_t)target;
                                                    flg_time_correct=true;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }


                break;

            //---- MODE: RANDOM NUMBER -----------------------------------
            case RANDOM:

                // SWITCH CONTROL : SWA
                if(!gpio_get(SWA_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWA_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し
                        operation_mode = CLOCK;
                        if(flg_off){
                            // OFFするときのアニメーション
                            for(j=0;j<(6*40+100);j++){
                                for(i=0;i<6;i++){
                                    if(j<(i*20)){
                                        disp[i] = disp[i];
                                    }else if(j<((i*40)+100)){
                                        disp[i] = (uint8_t)(rand()%10)+0x30;
                                    }else{
                                        disp[i]=10;
                                    }
                                }
                                sleep_ms(5);
                                multicore_fifo_push_blocking(0);
                            }
                        }

                    }else if(sw_count>10){
                        // 短押し
                        rtc_get_datetime(&t);
                        srand(t.month+t.day+t.hour+t.min+t.sec);
                        for(j=0;j<780;j++){
                            for(i=0;i<6;i++){
                                if(j<((i*80)+300)){
                                    disp[i] = (uint8_t)(rand()%10)+0x30;
                                }else{
                                    disp[i]=disp[i]&0x0F;
                                }
                            }
                            sleep_ms(10);
                            multicore_fifo_push_blocking(0);
                        }

                    }

                    while(!gpio_get(SWA_PIN));
                }

                // SWITCH CONTROL : SWB
                if(!gpio_get(SWB_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWB_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し
                        operation_mode = DEMO;

                    }else if(sw_count>10){
                        // 短押し

                    }

                    while(!gpio_get(SWB_PIN));
                }

                break;

            //---- MODE: DEMO  -----------------------------------
            case DEMO:
                rtc_get_datetime(&t);
                srand(t.month+t.day+t.hour+t.min+t.sec);
                for(j=0;j<780;j++){
                    for(i=0;i<6;i++){
                        if(j<((i*80)+300)){
                            disp[i] = (uint8_t)(rand()%10)+0x30;
                        }else{
                            disp[i]=disp[i]&0x0F;
                        }
                    }
                    sleep_ms(10);
                    multicore_fifo_push_blocking(0);
                }

                // SWITCH CONTROL : SWA 
                // ランダムの後にこの処理を行わないと表示が異常となる。
                if(!gpio_get(SWA_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWA_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し
                        operation_mode = CLOCK;
                        if(flg_off){
                            // OFFするときのアニメーション
                            for(j=0;j<(6*40+100);j++){
                                for(i=0;i<6;i++){
                                    if(j<(i*20)){
                                        disp[i] = disp[i];
                                    }else if(j<((i*40)+100)){
                                        disp[i] = (uint8_t)(rand()%10)+0x30;
                                    }else{
                                        disp[i]=10;
                                    }
                                }
                                sleep_ms(5);
                                multicore_fifo_push_blocking(0);
                            }
                        }

                    }else if(sw_count>10){
                        // 短押し

                    }

                    while(!gpio_get(SWA_PIN));
                }

                sleep_ms(1000);
                

                break;

            //---- MODE: SETUP--------------------------------------
            case SETUP:

                // SWITCH CONTROL : SWA
                if(!gpio_get(SWA_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWA_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し
                        operation_mode = CLOCK;
                        if(flash_data.flash_contents.sleep_on==0) flg_off=false; 
                        flash_write(flash_data.flash_byte);
                        if(flg_off){
                            // OFFするときのアニメーション
                            for(j=0;j<(6*40+100);j++){
                                for(i=0;i<6;i++){
                                    if(j<(i*20)){
                                        disp[i] = disp[i];
                                    }else if(j<((i*40)+100)){
                                        disp[i] = (uint8_t)(rand()%10)+0x30;
                                    }else{
                                        disp[i]=10;
                                    }
                                }
                                sleep_ms(5);
                                multicore_fifo_push_blocking(0);
                            }
                        }

                    }else if(sw_count>10){
                        // 短押し
                        blink_counter=1;
                        sleep_ms(40);
                        blink_counter=5;
                        if(cursor==0) cursor=5;
                        else cursor--;
                    }

                    while(!gpio_get(SWA_PIN));
                }

                // SWITCH CONTROL : SWB
                if(!gpio_get(SWB_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWB_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し

                    }else if(sw_count>10){
                        // 短押し
                        switch(cursor){
                            case 0:
                                if(flash_data.flash_contents.switch_mode==3) flash_data.flash_contents.switch_mode=0;
                                else flash_data.flash_contents.switch_mode++;
                                disp[0]=flash_data.flash_contents.switch_mode;
                                break;
                            case 1:
                                if(flash_data.flash_contents.hour_demo==1) flash_data.flash_contents.hour_demo=0;
                                else flash_data.flash_contents.hour_demo=1;
                                disp[1]=flash_data.flash_contents.hour_demo;
                                break;
                            case 2:
                                if(flash_data.flash_contents.sleep_on==1) flash_data.flash_contents.sleep_on=0;
                                else flash_data.flash_contents.sleep_on=1;
                                disp[2]=flash_data.flash_contents.sleep_on;
                                break;
                            case 3:
                                if(flash_data.flash_contents.fluc_amp>=900) flash_data.flash_contents.fluc_amp-=900;
                                else flash_data.flash_contents.fluc_amp+=100;
                                disp[3] = (uint8_t)(flash_data.flash_contents.fluc_amp/100);
                                disp[4] = (uint8_t)(flash_data.flash_contents.fluc_amp/10)%10;
                                disp[5] = (uint8_t)(flash_data.flash_contents.fluc_amp%10);
                                break;
                            case 4:
                                if((flash_data.flash_contents.fluc_amp%100)>=90) flash_data.flash_contents.fluc_amp-=90;
                                else flash_data.flash_contents.fluc_amp+=10;
                                disp[3] = (uint8_t)(flash_data.flash_contents.fluc_amp/100);
                                disp[4] = (uint8_t)(flash_data.flash_contents.fluc_amp/10)%10;
                                disp[5] = (uint8_t)(flash_data.flash_contents.fluc_amp%10);
                                break;
                            case 5:
                                if((flash_data.flash_contents.fluc_amp%10)>=9) flash_data.flash_contents.fluc_amp-=9;
                                else flash_data.flash_contents.fluc_amp++;
                                disp[3] = (uint8_t)(flash_data.flash_contents.fluc_amp/100);
                                disp[4] = (uint8_t)(flash_data.flash_contents.fluc_amp/10)%10;
                                disp[5] = (uint8_t)(flash_data.flash_contents.fluc_amp%10);
                                break;

                            
                        }
                        multicore_fifo_push_blocking(0);

                    }

                    while(!gpio_get(SWB_PIN));
                }

                // SWITCH CONTROL : SWC
                if(!gpio_get(SWC_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWC_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し

                    }else if(sw_count>10){
                        // 短押し
                        blink_counter=1;
                        sleep_ms(40);
                        blink_counter=5;
                        if(cursor==5) cursor=0;
                        else cursor++;
                    }

                    while(!gpio_get(SWC_PIN));
                }

                break;

            //---- MODE: LIGHT ADJUST ------------------------------
            case LIGHT_ADJ:

                // SWITCH CONTROL : SWA
                if(!gpio_get(SWA_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWA_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し
                        operation_mode = CLOCK;
                        if(flg_off){
                            // OFFするときのアニメーション
                            for(j=0;j<(6*40+100);j++){
                                for(i=0;i<6;i++){
                                    if(j<(i*20)){
                                        disp[i] = disp[i];
                                    }else if(j<((i*40)+100)){
                                        disp[i] = (uint8_t)(rand()%10)+0x30;
                                    }else{
                                        disp[i]=11;
                                    }
                                }
                                sleep_ms(5);
                                multicore_fifo_push_blocking(0);
                            }
                        }
                        flash_write(flash_data.flash_byte);

                    }else if(sw_count>10){
                        // 短押し
                        blink_counter=1;
                        sleep_ms(40);
                        blink_counter=5;
                        if(cursor==0) cursor=5;
                        else cursor--;
                    }

                    while(!gpio_get(SWA_PIN));
                }

                // SWITCH CONTROL : SWB
                if(!gpio_get(SWB_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWB_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し
                        num++;
                        if(num>9) num=0;

                        for(i=0;i<6;i++){
                            disp[i]=num;
                        }       
                        multicore_fifo_push_blocking(0);

                    }else if(sw_count>10){
                        // 短押し
                        flash_data.flash_contents.adj_duty[cursor]+=10;
                    }

                    while(!gpio_get(SWB_PIN));
                }
                
                // SWITCH CONTROL : SWC
                if(!gpio_get(SWC_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWC_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し
                        operation_mode=SLEEP_TIME;
                        disp[0] = flash_data.flash_contents.off_hour/10;
                        disp[1] = flash_data.flash_contents.off_hour%10;
                        disp[2] = flash_data.flash_contents.off_min/10;
                        disp[3] = flash_data.flash_contents.off_min%10;
                        disp[4] = flash_data.flash_contents.off_sec/10;
                        disp[5] = flash_data.flash_contents.off_sec%10;
                        multicore_fifo_push_blocking(0);

                    }else if(sw_count>10){
                        // 短押し
                        flash_data.flash_contents.adj_duty[cursor]-=10;
                    }

                    while(!gpio_get(SWC_PIN));
                }
                break;

            //---- MODE: SLEEP TIME SETTING ------------------------
            case SLEEP_TIME:

                // SWITCH CONTROL : SWA 
                if(!gpio_get(SWA_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWA_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し
                        operation_mode = CLOCK;
                        if(flg_off){
                            // OFFするときのアニメーション
                            for(j=0;j<(6*40+100);j++){
                                for(i=0;i<6;i++){
                                    if(j<(i*20)){
                                        disp[i] = disp[i];
                                    }else if(j<((i*40)+100)){
                                        disp[i] = (uint8_t)(rand()%10)+0x30;
                                    }else{
                                        disp[i]=11;
                                    }
                                }
                                sleep_ms(5);
                                multicore_fifo_push_blocking(0);
                            }
                        }
                        flash_write(flash_data.flash_byte);

                    }else if(sw_count>10){
                        // 短押し
                        flash_data.flash_contents.off_hour++;
                        if(flash_data.flash_contents.off_hour==24) flash_data.flash_contents.off_hour=0;
                        
                        disp[0] = flash_data.flash_contents.off_hour/10;
                        disp[1] = flash_data.flash_contents.off_hour%10;
                        disp[2] = flash_data.flash_contents.off_min/10;
                        disp[3] = flash_data.flash_contents.off_min%10;
                        disp[4] = flash_data.flash_contents.off_sec/10;
                        disp[5] = flash_data.flash_contents.off_sec%10;
                        multicore_fifo_push_blocking(0);
                    }

                    while(!gpio_get(SWA_PIN));
                }

                // SWITCH CONTROL : SWB
                if(!gpio_get(SWB_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWB_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し

                    }else if(sw_count>10){
                        // 短押し
                        flash_data.flash_contents.off_min++;
                        if(flash_data.flash_contents.off_min==60) flash_data.flash_contents.off_min=0;
                        
                        disp[0] = flash_data.flash_contents.off_hour/10;
                        disp[1] = flash_data.flash_contents.off_hour%10;
                        disp[2] = flash_data.flash_contents.off_min/10;
                        disp[3] = flash_data.flash_contents.off_min%10;
                        disp[4] = flash_data.flash_contents.off_sec/10;
                        disp[5] = flash_data.flash_contents.off_sec%10;
                        multicore_fifo_push_blocking(0);
                    }

                    while(!gpio_get(SWB_PIN));
                }

                // SWITCH CONTROL : SWC
                if(!gpio_get(SWC_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWC_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し
                        operation_mode=WAKE_TIME;
                        disp[0] = flash_data.flash_contents.on_hour/10;
                        disp[1] = flash_data.flash_contents.on_hour%10;
                        disp[2] = flash_data.flash_contents.on_min/10;
                        disp[3] = flash_data.flash_contents.on_min%10;
                        disp[4] = flash_data.flash_contents.on_sec/10;
                        disp[5] = flash_data.flash_contents.on_sec%10;
                        multicore_fifo_push_blocking(0);

                    }else if(sw_count>10){
                        // 短押し
                        flash_data.flash_contents.off_sec++;
                        if(flash_data.flash_contents.off_sec==60) flash_data.flash_contents.off_sec=0;

                        disp[0] = flash_data.flash_contents.off_hour/10;
                        disp[1] = flash_data.flash_contents.off_hour%10;
                        disp[2] = flash_data.flash_contents.off_min/10;
                        disp[3] = flash_data.flash_contents.off_min%10;
                        disp[4] = flash_data.flash_contents.off_sec/10;
                        disp[5] = flash_data.flash_contents.off_sec%10;
                        multicore_fifo_push_blocking(0);
                    }

                    while(!gpio_get(SWC_PIN));
                }

                break;

            //---- MODE: WAKEUP TIME SETTING -----------------------
            case WAKE_TIME:

                // SWITCH CONTROL : SWA 
                if(!gpio_get(SWA_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWA_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し
                        if(flg_off){
                            // OFFするときのアニメーション
                            for(j=0;j<(6*40+100);j++){
                                for(i=0;i<6;i++){
                                    if(j<(i*20)){
                                        disp[i] = disp[i];
                                    }else if(j<((i*40)+100)){
                                        disp[i] = (uint8_t)(rand()%10)+0x30;
                                    }else{
                                        disp[i]=11;
                                    }
                                }
                                sleep_ms(5);
                                multicore_fifo_push_blocking(0);
                            }
                        }
                        operation_mode = CLOCK;
                        flash_write(flash_data.flash_byte);

                    }else if(sw_count>10){
                        // 短押し
                        flash_data.flash_contents.on_hour++;
                        if(flash_data.flash_contents.on_hour==24) flash_data.flash_contents.on_hour=0;
                        
                        disp[0] = flash_data.flash_contents.on_hour/10;
                        disp[1] = flash_data.flash_contents.on_hour%10;
                        disp[2] = flash_data.flash_contents.on_min/10;
                        disp[3] = flash_data.flash_contents.on_min%10;
                        disp[4] = flash_data.flash_contents.on_sec/10;
                        disp[5] = flash_data.flash_contents.on_sec%10;
                        multicore_fifo_push_blocking(0);
                    }

                    while(!gpio_get(SWA_PIN));
                }

                // SWITCH CONTROL : SWB
                if(!gpio_get(SWB_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWB_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し

                    }else if(sw_count>10){
                        // 短押し
                        flash_data.flash_contents.on_min++;
                        if(flash_data.flash_contents.on_min==60) flash_data.flash_contents.on_min=0;
                        
                        disp[0] = flash_data.flash_contents.on_hour/10;
                        disp[1] = flash_data.flash_contents.on_hour%10;
                        disp[2] = flash_data.flash_contents.on_min/10;
                        disp[3] = flash_data.flash_contents.on_min%10;
                        disp[4] = flash_data.flash_contents.on_sec/10;
                        disp[5] = flash_data.flash_contents.on_sec%10;
                        multicore_fifo_push_blocking(0);
                    }

                    while(!gpio_get(SWB_PIN));
                }

                // SWITCH CONTROL : SWC
                if(!gpio_get(SWC_PIN)){
                    // チャタリング対策
                    sleep_ms(10);
                    sw_count=0;
                    while(!gpio_get(SWC_PIN)){
                        sw_count++;
                        if(sw_count==150){
                            break;
                        }
                        sleep_ms(10);
                    }

                    if(sw_count==150){
                        // 長押し
                        operation_mode = LIGHT_ADJ;
                        num=0;
                        for(i=0;i<6;i++){
                            disp[i]=num;
                        }       
                        multicore_fifo_push_blocking(0);

                    }else if(sw_count>10){
                        // 短押し
                        flash_data.flash_contents.on_sec++;
                        if(flash_data.flash_contents.on_sec==60) flash_data.flash_contents.on_sec=0;

                        disp[0] = flash_data.flash_contents.on_hour/10;
                        disp[1] = flash_data.flash_contents.on_hour%10;
                        disp[2] = flash_data.flash_contents.on_min/10;
                        disp[3] = flash_data.flash_contents.on_min%10;
                        disp[4] = flash_data.flash_contents.on_sec/10;
                        disp[5] = flash_data.flash_contents.on_sec%10;
                        multicore_fifo_push_blocking(0);
                    }

                    while(!gpio_get(SWC_PIN));
                }

                break;
            

        }

   }
}


void hardware_init(void){
    int i;

    //---- initialize clock ------------
    sleep_ms(10);
    xosc_init();
    clock_configure(clk_sys,
                    CLOCKS_CLK_REF_CTRL_SRC_VALUE_CLKSRC_CLK_REF_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    12*MHZ,
                    125*MHZ);
    clock_configure(clk_ref,
                    CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
                    CLOCKS_CLK_REF_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    12*MHZ,
                    125*MHZ);
    clock_configure(clk_rtc,
                    CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
                    CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
                    12*MHZ,
                    47*KHZ);

    //---- RTC -------------------------
    datetime_t t = {
        .year = 2022,
        .month = 04,
        .day = 18,
        .dotw = 1,
        .hour = 00,
        .min = 00,
        .sec = 00
    };

    rtc_init();
    rtc_set_datetime(&t);

    // Alarm once a minute
    datetime_t alarm = {
        .year  = -1,
        .month = -1,
        .day   = -1,
        .dotw  = -1,
        .hour  = -1,
        .min   = -1,
        .sec   = 01
    };
    rtc_set_alarm(&alarm, alarm_callback);

    //---- UART -------------------------
    // UART INIT(Debug)
    uart_init(UART_DEBUG, 2400);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    int __unused actual = uart_set_baudrate(UART_DEBUG, BAUD_RATE);
    uart_set_hw_flow(UART_DEBUG, false, false);
    uart_set_format(UART_DEBUG, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_DEBUG, false);
    uart_set_irq_enables(UART_DEBUG, false, false);

    // UART INIT(GPS)
    uart_init(UART_GPS, 9600);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);

    int __unused actual1 = uart_set_baudrate(UART_GPS, BAUD_RATE_GPS);
    uart_set_hw_flow(UART_GPS, false, false);
    uart_set_format(UART_GPS, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_GPS, false);
    int UART_IRQ = UART_GPS == uart0 ? UART0_IRQ : UART1_IRQ;

    uart_set_irq_enables(UART_GPS, true, false);
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, false);

    rx_counter = 0;
    rx_sentence_counter = 0;

    // for debug
    char buf[20];
    int *num;
    num = (CLOCKS_BASE+0x74);
    sprintf(buf, "%x\n\r", *num);
    uart_puts(UART_ID, buf);

    //---- GPIO ---------------------------
    gpio_init(HV_EN_PIN);
    gpio_set_dir(HV_EN_PIN, GPIO_OUT);
    gpio_put(HV_EN_PIN, 0);             // High voltage supply disable

    gpio_init(OUT_K3L_PIN);
    gpio_init(OUT_K3R_PIN);
    gpio_init(OUT_K4L_PIN);
    gpio_init(OUT_K4R_PIN);
    gpio_init(OUT_K5L_PIN);
    gpio_init(OUT_K5R_PIN);
    gpio_init(OUT_K6L_PIN);
    gpio_init(OUT_K6R_PIN);

    gpio_set_dir(OUT_K3L_PIN, GPIO_OUT);
    gpio_set_dir(OUT_K3R_PIN, GPIO_OUT);
    gpio_set_dir(OUT_K4L_PIN, GPIO_OUT);
    gpio_set_dir(OUT_K4R_PIN, GPIO_OUT);
    gpio_set_dir(OUT_K5L_PIN, GPIO_OUT);
    gpio_set_dir(OUT_K5R_PIN, GPIO_OUT);
    gpio_set_dir(OUT_K6L_PIN, GPIO_OUT);
    gpio_set_dir(OUT_K6R_PIN, GPIO_OUT);

    gpio_put(OUT_K3L_PIN, 0);
    gpio_put(OUT_K3R_PIN, 0);
    gpio_put(OUT_K4L_PIN, 0);
    gpio_put(OUT_K4R_PIN, 0);
    gpio_put(OUT_K5L_PIN, 0);
    gpio_put(OUT_K5R_PIN, 0);
    gpio_put(OUT_K6L_PIN, 0);
    gpio_put(OUT_K6R_PIN, 0);

    // Cathode Driver
    gpio_init(OE_PIN);
    gpio_init(LE_N_PIN);
    gpio_init(CLK_PIN);
    gpio_init(DATA_PIN);
    gpio_init(SWA_PIN);
    gpio_init(SWB_PIN);
    gpio_init(SWC_PIN);
    gpio_set_dir(OE_PIN, GPIO_OUT);
    gpio_set_dir(LE_N_PIN, GPIO_OUT);
    gpio_set_dir(CLK_PIN, GPIO_OUT);
    gpio_set_dir(DATA_PIN, GPIO_OUT);
    gpio_set_dir(SWA_PIN, GPIO_IN);
    gpio_set_dir(SWB_PIN, GPIO_IN);
    gpio_set_dir(SWC_PIN, GPIO_IN);
    gpio_put(OE_PIN, 1);
    gpio_put(LE_N_PIN, 1);
    gpio_put(CLK_PIN, 0);
    gpio_put(DATA_PIN, 0);
    gpio_pull_up(SWA_PIN);
    gpio_pull_up(SWB_PIN);
    gpio_pull_up(SWC_PIN);

    for(i=0; i<64; i++){
        gpio_put(DATA_PIN, 1);
        sleep_us(100);
        gpio_put(CLK_PIN, 1);           // clock rising
        sleep_us(100);
        gpio_put(CLK_PIN, 0);           // clock falling
        sleep_us(100);
    }

    //---- PWM -------------------------
    uint slice_num0 = pwm_gpio_to_slice_num(PWM0A_PIN);
    uint slice_num1 = pwm_gpio_to_slice_num(PWM1A_PIN);
    uint slice_num2 = pwm_gpio_to_slice_num(PWM2A_PIN);
    uint slice_num3 = pwm_gpio_to_slice_num(PWM3A_PIN);
    uint slice_num4 = pwm_gpio_to_slice_num(PWM4A_PIN);
    uint slice_num5 = pwm_gpio_to_slice_num(PWM5A_PIN);
    gpio_set_function(PWM0A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(PWM1A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(PWM2A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(PWM3A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(PWM4A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(PWM5A_PIN, GPIO_FUNC_PWM);

    pwm_set_wrap(slice_num0, 30000);
    pwm_set_chan_level(slice_num0, PWM_CHAN_A, 600+duty_comp[0]+flash_data.flash_contents.adj_duty[0]);
    pwm_set_enabled(slice_num0, true);

    pwm_set_wrap(slice_num1, 30000);
    pwm_set_chan_level(slice_num1, PWM_CHAN_A, 600+duty_comp[1]+flash_data.flash_contents.adj_duty[1]);
    pwm_set_enabled(slice_num1, true);

    pwm_set_wrap(slice_num2, 30000);
    pwm_set_chan_level(slice_num2, PWM_CHAN_A, 600+duty_comp[2]+flash_data.flash_contents.adj_duty[2]);
    pwm_set_enabled(slice_num2, true);

    pwm_set_wrap(slice_num3, 30000);
    pwm_set_chan_level(slice_num3, PWM_CHAN_A, 600+duty_comp[3]+flash_data.flash_contents.adj_duty[3]);
    pwm_set_enabled(slice_num3, true);

    pwm_set_wrap(slice_num4, 30000);
    pwm_set_chan_level(slice_num4, PWM_CHAN_A, 600+duty_comp[4]+flash_data.flash_contents.adj_duty[4]);
    pwm_set_enabled(slice_num4, true);
    
    pwm_set_wrap(slice_num5, 30000);
    pwm_set_chan_level(slice_num5, PWM_CHAN_A, 600+duty_comp[5]+flash_data.flash_contents.adj_duty[5]);
    pwm_set_enabled(slice_num5, true);

    //---- ADC ------------------------------
    adc_init();
    adc_gpio_init(LSEN_PIN);
    adc_select_input(3);

    //---- SPI(BME280) Setting --------------
    spi_init(spi_default, 500*1000);
    gpio_set_function(SENS_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SENS_SDI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SENS_SDO_PIN, GPIO_FUNC_SPI);
    bi_decl(bi_3pins_with_func(SENS_SDI_PIN, SENS_SDO_PIN, SENS_SCK_PIN, GPIO_FUNC_SPI));
    gpio_init(SENS_CSB_PIN);
    gpio_set_dir(SENS_CSB_PIN, GPIO_OUT);
    gpio_put(SENS_CSB_PIN, 1);
    bi_decl(bi_1pin_with_name(SENS_CSB_PIN, "SPI_CS"));

    
    gpio_put(HV_EN_PIN, 1);             // High voltage supply enable
}

void set_duty(uint16_t duty){
    int i;
    uint16_t temp;

    uint slice_num[6] = {pwm_gpio_to_slice_num(PWM0A_PIN),
                        pwm_gpio_to_slice_num(PWM1A_PIN),
                        pwm_gpio_to_slice_num(PWM2A_PIN),
                        pwm_gpio_to_slice_num(PWM3A_PIN),
                        pwm_gpio_to_slice_num(PWM4A_PIN),
                        pwm_gpio_to_slice_num(PWM5A_PIN)};
    
    for(i=0;i<6;i++){
        if((disp[i]&0xF0)==0){
            temp = duty+duty_comp[i]+flash_data.flash_contents.adj_duty[i]+duty_num_comp[(disp[i]&0x0F)];
        }else if(((disp[0]&0xF0)==0x10) || ((disp[0]&0xF0)==0x20)){
            temp = duty+duty_comp[i]+flash_data.flash_contents.adj_duty[i]+duty_num_comp[(disp[i]&0x0F)]+duty_num_comp[11];
        }else{
            temp = duty+duty_comp[i]+flash_data.flash_contents.adj_duty[i]+duty_num_comp[(disp[i]&0x0F)]+duty_num_comp[12]; 
        }

        if(temp>3000) temp=3000;        // 異常に明るくならないための安全策
        
        pwm_set_chan_level(slice_num[i], PWM_CHAN_A, temp);

    }
}

void set_dutys(uint16_t *duty){
    int i;
    uint16_t temp;

    uint slice_num[6] = {pwm_gpio_to_slice_num(PWM0A_PIN),
                        pwm_gpio_to_slice_num(PWM1A_PIN),
                        pwm_gpio_to_slice_num(PWM2A_PIN),
                        pwm_gpio_to_slice_num(PWM3A_PIN),
                        pwm_gpio_to_slice_num(PWM4A_PIN),
                        pwm_gpio_to_slice_num(PWM5A_PIN)};

    for(i=0;i<6;i++){
        if((disp[i]&0xF0)==0){
            temp = duty[i]+duty_comp[i]+flash_data.flash_contents.adj_duty[i]+duty_num_comp[(disp[i]&0x0F)];
        }else if(((disp[0]&0xF0)==0x10) || ((disp[0]&0xF0)==0x20)){
            temp = duty[i]+duty_comp[i]+flash_data.flash_contents.adj_duty[i]+duty_num_comp[(disp[i]&0x0F)]+duty_num_comp[11];
        }else{
            temp = duty[i]+duty_comp[i]+flash_data.flash_contents.adj_duty[i]+duty_num_comp[(disp[i]&0x0F)]+duty_num_comp[12];        
        }

        if(temp>3000) temp=3000;        // 異常に明るくならないための安全策
        
        pwm_set_chan_level(slice_num[i], PWM_CHAN_A, temp); 

    }
}