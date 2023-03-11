#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"

const uint KR_PIN = 0;
const uint KL_PIN = 1;
const uint K9_PIN = 2;
const uint K8_PIN = 3;
const uint K7_PIN = 4;
const uint K6_PIN = 5;
const uint K5_PIN = 6;
const uint K4_PIN = 7;
const uint K3_PIN = 8;
const uint K2_PIN = 9;
const uint K1_PIN = 10;
const uint K0_PIN = 11;
const uint DBG_TX_PIN = 12;
const uint DBG_RX_PIN = 13;
const uint VCONT_PIN = 14;
const uint DIGIT6_PIN = 15;
const uint DIGIT2_PIN = 16;
const uint DIGIT3_PIN = 17;
const uint DIGIT4_PIN = 18;
const uint DIGIT5_PIN = 19;
const uint PPS_PIN = 20;
const uint GPS_TX_PIN = 21;
const uint DIGIT1_PIN = 22;
const uint SWA_PIN = 23;
const uint SWB_PIN = 24;
const uint SWC_PIN = 25;
const uint DBGLED_PIN = 26;
const uint PPSLED_PIN = 27;
const uint HVEN_PIN = 28;
const uint LSENSOR_PIN = 29;

int main(){
    bi_decl(bi_program_description("This is a test program for nixie6."));

    stdio_init_all();

    gpio_init(KR_PIN);
    gpio_init(KL_PIN);
    gpio_init(K9_PIN);
    gpio_init(K8_PIN);
    gpio_init(K7_PIN);
    gpio_init(K6_PIN);
    gpio_init(K5_PIN);
    gpio_init(K4_PIN);
    gpio_init(K3_PIN);
    gpio_init(K2_PIN);
    gpio_init(K1_PIN);
    gpio_init(K0_PIN);
    gpio_set_dir(KR_PIN, GPIO_OUT);
    gpio_set_dir(KL_PIN, GPIO_OUT);
    gpio_set_dir(K9_PIN, GPIO_OUT);
    gpio_set_dir(K8_PIN, GPIO_OUT);
    gpio_set_dir(K7_PIN, GPIO_OUT);
    gpio_set_dir(K6_PIN, GPIO_OUT);
    gpio_set_dir(K5_PIN, GPIO_OUT);
    gpio_set_dir(K4_PIN, GPIO_OUT);
    gpio_set_dir(K3_PIN, GPIO_OUT);
    gpio_set_dir(K2_PIN, GPIO_OUT);
    gpio_set_dir(K1_PIN, GPIO_OUT);
    gpio_set_dir(K0_PIN, GPIO_OUT);
    gpio_put(KR_PIN,0);
    gpio_put(KL_PIN,0);
    gpio_put(K9_PIN,0);
    gpio_put(K8_PIN,0);
    gpio_put(K7_PIN,0);
    gpio_put(K6_PIN,0);
    gpio_put(K5_PIN,0);
    gpio_put(K4_PIN,0);
    gpio_put(K3_PIN,0);
    gpio_put(K2_PIN,0);
    gpio_put(K1_PIN,0);
    gpio_put(K0_PIN,0);

    gpio_init(DBG_TX_PIN);
    gpio_init(DBG_RX_PIN);
    gpio_set_dir(DBG_TX_PIN, GPIO_OUT);
    gpio_set_dir(DBG_RX_PIN, GPIO_IN);
    gpio_put(DBG_TX_PIN,0);

    gpio_init(VCONT_PIN);
    gpio_set_dir(VCONT_PIN, GPIO_OUT);
    gpio_put(VCONT_PIN,0);

    gpio_init(DIGIT1_PIN);
    gpio_init(DIGIT2_PIN);
    gpio_init(DIGIT3_PIN);
    gpio_init(DIGIT4_PIN);
    gpio_init(DIGIT5_PIN);
    gpio_init(DIGIT6_PIN);
    gpio_set_dir(DIGIT1_PIN, GPIO_OUT);
    gpio_set_dir(DIGIT2_PIN, GPIO_OUT);
    gpio_set_dir(DIGIT3_PIN, GPIO_OUT);
    gpio_set_dir(DIGIT4_PIN, GPIO_OUT);
    gpio_set_dir(DIGIT5_PIN, GPIO_OUT);
    gpio_set_dir(DIGIT6_PIN, GPIO_OUT);
    gpio_put(DIGIT1_PIN,0);
    gpio_put(DIGIT2_PIN,0);
    gpio_put(DIGIT3_PIN,0);
    gpio_put(DIGIT4_PIN,0);
    gpio_put(DIGIT5_PIN,0);
    gpio_put(DIGIT6_PIN,0);

    gpio_init(PPS_PIN);
    gpio_init(GPS_TX_PIN);
    gpio_set_dir(PPS_PIN, GPIO_IN);
    gpio_set_dir(GPS_TX_PIN, GPIO_IN);

    gpio_init(SWA_PIN);
    gpio_init(SWB_PIN);
    gpio_init(SWC_PIN);
    gpio_set_dir(SWA_PIN, GPIO_IN);
    gpio_set_dir(SWB_PIN, GPIO_IN);
    gpio_set_dir(SWC_PIN, GPIO_IN);

    gpio_init(DBGLED_PIN);
    gpio_init(PPSLED_PIN);
    gpio_set_dir(DBGLED_PIN, GPIO_OUT);
    gpio_set_dir(PPSLED_PIN, GPIO_OUT);
    gpio_put(DBGLED_PIN,1);
    gpio_put(PPSLED_PIN,1);

    gpio_init(HVEN_PIN);
    gpio_set_dir(HVEN_PIN, GPIO_OUT);
    gpio_put(HVEN_PIN,0);

    gpio_init(LSENSOR_PIN);
    gpio_set_dir(LSENSOR_PIN, GPIO_IN);

    while (1) {

    }
}