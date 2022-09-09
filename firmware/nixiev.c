#include "nixiev.h"

int32_t t_fine;

uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t dig_H1, dig_H3;
int8_t dig_H6;
int16_t dig_H2, dig_H4, dig_H5;

float z[MAX_Z];
float k[MAX_Z];

uint64_t get_shift_register_data(uint8_t *disp){
    uint64_t serial_data = 0xFFFFFFFFFFFFFFFF;
    uint8_t i;
    uint8_t temp[6];

    for(i=0;i<6;i++){
        temp[i]=disp[i]&0x0F;
    }

    switch(temp[0]){
        case 0:
            serial_data &= ~((uint64_t)1 << 36);
            break;
        case 1:
            serial_data &= ~((uint64_t)1 << 37);
            break;
        case 2:
            serial_data &= ~((uint64_t)1 << 38);
            break;
        case 3:
            serial_data &= ~((uint64_t)1 << 39);
            break;
        case 4:
            serial_data &= ~((uint64_t)1 << 40);
            break;
        case 5:
            serial_data &= ~((uint64_t)1 << 42);
            break;
        case 6:
            serial_data &= ~((uint64_t)1 << 43);
            break;
        case 7:
            serial_data &= ~((uint64_t)1 << 32);
            break;
        case 8:
            serial_data &= ~((uint64_t)1 << 34);
            break;
        case 9:
            serial_data &= ~((uint64_t)1 << 35);
            break;
        default:
            break;
    } 

    switch(temp[1]){
        case 0:
            serial_data &= ~((uint64_t)1 << 48);
            break;
        case 1:
            serial_data &= ~((uint64_t)1 << 49);
            break;
        case 2:
            serial_data &= ~((uint64_t)1 << 50);
            break;
        case 3:
            serial_data &= ~((uint64_t)1 << 51);
            break;
        case 4:
            serial_data &= ~((uint64_t)1 << 52);
            break;
        case 5:
            serial_data &= ~((uint64_t)1 << 54);
            break;
        case 6:
            serial_data &= ~((uint64_t)1 << 55);
            break;
        case 7:
            serial_data &= ~((uint64_t)1 << 44);
            break;
        case 8:
            serial_data &= ~((uint64_t)1 << 46);
            break;
        case 9:
            serial_data &= ~((uint64_t)1 << 47);
            break;
        default:
            break;
    } 

    switch(temp[2]){
        case 0:
            serial_data &= ~((uint64_t)1 << 59);
            break;
        case 1:
            serial_data &= ~((uint64_t)1 << 60);
            break;
        case 2:
            serial_data &= ~((uint64_t)1 << 61);
            break;
        case 3:
            serial_data &= ~((uint64_t)1 << 62);
            break;
        case 4:
            serial_data &= ~((uint64_t)1 << 63);
            break;
        case 5:
            serial_data &= ~((uint64_t)1 << 0);
            break;
        case 6:
            serial_data &= ~((uint64_t)1 << 1);
            break;
        case 7:
            serial_data &= ~((uint64_t)1 << 56);
            break;
        case 8:
            serial_data &= ~((uint64_t)1 << 57);
            break;
        case 9:
            serial_data &= ~((uint64_t)1 << 58);
            break;
        default:
            break;
    } 

    switch(temp[3]){
        case 0:
            serial_data &= ~((uint64_t)1 << 5);
            break;
        case 1:
            serial_data &= ~((uint64_t)1 << 6);
            break;
        case 2:
            serial_data &= ~((uint64_t)1 << 7);
            break;
        case 3:
            serial_data &= ~((uint64_t)1 << 8);
            break;
        case 4:
            serial_data &= ~((uint64_t)1 << 9);
            break;
        case 5:
            serial_data &= ~((uint64_t)1 << 10);
            break;
        case 6:
            serial_data &= ~((uint64_t)1 << 11);
            break;
        case 7:
            serial_data &= ~((uint64_t)1 << 2);
            break;
        case 8:
            serial_data &= ~((uint64_t)1 << 3);
            break;
        case 9:
            serial_data &= ~((uint64_t)1 << 4);
            break;
        default:
            break;
    } 

    switch(temp[4]){
        case 0:
            serial_data &= ~((uint64_t)1 << 15);
            break;
        case 1:
            serial_data &= ~((uint64_t)1 << 16);
            break;
        case 2:
            serial_data &= ~((uint64_t)1 << 17);
            break;
        case 3:
            serial_data &= ~((uint64_t)1 << 18);
            break;
        case 4:
            serial_data &= ~((uint64_t)1 << 19);
            break;
        case 5:
            serial_data &= ~((uint64_t)1 << 21);
            break;
        case 6:
            serial_data &= ~((uint64_t)1 << 20);
            break;
        case 7:
            serial_data &= ~((uint64_t)1 << 12);
            break;
        case 8:
            serial_data &= ~((uint64_t)1 << 13);
            break;
        case 9:
            serial_data &= ~((uint64_t)1 << 14);
            break;
        default:
            break;
    } 

    switch(temp[5]){
        case 0:
            serial_data &= ~((uint64_t)1 << 25);
            break;
        case 1:
            serial_data &= ~((uint64_t)1 << 26);
            break;
        case 2:
            serial_data &= ~((uint64_t)1 << 27);
            break;
        case 3:
            serial_data &= ~((uint64_t)1 << 28);
            break;
        case 4:
            serial_data &= ~((uint64_t)1 << 29);
            break;
        case 5:
            serial_data &= ~((uint64_t)1 << 30);
            break;
        case 6:
            serial_data &= ~((uint64_t)1 << 31);
            break;
        case 7:
            serial_data &= ~((uint64_t)1 << 22);
            break;
        case 8:
            serial_data &= ~((uint64_t)1 << 23);
            break;
        case 9:
            serial_data &= ~((uint64_t)1 << 24);
            break;
        default:
            break;
    } 

    temp[0] = disp[0]>>4;

    switch(temp[0]){
        case 1: // R
            serial_data &= ~((uint64_t)1 << 41);
            break;
        case 2: // L
            serial_data &= ~((uint64_t)1 << 33);
            break;
        case 3: // both
            serial_data &= ~((uint64_t)1 << 41);
            serial_data &= ~((uint64_t)1 << 33);
            break;
        default:
            break;
    }

    temp[1] = disp[1]>>4;

    switch(temp[1]){
        case 1: // R
            serial_data &= ~((uint64_t)1 << 53);
            break;
        case 2: // L
            serial_data &= ~((uint64_t)1 << 45);
            break;
        case 3: // both
            serial_data &= ~((uint64_t)1 << 53);
            serial_data &= ~((uint64_t)1 << 45);
            break;
        default:
            break;
    }

    return serial_data;
}

void disp_num(uint8_t *disp){
    int i;
    uint64_t serial_data;

    serial_data = get_shift_register_data(disp);

    switch(disp[2]>>4){
        case 1: // R
            gpio_put(OUT_K3R_PIN, 1);
            gpio_put(OUT_K3L_PIN, 0);
            break;
        case 2: // L
            gpio_put(OUT_K3R_PIN, 0);
            gpio_put(OUT_K3L_PIN, 1);
            break;
        case 3: // both
            gpio_put(OUT_K3R_PIN, 1);
            gpio_put(OUT_K3L_PIN, 1);
            break;
        default:
            gpio_put(OUT_K3R_PIN, 0);
            gpio_put(OUT_K3L_PIN, 0);
            break;
    }
    switch(disp[3]>>4){
        case 1: // R
            gpio_put(OUT_K4R_PIN, 1);
            gpio_put(OUT_K4L_PIN, 0);
            break;
        case 2: // L
            gpio_put(OUT_K4R_PIN, 0);
            gpio_put(OUT_K4L_PIN, 1);
            break;
        case 3: // both
            gpio_put(OUT_K4R_PIN, 1);
            gpio_put(OUT_K4L_PIN, 1);
            break;
        default:
            gpio_put(OUT_K4R_PIN, 0);
            gpio_put(OUT_K4L_PIN, 0);
            break;
    }
    switch(disp[4]>>4){
        case 1: // R
            gpio_put(OUT_K5R_PIN, 1);
            gpio_put(OUT_K5L_PIN, 0);
            break;
        case 2: // L
            gpio_put(OUT_K5R_PIN, 0);
            gpio_put(OUT_K5L_PIN, 1);
            break;
        case 3: // both
            gpio_put(OUT_K5R_PIN, 1);
            gpio_put(OUT_K5L_PIN, 1);
            break;
        default:
            gpio_put(OUT_K5R_PIN, 0);
            gpio_put(OUT_K5L_PIN, 0);
            break;
    }
    switch(disp[5]>>4){
        case 1: // R
            gpio_put(OUT_K6R_PIN, 1);
            gpio_put(OUT_K6L_PIN, 0);
            break;
        case 2: // L
            gpio_put(OUT_K6R_PIN, 0);
            gpio_put(OUT_K6L_PIN, 1);
            break;
        case 3: // both
            gpio_put(OUT_K6R_PIN, 1);
            gpio_put(OUT_K6L_PIN, 1);
            break;
        default:
            gpio_put(OUT_K6R_PIN, 0);
            gpio_put(OUT_K6L_PIN, 0);
            break;
    }
    gpio_put(LE_N_PIN, 0);
    for(i=0; i<64; i++){
        if(((serial_data>>i) & (uint64_t)1)==0){
            gpio_put(DATA_PIN, 0);
        }else{
            gpio_put(DATA_PIN, 1);
        }
        sleep_us(1);
        gpio_put(CLK_PIN, 1);           // clock rising
        sleep_us(1);
        gpio_put(CLK_PIN, 0);           // clock falling
        sleep_us(1);
    }
    gpio_put(LE_N_PIN, 1);
}


//--------------------------------------------------------
//- SPI BME280
//--------------------------------------------------------


void read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.
    reg |= READ_BIT;
    cs_select();
    spi_write_blocking(spi_default, &reg, 1);
    spi_read_blocking(spi_default, 0, buf, len);
    cs_deselect();
}

void write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg & 0x7f;  // remove read bit as this is a write
    buf[1] = data;
    cs_select();
    spi_write_blocking(spi_default, buf, 2);
    cs_deselect();
}

void bme280_read_raw(int32_t *humidity, int32_t *pressure, int32_t *temperature) {
    uint8_t buffer[8];

    read_registers(0xF7, buffer, 8);
    *pressure = ((uint32_t) buffer[0] << 12) | ((uint32_t) buffer[1] << 4) | (buffer[2] >> 4);
    *temperature = ((uint32_t) buffer[3] << 12) | ((uint32_t) buffer[4] << 4) | (buffer[5] >> 4);
    *humidity = (uint32_t) buffer[6] << 8 | buffer[7];
}

int32_t compensate_temp(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t) dig_T1 << 1))) * ((int32_t) dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t) dig_T1)) * ((adc_T >> 4) - ((int32_t) dig_T1))) >> 12) * ((int32_t) dig_T3))
            >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t compensate_pressure(int32_t adc_P) {
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t) t_fine) >> 1) - (int32_t) 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t) dig_P6);
    var2 = var2 + ((var1 * ((int32_t) dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t) dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t) dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t) dig_P1)) >> 15);
    if (var1 == 0)
        return 0;

    p = (((uint32_t) (((int32_t) 1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000)
        p = (p << 1) / ((uint32_t) var1);
    else
        p = (p / (uint32_t) var1) * 2;

    var1 = (((int32_t) dig_P9) * ((int32_t) (((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t) (p >> 2)) * ((int32_t) dig_P8)) >> 13;
    p = (uint32_t) ((int32_t) p + ((var1 + var2 + dig_P7) >> 4));

    return p;
}

uint32_t compensate_humidity(int32_t adc_H) {
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t) 76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t) dig_H4) << 20) - (((int32_t) dig_H5) * v_x1_u32r)) +
                   ((int32_t) 16384)) >> 15) * (((((((v_x1_u32r * ((int32_t) dig_H6)) >> 10) * (((v_x1_u32r *
                                                                                                  ((int32_t) dig_H3))
            >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) *
                                                 ((int32_t) dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t) dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t) (v_x1_u32r >> 12);
}

/* This function reads the manufacturing assigned compensation parameters from the device */
void read_compensation_parameters() {
    uint8_t buffer[26];

    read_registers(0x88, buffer, 24);

    dig_T1 = buffer[0] | (buffer[1] << 8);
    dig_T2 = buffer[2] | (buffer[3] << 8);
    dig_T3 = buffer[4] | (buffer[5] << 8);

    dig_P1 = buffer[6] | (buffer[7] << 8);
    dig_P2 = buffer[8] | (buffer[9] << 8);
    dig_P3 = buffer[10] | (buffer[11] << 8);
    dig_P4 = buffer[12] | (buffer[13] << 8);
    dig_P5 = buffer[14] | (buffer[15] << 8);
    dig_P6 = buffer[16] | (buffer[17] << 8);
    dig_P7 = buffer[18] | (buffer[19] << 8);
    dig_P8 = buffer[20] | (buffer[21] << 8);
    dig_P9 = buffer[22] | (buffer[23] << 8);

    dig_H1 = buffer[25];

    read_registers(0xE1, buffer, 8);

    dig_H2 = buffer[0] | (buffer[1] << 8);
    dig_H3 = (int8_t) buffer[2];
    dig_H4 = buffer[3] << 4 | (buffer[4] & 0xf);
    dig_H5 = (buffer[5] >> 4) | (buffer[6] << 4);
    dig_H6 = (int8_t) buffer[7];
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(SENS_CSB_PIN, 1);
    asm volatile("nop \n nop \n nop");
}

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(SENS_CSB_PIN, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

//---- pink filter -----------------------
void init_pink() {
    extern float   z[MAX_Z];
    extern float   k[MAX_Z];
    int             i;

    for (i = 0; i < MAX_Z; i++)
        z[i] = 0;
    k[MAX_Z - 1] = 0.5;
    for (i = MAX_Z - 1; i > 0; i--)
        k[i - 1] = k[i] * 0.25;
}

float pinkfilter(float in) {
    extern float   z[MAX_Z];
    extern float   k[MAX_Z];
    static float   t = 0.0;
    float          q;
    int             i;

    q = in;
    for (i = 0; i < MAX_Z; i++) {
        z[i] = (q * k[i] + z[i] * (1.0 - k[i]));
        q = (q + z[i]) * 0.5;
    }
    return (t = 0.75 * q + 0.25 * t); /* add 1st order LPF */
}

//---- Flash Programming -----------------
// uint8_t write_data[FLASH_PAGE_SIZE]

const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

bool flash_write(uint8_t *write_data){

    // Note that a whole number of sectors must be erased at a time.
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
//    sleep_ms(10);
    flash_range_program(FLASH_TARGET_OFFSET, write_data, FLASH_PAGE_SIZE);

    bool mismatch = false;
    int i;
    for (i = 0; i < FLASH_PAGE_SIZE; i++) {
        if (write_data[i] != flash_target_contents[i]){
            mismatch = true;
        }
    }
    if (mismatch)
        return false;
    else
        return true;
}

void flash_read(uint8_t *read_data){
    int i;

    for(i=0;i<FLASH_PAGE_SIZE;i++){
        read_data[i] = flash_target_contents[i];
    }
}