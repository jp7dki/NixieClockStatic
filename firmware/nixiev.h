#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/flash.h"
#include "pico/binary_info.h"

#ifndef NIXIE_H
#define NIXIE_H

//---------------------------------------
//- PIN Definition
//---------------------------------------
#define UART_TX_PIN 12
#define UART_RX_PIN 13

#define PWM0A_PIN 0
#define PWM1A_PIN 2
#define PWM2A_PIN 4
#define PWM3A_PIN 6
#define PWM4A_PIN 8
#define PWM5A_PIN 10

#define OUT_K3R_PIN 3
#define OUT_K3L_PIN 1
#define OUT_K4R_PIN 5
#define OUT_K4L_PIN 7
#define OUT_K5R_PIN 9
#define OUT_K5L_PIN 11
#define OUT_K6R_PIN 14
#define OUT_K6L_PIN 15

#define SWA_PIN 16
#define SWB_PIN 22
#define SWC_PIN 23

#define SENS_SCK_PIN 18
#define SENS_CSB_PIN 17
#define SENS_SDI_PIN 19
#define SENS_SDO_PIN 20

#define GPS_RX_PIN 21

#define HV_EN_PIN 28

#define OE_PIN 24
#define LE_N_PIN 25
#define CLK_PIN 26
#define DATA_PIN 27

#define LSEN_PIN 29

// UART0(DEBUG) 
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// UART1(GPS)
#define UART_ID_GPS uart1
#define BAUD_RATE_GPS 9600

// SPI(BME280)
#define READ_BIT 0x80

#define MAX_Z 16

// FLASH
#define FLASH_TARGET_OFFSET (512 * 1024)

//------------------------------------------
//- Global Variable
//------------------------------------------
extern int32_t t_fine;

extern uint16_t dig_T1;
extern int16_t dig_T2, dig_T3;
extern uint16_t dig_P1;
extern int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
extern uint8_t dig_H1, dig_H3;
extern int8_t dig_H6;
extern int16_t dig_H2, dig_H4, dig_H5;

extern float z[MAX_Z];
extern float k[MAX_Z];

//-----------------------------------------
//- Function Prototyping
//-----------------------------------------
uint64_t get_shift_register_data(uint8_t *disp);
void disp_num(uint8_t *disp);
static inline void cs_deselect();
static inline void cs_select();
void read_registers(uint8_t reg, uint8_t *buf, uint16_t len);
void write_register(uint8_t reg, uint8_t data);
void bme280_read_raw(int32_t *humidity, int32_t *pressure, int32_t *temperature);
int32_t compensate_temp(int32_t adc_T);
uint32_t compensate_pressure(int32_t adc_P);
uint32_t compensate_humidity(int32_t adc_H);
void read_compensation_parameters();

void init_pink();
float pinkfilter(float in);

bool flash_write(uint8_t *write_data);
void flash_read(uint8_t *read_data);

#endif