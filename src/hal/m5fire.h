// clang-format off
// upload_speed 921600
// board m5stack-fire

#ifndef _M5FIRE_H
#define _M5FIRE_H

#include <stdint.h>

#define HAS_LORA 1 // comment out if device shall not send data via LoRa or has no M5 RA01 LoRa module
#define LORA_SCK  SCK
#define LORA_CS   SS
#define LORA_MISO MISO
#define LORA_MOSI MOSI
#define LORA_RST  GPIO_NUM_26
#define LORA_IRQ  GPIO_NUM_36
#define LORA_IO1  GPIO_NUM_34 // must be wired by you on PCB!
#define LORA_IO2  LMIC_UNUSED_PIN


// enable only if you want to store a local paxcount table on the device
#define HAS_SDCARD  1      // this board has an SD-card-reader/writer
#define SDCARD_CS    GPIO_NUM_4
#define SDCARD_MOSI  MOSI
#define SDCARD_MISO  MISO
#define SDCARD_SCLK  SCK

// user defined sensors
//#define HAS_SENSORS 1 // comment out if device has user defined sensors

#define CFG_sx1276_radio 1 // select LoRa chip
#define BOARD_HAS_PSRAM // use if board has external PSRAM
#define DISABLE_BROWNOUT 1 // comment out if you want to keep brownout feature

//#define HAS_DISPLAY 2 // TFT-LCD, support work in progess, not ready yet
//#define MY_DISPLAY_FLIP  1 // use if display is rotated
//#define BAT_MEASURE_ADC ADC1_GPIO35_CHANNEL // battery probe GPIO pin -> ADC1_CHANNEL_7
//#define BAT_VOLTAGE_DIVIDER 2 // voltage divider 100k/100k on board

#define HAS_LED NOT_A_PIN // no on board LED
#define RGB_LED_COUNT 5 // we use 5 of 10 LEDs (1 side)
#define HAS_RGB_LED SmartLed rgb_led(LED_SK6812, RGB_LED_COUNT, GPIO_NUM_15) // LED_SK6812 RGB LED on GPIO15
#define HAS_BUTTON (39) // on board button A

// GPS settings
#define HAS_GPS 0 // use on board GPS
#define GPS_SERIAL 9600, SERIAL_8N1, RXD2, TXD2 // UBlox NEO 6M RX, TX
// #define GPS_INT GPIO_NUM_35 // 30ns accurary timepulse, to be external wired on pcb: shorten R12!

// Display Settings
#define MY_DISPLAY_WIDTH 320
#define MY_DISPLAY_HEIGHT 240
#define MY_DISPLAY_TYPE LCD_ILI9341
#define MY_DISPLAY_CS GPIO_NUM_14 // Display CS pin
#define MY_DISPLAY_CLK GPIO_NUM_18 // SPI CLOCK pin
#define MY_DISPLAY_DC GPIO_NUM_27 // Display command/data pin
#define MY_DISPLAY_MOSI GPIO_NUM_23 // SPI MOSI
#define MY_DISPLAY_MISO GPIO_NUM_19 // SPI MISO
#define MY_DISPLAY_BL GPIO_NUM_32 // backlight control
#define MY_DISPLAY_RST GPIO_NUM_33 // RESET control

#endif