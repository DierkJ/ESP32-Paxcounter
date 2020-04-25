#ifndef _SOLAR_H
#define _SOLAR_H

#include <Arduino.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include "i2c.h"
#include "reset.h"

#ifdef HAS_SOLAR

#include <Adafruit_INA219.h>

enum chargeState_t { CHARGING, DISCHARGING };

//
// solar charger status
//
typedef struct {
  float fSolarVoltage;        // voltage on solar panel [V]
  float fSolarCurrent;        // Current [mA]   +/- 800mA (Gain Setting: /2)
  float fSolarPower;          // Power [mW]
  float fSoC_mAh;             // state of charge [mAh]
  float fSoC;                 // state of charge [%]
  float fBatteryCurrent;      // [mA], positive values: charging,  negative values: discharging
  float fBatteryVoltage;      //  ESP… AD-Wandler?
  float fESPVoltage;          // CPU Voltage [V]
  float fTemperature;         // Temperature of battery ?
  uint8_t Charging;           // 0: Charging, 1: Discharging
  uint16_t iBatCycles;        // Charge / Discharge cycles ?
} solarStatus_t;

extern solarStatus_t solarStatus;
extern Ticker solarcycler;

void Coloumb_PowerEvent_IRQ(void); 
int solar_init(void);
void solarcycle(void);
void solar_storedata(solarStatus_t *bme_store);

#endif // HAS_SOLAR

#endif