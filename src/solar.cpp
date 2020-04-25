// Basic config
#include "globals.h"
#include "solar.h"

// Local logging tag
static const char TAG[] = __FILE__;

#ifdef HAS_SOLAR

Ticker solarcycler;

solarStatus_t solarStatus = {0};
Adafruit_INA219 ina219;

#define MAX_BATTERY_CAPACITY  (2600.0)      // mAh
#define AH_QUANTA             (0.17067759)  // mAh for each INT
double dAh_quanta = AH_QUANTA;  // mAh for each INT
double dPercent_quanta;         // same in %


// TOOD:  Save/Restore this values in Non Volatile Memory
double dSoC_mAh = MAX_BATTERY_CAPACITY;   // initialize with full charge
double dSoC_percent = 100.0;              // and 100%
double dBattCurrent;                      // battery current [mA] estimated from charge counter
enum chargeState_t lastCharge;            // last direction
uint32_t uTLastInt = 0;                   // ms of last interrupt

void solarcycle() { xTaskNotify(irqHandlerTask, SOLAR_IRQ, eSetBits); }

//
// IRQ Handler for LTC4150
// one INT every AH_QUANTA
//
void Coloumb_PowerEvent_IRQ(void) 
{
  uint32_t tnow = millis();

  if (digitalRead(PMU_DIR))
  {
    ESP_LOGI(TAG, "Battery Charging");
    lastCharge = CHARGING;
    dSoC_mAh += dAh_quanta;
    dSoC_percent += dPercent_quanta;
  }
  else
  {
    ESP_LOGI(TAG, "Battery Discharging");
    lastCharge = DISCHARGING;
    dSoC_mAh -= dAh_quanta;
    dSoC_percent -= dPercent_quanta;
    if (dSoC_mAh < 0.0)
    {
      dSoC_mAh = 0.0;
      dSoC_percent = 0.0;
    }
  }
  if (uTLastInt > 0)
  {
    // only if there was a previous int
    dBattCurrent = 614.4/((tnow-uTLastInt)/1000000.0);
    if (lastCharge == DISCHARGING)
      dBattCurrent *= -1.0;
  }
  uTLastInt = tnow;
}


int solar_init(void) 
{
  int rc = 1;
 
  // block i2c bus access
  if (I2C_MUTEX_LOCK()) 
  {
    if (! ina219.begin()) 
    {
      ESP_LOGE(TAG, "INA219 current measurement not found");
      rc = 0;
      goto finish;
    }
    ina219.setCalibration_32V_1A(); // scale to 1A
    ESP_LOGI(TAG, "INA219 found and initialized");
    // 
    // some inits for coulomb counter
    dPercent_quanta = 1.0/(MAX_BATTERY_CAPACITY/1000.0*5859.0/100.0);
    //
    // TODO: Restore SoC ??
    //
  
  } 
  else 
  {
    ESP_LOGE(TAG, "I2c bus busy - INA219 initialization error");
    rc = 0;
    goto finish;
  }

#ifdef PMU_INT

    pinMode(PMU_INT, INPUT_PULLUP);
//    attachInterrupt(digitalPinToInterrupt(PMU_INT), PMUIRQ, FALLING);
    
    pinMode(PMU_DIR, INPUT_PULLUP);
    if (digitalRead(PMU_DIR))
      lastCharge = CHARGING;
    else
      lastCharge = DISCHARGING;


#endif // PMU_INT

finish:
  I2C_MUTEX_UNLOCK(); // release i2c bus access

  if (rc)
    solarcycler.attach(SOLARCYCLE, solarcycle);

  return rc;
  ESP_LOGI(TAG, "Solar initialized");
  
}

void solar_storedata(solarStatus_t *solar_store) {

  if ((cfg.payloadmask & BATT_DATA) &&
      (I2C_MUTEX_LOCK())) 
  { 
    // block i2c bus access
    solar_store->fSolarCurrent = ina219.getCurrent_mA();
    solar_store->fSolarVoltage = ina219.getBusVoltage_V();
    solar_store->fSolarPower = ina219.getPower_mW();
    solar_store->fSoC_mAh = dSoC_mAh;
    solar_store->fSoC = dSoC_percent;
    solar_store->fBatteryVoltage = (float)read_voltage() / 1000.0;  // in [V]
    solar_store->fBatteryCurrent = dBattCurrent;
    solar_store->Charging = lastCharge;
   
    I2C_MUTEX_UNLOCK(); // release i2c bus access
  }

} // solar_storedata()


#endif // HAS_SOLAR




