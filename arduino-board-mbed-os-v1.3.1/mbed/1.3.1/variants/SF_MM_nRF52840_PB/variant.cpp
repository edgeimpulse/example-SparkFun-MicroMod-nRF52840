#include "Arduino.h"

/* wiring_analog variables definition */
/* Flag to indicate whether the ADC config has been changed from the default one */
bool isAdcConfigChanged = false;

/* 
 * Configuration used for all the active ADC channels, it is initialized with the mbed default values
 * When it is changed, all the ADC channels are reconfigured accordingly 
 */
analogin_config_t adcCurrentConfig = {
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
    .gain = NRF_SAADC_GAIN1_4,
    .reference = NRF_SAADC_REFERENCE_VDD4,
    .acq_time = NRF_SAADC_ACQTIME_10US,
    .mode = NRF_SAADC_MODE_SINGLE_ENDED,
    .burst = NRF_SAADC_BURST_DISABLED,
    .pin_p = NRF_SAADC_INPUT_DISABLED,
    .pin_n = NRF_SAADC_INPUT_DISABLED};

void analogReference(uint8_t mode)
{
  nrf_saadc_reference_t reference = NRF_SAADC_REFERENCE_VDD4;
  nrf_saadc_gain_t gain = NRF_SAADC_GAIN1_4;
  if (mode == AR_VDD)
  {
    reference = NRF_SAADC_REFERENCE_VDD4;
    gain = NRF_SAADC_GAIN1_4;
  }
  else if (mode == AR_INTERNAL)
  {
    reference = NRF_SAADC_REFERENCE_INTERNAL;
    gain = NRF_SAADC_GAIN1;
  }
  else if (mode == AR_INTERNAL1V2)
  {
    reference = NRF_SAADC_REFERENCE_INTERNAL;
    gain = NRF_SAADC_GAIN1_2;
  }
  else if (mode == AR_INTERNAL2V4)
  {
    reference = NRF_SAADC_REFERENCE_INTERNAL;
    gain = NRF_SAADC_GAIN1_4;
  }
  adcCurrentConfig.reference = reference;
  adcCurrentConfig.gain = gain;
  analogUpdate();
}

void analogAcquisitionTime(uint8_t time)
{
  nrf_saadc_acqtime_t acqTime = NRF_SAADC_ACQTIME_10US;
  if (time == AT_3_US)
  {
    acqTime = NRF_SAADC_ACQTIME_3US;
  }
  else if (time == AT_5_US)
  {
    acqTime = NRF_SAADC_ACQTIME_5US;
  }
  else if (time == AT_10_US)
  {
    acqTime = NRF_SAADC_ACQTIME_10US;
  }
  else if (time == AT_15_US)
  {
    acqTime = NRF_SAADC_ACQTIME_15US;
  }
  else if (time == AT_20_US)
  {
    acqTime = NRF_SAADC_ACQTIME_20US;
  }
  else if (time == AT_40_US)
  {
    acqTime = NRF_SAADC_ACQTIME_40US;
  }
  adcCurrentConfig.acq_time = acqTime;
  analogUpdate();
}

AnalogPinDescription g_AAnalogPinDescription[] = {
    {P0_4, NULL},   //A0
    {P0_5, NULL},   //A1
    {P0_29, NULL},  //A2 / G0
    {P0_28, NULL},  //A3 / SCK
    {P0_30, NULL},  //A4 / BATT_VIN/3
    {P0_31, NULL} //A5 / COPI
};

PinDescription g_APinDescription[] = {
    {P0_0, NULL, NULL, NULL},  //XL1
    {P0_1, NULL, NULL, NULL},  //XL2
    {P0_2, NULL, NULL, NULL},  //CIPO
    {P0_3, NULL, NULL, NULL},  //GPIO1
    {P0_4, NULL, NULL, NULL},  //ADC0
    {P0_5, NULL, NULL, NULL},  //ADC1
    {P0_6, NULL, NULL, NULL},  //PWM0
    {P0_7, NULL, NULL, NULL},  //!BOOT!
    {P0_8, NULL, NULL, NULL},  //SDA
    {P0_9, NULL, NULL, NULL},  //GPIO9 / NFC1
    {P0_10, NULL, NULL, NULL}, //GPIO10 / NFC2
    {P0_11, NULL, NULL, NULL}, //SCL
    {P0_12, NULL, NULL, NULL}, //FLASH_!CS!
    {P0_13, NULL, NULL, NULL}, //LED_BUILTIN
    {P0_14, NULL, NULL, NULL}, //COPI1 / SDIO_CMD
    {P0_15, NULL, NULL, NULL}, //I2C_!INT!
    {P0_16, NULL, NULL, NULL}, //PWM1
    {P0_17, NULL, NULL, NULL}, //GPIO5
    {P0_18, NULL, NULL, NULL}, //!RESET!
    {P0_19, NULL, NULL, NULL}, //SCK1 / SDIO_CLK
    {P0_20, NULL, NULL, NULL}, //!CS!
    {P0_21, NULL, NULL, NULL}, //CIPO1 / SDIO_DATA0
    {P0_22, NULL, NULL, NULL}, //SDIO_DATA1
    {P0_23, NULL, NULL, NULL}, //SDIO_DATA2
    {P0_24, NULL, NULL, NULL}, //SCL1
    {P0_25, NULL, NULL, NULL}, //PDM_CLK
    {P0_26, NULL, NULL, NULL}, //PDM_DATA
    {P0_27, NULL, NULL, NULL}, //D0
    {P0_28, NULL, NULL, NULL}, //SCK
    {P0_29, NULL, NULL, NULL}, //GPIO0
    {P0_30, NULL, NULL, NULL}, //BATT_VIN/3
    {P0_31, NULL, NULL, NULL}, //COPI
    {P1_0, NULL, NULL, NULL},  //!CS1! / SDIO_DATA3
    {P1_1, NULL, NULL, NULL},  //SDA1
    {P1_2, NULL, NULL, NULL},  //RTS1
    {P1_3, NULL, NULL, NULL},  //TX1
    {P1_4, NULL, NULL, NULL},  //GPIO7
    {P1_5, NULL, NULL, NULL},  //RX2
    {P1_6, NULL, NULL, NULL},  //GPIO6
    {P1_7, NULL, NULL, NULL},  //TX2
    {P1_8, NULL, NULL, NULL},  //D1
    {P1_9, NULL, NULL, NULL},  //CTS1
    {P1_10, NULL, NULL, NULL}, //RX1
    {P1_11, NULL, NULL, NULL}, //GPIO4
    {P1_12, NULL, NULL, NULL}, //GPIO3
    {P1_13, NULL, NULL, NULL}, //GPIO2
    {P1_14, NULL, NULL, NULL}, //GPIO8
    {P1_15, NULL, NULL, NULL}  //3.3V_EN
};

extern "C"
{
  unsigned int PINCOUNT_fn()
  {
    return (sizeof(g_APinDescription) / sizeof(g_APinDescription[0]));
  }
}

#include "nrf_rtc.h"

void initVariant()
{
  // Errata Nano33BLE - I2C pullup is on SWO line, need to disable TRACE
  // was being enabled by nrfx_clock_anomaly_132
  CoreDebug->DEMCR = 0;
  NRF_CLOCK->TRACECONFIG = 0;

  // FIXME: bootloader enables interrupt on COMPARE[0], which we don't handle
  // Disable it here to avoid getting stuck when OVERFLOW irq is triggered
  nrf_rtc_event_disable(NRF_RTC1, NRF_RTC_INT_COMPARE0_MASK);
  nrf_rtc_int_disable(NRF_RTC1, NRF_RTC_INT_COMPARE0_MASK);

  NRF_PWM_Type *PWM[] = {
      NRF_PWM0, NRF_PWM1, NRF_PWM2
#ifdef NRF_PWM3
      ,
      NRF_PWM3
#endif
  };

  for (unsigned int i = 0; i < (sizeof(PWM) / sizeof(PWM[0])); i++)
  {
    PWM[i]->ENABLE = 0;
    PWM[i]->PSEL.OUT[0] = 0xFFFFFFFFUL;
  }
}

#ifdef SERIAL_CDC

static void utox8(uint32_t val, uint8_t *s)
{
  for (int i = 0; i < 16; i = i + 2)
  {
    int d = val & 0XF;
    val = (val >> 4);

    s[15 - i - 1] = d > 9 ? 'A' + d - 10 : '0' + d;
    s[15 - i] = '\0';
  }
}

uint8_t getUniqueSerialNumber(uint8_t *name)
{
#define SERIAL_NUMBER_WORD_0 NRF_FICR->DEVICEADDR[1]
#define SERIAL_NUMBER_WORD_1 NRF_FICR->DEVICEADDR[0]

  utox8(SERIAL_NUMBER_WORD_0, &name[0]);
  utox8(SERIAL_NUMBER_WORD_1, &name[16]);

  return 32;
}

void _ontouch1200bps_()
{
  __disable_irq();
  NRF_POWER->GPREGRET = DFU_MAGIC_SERIAL_ONLY_RESET;
  NVIC_SystemReset();
}

#endif
