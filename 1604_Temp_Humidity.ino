#include <Arduino.h>
#include <RF24.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <avr\sleep.h>
//#include <avr/wdt.h>
#include <avr/io.h>

#define SENSORID 2

/*
   sleep interval is 32 seconds
   30 minutes is 1800 seconds
   1800 s / 32 s = 56.25 or 57
   total sleep time is 30.4 minutes
   RTC must be divided by 32 for 1.024kHz
   PIT must be 32768 cycles
*/

#define SLEEPTIME 57

void rtcInt(void);

RF24 radio(3, 0);

DHT dht(5, DHT22);

uint64_t address[] = {0x1, 0x2, 0x3};

uint64_t sleepTime = 0;

struct data
{
  uint16_t humidity;
  uint16_t temp;
  uint8_t sensorID;
  uint16_t batteryVoltage;
};

data sensorData;

void ADCSetup()
{
  VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;
  ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV256_gc; // 78kHz clock
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;                      // Measure INTREF
  ADC0.CTRLA = ADC_ENABLE_bm;                              // Single, 10-bit
}

uint16_t MeasureVoltage()
{
  ADC0.COMMAND = ADC_STCONV_bm; // Start conversion
  while (ADC0.COMMAND & ADC_STCONV_bm)
    ;                              // Wait for completion
  uint16_t adc_reading = ADC0.RES; // ADC conversion result
  uint16_t voltage = 11264 / adc_reading;
  // radio.write(&voltage, sizeof(voltage));
  return voltage;
  // Buffer[0] = voltage/10; Buffer[1]= voltage%10;
}

void radioSetup()
{
  radio.begin();

  radio.setChannel(101);

  // radio.setDataRate(RF24_2MBPS);

  radio.setRetries(15, 15);

  radio.setPALevel(RF24_PA_HIGH);

  radio.openWritingPipe(address[2]);

  radio.stopListening();
}

void rtcInit(void)
{

  while (RTC.STATUS > 0)
  {
    ;
  }

  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;

  RTC.PITINTCTRL = RTC_PI_bm;

  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm;
}

ISR(RTC_PIT_vect)
{
  RTC.PITINTFLAGS = RTC_PI_bm;
}

void sleepinit(void)
{
  SLPCTRL.CTRLA |= SLPCTRL_SMODE_PDOWN_gc;
  SLPCTRL.CTRLA |= SLPCTRL_SEN_bm;
}

void ADCInit(void)
{

  ADC0.CTRLA = ADC_ENABLE_bm;
  ADC0.MUXPOS = ADC_MUXPOS_AIN6_gc;
  ADC0.COMMAND = ADC_STCONV_bm;

  // while(ADC0_R)
}


void wdt_enable() {
  _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_8KCLK_gc); // no window, 8 seconds
}

void wdt_reset() {
  __asm__ __volatile__ ("wdr"::);
}

void wdt_disable() {
  _PROTECTED_WRITE(WDT.CTRLA, 0);
}

void setup()
{
  if (SENSORID >= 2)
  {
    pinMode(1, OUTPUT);

    digitalWrite(1, HIGH);

  }

  rtcInit();

  sleepinit();

  ADCSetup();

  sei();

  dht.begin();

  radioSetup();

}

void loop()
{
  // put your main code here, to run repeatedly:

  wdt_disable();

  sleepTime = 0;

  radio.powerDown();

  //digitalWrite(1, HIGH);

  while (sleepTime < SLEEPTIME)
  {
    sleep_cpu();

    sleepTime++;
  }
  
  delay(500);
  
  wdt_enable();

  radio.powerUp();

  if (radio.failureDetected)
  {
    radioSetup();
  }

  dht.begin();

  wdt_reset();

  sensorData.humidity = dht.readHumidity() * 100 ;

  sensorData.temp = dht.readTemperature(true) * 100 ;

  sensorData.batteryVoltage = MeasureVoltage();

  sensorData.sensorID = SENSORID;

  radio.write(&sensorData, sizeof(sensorData));

  wdt_reset();
}
