#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <RCSwitch.h>

// Configuration
#define TRACE 0 
#define RF_REPEAT 10
const int EMIT_PIN = 10;
const int BME_ADDRESS = 0x76; 

// RF Codes 
const long CODE_HUM  = 81000;
const long CODE_TEMP = 83000;
const long CODE_VOLT = 850000;
const long CODE_ERR  = 99999;

const int COUNT_LIMIT = 40;
const int COUNT_VOLTS_LIMIT = 5;

// Global Objects
RCSwitch mySwitch = RCSwitch();
Adafruit_BME280 bme;

// Calibration
const float TEMP_CALIBRATION = 0.0;

// Volatile variables for ISR
volatile uint8_t f_wdt = 1;
int count = 999;
int countVolts = 999;

void setup() {
  // --- 328PB GPIO HANDLING ---
  // Set standard pins LOW
  for (uint8_t i = 0; i <= A5; i++) {
    if (i != SDA && i != SCL && i != EMIT_PIN) {
       pinMode(i, OUTPUT);
       digitalWrite(i, LOW);
    }
  }
  
  // 328PB Specific: Port E (PE0-PE3)
  // If not handled, these floating pins waste power.
  // Depending on your pinout variant, these might be mapped to digital pins,
  // but writing directly to the register is safest for the PB.
  #ifdef PORTE
    DDRE  = 0xFF; // Set Port E direction to Output
    PORTE = 0x00; // Set Port E low
  #endif

  if (TRACE) {
    Serial.begin(57600);
    while(!Serial);
    trc("Start 328PB");
  }

  // --- WDT SETUP ---
  cli();
  MCUSR &= ~(1 << WDRF); // Clear Reset Flag
  
  // Enter Watchdog Configuration mode
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  
  // Set Interrupt Enable (WDIE) and 8 Seconds (WDP3+WDP0)
  // IMPORTANT: We do NOT set WDE (System Reset Enable) here, only Interrupts.
  WDTCSR = (1 << WDP0) | (1 << WDP3);
  sei();

  if (!bme.begin(BME_ADDRESS)) {
    trc("BME280 missing");
  }
  
  // Forced mode is correct for sleep/wake cycles
  bme.setSampling(Adafruit_BME280::MODE_FORCED);
                    
  mySwitch.enableTransmit(EMIT_PIN);
  mySwitch.setRepeatTransmit(RF_REPEAT);
  if(TRACE) {
    Serial.begin(57600);
    while(!Serial);
  }
}

void loop() {
  if (f_wdt == 1) {
    // 40 * 8s = ~5.3 mins
    if (++count > COUNT_LIMIT) { 
      count = 0;
      if(TRACE) {
        Serial.begin(57600);
        while(!Serial);
        trc("Wake");
      }
      // digitalWrite(EMIT_PIN, LOW);
      // pinMode(EMIT_PIN, OUTPUT);
      // Wake BME and measure
      bme.takeForcedMeasurement(); 
      float t = bme.readTemperature();
      float h = bme.readHumidity();
      t += TEMP_CALIBRATION;

      if(++countVolts > COUNT_VOLTS_LIMIT){
        countVolts = 0;
        long voltage = readVcc();
        sendData(voltage, CODE_VOLT);
      }
      sendTempAndHum(t, h);
      
      // digitalWrite(EMIT_PIN, LOW);
      // pinMode(EMIT_PIN, INPUT);

      trc("Sleep");
      if(TRACE) {
        Serial.println("");
        Serial.flush(); 
      }
    }

    f_wdt = 0;
    enterSleep();
  }
}

// Watchdog Interrupt Service Routine
ISR(WDT_vect) {
  f_wdt = 1;
}

void enterSleep(void) {

  // 1. Turn off ADC
  ADCSRA &= ~(1 << ADEN);
   // Enable the Watchdog interrupt before going to sleep (no WDE)
  WDTCSR |= _BV(WDIE);
  // 2. Configure Sleep Mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  
  // 3. Disable Peripherals
  // Note: disabling Timer0 (millis) means millis() stops counting
  power_all_disable();
  
  // 328PB Specific: Disable extra peripherals
  // Ensure your board definition (e.g., MiniCore) supports these constants.
  // If compile fails, comment these lines out, but you lose some power saving.
  #ifdef PRR1
    PRR1 |= _BV(PRTWI1) | _BV(PRPTC) | _BV(PRTIM4) | _BV(PRSPI1) | _BV(PRTIM3); 
  #endif

  cli();
  // 4. Enable Sleep and Enter
  sleep_enable();
  // sleep_bod_disable();
  // CRITICAL FIX: Do not use cli() here. 
  // We need interrupts enabled to wake up via WDT.
  // Ideally, we turn off Brown Out Detection (BOD) right before sleep
  
  // Determines if BOD disable is supported
  #if defined(BODS) && defined(BODSE)
    MCUCR = _BV(BODS) | _BV(BODSE);
    MCUCR = _BV(BODS);
  #endif
  sei();
  
  sleep_cpu(); // <--- System sleeps here until WDT fires

  // --- WAKE UP ---
  
  sleep_disable();
  sei();
  // 5. Re-enable Peripherals
  power_all_enable(); 
  
  // 328PB Specific: Re-enable USART0 manually
  #ifdef PRR0
    PRR0 &= ~_BV(PRUSART0); 
  #endif

  if(countVolts >= COUNT_VOLTS_LIMIT) {
    // 6. Re-enable ADC
    ADCSRA |= (1 << ADEN);
  }
}

long readVcc() {
  long result;
  // REFS0 = AVCC, MUX3:1 = 1110 (1.1V Bandgap)
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); 
  ADCSRA |= _BV(ADSC); 
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  
  if (result == 0) return 0; 
  result = 1125300L / result; 
  return result;
}

void sendData(long dataValue, long dataTypeOffset) {
  long sum = CODE_ERR;
  if (dataValue != CODE_ERR) {
    sum = dataValue + dataTypeOffset;
  }
  if (TRACE) {
    Serial.print(F("Tx: ")); // F() macro saves RAM
    Serial.println(sum);
  }
  mySwitch.send(sum, 24);
}

void sendTempAndHum(float t, float h) {
  if (TRACE) {
    Serial.print(F("sendTempAndHum: t ")); // F() macro saves RAM
    Serial.print(t);
    Serial.print(F(", h "));
    Serial.println(h);
    Serial.flush();
  }
  if (isnan(h) || isnan(t)) {
    trc("Error: BME read failed");
    sendData(CODE_ERR, CODE_TEMP);
    sendData(CODE_ERR, CODE_HUM);
  } else {
    sendData((long)(t * 10), CODE_TEMP);
    sendData((long)(h * 10), CODE_HUM);
  }
}

void trc(String msg) {
  if (TRACE) {
    Serial.println(msg);
    Serial.flush(); 
  }
}
