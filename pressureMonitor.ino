#include <ESP32_LoRaWAN.h>
#include "Arduino.h"

float calibrationPressure = 0.0;
float maximumSensorRange = 10.0;
float maxCriticalValue = 3.2;
float minCriticalValue = 2.8;

#define Vext 21

int counter = 0;

// -----------------------------LoRaWAN Parameters start-----------------------------
// Define LoRaWAN/TTN Licences and OTAA parameters
/*license for Heltec ESP32 LoRaWan, query your ChipID for relevant license: http://resource.heltec.cn/search */
uint32_t  license[4] = {0x05D87083, 0xF3758DC2, 0xD2C15684, 0x1282E3B1};

/* OTAA para*/
uint8_t DevEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // Fill those with TTN Info
uint8_t AppEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t AppKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* ABP para*/
uint8_t NwkSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // Fill those with TTN Info
uint8_t AppSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint32_t DevAddr =  ( uint32_t ) 0x00000000;
// LoRaWAN Parameters
/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };
/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;
/*the application data transmission duty cycle.  value in [ms].*/
uint32_t dutyMultiplier = 60000; // 1000 for seconds, 60000 for minutes
uint32_t t = 15;
uint32_t appTxDutyCycle = t * dutyMultiplier;
/*OTAA or ABP*/
bool overTheAirActivation = true;
/*ADR enable*/
bool loraWanAdr = true;
/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;
/* Application port */
uint8_t appPort = 2;
/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:

  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)

  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 8;
/*LoraWan debug level, select in arduino IDE tools.
  None : print basic info.
  Freq : print Tx and Rx freq, DR info.
  Freq && DIO : print Tx and Rx freq, DR, DIO0 interrupt and DIO1 interrupt info.
  Freq && DIO && PW: print Tx and Rx freq, DR, DIO0 interrupt, DIO1 interrupt and MCU deepsleep info.
*/
uint8_t debugLevel = LoRaWAN_DEBUG_LEVEL;
/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
// ------------------------------LoRaWAN Parameters end------------------------------


void setup() {
  // Begin Serial
  Serial.begin(115200);
  // Initialize analog to digital converter
  /*
    analogReadResolution(12);             // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
    analogSetWidth(12);                   // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
                                      //  9-bit gives an ADC range of 0-511
                                      // 10-bit gives an ADC range of 0-1023
                                      // 11-bit gives an ADC range of 0-2047
                                      // 12-bit gives an ADC range of 0-4095
    analogSetCycles(8);                   // Set number of cycles per sample, default is 8 and provides an optimal result, range is 1 - 255
    analogSetSamples(1);                  // Set number of samples in the range, default is 1, it has an effect on sensitivity has been multiplied
    analogSetClockDiv(1);                 // Set the divider for the ADC clock, default is 1, range is 1 - 255
    analogSetAttenuation(ADC_11db);       // Sets the input attenuation for ALL ADC inputs, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db
    analogSetPinAttenuation(VP,ADC_11db); // Sets the input attenuation, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db
                                      // ADC_0db provides no attenuation so IN/OUT = 1 / 1 an input of 3 volts remains at 3 volts before ADC measurement
                                      // ADC_2_5db provides an attenuation so that IN/OUT = 1 / 1.34 an input of 3 volts is reduced to 2.238 volts before ADC measurement
                                      // ADC_6db provides an attenuation so that IN/OUT = 1 / 2 an input of 3 volts is reduced to 1.500 volts before ADC measurement
                                      // ADC_11db provides an attenuation so that IN/OUT = 1 / 3.6 an input of 3 volts is reduced to 0.833 volts before ADC measurement
    adcAttachPin(VP);                     // Attach a pin to ADC (also clears any other analog mode that could be on), returns TRUE/FALSE result
    adcStart(VP);                         // Starts an ADC conversion on attached pin's bus
    adcBusy(VP);                          // Check if conversion on the pin's ADC bus is currently running, returns TRUE/FALSE result
    adcEnd(VP);                           // Get the result of the conversion (will wait if it have not finished), returns 16-bit integer result
  */
  adcAttachPin(13); // Attach adc to pin 13 (Sensor)
  adcAttachPin(37); // Attach adc to Pin 37 (Battery)
  analogSetWidth(12); // Set 12bit resolution
  analogSetClockDiv(255); // 1338mS
  pinMode(Vext, OUTPUT);

  // LoRaWAN OTAA Setup
  SPI.begin(SCK, MISO, MOSI, SS);
  Mcu.init(SS, RST_LoRa, DIO0, DIO1, license);
  deviceState = DEVICE_STATE_INIT;

  // Set warning lights pin
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
}

// Reads the sensor value twice and calcualtes the Pressure
float readAndCalculatePressure() {
  // Read sensor Value twice for a less noisy reading and average the values.
  int sensorValue1 = analogRead(13);
  delay(2);
  int sensorValue2 = analogRead(13);
  int sensorValueAvg = (sensorValue1 + sensorValue2) / 2;
  // Calculate pressure from sensor reading with pressure = ((sensorReading/maximumResolution)/maxPressureRangeOfTheSensor)
  // where maximum Resolution is 4095 at 3.3 V Output and the maximum range of the sensor is 10 bar.
  float pressure = ((sensorValueAvg / 4095.0) * maximumSensorRange) - calibrationPressure;
  return pressure;
}

// Calculates the battery voltage
uint16_t readBatteryVoltage(){
  digitalWrite(Vext, LOW);
  delay(10);
  // Calculate battery voltage by multiplying analog read with 0.0025*1000 for mV reading
  uint16_t voltage = analogRead(37) * 0.0025 * 1000;
  digitalWrite(Vext, HIGH);
  return voltage;
}

void warningState(){
  float pressure = 0.0;
  do{
    pressure = readAndCalculatePressure();
    Serial.println("WARNING! PRESSURE CRITICAL!" + String(pressure));
    digitalWrite(12, HIGH);
    delay(200);
    digitalWrite(12, LOW);
    delay(200);
  }while(pressure <= minCriticalValue || pressure >= maxCriticalValue );
}

// Packs the given pressure data and packs them to be send via LoRaWAN (May send more later)
static void packData(float value, uint16_t batteryVoltage) {
  appDataSize = 4;
  // Pack value
  uint16_t valueInt = round(value * 1000);
  appData[0] = (char) (highByte(valueInt));
  appData[1] = (char) (lowByte(valueInt));
  // Pack battery voltage
  appData[2] = (char) (highByte(batteryVoltage));
  appData[3] = (char) (lowByte(batteryVoltage));
}

void loop() {
  switch ( deviceState )
  {
    case DEVICE_STATE_INIT:
      {
        LoRaWAN.init(loraWanClass, loraWanRegion);
        break;
      }
    case DEVICE_STATE_JOIN:
      {              
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        // Read pressure
        float pressure = readAndCalculatePressure();
        Serial.println("Measured Pressure: " + String(pressure) + " bar.");
        // Read battery Voltage
        uint16_t voltage = readBatteryVoltage();
        Serial.println("Measured battery voltage: " + String(voltage) + " mV.");
        packData(pressure, voltage); 
        Serial.println("Start sending");
        LoRaWAN.send(loraWanClass);
        Serial.println("Sending complete");
        deviceState = DEVICE_STATE_CYCLE;   
        // Go into warning state after transmit
        if(pressure <= minCriticalValue || pressure >= maxCriticalValue){
          warningState();   
        }
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
        Serial.println("Start cycle");
        LoRaWAN.cycle(txDutyCycleTime);
        Serial.println("Cycle complete");
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        LoRaWAN.sleep(loraWanClass, debugLevel);
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}
