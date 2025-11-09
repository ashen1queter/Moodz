#include <ArduinoBLE.h>
//#include <Wire.h>
//#include "MAX30105.h"
//#include "heartRate.h"
#include <Crypto.h>
#include <SHA256.h>
#include "nrf.h"
#include "nrf_nvmc.h"
#include "nrf_soc.h"
#include "nrf_power.h"

#define FLASH_PAGE_SIZE    4096
#define FLASH_SIZE         (1024 * 1024)
#define USER_FLASH_START   (FLASH_SIZE - 2 * FLASH_PAGE_SIZE)  // 2nd last page
#define CALIB_FLASH_ADDR   (FLASH_SIZE - 3 * FLASH_PAGE_SIZE)

#define CALIBRATION_DURATION 5000   
#define CALIBRATION_INTERVAL 50     
#define CALIBRATION_ALPHA 0.2f

struct MoodHash {
  char mood[16];
  uint8_t hash[32];
  uint8_t salt[32];
};

struct FlashData {
  uint32_t magic;             
  bool hashz_flashz_done;     
  uint8_t padding[3];
  MoodHash moods[4];         
};

struct CalibrationData {
  float heartbeat;
  float temperature;
  float gsr;
  bool calib_done;
  uint32_t magic;
};

CalibrationData calib;
CalibrationData *flashCalib = (CalibrationData *)CALIB_FLASH_ADDR;
CalibrationData flashCalib_ram;

FlashData *flashData = (FlashData *)USER_FLASH_START;
FlashData ramData;

SHA256 sha256;
//MAX30105 particleSensor;

#define DEBUG 1

#ifdef DEBUG
  #define DEBUG_PRINT(x)   Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

BLEService myService("12345678-1234-5678-1234-56789abcdef0");
BLECharacteristic txChar("12345678-1234-5678-1234-56789abcdef1", BLENotify, 50);
BLECharacteristic rxChar("12345678-1234-5678-1234-56789abcdef2", BLEWrite, 50);

void Heartbeat_sensor();
void BLE_rssi();
void GSR_sensor();
void Temp_sensor();
void Battery_charge();
void hashz_flashz(const char *mood, MoodHash *slot);
void loadFromFlash();
void saveToFlash();
void loadCalibrationFromFlash();
void saveCalibrationToFlash();
void deep_sleep();

bool isCharging = false;
bool isCharged = false;

bool ca = false;
bool calib_donez = false;

bool sleep_modez = true;

float temperature;

int currentHashIndex = 0;

const int ADC_PIN = 14;       // P0.14 (A0 / D14)
const int CHARGE_DETECT_PIN = 31; // P0.31 (connected to charging indicator / limit)

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

int rates_gsr[RATE_SIZE]; //Array of gsr readings
const int GSR_PIN = A0;
int gsrAvg;
int rateSpot_gsr = 0;
//float gsrFiltered = 0.0;
//float gsrAlpha = 0.1;

bool sendingEnabled = false;
bool sendinghashEnabled = false;
bool hashSent = false;

uint8_t calib_done = 0x0C;

unsigned long previousMillis = 0;
const unsigned long interval = 2000;
//char lastMessage[64] = "Hello from XIAO Sense!";
byte received;

BLEDevice central;

void setup() {
  //pinMode (P0_13, OUTPUT);
  //Battery_charge();

  //analogReadResolution(10);

  #ifdef DEBUG
    Serial.begin(9600);
    while (!Serial);
  #endif

  //nrf_nvmc_page_erase((uint32_t)flashData);
  DEBUG_PRINTLN(" ");
  DEBUG_PRINTLN("YO!");

  /**
  ramData.magic = 0xDEADBEEF;
  //ramData.hashz_flashz_done = false;

  hashz_flashz("happy", &ramData.moods[0]);
  hashz_flashz("sad", &ramData.moods[1]);
  hashz_flashz("calm", &ramData.moods[2]);
  hashz_flashz("energetic", &ramData.moods[3]);

  //ramData.hashz_flashz_done = true;

  saveToFlash();

  DEBUG_PRINTLN(F("New mood hashes generated and saved to flash."));
  loadFromFlash();
  **/

  loadCalibrationFromFlash();

  // Initialize MAX30105 sensor
  /**
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  DEBUG_PRINTLN("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
  particleSensor.enableDIETEMPRDY();
  **/

  // Initialize BLE
  if (!BLE.begin()) while (1);
  BLE.setLocalName("XIAO_Sense");
  BLE.setAdvertisedService(myService);
  myService.addCharacteristic(txChar);
  myService.addCharacteristic(rxChar);
  BLE.addService(myService);
  BLE.advertise();

  //DEBUG_PRINTLN("BLE started, advertising as XIAO_Sense");
}

void loop() {
  deep_sleep();
  //checkBatteryStatus();
  /**
  if (isCharging) {
    DEBUG_PRINTLN("Charging in progress, skipping operations...");
    delay(1000);
    return;
  }
  **/
  //BLE_rssi();
  central = BLE.central();

  if (central && !sleep_modez) {
    while (central.connected()) {
      //unsigned long currentMillis = millis();

      // Periodic BLE sending
      //if (sendingEnabled && currentMillis - previousMillis >= interval) {
        //previousMillis = currentMillis;

        //Heartbeat_sensor();
        //GSR_sensor();
        //Temp_sensor();
        //Mood_detection
        /**
        snprintf(lastMessage, sizeof(lastMessage),
                 "HR: %.1f bpm | Avg: %d | GSR: %.1f | Temp: %.2f°C",
                 beatsPerMinute, beatAvg, gsrFiltered, temperature);
        **/
       //snprintf(lastMessage, sizeof(lastMessage), "Jesica");
        
        //txChar.writeValue(sent); //mood is sent here //txChar.writeValue(ramData.moods[i].hash, 32);
        //DEBUG_PRINTLN(sent);

        /**
        if(sendinghashEnabled) {
          for (int i = 0; i < 4; i++)
          txChar.writeValue(ramData.moods[i].hash, 32);
        }
        sendinghashEnabled = false;
        DEBUG_PRINTLN("All hashes sent over BLE");
      }
      **/
      //}
      if(sendingEnabled && hashSent) {
        txChar.writeValue(ramData.moods[1].hash, 32); //mood is sent here //txChar.writeValue(ramData.moods[i].hash, 32);
        DEBUG_PRINTLN("Mood sent!");
      }
      /**
      if(calib_donez && !ca) {
        txChar.writeValue(&calib_done, 1);
        DEBUG_PRINTLN("Calibration done!");
        ca = true;
      }
      **/
      // Handle BLE RX commands
      if (rxChar.written()) {
        rxChar.readValue(&received, 1);

        DEBUG_PRINT("Received command: ");
        DEBUG_PRINTLN(received);

        /**
        if (received == 0x01 || received == 0x02) {
          sendingEnabled = true;
          sleep_modez = false;
          DEBUG_PRINTLN("Sending started/resumed.");
        }
        **/
        
        if (!hashSent && received == 0x01) {
          sendingEnabled = true;
          sleep_modez = false;
          currentHashIndex = 0;
          sendinghashEnabled = true;
          hashSent = false;

          ramData.magic = 0xDEADBEEF;
            //ramData.hashz_flashz_done = false;

          hashz_flashz("happy", &ramData.moods[0]);
          hashz_flashz("sad", &ramData.moods[1]);
          hashz_flashz("calm", &ramData.moods[2]);
          hashz_flashz("energetic", &ramData.moods[3]);

            //ramData.hashz_flashz_done = true;

          saveToFlash();

          DEBUG_PRINTLN(F("New mood hashes generated and saved to flash."));
          loadFromFlash();
        }
        else if (received == 0x03) {
          //txChar.writeValue(lastMessage);
          DEBUG_PRINTLN("Retried last message.");
        } 
        else if (received == 0x04) {
          sendingEnabled = false;
          DEBUG_PRINTLN("Stopped sending.");
        }
        else if (received == 0x05) {
          sendinghashEnabled = true;
          txChar.writeValue(ramData.moods[currentHashIndex].hash, 32);
          DEBUG_PRINT("Sent hash index: ");
          DEBUG_PRINTLN(currentHashIndex);
        }
        else if (received == 0x06 && sendinghashEnabled) {
          currentHashIndex++;
          if (currentHashIndex < 4) {
            txChar.writeValue(ramData.moods[currentHashIndex].hash, 32);
            DEBUG_PRINT("Sent next hash: ");
            DEBUG_PRINTLN(currentHashIndex);
          } else {
            sendinghashEnabled = false;
            hashSent = true;
            DEBUG_PRINTLN("All hashes sent. Sync complete.");
            DEBUG_PRINTLN(hashSent);
          }
        }
        else if (received == 0x0A) {
          DEBUG_PRINTLN("Ok bye!");
          sleep_modez = true;
          BLE.disconnect();   
        }
        else if (received == 0x0C) {
          calibrationz();
          txChar.writeValue(&calib_done, 1);
          DEBUG_PRINTLN("Calibration complete");
        }
        /**
        else if (received == 0x0A) {
          deep_sleep();
        }
        **/
        else {
          DEBUG_PRINTLN("Unknown command.");
          sleep_modez = true;
        }
      }
      //DEBUG_PRINTLN("Disconnected.");
    }
  }
}

/**
void GSR_sensor() {
  static bool calibrated = false;
  static int sampleCount = 0;
  static unsigned long prevMillis = 0;
  static int sum = 0;

  unsigned long currentMillis = millis();
  /**
  if (!calibrated) {
    if (currentMillis - prevMillis >= 2) {
      prevMillis = currentMillis;
      sum += analogRead(GSR_PIN);
      sampleCount++;

      if (sampleCount >= 100) {
        gsrFiltered = sum / 100.0;
        calibrated = true;
        sum = 0;
        sampleCount = 0;
      }
    }
  } else {

  int raw = analogRead(GSR_PIN);
  if(raw >= 0) {
    rates_gsr[rateSpot_gsr] += analogRead(GSR_PIN); 
    rateSpot_gsr %= RATE_SIZE;

    for(int i = 0; i < RATE_SIZE; i++) {
      int raw = analogRead(GSR_PIN);
      rates_gsr[i] += raw; 
  }
    gsrAvg /= RATE_SIZE;
  }
}

void Heartbeat_sensor() {
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
}

void Temp_sensor() {
  temperature = particleSensor.readTemperature();
}

void Battery_charge() {
  digitalWrite(P0_13, HIGH);
  pinMode(ADC_PIN, INPUT);
  pinMode(CHARGE_DETECT_PIN, INPUT);
}

void checkBatteryStatus() {
  int chgState = digitalRead(CHARGE_DETECT_PIN);

  if (chgState == LOW) {
    if (!isCharging) {
      isCharging = true;
      isCharged = false;
      DEBUG_PRINTLN("Battery is charging...");
      pinMode(ADC_PIN, INPUT);    // prevent high drive
      BLE.disconnect();
      BLE.end();
      particleSensor.setPulseAmplitudeRed(0);
      particleSensor.setPulseAmplitudeGreen(0);
    }
  } 
  else { 
    if (isCharging) {
      isCharging = false;
      isCharged = true;
      DEBUG_PRINTLN("Battery is fully charged!");
      particleSensor.setPulseAmplitudeRed(0x0A);
      BLE.begin();
      BLE.advertise();
    }
  }
}
**/

void BLE_rssi() {
  DEBUG_PRINTLN(BLE.rssi());
  //Link is weak then disconnect
}

void RNG_(uint8_t *buf, size_t len) {
  NRF_RNG->TASKS_START = 1;

  for (size_t i = 0; i < len; i++) {
    while (NRF_RNG->EVENTS_VALRDY == 0);
    NRF_RNG->EVENTS_VALRDY = 0;
    buf[i] = NRF_RNG->VALUE;
  }


  NRF_RNG->TASKS_STOP = 1;
}

void hashz_flashz(const char *mood, MoodHash *slot) {
  uint8_t salt[32];
  uint8_t hash[32];

  RNG_(salt, sizeof(salt));

  sha256.reset();
  sha256.update((const uint8_t*)mood, strlen(mood));
  sha256.update(salt, sizeof(salt));
  sha256.finalize(hash, sizeof(hash));

  strncpy(slot->mood, mood, sizeof(slot->mood) - 1);
  memcpy(slot->hash, hash, sizeof(hash));
  memcpy(slot->salt, salt, sizeof(salt));

  DEBUG_PRINTLN(F("Generated hash for "));
  DEBUG_PRINTLN(mood);
}

/**
void saveFlagToFlash() {
  nrf_nvmc_page_erase((uint32_t)flashFlag);
  nrf_nvmc_write_words(
    (uint32_t)flashFlag,
    (uint32_t *)&ramFlag,
    (sizeof(FlagData) + 3) / 4
  );
  DEBUG_PRINTLN("Hash done flag saved to flash");
}

bool loadFlagFromFlash() {
  if (flashFlag->magic != 0xDEADBEEF) {
    return false;
  }
  memcpy(&ramFlag, flashFlag, sizeof(FlagData));
  return true;
}
**/

void saveToFlash() {
  nrf_nvmc_page_erase((uint32_t)flashData);
  nrf_nvmc_write_words((uint32_t)flashData, (uint32_t *)&ramData, (sizeof(FlashData) + 3) / 4);
  DEBUG_PRINTLN(F("All mood hashes saved to flash"));
}

void printHash(const uint8_t *hash, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (hash[i] < 0x10) DEBUG_PRINT('0');
    Serial.print(hash[i], HEX);
  }
  DEBUG_PRINTLN();
}

void loadFromFlash() {
  memcpy(&ramData, flashData, sizeof(FlashData));

  DEBUG_PRINTLN(F("Loaded hashes into RAM:"));
  for (int i = 0; i < 4; i++) {
    DEBUG_PRINT(F("  - "));
    DEBUG_PRINT(ramData.moods[i].mood);
    DEBUG_PRINT(": ");
    printHash(ramData.moods[i].hash, 32);
  }
}

void calibrationz() {
  /**
  unsigned long startTime = millis();

  while (millis() - startTime < CALIBRATION_DURATION) {
    float hb = Heartbeat_sensor();
    float tp = Temp_sensor();
    float gs = GSR_sensor();

    calib.heartbeat = (CALIBRATION_ALPHA * hb) + ((1 - CALIBRATION_ALPHA) * calib.heartbeat);
    calib.temperature = (CALIBRATION_ALPHA * tp) + ((1 - CALIBRATION_ALPHA) * calib.temperature);
    calib.gsr = (CALIBRATION_ALPHA * gs) + ((1 - CALIBRATION_ALPHA) * calib.gsr);
  }
  **/
  nrf_nvmc_page_erase((uint32_t)flashCalib);
  calib.heartbeat = 1.0;
  calib.temperature = 2.0;
  calib.gsr = 3.0;
  calib.calib_done = true;
  calib.magic = 0xDEADBEEF;
  saveCalibrationToFlash(); 
  calib_donez = true;
}

void saveCalibrationToFlash() {
  nrf_nvmc_page_erase((uint32_t)flashCalib);
  nrf_nvmc_write_words((uint32_t)flashCalib, (uint32_t *)&calib, (sizeof(CalibrationData) + 3) / 4);
  DEBUG_PRINTLN(F("Calibration saved to flash"));
}

void loadCalibrationFromFlash() {
  memcpy(&calib, flashCalib, sizeof(CalibrationData));

  if (calib.calib_done) {
    DEBUG_PRINTLN(F("Loaded calibration from flash:"));

    DEBUG_PRINT(F("Heartbeat: "));
    Serial.print(calib.heartbeat, 2);
    Serial.println();

    DEBUG_PRINT(F("Temperature: "));
    Serial.print(calib.temperature, 2);
    Serial.println();

    DEBUG_PRINT(F("GSR: "));
    Serial.print(calib.gsr, 2);
    Serial.println();
  } 
  else {
    Serial.println(F("No valid calibration in flash."));
  }
}

void deep_sleep() {
  while (sleep_modez) {
    central = BLE.central();
    sd_app_evt_wait();
    DEBUG_PRINTLN("Good night!");

    if (central) {
      DEBUG_PRINT("Connected to: ");
      DEBUG_PRINTLN(central.address());
      while (central.connected()) {
        if (rxChar.written()) {
            rxChar.readValue(&received, 1);

            DEBUG_PRINT("Received command: ");
            DEBUG_PRINTLN(received);

            if (received == 0x01) {
            sendingEnabled = true;
            sleep_modez = false;
            currentHashIndex = 0;
            sendinghashEnabled = true;
            hashSent = false;

            ramData.magic = 0xDEADBEEF;
            //ramData.hashz_flashz_done = false;

            hashz_flashz("happy", &ramData.moods[0]);
            hashz_flashz("sad", &ramData.moods[1]);
            hashz_flashz("calm", &ramData.moods[2]);
            hashz_flashz("energetic", &ramData.moods[3]);

            //ramData.hashz_flashz_done = true;

            saveToFlash();

            DEBUG_PRINTLN(F("New mood hashes generated and saved to flash."));
            loadFromFlash();
            DEBUG_PRINTLN("Sending started/resumed.");
          }
            else if (received == 0x0C) {
              sleep_modez = false;
              calibrationz();
              txChar.writeValue(&calib_done, 1);
              DEBUG_PRINTLN("Calibration complete");
            }
            break;
          }
        }
      }
    }
  }
