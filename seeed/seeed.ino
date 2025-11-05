#include <ArduinoBLE.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

//#define DEBUG 1

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
void GSR_sensor();
void Temp_sensor();
void Battery_charge();

bool isCharging = false;

float temperature;

const int ADC_PIN = 14;       // P0.14 (A0 / D14)
const int CHARGE_DETECT_PIN = 31; // P0.31 (connected to charging indicator / limit)

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

const int GSR_PIN = A0;
float gsrFiltered = 0.0;
float gsrAlpha = 0.1;

bool sendingEnabled = false;
unsigned long previousMillis = 0;
const unsigned long interval = 2000;
char lastMessage[64] = "Hello from XIAO Sense!";
char received[64];

void setup() {
  pinMode (P0_13, OUTPUT);
  Battery_charge();

  analogReadResolution(10);

  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial);
  #endif

  // Initialize MAX30105 sensor
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

  // Initialize BLE
  if (!BLE.begin()) while (1);
  BLE.setLocalName("XIAO_Sense");
  BLE.setAdvertisedService(myService);
  myService.addCharacteristic(txChar);
  myService.addCharacteristic(rxChar);
  BLE.addService(myService);
  BLE.advertise();

  DEBUG_PRINTLN("BLE started, advertising as XIAO_Sense");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    DEBUG_PRINT("Connected to: ");
    DEBUG_PRINTLN(central.address());

    while (central.connected()) {
      unsigned long currentMillis = millis();

      // Periodic BLE sending
      if (sendingEnabled && currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        Heartbeat_sensor();
        GSR_sensor();
        Temp_sensor();

        snprintf(lastMessage, sizeof(lastMessage),
                 "HR: %.1f bpm | Avg: %d | GSR: %.1f | Temp: %.2f°C",
                 beatsPerMinute, beatAvg, gsrFiltered, temperature);

        txChar.writeValue(lastMessage);
        DEBUG_PRINTLN(lastMessage);
      }

      // Handle BLE RX commands
      if (rxChar.written()) {
        DEBUG_PRINT("Received command: ");
        DEBUG_PRINTLN(received);

        if (received == "start" || received == "resume") {
          sendingEnabled = true;
          DEBUG_PRINTLN("Sending started/resumed.");
        } 
        else if (received == "retry") {
          txChar.writeValue(lastMessage);
          DEBUG_PRINTLN("Retried last message.");
        } 
        else if (received == "stop") {
          sendingEnabled = false;
          DEBUG_PRINTLN("Stopped sending.");
        } 
        else {
          DEBUG_PRINTLN("Unknown command.");
        }
      }
    }

    DEBUG_PRINTLN("Disconnected.");
  }
}

void GSR_sensor() {
  static bool calibrated = false;
  static int sampleCount = 0;
  static unsigned long prevMillis = 0;
  static int sum = 0;

  unsigned long currentMillis = millis();

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
    gsrFiltered += gsrAlpha * (raw - gsrFiltered);
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

void is_Battery_charge() {
  isCharging = (digitalRead(CHARGE_DETECT_PIN) == LOW);
  if(isCharging){
    pinMode(ADC_PIN, INPUT);
  }
}