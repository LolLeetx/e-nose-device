// Include the Arduino library here (something like your_project_inference.h) 
// In the Arduino IDE see **File > Examples > Your project name - Edge Impulse > Static buffer** to get the exact name
#include <esp32_fruit_inferencing.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLE2901.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLE2901 *descriptor_2901 = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};


static unsigned long last_interval_ms = 0;
// to classify 1 frame of data you need EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE values
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
// keep track of where we are in the feature array
size_t feature_ix = 0;


void setup() {
  Serial.begin(115200);

  if (!ads.begin(0x48)) { // 0x48 default, check your ADDR pin
    Serial.println("ADS1115 not found. Check wiring!");
    while (1);
  }

  // Full-scale range = ±6.144 V (smallest gain, no clipping with 5V sensors)
  ads.setGain(GAIN_TWOTHIRDS);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  display.display();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Waiting for data...");
  display.display();


  // Create the BLE Device
  BLEDevice::init("ESP32");
  BLEDevice::setMTU(512);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE
  );

  // Creates BLE Descriptor 0x2902: Client Characteristic Configuration Descriptor (CCCD)
  pCharacteristic->addDescriptor(new BLE2902());
  // Adds also the Characteristic User Description - 0x2901 descriptor
  descriptor_2901 = new BLE2901();
  descriptor_2901->setDescription("My own description for this characteristic.");
  descriptor_2901->setAccessPermissions(ESP_GATT_PERM_READ);  // enforce read only - default is Read|Write
  pCharacteristic->addDescriptor(descriptor_2901);

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
  ei_printf("Enter 4 floats CH4,CO,Odor,CH2O on Serial\n");
}


void loop() {
  int16_t rawMQ7   = ads.readADC_SingleEnded(0); // A0 → MQ-7 (CO)
  int16_t rawMQ8   = ads.readADC_SingleEnded(1); // A1 → MQ-8 (H2)
  int16_t rawMQ135 = ads.readADC_SingleEnded(2); // A2 → MQ-135

  Serial.print("MQ-7 raw: ");
  Serial.print(rawMQ7);
  Serial.print(" | MQ-8 raw: ");
  Serial.print(rawMQ8);
  Serial.print(" | MQ-135 raw: ");
  Serial.println(rawMQ135);


  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  // Parse 4 comma-separated values
  float ch4, co, odor, ch2o;
  char buf[64];
  line.toCharArray(buf, sizeof(buf));
  char *tok = strtok(buf, ",");
  int count = 0;
  float vals[4];
  while (tok && count < 4) {
    vals[count++] = atof(tok);
    tok = strtok(NULL, ",");
  }
  if (count != 4) {
    ei_printf("Expected 4 values, got %d\n", count);
    return;
  }

  ch4  = vals[0];
  co   = vals[1];
  odor = vals[2];
  ch2o = vals[3];

  // fill the features buffer
  for (int _ = 0; _ < EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME; _++){
    features[feature_ix++] = ch4;
    features[feature_ix++] = co;
    features[feature_ix++] = odor;
    features[feature_ix++] = ch2o;
  }


  // features buffer full? then classify!
  if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
      ei_impulse_result_t result;

      // create signal from features frame
      signal_t signal;
      numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

      // run classifier
      EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
      ei_printf("run_classifier returned: %d\n", res);
      if (res != 0) return;

      // print predictions
      ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
          result.timing.dsp, result.timing.classification, result.timing.anomaly);

      // print the predictions
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
          ei_printf("%s:\t%.5f\n", result.classification[ix].label, result.classification[ix].value);
      }
  #if EI_CLASSIFIER_HAS_ANOMALY == 1
      ei_printf("anomaly:\t%.3f\n", result.anomaly);
  #endif
  char ble_buf[128];
  int pos = 0;
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
      pos += snprintf(ble_buf + pos,
                      sizeof(ble_buf) - pos,
                      "%s:%.3f,",
                      result.classification[i].label,
                      result.classification[i].value);
  }
  
  pos += snprintf(ble_buf + pos,
              sizeof(ble_buf) - pos,
              "CH4:%.3f,CO:%.3f,Odor:%.3f,CH2O:%.3f",
              ch4, co, odor, ch2o);
              
  if (pos > 0) ble_buf[pos - 1] = '\0'; // remove trailing comma

  // Transmit over BLE notifications
  pCharacteristic->setValue((uint8_t*)ble_buf, strlen(ble_buf));
  pCharacteristic->notify();

  ei_printf("Sent over BLE: %s\n\n", ble_buf);

  display.clearDisplay();
  display.setCursor(0, 0);

  // If the message is longer than fits on one line,
  // you can split it into two lines:
  String s = String(ble_buf);
  // if (s.length() > 20) {
  //   display.println(s.substring(0, 20));
  //   display.println(s.substring(20));
  // } else {
  //   display.println(s);
  // }

  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    // make a small buffer to hold each formatted line
    char buf[32];
    // format into buf: "label: 0.12345"
    snprintf(buf, sizeof(buf), "%s: %.5f",
            result.classification[ix].label,
            result.classification[ix].value);
    // print the formatted line
    display.println(buf);
  }

  const int rowHeight = 8;
  // y-coordinate of the first of your 4 bottom rows
  int16_t yStart = SCREEN_HEIGHT - rowHeight * 4;

  display.setCursor(0, yStart);
  char sensorBuf[32];

  snprintf(sensorBuf, sizeof(sensorBuf), "CH4: %.3f", ch4);
  display.println(sensorBuf);

  snprintf(sensorBuf, sizeof(sensorBuf), "CO:  %.3f", co);
  display.println(sensorBuf);

  snprintf(sensorBuf, sizeof(sensorBuf), "Odor:%.3f", odor);
  display.println(sensorBuf);

  snprintf(sensorBuf, sizeof(sensorBuf), "CH2O:%.3f", ch2o);
  display.println(sensorBuf);


  // Finally push everything to the OLED
  display.display();



  // reset features frame
  feature_ix = 0;
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }


}

void ei_printf(const char *format, ...) {
    static char print_buf[1024] = { 0 };

    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    if (r > 0) {
        Serial.write(print_buf);
    }
}