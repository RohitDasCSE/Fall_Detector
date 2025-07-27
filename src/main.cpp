#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <audio_inferencing.h>
#include <driver/i2s.h>
#include <edge-impulse-sdk/classifier/ei_run_classifier.h>
#include <edge-impulse-sdk/dsp/numpy.hpp>

// Function prototypes
void        handleMonitoringState();
void        handleFreeFallState(unsigned long currentTime);
void        handleImpactState(unsigned long currentTime);
void        handleStaticCheckState(unsigned long currentTime);
void        handleAlertActiveState(unsigned long currentTime);
void        handleSendingAlertsState();

void        initAudioInference();
void        audioInferenceTask(void *pvParameters);
void        runAudioInference();
static int  microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr);

void        handleSosButtonPress(unsigned long currentTime);
void        resetDetectionStates();
void        checkWiFiConnection();

bool        sendATCommand(const char* cmd, const char* expected, unsigned long timeout);
void        sendSMSAlert(const char* number, const String& timeStr);
void        makeEmergencyCall(const char* number);
void        sendServerAlert(const String& timeStr);

// System states
enum SystemState {
  STATE_MONITORING,
  STATE_FREE_FALL,
  STATE_IMPACT_DETECTED,
  STATE_STATIC_CHECK,
  STATE_ALERT_ACTIVE,
  STATE_SENDING_ALERTS
};

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
SystemState currentState = STATE_MONITORING;

// GPIO pins
const int buzzerPin = 12;
const int cancelPin = 13;
const int sosPin = 14;  
const int ledPin = 15;         

// SIM800L serial pins and caregiver numbers
const int SIM_RX_PIN = 32;   // ESP32 receives from SIM800L TX here
const int SIM_TX_PIN = 33;   // ESP32 transmits to SIM800L RX here
const char* CAREGIVER_NUMBERS[] = {"+1234567890", "+1987654320"};
const int NUM_CAREGIVERS = 2;

// Create HardwareSerial instance for SIM800L on UART1
HardwareSerial SIM800(1);

// Thresholds
const float             FREE_FALL_THRESH        =     0.7f;                   // g
const float             IMPACT_THRESH           =     2.4f;                   // g
const unsigned long     MAX_IMPACT_INTERVAL     =     800;                    // ms
const unsigned long     STATIC_DURATION         =     10000;                  // ms
const float             GYRO_MOVEMENT_THRESH    =     10.0f * PI / 180.0f;    // rad/s

// Wi-Fi credentials
const char ssid[] = "James Bond";
const char password[] = "heheheheh";

// Server configuration
const char server[] = "domain-name.com";
const uint16_t serverPort = 443;
const char serverPath[] = "/fallAlert";

// Timing constants
const unsigned long     BLINK_INTERVAL        =   500;    // ms 
const unsigned long     SOS_DEBOUNCE_TIME     =   300;    // ms
const unsigned long     WIFI_CHECK_INTERVAL   =   30000;  // ms
const unsigned long     AUDIO_COOLDOWN        =   10000;  // ms

// State timers
unsigned long stateStartTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long lastWiFiCheck = 0;
unsigned long lastSosCheck = 0;
unsigned long lastAudioTrigger = 0;

// System flags
bool freeFallDetected = false;
bool impactDetected = false;
bool staticCondition = false;
bool alertOutputState = false; // buzzer/LED state
volatile bool audioHelpDetected = false;
String locationCoords = "unknown";

// Audio inference variables
static bool record_status = true;
static bool debug_nn = false;

// Audio buffer configuration
const int SAMPLE_BUFFER_SIZE = 2048;
static signed short sampleBuffer[SAMPLE_BUFFER_SIZE];
const int INFERENCE_BUFFER_SIZE = EI_CLASSIFIER_SLICE_SIZE;

// Audio inference structure
typedef struct {
    signed short *buffers[2];       // array of two pointers to the two buffers
    unsigned char buf_select;       // which buffer index is currently being filled
    unsigned char buf_ready;        // 1 means buffer is ready and can be processed
    unsigned int buf_count;         // no. of samples written so far in current buffer
    unsigned int n_samples;         // samples per buffer (slice size)
} inference_t;

static inference_t inference;

// I2S configuration for INMP441
const i2s_port_t I2S_PORT = I2S_NUM_0;
const int I2S_BCK_PIN = 26;
const int I2S_WS_PIN = 25;
const int I2S_DOUT_PIN = 27;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize I2C
  #ifdef ESP32
    Wire.setPins(21, 22);
  #endif

  // MPU6050 initialization
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(1000);
      Serial.println("Check MPU6050 wiring and pull-up resistors!");
    }
  }
  
  // Configure sensor ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize GPIO
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(cancelPin, INPUT_PULLUP);
  pinMode(sosPin, INPUT_PULLUP);

  // GPS initialization on UART2
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  // SIM800L initialization
  SIM800.begin(9600, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  delay(1000);
  sendATCommand("AT", "OK", 1000);
  sendATCommand("AT+CMGF=1", "OK", 1000);   // sms mode = text
  sendATCommand("AT+CLIP=1", "OK", 1000);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected. IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nFailed to connect! Continuing without WiFi");
  }

  // Initialize audio inference
  initAudioInference();

  // Calibration delay
  Serial.println("Calibrating sensors...");
  delay(2000);
  Serial.println("System ready");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Feed GPS data continuously
  while (Serial2.available()) {
    gps.encode(Serial2.read());
  }
  
  // Update location when valid
  if (gps.location.isValid() && gps.location.isUpdated()) {
    locationCoords = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
  }

  // Maintain WiFi connection
  if (currentTime - lastWiFiCheck > WIFI_CHECK_INTERVAL) {
    checkWiFiConnection();
    lastWiFiCheck = currentTime;
  }

  // Check SOS button with debouncing
  if (currentTime - lastSosCheck > SOS_DEBOUNCE_TIME) {
    lastSosCheck = currentTime;
    if (digitalRead(sosPin) == LOW) {
      handleSosButtonPress(currentTime);
    }
  }

  // Check audio detection ("help")
  if (audioHelpDetected && currentState == STATE_MONITORING && 
      (currentTime - lastAudioTrigger > AUDIO_COOLDOWN)) {
    Serial.println("Audio help detected - Manual alert activated!");
    currentState = STATE_ALERT_ACTIVE;
    stateStartTime = currentTime;
    lastAudioTrigger = currentTime;
    audioHelpDetected = false;
    
    // Initialize alert outputs
    alertOutputState = false;
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledPin, LOW);
    lastBlinkTime = currentTime;
  }

  // state machine
  switch (currentState) {
    case STATE_MONITORING:
      handleMonitoringState();
      break;
    case STATE_FREE_FALL:
      handleFreeFallState(currentTime);
      break;
    case STATE_IMPACT_DETECTED:
      handleImpactState(currentTime);
      break;
    case STATE_STATIC_CHECK:
      handleStaticCheckState(currentTime);
      break;
    case STATE_ALERT_ACTIVE:
      handleAlertActiveState(currentTime);
      break;
    case STATE_SENDING_ALERTS:
      handleSendingAlertsState();
      break;
  }
  
  delay(10);
}

// Audio Inference Initialization
void initAudioInference() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DOUT_PIN
  };
  
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);

  // Allocate inference buffers
  inference.buffers[0] = (signed short *)malloc(INFERENCE_BUFFER_SIZE * sizeof(signed short));
  inference.buffers[1] = (signed short *)malloc(INFERENCE_BUFFER_SIZE * sizeof(signed short));
  
  if (!inference.buffers[0] || !inference.buffers[1]) {
    Serial.println("Failed to allocate audio buffers");
    return;
  }
  
  inference.buf_select = 0;
  inference.buf_count = 0;
  inference.n_samples = INFERENCE_BUFFER_SIZE;
  inference.buf_ready = 0;
  
  // Start audio inference task
  xTaskCreatePinnedToCore(
    audioInferenceTask,    // Task function
    "AudioInference",      // Task name
    8192,                  // Stack size (bytes)
    NULL,                  // Task parameters
    1,                     // Task priority
    NULL,                  // Task handle
    0                      // Core ID (core 0)
  );
}

void audioInferenceTask(void *pvParameters) {
  size_t bytes_read;
  const size_t bytes_to_read = SAMPLE_BUFFER_SIZE * sizeof(int16_t);
  
  while (true) {
    // Read audio data from I2S
    i2s_read(I2S_PORT, (void*)sampleBuffer, bytes_to_read, &bytes_read, portMAX_DELAY);
    int samples_read = bytes_read / sizeof(int16_t);

    if (samples_read <= 0) continue;
    
    // Process audio samples
    for (int i = 0; i < samples_read; i++) {
      inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];
      
      if (inference.buf_count >= inference.n_samples) {
        inference.buf_select ^= 1;
        inference.buf_count = 0;
        inference.buf_ready = 1;
        
        // Run inference when buffer is ready
        if (inference.buf_ready) {
          runAudioInference();
          inference.buf_ready = 0;
        }
      }
    }
  }
}

void runAudioInference() {
  signal_t signal;
  signal.total_length = INFERENCE_BUFFER_SIZE;
  signal.get_data = &microphone_audio_signal_get_data;
  
  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR err = run_classifier_continuous(&signal, &result, debug_nn);
  
  if (err != EI_IMPULSE_OK) {
    Serial.print("Audio inference error: ");
    Serial.println(err);
    return;
  }
  
  // Check for "help" detection
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    if (strcmp(result.classification[ix].label, "help") == 0) {
      if (result.classification[ix].value > 0.75f) { 
        audioHelpDetected = true;
        Serial.print("Help detected! Confidence: ");
        Serial.println(result.classification[ix].value);
      }
    }
  }
}

// Audio signal processing
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
  numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
  return 0;
}

void handleSosButtonPress(unsigned long currentTime) {
  // Only trigger in monitoring state
  if (currentState == STATE_MONITORING) {
    Serial.println("SOS button pressed - Manual alert activated!");
    
    // Start alert sequence
    currentState = STATE_ALERT_ACTIVE;
    stateStartTime = currentTime;
    
    // Initialize alert outputs
    alertOutputState = false;
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledPin, LOW);
    lastBlinkTime = currentTime;
    
    // Reset detection flags
    freeFallDetected = false;
    impactDetected = false;
    staticCondition = false;
  }
}

// State Handlers
void handleMonitoringState() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Compute acceleration magnitude in g
  float ax = a.acceleration.x / 9.80665f;
  float ay = a.acceleration.y / 9.80665f;
  float az = a.acceleration.z / 9.80665f;
  float accelMag = sqrtf(ax*ax + ay*ay + az*az);
  
  // Detect free fall condition
  if (accelMag < FREE_FALL_THRESH) {
    freeFallDetected = true;
    currentState = STATE_FREE_FALL;
    stateStartTime = millis();
    Serial.println("Free fall detected!");
  }
}

void handleFreeFallState(unsigned long currentTime) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  float ax = a.acceleration.x / 9.80665f;
  float ay = a.acceleration.y / 9.80665f;
  float az = a.acceleration.z / 9.80665f;
  float accelMag = sqrtf(ax*ax + ay*ay + az*az);
  
  // Check for impact
  if (accelMag > IMPACT_THRESH) {
    impactDetected = true;
    currentState = STATE_IMPACT_DETECTED;
    stateStartTime = currentTime;
    Serial.println("Impact detected!");
    return;
  }
  
  // Timeout free-fall detection
  if (currentTime - stateStartTime > MAX_IMPACT_INTERVAL) {
    freeFallDetected = false;
    currentState = STATE_MONITORING;
    Serial.println("Free fall timeout");
  }
}

void handleImpactState(unsigned long currentTime) {
  if (currentTime - stateStartTime < 200) return;
  
  currentState = STATE_STATIC_CHECK;
  stateStartTime = currentTime;
  staticCondition = true;
  Serial.println("Starting static check");
}

void handleStaticCheckState(unsigned long currentTime) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Check for movement
  if (fabsf(g.gyro.x) > GYRO_MOVEMENT_THRESH ||
      fabsf(g.gyro.y) > GYRO_MOVEMENT_THRESH ||
      fabsf(g.gyro.z) > GYRO_MOVEMENT_THRESH) {
    staticCondition = false;
  }
  
  // Check cancel button
  if (digitalRead(cancelPin) == LOW) {
    staticCondition = false;
    Serial.println("Fall cancelled by button");
  }
  
  // Check static duration completion
  if (currentTime - stateStartTime > STATIC_DURATION) {
    if (staticCondition) {
      currentState = STATE_ALERT_ACTIVE;
      stateStartTime = currentTime;
      
      // Initialize alert outputs
      alertOutputState = false;
      digitalWrite(buzzerPin, LOW);
      digitalWrite(ledPin, LOW);
      lastBlinkTime = currentTime;
      
      Serial.println("Static condition confirmed - Alert activated!");
    } else {
      resetDetectionStates();
      Serial.println("Movement detected - Cancelling alert");
    }
  }
}

void handleAlertActiveState(unsigned long currentTime) {
  // Handle blinking for buzzer and LED
  if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
    alertOutputState = !alertOutputState;
    digitalWrite(buzzerPin, alertOutputState ? HIGH : LOW);
    digitalWrite(ledPin, alertOutputState ? HIGH : LOW);
    lastBlinkTime = currentTime;
  }
  
  // Check for manual cancellation
  if (digitalRead(cancelPin) == LOW) {
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledPin, LOW);
    resetDetectionStates();
    Serial.println("Alert cancelled by button");
    return;
  }
  
  // Timeout after 10 seconds and proceed to alerts
  if (currentTime - stateStartTime > 10000) {
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledPin, LOW);
    currentState = STATE_SENDING_ALERTS;
    Serial.println("Proceeding to send alerts");
  }
}

void handleSendingAlertsState() {
  // Get current time string
  String timeStr = "unknown";
  if (gps.time.isValid()) {
    timeStr = String(gps.time.hour()) + ":" + 
              String(gps.time.minute()) + ":" + 
              String(gps.time.second());
  }
  
  // Send alerts to all caregivers
  for (int i = 0; i < NUM_CAREGIVERS; i++) {
    sendSMSAlert(CAREGIVER_NUMBERS[i], timeStr);
    makeEmergencyCall(CAREGIVER_NUMBERS[i]);
  }
  
  // Send server alert if WiFi available
  if (WiFi.status() == WL_CONNECTED) {
    sendServerAlert(timeStr);
  }
  
  // Return to monitoring
  resetDetectionStates();
}

// resets system flags and sets current state as monitoring
void resetDetectionStates() {
  freeFallDetected = false;
  impactDetected = false;
  staticCondition = false;
  currentState = STATE_MONITORING;
}

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nReconnected. IP: " + WiFi.localIP().toString());
    } else {
      Serial.println("\nFailed to reconnect!");
    }
  }
}

bool sendATCommand(const char* cmd, const char* expected, unsigned long timeout) {
  SIM800.println(cmd);
  Serial.println("SIM800 << " + String(cmd));
  
  unsigned long start = millis();
  String response = "";
  
  while (millis() - start < timeout) {
    while (SIM800.available()) {
      char c = SIM800.read();
      response += c;
      Serial.write(c);
    }
    
    if (response.indexOf(expected) != -1) {
      return true;
    }
  }
  
  Serial.println("SIM800 command timeout: " + String(cmd));
  return false;
}

void sendSMSAlert(const char* number, const String& timeStr) {
  String message = "Fall detected at " + timeStr + 
                   "\nCoordinates: " + locationCoords + 
                   "\nHelp needed!";
  
  if (!sendATCommand("AT+CMGF=1", "OK", 1000)) return;
  
  SIM800.print("AT+CMGS=\"");
  SIM800.print(number);
  SIM800.println("\"");
  delay(100);
  
  SIM800.print(message);
  SIM800.write(0x1A);
  
  if (!sendATCommand("", "OK", 5000)) {
    Serial.println("SMS failed to send to " + String(number));
  }
}

void makeEmergencyCall(const char* number) {
  SIM800.print("ATD");
  SIM800.print(number);
  SIM800.println(";");
  
  // Wait for call to connect (20 seconds max)
  unsigned long start = millis();
  while (millis() - start < 20000) {
    if (SIM800.available()) {
      String response = SIM800.readString();
      if (response.indexOf("BUSY") != -1 || 
          response.indexOf("NO ANSWER") != -1) {
        break;
      }
      if (response.indexOf("OK") != -1) {
        // Play audio alert if available
        sendATCommand("AT+CREC=3,0,\"FALL.AMR\"", "OK", 3000);
        delay(10000);  // Play for 10 seconds
        break;
      }
    }
    delay(100);
  }
  
  sendATCommand("ATH", "OK", 1000);  // Hang up
}

void sendServerAlert(const String& timeStr) {
  HTTPClient https;
  String url = "https://" + String(server) + serverPath;
  
  // Create JSON payload
  DynamicJsonDocument doc(256);
  doc["device_id"] = WiFi.macAddress();
  doc["timestamp"] = timeStr;
  doc["coordinates"] = locationCoords;
  doc["alert_type"] = (freeFallDetected) ? "auto" : "manual";
  
  String payload;
  serializeJson(doc, payload);
  
  https.begin(url);
  https.addHeader("Content-Type", "application/json");
  
  int httpCode = https.POST(payload);
  if (httpCode == HTTP_CODE_OK) {
    Serial.println("Server alert sent successfully");
  } else {
    Serial.printf("Server alert failed, error: %s\n", https.errorToString(httpCode).c_str());
  }
  https.end();
}

