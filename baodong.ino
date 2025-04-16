#define BLYNK_TEMPLATE_ID "TMPL62xTT4VQM"
#define BLYNK_TEMPLATE_NAME "Nhom4"
#define BLYNK_AUTH_TOKEN "YojosDhPZMZzVxq-dGws31wrZ2yFzLqj"
#define BLYNK_PRINT Serial
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <BlynkSimpleEsp32.h>

// --- CẤU HÌNH BLYNK ---

// Cấu hình phần cứng
const int trig = 13;
const int echo = 12;
unsigned long thoigian;
int khoangcach;
const int outPin = 14;
int trangthai;
const int coiPin = 27;
const int denPin = 26;
const int duration = 200;

const int buttonPin = 25;
volatile bool systemEnabled = false;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

volatile bool alarmActive = false;

// Cấu hình WiFi và ThingSpeak
const char* ssid = "Mi 11 Lite";
const char* password = "88888888";
const char* thingspeak_server = "api.thingspeak.com";
String apiKey = "FEA9XT4MFQ7MPZ3K"; // Write API Key
String readApiKey = "V6PHEB4FRJAUJQW1"; // Read API Key
unsigned long lastThingSpeakUpdate = 0;
const unsigned long thingSpeakDelay = 15000;

// Task handles
TaskHandle_t buttonTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t alarmTaskHandle = NULL;
TaskHandle_t thingSpeakTaskHandle = NULL;

bool remoteControlActive = false;
QueueHandle_t alarmQueue;
QueueHandle_t thingSpeakQueue;

struct SensorData {
  int doorStatus;    // 0 = closed, 1 = open
  int motionStatus;  // 0 = no motion, 1 = motion detected
  int systemStatus;  // 0 = off, 1 = on
  int alarmStatus;   // 0 = off, 1 = on
};

SemaphoreHandle_t systemStateMutex;
SemaphoreHandle_t alarmStateMutex;

bool prevSystemEnabled = false;
bool prevMotionState = false;
int prevDoorStatus = -1;
bool printNeeded = false;

// Hàm báo động
void baodong() {
  //nhịp báo S.O.S là 3 tiếng ngắn 3 tiếng dài
  for(int t=0; t < 3; t++) { // lặp lại nhịp báo 3 lần (5,6s/lần)
    //3 tiếng ngắn
    for(int i=0; i<3; i++) {
      digitalWrite(coiPin, LOW);    // Bật relay (active LOW)
      digitalWrite(denPin, LOW);    // Bật relay (active LOW)
      vTaskDelay(duration / portTICK_PERIOD_MS);
      digitalWrite(coiPin, HIGH);   // Tắt relay
      digitalWrite(denPin, HIGH);   // Tắt relay
      vTaskDelay(duration / portTICK_PERIOD_MS);
    }
    vTaskDelay((duration*5) / portTICK_PERIOD_MS);
    
    //3 tiếng dài
    for(int i=0; i<3; i++) {
      digitalWrite(coiPin, LOW);    // Bật relay (active LOW)
      digitalWrite(denPin, LOW);    // Bật relay (active LOW)
      vTaskDelay((duration*3) / portTICK_PERIOD_MS);
      digitalWrite(coiPin, HIGH);   // Tắt relay
      digitalWrite(denPin, HIGH);   // Tắt relay
      vTaskDelay(duration / portTICK_PERIOD_MS);
    }
    vTaskDelay((duration*5) / portTICK_PERIOD_MS);

    //3 tiếng ngắn
    for(int i=0; i<3; i++) {
      digitalWrite(coiPin, LOW);    // Bật relay (active LOW)
      digitalWrite(denPin, LOW);    // Bật relay (active LOW)
      vTaskDelay(duration / portTICK_PERIOD_MS);
      digitalWrite(coiPin, HIGH);   // Tắt relay
      digitalWrite(denPin, HIGH);   // Tắt relay
      vTaskDelay(duration / portTICK_PERIOD_MS);
    }
    vTaskDelay((duration*10) / portTICK_PERIOD_MS);
  }
  digitalWrite(coiPin, HIGH);       // Đảm bảo relay tắt khi kết thúc
  digitalWrite(denPin, HIGH);       // Đảm bảo relay tắt khi kết thúc
}

// Task xử lý nút nhấn
void buttonTask(void *parameter) {
  while(1) {
    // Đọc trạng thái nút nhấn
    bool reading = digitalRead(buttonPin);
    
    // Nếu nút được nhấn (LOW)
    if (reading == LOW) {
      vTaskDelay(50 / portTICK_PERIOD_MS); // Debounce
      if (digitalRead(buttonPin) == LOW) {  // Đọc lại để xác nhận
        // Đảm bảo thread-safe khi thay đổi biến systemEnabled
        xSemaphoreTake(systemStateMutex, portMAX_DELAY);
        systemEnabled = !systemEnabled;  // Đảo trạng thái
        xSemaphoreGive(systemStateMutex);
        
        Serial.print("System: ");
        Serial.println(systemEnabled ? "BẬT" : "TẮT");
        
        // System state has changed - note that we need to print status
        printNeeded = true;
        
        // Create and send an immediate update to ThingSpeak
        SensorData sensorData;
        sensorData.systemStatus = systemEnabled ? 1 : 0;
        
        xSemaphoreTake(alarmStateMutex, portMAX_DELAY);
        sensorData.alarmStatus = alarmActive ? 1 : 0;
        xSemaphoreGive(alarmStateMutex);
        
        // Get the latest sensor readings for the update
        digitalWrite(trig, 0);
        delayMicroseconds(2);
        digitalWrite(trig, 1);
        delayMicroseconds(10);
        digitalWrite(trig, 0);
        thoigian = pulseIn(echo, HIGH);
        khoangcach = int(thoigian/2/29.412);
        sensorData.doorStatus = (khoangcach < 20) ? 1 : 0;
        
        trangthai = digitalRead(outPin);
        sensorData.motionStatus = trangthai;
        
        // Queue the update
        xQueueSend(thingSpeakQueue, &sensorData, 0);
        
        while(digitalRead(buttonPin) == LOW) {  // Chờ nhả nút
          vTaskDelay(10 / portTICK_PERIOD_MS);
        }
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// Task đọc cảm biến
void sensorTask(void *parameter) {
  bool isEnabled;
  bool alarmTriggered;
  SensorData sensorData;
  bool systemStateChanged = false;
  bool motionStateChanged = false;
  bool doorStateChanged = false;
  bool newAlarmActivation = false;
  
  while(1) {
    // Reset status change flags
    systemStateChanged = false;
    motionStateChanged = false;
    doorStateChanged = false;
    newAlarmActivation = false;
    
    // Kiểm tra trạng thái hệ thống
    xSemaphoreTake(systemStateMutex, portMAX_DELAY);
    isEnabled = systemEnabled;
    sensorData.systemStatus = systemEnabled ? 1 : 0;
    // Check if system state changed
    if (isEnabled != prevSystemEnabled) {
      systemStateChanged = true;
      prevSystemEnabled = isEnabled;
    }
    xSemaphoreGive(systemStateMutex);
    
    // Kiểm tra trạng thái báo động
    xSemaphoreTake(alarmStateMutex, portMAX_DELAY);
    sensorData.alarmStatus = alarmActive ? 1 : 0;
    // Check if there's a new alarm activation
    if (alarmActive && !newAlarmActivation) {
      newAlarmActivation = true;
    }
    xSemaphoreGive(alarmStateMutex);
    
    // Đọc cảm biến siêu âm
    digitalWrite(trig, 0);
    delayMicroseconds(2);
    digitalWrite(trig, 1);
    delayMicroseconds(10);
    digitalWrite(trig, 0);

    thoigian = pulseIn(echo, HIGH);
    khoangcach = int(thoigian/2/29.412);

    // Validate readings - ignore zero readings which are likely errors
    if (khoangcach == 0) {
      // Skip this iteration if we get a zero reading
      vTaskDelay(300 / portTICK_PERIOD_MS);
      continue;
    }

    // Xác định trạng thái cửa (field1)
    sensorData.doorStatus = (khoangcach < 20) ? 1 : 0; // 1 = open, 0 = closed
    
    // Check if door status changed
    if (sensorData.doorStatus != prevDoorStatus) {
      doorStateChanged = true;
      prevDoorStatus = sensorData.doorStatus;
    }
    
    // Đọc cảm biến chuyển động
    trangthai = digitalRead(outPin);
    sensorData.motionStatus = trangthai;
    
    // Check if motion status changed
    if ((bool)sensorData.motionStatus != prevMotionState) {
      motionStateChanged = true;
      prevMotionState = (bool)sensorData.motionStatus;
    }
    
    // Print if any condition is met or if printNeeded flag is set
    if (systemStateChanged || motionStateChanged || doorStateChanged || 
        newAlarmActivation || printNeeded || 
        (sensorData.doorStatus == 1)) { // Also print if door is open
        
      // In giá trị cảm biến trên cùng một dòng
      Serial.print("Sensors | Khoảng cách: ");
      Serial.print(khoangcach);
      Serial.print(" cm | Chuyển động: ");
      Serial.print(trangthai);
      Serial.print(" | Cửa: ");
      Serial.print(sensorData.doorStatus == 1 ? "Mở" : "Đóng");
      Serial.print(" | Hệ thống: ");
      Serial.print(isEnabled ? "BẬT" : "TẮT");
      Serial.print(" | Báo động: ");
      Serial.println(sensorData.alarmStatus ? "ON" : "OFF");
      
      // Add explanation of why we're printing
      if (systemStateChanged) Serial.println(" > Trạng thái hệ thống thay đổi");
      if (motionStateChanged) Serial.println(" > Trạng thái chuyển động thay đổi");
      if (doorStateChanged) Serial.println(" > Trạng thái cửa thay đổi");
      if (newAlarmActivation) Serial.println(" > CẢNH BÁO TRỘM");
      if (sensorData.doorStatus == 1) Serial.println(" > CỬA ĐANG MỞ");

      // Reset the print needed flag
      printNeeded = false;
    }

    // Gửi dữ liệu lên ThingSpeak
    xQueueSend(thingSpeakQueue, &sensorData, 0);
    
    // Chỉ xử lý cảm biến khi hệ thống được bật
    if (isEnabled) {
      // Add explicit check to avoid triggering alarm on zero readings
      if(khoangcach < 20 && khoangcach > 0 && trangthai == 1) {
        Serial.println("CANH BAO TROM");
        alarmTriggered = true;
        
        // Cập nhật trạng thái báo động
        xSemaphoreTake(alarmStateMutex, portMAX_DELAY);
        alarmActive = true;
        xSemaphoreGive(alarmStateMutex);
        
        // Gửi thông báo qua queue
        xQueueSend(alarmQueue, &alarmTriggered, 0);
      }
    }
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}

// Task báo động
void alarmTask(void *parameter) {
  bool alarmFlag;
  
  while(1) {
    // Đợi thông báo từ queue
    if (xQueueReceive(alarmQueue, &alarmFlag, portMAX_DELAY) == pdTRUE) {
      if (alarmFlag) {
        // Cập nhật trạng thái báo động
        if (!remoteControlActive) {
          xSemaphoreTake(alarmStateMutex, portMAX_DELAY);
          alarmActive = true;
          xSemaphoreGive(alarmStateMutex);
        }
        
        // Execute alarm unless remotely disabled
        xSemaphoreTake(alarmStateMutex, portMAX_DELAY);
        bool shouldSound = alarmActive;
        xSemaphoreGive(alarmStateMutex);
        
        if (shouldSound) {
          baodong();
        }
        
        // Reset alarm state when done if not remotely controlled
        if (!remoteControlActive) {
          xSemaphoreTake(alarmStateMutex, portMAX_DELAY);
          alarmActive = false;
          xSemaphoreGive(alarmStateMutex);
        }
      }
    }
  }
}

// Function to update ThingSpeak
void updateThingSpeak(int field1, int field2, int field3, int field4) {
  HTTPClient http;
  String url = "http://api.thingspeak.com/update?api_key=" + apiKey +
               "&field1=" + String(field1) +
               "&field2=" + String(field2) +
               "&field3=" + String(field3) +
               "&field4=" + String(field4);
  
  http.begin(url);
  int httpResponseCode = http.GET();
  
  if (httpResponseCode > 0) {
    // Success, but no need to print the response
  } else {
    Serial.print("Error on sending GET: ");
    Serial.println(httpResponseCode);
  }
  
  http.end();
  
  // Add a longer delay after HTTP operations
  vTaskDelay(500 / portTICK_PERIOD_MS);  // Increased from 100ms to 500ms
}

// Task to handle ThingSpeak updates
void thingSpeakTask(void *parameter) {
  SensorData data;
  
  while(1) {
    if (WiFi.status() == WL_CONNECTED) {
      if (xQueueReceive(thingSpeakQueue, &data, portMAX_DELAY) == pdTRUE) {
        // Make sure we don't update too frequently (ThingSpeak limit)
        unsigned long currentMillis = millis();
        if (currentMillis - lastThingSpeakUpdate >= thingSpeakDelay) {
          updateThingSpeak(data.doorStatus, data.motionStatus, data.systemStatus, data.alarmStatus);
          lastThingSpeakUpdate = currentMillis;
        }
      }
    } else {
      // Try to reconnect to WiFi
      Serial.println("WiFi disconnected, reconnecting...");
      WiFi.begin(ssid, password);
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// BLYNK CONTROL FUNCTIONS
BLYNK_WRITE(V0) { // Điều khiển đèn
  int value = param.asInt();
  digitalWrite(denPin, value == 1 ? LOW : HIGH);
}

BLYNK_WRITE(V1) { // Điều khiển còi
  int value = param.asInt();
  digitalWrite(coiPin, value == 1 ? LOW : HIGH);
}

BLYNK_WRITE(V2) { // Điều khiển hệ thống
  int value = param.asInt();
  xSemaphoreTake(systemStateMutex, portMAX_DELAY);
  systemEnabled = (value == 1);
  xSemaphoreGive(systemStateMutex);
  
  // Update ThingSpeak immediately
  SensorData sensorData;
  sensorData.systemStatus = systemEnabled ? 1 : 0;
  
  xSemaphoreTake(alarmStateMutex, portMAX_DELAY);
  sensorData.alarmStatus = alarmActive ? 1 : 0;
  xSemaphoreGive(alarmStateMutex);
  
  // Get current sensor readings
  digitalWrite(trig, 0);
  delayMicroseconds(2);
  digitalWrite(trig, 1);
  delayMicroseconds(10);
  digitalWrite(trig, 0);
  thoigian = pulseIn(echo, HIGH);
  khoangcach = int(thoigian/2/29.412);
  sensorData.doorStatus = (khoangcach < 20) ? 1 : 0;
  
  trangthai = digitalRead(outPin);
  sensorData.motionStatus = trangthai;
  
  xQueueSend(thingSpeakQueue, &sensorData, 0);
}

BLYNK_WRITE(V3) { // Điều khiển báo động từ xa
  int value = param.asInt();
  xSemaphoreTake(alarmStateMutex, portMAX_DELAY);
  alarmActive = (value == 1);
  remoteControlActive = true;
  xSemaphoreGive(alarmStateMutex);
  
  if (alarmActive) {
    bool alarmTriggered = true;
    xQueueSend(alarmQueue, &alarmTriggered, 0);
  }
}

void setup() {
  Serial.begin(9600);
  
  // Khởi tạo các chân GPIO
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(outPin, INPUT);
  pinMode(coiPin, OUTPUT);
  pinMode(denPin, OUTPUT);
  digitalWrite(coiPin, HIGH);
  digitalWrite(denPin, HIGH);
  pinMode(buttonPin, INPUT_PULLUP);

  // Kết nối WiFi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed");
  }

  // Khởi tạo Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);

  // Khởi tạo mutex và queue
  systemStateMutex = xSemaphoreCreateMutex();
  alarmStateMutex = xSemaphoreCreateMutex();
  alarmQueue = xQueueCreate(5, sizeof(bool));
  thingSpeakQueue = xQueueCreate(3, sizeof(SensorData));
  
  // Tạo các task
  xTaskCreate(buttonTask, "ButtonTask", 2048, NULL, 3, &buttonTaskHandle);
  xTaskCreate(sensorTask, "SensorTask", 2048, NULL, 2, &sensorTaskHandle);
  xTaskCreate(alarmTask, "AlarmTask", 2048, NULL, 2, &alarmTaskHandle);
  xTaskCreate(thingSpeakTask, "ThingSpeakTask", 4096, NULL, 1, &thingSpeakTaskHandle);
  
  Serial.println("Hệ thống khởi động hoàn tất");
}

void loop() {
  Blynk.run();
  vTaskDelay(10 / portTICK_PERIOD_MS);
}