/* 
 * MIROSLAV Firmware v0.5.0
 * Arduino firmware for the Multicage InfraRed Open Source Locomotor Activity eValuator - MIROSLAV
 * An open source device for high-throughput monitoring of rodent circadian activity
 */

/* 
 * One complete transmission looks like this:
 * START 2022-01-04 14:22:11,1111111111111111,1111111111111111,1111111111111111, END
 * Each bit series corresponds to:
 * PH7|PH6|PH5|PH4|PH3|PH2|PH1|PH0|PL0|PL1|PL2|PL3|PL4|PL5|PL6|PL7
 * The order of bit series corresponds to the order of MCP addresses in mcps[]
 */

/*
 * To upload through terminal you can use: curl -F "image=@firmware.bin" DEVICE_NAME.local/update
 */

#include "config_test.h"

#ifndef USE_WIFI
  #undef USE_MQTT
#endif

#ifndef USE_MQTT
  #define MSG_SERIAL
#endif

// This is used for debugging purposes only: it will print out fixed values
// without reading the input serialisation stack, so the board can be tested
// without a stack connected.
#define DEBUG_MCP

#ifdef USE_WIFI
  #include <WiFi.h>
  #include <WiFiClient.h>
  #include <WiFiUdp.h>
  #include <ESPmDNS.h>
  #include <HTTPUpdateServer.h>
#endif

#include <Wire.h>
#include "BH1750FVI.h"

#include "time.h"
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
  #include "freertos/task.h"
}
#ifdef USE_MQTT
  #include <AsyncMqttClient.h>
#endif

#define MCP_I2C_SDA 10
#define MCP_I2C_SCL 11
#define ENV_I2C_SDA 35
#define ENV_I2C_SCL 34

#define MCP_L 0x20
#define MCP_R 0x24

#define MCP_IODIRA 0x00
#define MCP_IODIRB 0x01
#define MCP_GPPUA 0x0C
#define MCP_GPPUB 0x0D
// GPIOA0-7 -> PH0-7
#define MCP_GPIOA 0x12
// GPIOB0-7 -> PL7-0 !! notice the reversed order !!
#define MCP_GPIOB 0x13

#define TEMPHUM_ADDR 0x40
#define LUX_ADDR 0x5C

#define PIR_PIN 14

int32_t mcps[] = {0x20+MCP_ID_TOP,
  #ifdef MCP_ID_2
                  0x20+MCP_ID_2,
  #endif
  #ifdef MCP_ID_3
                  0x20+MCP_ID_3,
  #endif
  #ifdef MCP_ID_4
                  0x20+MCP_ID_4,
  #endif
  #ifdef MCP_ID_5
                  0x20+MCP_ID_5,
  #endif
  #ifdef MCP_ID_6
                  0x20+MCP_ID_6,
  #endif
  #ifdef MCP_ID_7
                  0x20+MCP_ID_7,
  #endif
  #ifdef MCP_ID_8
                  0x20+MCP_ID_8,
  #endif
                  -1};

#define MCP_NO (sizeof(mcps)/sizeof(mcps[0]))-1

TwoWire pirI2C = TwoWire(0);
#ifdef DO_ENV
  TwoWire envI2C = TwoWire(1);
  BH1750FVI LuxSensor(LUX_ADDR, &envI2C);
#endif

#ifdef USE_MQTT
  AsyncMqttClient mqttClient;
  TimerHandle_t mqttReconnectTimer;
#endif
TimerHandle_t wifiReconnectTimer;
TaskHandle_t handleHttpHandler = NULL;

TaskHandle_t handlePollPir;
#ifdef DO_ENV
  TaskHandle_t handlePollEnv;
  TaskHandle_t handlePollEnvPir;
#endif

uint32_t time_us;
// careful. this needs to be precisely 20 - 19 for ISO time, 1 for \0
char isotime[20]; 

#ifdef USE_WIFI
  WebServer httpServer(80);
  HTTPUpdateServer httpUpdater;
#endif

// function declarations
void send_log(const String text);
void send_log(const int64_t number);
void utobs(uint16_t number, char *binstring);
#ifdef USE_WIFI
  int updateTime(char *iso_str, uint32_t *micro_int);
  void connectToWifi();
  void WiFiEvent(WiFiEvent_t event);
#endif
#ifdef USE_MQTT
  void connectToMqtt();
  void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
  void onMqttSubscribe(uint16_t packetId, uint8_t qos);
#endif
#ifdef DO_ENV
  void setup_env();
  void hdc_trigger();
  void setup_env_pir();
#endif
void setup_pir();
void write_mcp_reg(uint8_t addr, uint8_t reg, uint8_t val);
uint16_t read_mcp(uint8_t addr);

#ifdef USE_WIFI
  void TaskHttpHandler(void *pvParameters);
  void handle_root();
#endif
void TaskPollPir(void *pvParameters);
#ifdef DO_ENV
  void TaskPollEnv(void *pvParameters);
  void TaskPollEnvPir(void *pvParameters);
  SemaphoreHandle_t envpir_semaphore;
#endif

BaseType_t task_create_response;
#ifndef USE_WIFI
  hw_timer_t *timer = NULL;
#endif

#ifdef DO_ENV
  volatile uint32_t global_envpir_readings;
  volatile uint32_t global_envpir_n_readings;
#endif

void setup() {
  // Set up serial (communication with the PC via USB)
  Serial.begin(115200);
  Serial.println("Booting");

  // Set up Wi-Fi
  #ifdef USE_WIFI
    WiFi.mode(WIFI_STA);
    if (!WiFi.config(LOCAL_IP, GATEWAY, SUBNET, DNS_PRIM, DNS_SECOND)) {
      Serial.println("STA Failed to configure");
    }
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
    WiFi.onEvent(WiFiEvent);
  #endif

  // Set up message sending via MQTT
  #ifdef USE_MQTT
    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onSubscribe(onMqttSubscribe);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setClientId(MQTT_CLIENTID);
    mqttClient.setCredentials(MQTT_USER, MQTT_PASS);
  #endif

  // Connect to Wi-Fi
  #ifdef USE_WIFI
    connectToWifi();
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  #endif

  // Set up OTA code uploading via http (web browser)
  #ifdef USE_WIFI
    httpUpdater.setup(&httpServer);
    httpServer.on("/", handleRoot);
    httpServer.begin();
    Serial.printf("HTTPUpdateServer is set up. Open http://%s/update in your browser\n", WiFi.localIP().toString());
  #endif
  
  // Set up NTP time synchronisation
  #ifdef USE_WIFI
    configTime(NTP_OFFSET_GMT, NTP_OFFSET_DST, NTP_SERVER);
    uint8_t exit_signal;
    for (uint8_t i = 0; i < 10; i++) {
      exit_signal = updateTime(isotime, &time_us);
      if (exit_signal == 0) break;
    } 
    if (exit_signal == 1) ESP.restart();
    Serial.print("Time set: ");
    Serial.print(isotime);
    Serial.print(", ");
    Serial.println(time_us);
  #else
    timer = timerBegin(1000);
  #endif

  // Connect to MQTT
  #ifdef USE_MQTT
    Serial.println("Connecting to MQTT...");
    connectToMqtt();
    send_log("Hello! I have connected successfully.");
  #endif
  
  // Set up I2C communication
  // ...with the input serialisation stack
  setup_pir();
  // ...with the environmental sensors
  #ifdef DO_ENV
    setup_env();
    setup_env_pir();
  #endif

  // Create tasks that will...
  // ...poll the input serialisation stack for cage sensor array data
  task_create_response = xTaskCreate(TaskPollPir,
                                     "TPir",  // A name just for humans
                                     4096,
                                     NULL,
                                     1,  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                                     &handlePollPir);
  if (task_create_response != pdPASS) {
    send_log("ERROR while creating TaskExperimenter: ");
    send_log(task_create_response);
  }

  // ...poll the environmental sensors
  #ifdef DO_ENV
    envpir_semaphore = xSemaphoreCreateMutex();
    if (envpir_semaphore == NULL) {
      send_log("ERROR while creating envpir_mutex");
    }
    task_create_response = xTaskCreate(TaskPollEnv,
                                       "TEnv",  // A name just for humans
                                       4096,    // 2022-12-08 stack: 8192 shwm: 7412
                                       NULL,
                                       1,  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                                       &handlePollEnv);
    if (task_create_response != pdPASS) {
      send_log("ERROR while creating TaskExperimenter: ");
      send_log(task_create_response);
    }
  #endif

  // ...handle HTTP requests for OTA firmware updates
  #ifdef USE_WIFI
    Serial.println("Starting TaskHttpHandler");
    task_create_response = xTaskCreate(TaskHttpHandler,
                                      "THttp",  // A name just for humans
                                      4096,
                                      NULL,
                                      1,  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                                      &handleHttpHandler);
    if (task_create_response != pdPASS) {
      Serial.println("ERROR while creating TaskHttpHandler: ");
      Serial.println(task_create_response);
    }
  #endif
}

void loop() {
  // all repetitive tasks are handled by timers.
}

void send_log(const String text) {
  size_t msg_length = 37+text.length()+1;
  char msg[msg_length];
  
  #ifdef USE_WIFI
    updateTime(isotime, &time_us);
    snprintf(msg, sizeof(msg), "START %s.%06lu:%s END", isotime, time_us, text.c_str());  
  #else
    uint64_t time_ms = timerReadMilis(timer);
    snprintf(msg, sizeof(msg), "START %llu:%s END", time_ms, text.c_str());
  #endif

  Serial.print(TOPIC_STATUS);
  Serial.print(":");
  Serial.println(msg);
  #ifdef USE_MQTT
    mqttClient.publish(TOPIC_STATUS, 1, false, msg);
  #endif
}

void send_log(const int64_t number) {
  char msg[50];
  
  #ifdef USE_WIFI
    updateTime(isotime, &time_us);
    snprintf(msg, sizeof(msg), "START %s.%06lu:%lld END", isotime, time_us, number);  
  #else
    uint64_t time_ms = timerReadMilis(timer);
    snprintf(msg, sizeof(msg), "START %llu:%lld END", time_ms, number);
  #endif

  Serial.print(TOPIC_STATUS);
  Serial.print(":");
  Serial.println(msg);
  #ifdef USE_MQTT
    mqttClient.publish(TOPIC_STATUS, 1, false, msg);
  #endif
}

//void utobs(uint16_t number, char *binstring, bool reverse=0) {
void utobs(uint16_t number, char *binstring) {
  //if (!reverse) {
  int8_t i = 0;
  for (uint16_t mask = 0b1000000000000000; mask > 0; mask >>= 1) {
    binstring[i] = (number & mask) ? '1' : '0';
    i++;
  }
//  } else {
//    int8_t i = 15;
//    for (uint16_t mask = 0b1000000000000000; mask > 0; mask >>= 1) {
//      binstring[i] = (number & mask) ? '1' : '0';
//      i--;
//    }
//  }
  binstring[16] = '\0';
}

#ifdef USE_WIFI
  int updateTime(char *iso_str, uint32_t *micro_int) {
    struct tm timeinfo;
    struct timeval tv;
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
      return 1;
    }
    if (gettimeofday(&tv, NULL) != 0) {
      Serial.println("Failed to obtain time (us)");
      return 1;
    }
    strftime(iso_str, 20, "%Y-%m-%d %H:%M:%S", &timeinfo);
    *micro_int = tv.tv_usec;
    return 0;
  }
#endif

#ifdef USE_WIFI
  void connectToWifi() {
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    for (int i=0; i<600; i++) {
      if (WiFi.status() == WL_CONNECTED) break;
      delay(100);
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Failed to connect. Restarting...");
      ESP.restart();
    } else {
      Serial.println("Connected.");
    }
  }
#endif

#ifdef USE_WIFI
  void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      #ifdef USE_MQTT
        connectToMqtt();
      #endif
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      #ifdef USE_MQTT
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      #endif
      xTimerStart(wifiReconnectTimer, 0);
      break;
    }
  }
#endif

#ifdef USE_MQTT
  void connectToMqtt() {
    Serial.println("Connecting to MQTT...");
    mqttClient.connect();
  }
#endif

#ifdef USE_MQTT
  void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.println("Disconnected from MQTT.");

    if (WiFi.isConnected()) {
      xTimerStart(mqttReconnectTimer, 0);
    }
  }
#endif

#ifdef USE_MQTT
  void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
    Serial.println("Subscribe acknowledged.");
    Serial.print("  packetId: ");
    Serial.println(packetId);
    Serial.print("  qos: ");
    Serial.println(qos);
  }
#endif

#ifdef DO_ENV
  void setup_env() {
    envI2C.setPins(ENV_I2C_SDA, ENV_I2C_SCL);
    envI2C.setTimeout(200);
    envI2C.begin();
    LuxSensor.powerOn();
    LuxSensor.setContHighRes();
    envI2C.beginTransmission(TEMPHUM_ADDR);
    envI2C.write(0x02);
    envI2C.write(0x10);
    envI2C.endTransmission();
    // pinMode(PIR_PIN, INPUT);

    delay(1000);
  }
#endif

#ifdef DO_ENV
  void hdc_trigger() {
    envI2C.beginTransmission(TEMPHUM_ADDR);
    envI2C.write(0x00);
    envI2C.endTransmission();
  }
#endif

#ifdef DO_ENV
  void setup_env_pir() {
    pinMode(PIR_PIN, INPUT_PULLUP);
  }
#endif

void setup_pir() {
  #ifndef DEBUG_MCP
    pirI2C.setPins(MCP_I2C_SDA, MCP_I2C_SCL);
    pirI2C.begin();
    for (uint8_t i = 0; i < MCP_NO; i++) {
      write_mcp_reg(mcps[i], MCP_IODIRA, 0xff);
      write_mcp_reg(mcps[i], MCP_IODIRB, 0xff);
      write_mcp_reg(mcps[i], MCP_GPPUA, 0xff);
      write_mcp_reg(mcps[i], MCP_GPPUB, 0xff);
    }
  #endif
}

void write_mcp_reg(uint8_t addr, uint8_t reg, uint8_t val) {
  #ifndef DEBUG_MCP
    pirI2C.beginTransmission((uint8_t)addr);
    pirI2C.write(reg);
    pirI2C.write(val);
    pirI2C.endTransmission();
  #endif
}

uint16_t read_mcp(uint8_t addr) {
  #ifdef DEBUG_MCP
    return 0b1101111010101101;
  #else
    uint16_t reading = 0;
    pirI2C.beginTransmission((uint8_t)addr);
    pirI2C.write(MCP_GPIOA);
    pirI2C.endTransmission();
    delayMicroseconds(50);
    pirI2C.requestFrom((uint8_t)addr, (uint8_t)1);
    reading = pirI2C.read();
    reading <<= 8;
    
    pirI2C.beginTransmission((uint8_t)addr);
    pirI2C.write(MCP_GPIOB);
    pirI2C.endTransmission();
    delayMicroseconds(50);
    pirI2C.requestFrom((uint8_t)addr, (uint8_t)1);
    reading |= pirI2C.read();
    
    return reading;
  #endif
}

#ifdef USE_WIFI
void handleRoot() {
  const char* root = "<html><body>Hello, I am a MIROSLAV.<br>I am alive, and my name is <b>" DEVICE_NAME "</b>.<br><br>"
                     "<a href='/update'>You can update my firmware here.</a></body></html>";
  httpServer.send(200, "text/html", root);
}
#endif

// FreeRTOS tasks:
// - TaskHttpHandler: handles HTTP requests for OTA firmware updates
// - TaskPollPir: polls the input serialisation stack for cage sensor array data
// - TaskPollEnv: polls the environmental sensors
// - TaskPollEnvPir: polls the standalone habitat PIR for room entrances

#ifdef USE_WIFI
  void TaskHttpHandler(void *pvParameters) {
    while (1) {
      httpServer.handleClient();
      delay(100);
    }
  }
#endif

void TaskPollPir(void *pvParameters) {
  // Maximum message size should be
  //  - 6 for "START "
  //  - 26+1 for the timestamp and a comma
  //  - 8*(16+1) for up to 8 MCP readings followed by commas
  //  - 4 for " END"
  //  - 1 for the null terminator
  // Totalling 174 characters. We round it off to 200.
  const uint32_t message_length = 200;
  //const int message_length = 37+MCP_NO*17+1;
  TickType_t last_loop_time;
  BaseType_t was_delayed;
  while (1) {
    last_loop_time = xTaskGetTickCount();
    uint16_t mcp_readings[MCP_NO] = {0};
    char c_reading[17];
    char payload[message_length];
    // String payload;
    for (uint8_t i = 0; i < MCP_NO; i++) {
      mcp_readings[i] = read_mcp(mcps[i]);
    }
    
    // utobs(u_reading_left, c_reading_left);
    // utobs(u_reading_right, c_reading_right);

    uint32_t offset = 0;
    #ifdef USE_WIFI
      updateTime(isotime, &time_us);
      offset += snprintf(payload, sizeof(payload), "START %s.%06lu,", isotime, time_us);
    #else
      uint64_t time_ms = timerReadMilis(timer);
      offset += snprintf(payload, sizeof(payload), "START %llu,", time_ms);
    #endif

    for (uint8_t i = 0; i < MCP_NO; i++) {
        utobs(mcp_readings[i], c_reading);
        offset += snprintf(payload+offset, sizeof(payload)-offset, "%s,", c_reading);
    }

    snprintf(payload+offset, sizeof(payload)-offset, " END");

    // payload = String("START ")+String(isotime)+String(".")+String(time_us)+String(",");
    // for (uint8_t i = 0; i < MCP_NO; i++) {
    //   utobs(mcp_readings[i], c_reading);
    //   payload += String(c_reading)+String(",");
    // }
    // payload += " END";
    #ifdef USE_MQTT
      mqttClient.publish(TOPIC_PIR, 1, true, payload);
    #endif
    #ifdef MSG_SERIAL
      Serial.print(TOPIC_PIR);
      Serial.print(":");
      Serial.println(payload);
    #endif
    was_delayed = xTaskDelayUntil(&last_loop_time, pdMS_TO_TICKS(POLLRATE_PIR));
    if (was_delayed == pdFALSE) {
      send_log("TaskPollPir missed its deadline!");
    }
  }
}

#ifdef DO_ENV
  void TaskPollEnv(void *pvParameters) {
    BaseType_t envpir_task_create_response = xTaskCreate(TaskPollEnvPir,
                                                  "TEnP",  // A name just for humans
                                                  4096,
                                                  NULL,
                                                  1,  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                                                  &handlePollEnvPir);
    if (envpir_task_create_response != pdPASS) {
      send_log("ERROR while creating TaskPollEnvPir: ");
      send_log(envpir_task_create_response);
    }

    // see explanation below as to why this is set to 2
    const uint32_t envpir_avg_shift = 2;
    uint32_t temp_raw;
    uint32_t hum_raw;
    float temp;
    float hum;
    uint32_t envpir_readings;
    uint32_t envpir_n_readings;
    uint32_t envpir_n_shifted;
    uint32_t envpir_average;
    uint32_t room_pir;
    TickType_t last_loop_time;
    BaseType_t was_delayed;
    while (1) {
      last_loop_time = xTaskGetTickCount();
      hdc_trigger();
      delay(100);
      envI2C.requestFrom(TEMPHUM_ADDR, 4);
      temp_raw = envI2C.read();
      temp_raw <<= 8;
      temp_raw |= envI2C.read();
      hum_raw = envI2C.read();
      hum_raw <<= 8;
      hum_raw |= envI2C.read();

      temp = ((float)temp_raw)*165/65536 - 40;
      hum = ((float)hum_raw)*100/65536;

      envpir_readings = 0;
      envpir_n_readings = 0;
      if (xSemaphoreTake(envpir_semaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        envpir_readings = global_envpir_readings;
        envpir_n_readings = global_envpir_n_readings;
        global_envpir_readings = 0;
        global_envpir_n_readings = 0;
        xSemaphoreGive(envpir_semaphore);
      }
      // this will make room_pir equal 1 if the sensor has been active for more than 25% of the time
      // envpir_avg_shift = 0 -> room_pir = 1 if sensor active for no less than 100% of the time
      // envpir_avg_shift = 1 -> room_pir = 1 if sensor active for more than 50% of the time
      // envpir_avg_shift = 2 -> room_pir = 1 if sensor active for more than 25% of the time
      // ...and so on. 
      // Serial.printf("envpir_readings: %u, envpir_n_readings: %u\n", envpir_readings, envpir_n_readings);
      envpir_n_shifted = envpir_n_readings >> envpir_avg_shift;
      if (envpir_n_shifted == 0) envpir_n_shifted = 1;
      envpir_average = envpir_readings/envpir_n_shifted;
      uint32_t room_pir;
      if (envpir_average >= 2) {
        room_pir = 1;
      } else {
        room_pir = 0;
      }

      float lux = LuxSensor.getLux();
      
      char payload[100];
      #ifdef USE_WIFI
        updateTime(isotime, &time_us);
        snprintf(payload, sizeof(payload), "START %s.%06lu,PIR:%u,lx:%.2f,degC:%.2f,pctRH:%.2f%%, END",
                                           isotime, time_us, room_pir, lux, temp, hum);
      #else
        uint64_t time_ms = timerReadMilis(timer);
        snprintf(payload, sizeof(payload), "START %llu,PIR:%u,lx:%.2f,degC:%.2f,pctRH:%.2f%%, END",
                                           time_ms, room_pir, lux, temp, hum);
      #endif

      #ifdef USE_MQTT
        mqttClient.publish(TOPIC_ENV, 1, true, payload);
      #endif
      #ifdef MSG_SERIAL
        Serial.print(TOPIC_ENV);
        Serial.print(":");
        Serial.println(payload);
      #endif

      was_delayed = xTaskDelayUntil(&last_loop_time, pdMS_TO_TICKS(POLLRATE_ENV));
      if (was_delayed == pdFALSE) {
        send_log("TaskPollEnv missed its deadline!");
      }
    }
  }
#endif

#ifdef DO_ENV
  void TaskPollEnvPir(void *pvParameters) {
    TickType_t last_loop_time;
    BaseType_t was_delayed;
    uint32_t reading;
    while (1) {
      last_loop_time = xTaskGetTickCount();
      reading = digitalRead(PIR_PIN);
      if (xSemaphoreTake(envpir_semaphore, pdMS_TO_TICKS(60000)) == pdTRUE) {
        global_envpir_readings = global_envpir_readings + reading;
        global_envpir_n_readings = global_envpir_n_readings + 1;
        // Serial.printf("global_envpir_readings: %u, global_envpir_n_readings: %u\n", global_envpir_readings, global_envpir_n_readings);
        xSemaphoreGive(envpir_semaphore);
      }
      was_delayed = xTaskDelayUntil(&last_loop_time, pdMS_TO_TICKS(POLLRATE_PIR));
      if (was_delayed == pdFALSE) {
        send_log("TaskPollEnvPir missed its deadline!");
      }
    }
    
  }
#endif
