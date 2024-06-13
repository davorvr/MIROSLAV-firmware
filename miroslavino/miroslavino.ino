/* 
 * MIROSLAV Firmware v0.4.2
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

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <HTTPUpdateServer.h>
// #include <ArduinoOTA.h>

#include <Wire.h>

#include "time.h"
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
  #include "freertos/task.h"
}
#include <AsyncMqttClient.h>

#include "config.h"

#define MCP_I2C_SDA 10
#define MCP_I2C_SCL 11
#define ENV_I2C_SDA 35
#define ENV_I2C_SCL 34

#define MCP_L 0x20
#define MCP_R 0x24

#define MCP_ADDR_TOP 0x20+MCP_ID_TOP
#define MCP_ADDR_MID 0x20+MCP_ID_MID
#define MCP_ADDR_BOT 0x20+MCP_ID_BOT
#define MCP_IODIRA 0x00
#define MCP_IODIRB 0x01
#define MCP_GPPUA 0x0C
#define MCP_GPPUB 0x0D
// GPIOA0-7 -> PH0-7
#define MCP_GPIOA 0x12
// GPIOB0-7 -> PL7-0 !! notice the reversed order !!
#define MCP_GPIOB 0x13

#define TEMPHUM_ADDR 0x5C
#define LUX_ADDR 0x40

#define PIR_PIN 14

int mcps[] = {MCP_ADDR_TOP, MCP_ADDR_MID, MCP_ADDR_BOT};

#define MCP_NO sizeof(mcps)/sizeof(mcps[0])

TwoWire pirI2C = TwoWire(0);
#ifndef SKIP_ENV
  TwoWire envI2C = TwoWire(1);
#endif


AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
TaskHandle_t handleHttpHandler = NULL;

BaseType_t task_create_response;

TimerHandle_t pollPirTimer;
TaskHandle_t handlePollPir;
#ifndef SKIP_ENV
  TimerHandle_t pollEnvTimer;
  TaskHandle_t handlePollEnv;
#endif
#ifdef DO_ENV_PIR
  TimerHandle_t pollEnvPirTimer;
  TaskHandle_t handlePollEnvPir;
#endif
// TimerHandle_t handleOTATimer;

uint32_t time_us;
// careful. this needs to be precisely 20 - 19 for ISO time, 1 for \0
char isotime[20]; 

WebServer httpServer(80);
HTTPUpdateServer httpUpdater;

void send_log(const String text) {
  String msg = String("{\"message\":\"")+text+String("\"}");
  #ifdef LOG_TO_SERIAL
  Serial.println(msg);
  #endif
  #ifdef LOG_TO_MQTT
  mqttClient.publish(TOPIC_STATUS, 1, false, msg.c_str());
  #endif
}

void send_log(const int number) {
  String msg = String("{\"message\":")+String(number)+String("}");
  #ifdef LOG_TO_SERIAL
  Serial.println(msg);
  #endif
  #ifdef LOG_TO_MQTT
  mqttClient.publish(TOPIC_STATUS, 1, false, msg.c_str());
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

int updateTime(char *iso_str, uint32_t *micro_int)
{
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

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

//void onMqttConnect(bool sessionPresent) {
//  Serial.println("Connected to MQTT.");
//  Serial.print("Session present: ");
//  Serial.println(sessionPresent);
//  uint16_t packetIdSub = mqttClient.subscribe("test/lol", 2);
//  Serial.print("Subscribing at QoS 2, packetId: ");
//  Serial.println(packetIdSub);
//  mqttClient.publish("test/lol", 0, true, "test 1");
//  Serial.println("Publishing at QoS 0");
//  uint16_t packetIdPub1 = mqttClient.publish("test/lol", 1, true, "test 2");
//  Serial.print("Publishing at QoS 1, packetId: ");
//  Serial.println(packetIdPub1);
//  uint16_t packetIdPub2 = mqttClient.publish("test/lol", 2, true, "test 3");
//  Serial.print("Publishing at QoS 2, packetId: ");
//  Serial.println(packetIdPub2);
//}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void TaskHttpHandler(void *pvParameters) {
  while (1) {
    httpServer.handleClient();
    delay(100);
  }
}

void handleOTA() {
  httpServer.handleClient();
  // ArduinoOTA.handle();
  //xTimerStart(handleOTATimer, pdMS_TO_TICKS(3));
}

void write_mcp_reg(uint8_t addr, uint8_t reg, uint8_t val) {
  pirI2C.beginTransmission((uint8_t)addr);
  pirI2C.write(reg);
  pirI2C.write(val);
  pirI2C.endTransmission();
}

uint16_t read_mcp(uint8_t addr) {
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
}

//void write_env_reg(uint8_t addr, uint8_t reg, uint8_t val) {
//  envI2C.beginTransmission((uint8_t)addr);
//  envI2C.write(reg);
//  envI2C.write(val);
//  envI2C.endTransmission();
//}

void setup_pir() {
  pirI2C.begin(MCP_I2C_SDA, MCP_I2C_SCL);
  for (uint8_t i = 0; i < MCP_NO; i++) {
    write_mcp_reg(mcps[i], MCP_IODIRA, 0xff);
    write_mcp_reg(mcps[i], MCP_IODIRB, 0xff);
    write_mcp_reg(mcps[i], MCP_GPPUA, 0xff);
    write_mcp_reg(mcps[i], MCP_GPPUB, 0xff);
  }
}

//void setup_env() {
//  envI2C.begin(ENV_I2C_SDA, ENV_I2C_SCL);
//  write_env_reg(TEMPHUM_ADDR, 0x02, 0x10);
//  pinMode(PIR_PIN, INPUT);
//  adc1_config_width(ADC_WIDTH_BIT_11);   //Range 0-2047 
//  adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);  //ADC_ATTEN_DB_11 = 0-3,6V
//}

#ifdef DO_ENV_PIR
void setup_env_pir() {
  pinMode(PIR_PIN, INPUT_PULLUP);
}
#endif

//void hdc_trigger() {
//  envI2C.beginTransmission(TEMPHUM_ADDR);
//  envI2C.write(0x00);
//  envI2C.endTransmission();
//}

//void poll_env() {
//  int room_pir;
//  int reading;
//  float lux = 0;
//  uint16_t temp_raw;
//  float temp;
//  uint16_t hum_raw;
//  float hum;
//  
//  hdc_trigger();
//  delay(20);
//  envI2C.requestFrom(TEMPHUM_ADDR, 4);
//  temp_raw = envI2C.read();
//  temp_raw <<= 8;
//  temp_raw |= envI2C.read();
//  hum_raw = envI2C.read();
//  hum_raw <<= 8;
//  hum_raw |= envI2C.read();
//
//  temp = ((float)temp_raw)*165/65536 - 40;
//  hum = ((float)hum_raw)*100/65536;
//  
//  room_pir = digitalRead(PIR_PIN);
//  
//  for (uint8_t i=0; i<LUX_SAMPLES; i++) {
//    lux += adc1_get_raw(ADC1_CHANNEL_0);
//  }
//  lux /= LUX_SAMPLES;
//  
//  updateTime(isotime, &time_us);
//  String payload = String(isotime)+String(",")+String(time_us)+String(",")
//                  +String(room_pir)+String(",")
//                  +String(lux, 2)+String(",")
//                  +String(temp, 2)+String(",")
//                  +String(hum, 2);
//  Serial.println(payload);
//  mqttClient.publish(TOPIC_ENV, 1, true, payload.c_str());
//}

void TaskPollPir(void *pvParameters) {
  while (1) {
    uint16_t mcp_readings[MCP_NO] = {0};
    char c_reading[17];
    String payload;
    for (uint8_t i = 0; i < MCP_NO; i++) {
      mcp_readings[i] = read_mcp(mcps[i]);
    }
  //  uint16_t u_reading_left = read_mcp(MCP_L);
  //  uint16_t u_reading_right = read_mcp(MCP_R);
    
    
  //  utobs(u_reading_left, c_reading_left);
  //  utobs(u_reading_right, c_reading_right);
    updateTime(isotime, &time_us);
    payload = String("START ")+String(isotime)+String(".")+String(time_us)+String(",");
  //  mqttClient.publish(TOPIC_PIR, 1, true, payload.c_str());
  //  delay(10);
  //  Serial.print(payload);
    for (uint8_t i = 0; i < MCP_NO; i++) {
      utobs(mcp_readings[i], c_reading);
      payload += String(c_reading)+String(",");
  //    mqttClient.publish(TOPIC_PIR, 1, true, payload.c_str());
  //    Serial.print(payload);
  //    delay(10);
    }
    payload += " END";
    mqttClient.publish(TOPIC_PIR, 1, true, payload.c_str());
    // Serial.print(TOPIC_PIR);
    // Serial.print(" - ");
    // Serial.println(payload);
    delay(POLLRATE_PIR);
  }
}

#ifdef DO_ENV_PIR
void TaskPollEnvPir(void *pvParameters) {
  while (1) {
    bool reading;
    String payload;
    reading = digitalRead(PIR_PIN);
    updateTime(isotime, &time_us);
    payload = String("START ")+String(isotime)+String(".")+String(time_us)+String(",")
              +String(reading)+String(",")+String(" END");
    mqttClient.publish(TOPIC_ENV, 1, true, payload.c_str());
    // Serial.print(TOPIC_ENV);
    // Serial.print(" - ");
    // Serial.println(payload);
    delay(POLLRATE_ENV);
  }
}
#endif

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  if (!WiFi.config(LOCAL_IP, GATEWAY, SUBNET, DNS_PRIM, DNS_SECOND)) {
    Serial.println("STA Failed to configure");
  }

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  // handleOTATimer = xTimerCreate("OTATimer", pdMS_TO_TICKS(10), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(handleOTA));
  
  WiFi.onEvent(WiFiEvent);

  //mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
//  mqttClient.onSubscribe(onMqttSubscribe);
//  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setClientId(MQTT_CLIENTID);
  mqttClient.setCredentials(MQTT_USER, MQTT_PASS);

  connectToWifi();
  


  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // ArduinoOTA.setHostname(DEVICE_NAME);
  // ArduinoOTA.setPassword(OTA_PASS);
  // ArduinoOTA
  //   .onStart([]() {
  //     String type;
  //     if (ArduinoOTA.getCommand() == U_FLASH)
  //       type = "sketch";
  //     else // U_SPIFFS
  //       type = "filesystem";
  
  //     // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
  //     Serial.println("Start updating " + type);
  //   })
  //   .onEnd([]() {
  //     Serial.println("\nEnd");
  //   })
  //   .onProgress([](unsigned int progress, unsigned int total) {
  //     Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  //   })
  //   .onError([](ota_error_t error) {
  //     Serial.printf("Error[%u]: ", error);
  //     if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
  //     else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
  //     else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
  //     else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
  //     else if (error == OTA_END_ERROR) Serial.println("End Failed");
  //   });

  httpUpdater.setup(&httpServer);
  // httpServer.on("/", handleRoot);
  httpServer.begin();
  Serial.printf("HTTPUpdateServer is set up. Open http://%s/update in your browser\n", WiFi.localIP().toString());
  
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

  Serial.println("Connecting to MQTT...");
  connectToMqtt();
  
//  xTimerStart(handleOTATimer, pdMS_TO_TICKS(3));
  
  setup_pir();
  #ifndef SKIP_ENV
    setup_env();
  #endif

  // ArduinoOTA.begin();
  
  // xTimerStart(pollPirTimer, 0);
  // #ifndef SKIP_ENV
  //   xTimerStart(pollEnvTimer, 0);
  // #endif
  // #ifdef DO_ENV_PIR
  //   xTimerStart(pollEnvPirTimer, 0);
  // #endif
  // // xTimerStart(handleOTATimer, 0);

    // pollPirTimer = xTimerCreate("pirTimer", pdMS_TO_TICKS(POLLRATE_PIR), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(poll_pir));
  task_create_response = xTaskCreate(TaskPollPir,
                                    "TPir",  // A name just for humans
                                    4096,    // 2022-12-08 stack: 8192 shwm: 7412
                                    NULL,
                                    2,  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                                    &handlePollPir);
  if (task_create_response != pdPASS) {
    send_log("ERROR while creating TaskExperimenter: ");
    send_log(task_create_response);
  }
  #ifndef SKIP_ENV
    // pollEnvTimer = xTimerCreate("envTimer", pdMS_TO_TICKS(POLLRATE_ENV), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(poll_env));
    task_create_response = xTaskCreate(TaskPollEnv,
                                    "TEnv",  // A name just for humans
                                    4096,    // 2022-12-08 stack: 8192 shwm: 7412
                                    NULL,
                                    2,  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                                    &handlePollEnv);
    if (task_create_response != pdPASS) {
      send_log("ERROR while creating TaskExperimenter: ");
      send_log(task_create_response);
    }
  #endif
  #ifdef DO_ENV_PIR
    // pollEnvPirTimer = xTimerCreate("pirEnvTimer", pdMS_TO_TICKS(POLLRATE_ENV), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(poll_env_pir));
    task_create_response = xTaskCreate(TaskPollEnvPir,
                                    "TEnP",  // A name just for humans
                                    4096,    // 2022-12-08 stack: 8192 shwm: 7412
                                    NULL,
                                    2,  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                                    &handlePollEnvPir);
    if (task_create_response != pdPASS) {
      send_log("ERROR while creating TaskExperimenter: ");
      send_log(task_create_response);
    }
  #endif
  //pollEnvTimer = xTimerCreate("timeTimer", pdMS_TO_TICKS(POLLRATE_ENV), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(poll_and_send));

  Serial.println("Starting TaskHttpHandler");
  BaseType_t task_create_response = xTaskCreate(TaskHttpHandler,
                                    "THttp",  // A name just for humans
                                    4096,     // 2022-12-08 stack: 8192 shwm: 7460
                                    NULL,
                                    2,  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                                    &handleHttpHandler);
  if (task_create_response != pdPASS) {
    Serial.println("ERROR while creating TaskHttpHandler: ");
    Serial.println(task_create_response);
  }
}

void loop() {
  // all repetitive tasks are handled by timers.
}
