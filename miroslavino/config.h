// Whether to use Wi-Fi at all. Disabling this overrides USE_MQTT and auto-enables MSG_SERIAL
// so messages will be sent via USB connection only. Use with caution, as timestamps will be
// sent as seconds since boot, and not as actual clock time. This is because MIROSLAV uses
// Wi-Fi to get the current time from an NTP server. It is highly recommended to use Wi-Fi.
#define USE_WIFI

// Whether to use MQTT, i.e. send readings via Wi-Fi. Disabling this auto-enables MSG_SERIAL.
// It is highly recommended to use MQTT, as it provides a more reliable way of sending data.
#define USE_MQTT

// If this is enabled, all sensor readings will be printed via USB.
//#define MSG_SERIAL

// If this is used, environmental monitoring is enabled.
#define DO_ENV

//#define DEVICE_ID 1
#define DEVICE_NAME "rack_M"
// Password for uploading new firmware via a web browser
#define OTA_PASS "hesoyam"
// MQTT credentials - disregard if your MQTT allows anonymous connections
#define MQTT_USER "mqtt"
#define MQTT_PASS "hesoyam"
// Static IP address that this MIROSLAV will use
//#define LOCAL_IP IPAddress(192, 168, 1, 10+DEVICE_ID)
#define LOCAL_IP IPAddress(192, 168, 1, 11)

#define WIFI_SSID "wifi_name"
#define WIFI_PASS "wifi_password"

#define GATEWAY IPAddress(192, 168, 1, 1)
#define SUBNET IPAddress(255, 255, 255, 0)
#define DNS_PRIM IPAddress(192, 168, 1, 1)
#define DNS_SECOND IPAddress(8, 8, 8, 8)

// IP address of the MQTT broker device
#define MQTT_HOST IPAddress(192, 168, 1, 2)
#define MQTT_PORT 1883
#define MQTT_CLIENTID DEVICE_NAME

#define POLLRATE_PIR 100
#define POLLRATE_ENV 1000

/* IMPORTANT!
    You can have up to 8 input boards on the input serialisation stack, but each must have
    its own unique ID, i.e. a unique combination of 0s and 1s on the three ADDR solder jumpers.
    Each ID, as defined here, must start with "0b", followed by the values of ADDR1, ADDR2, and ADDR3.
    You must define MCPs in order. MCP_ID_TOP must be defined, and if you need three MCPs in total,
    you need to define MCP_ID_2, and MCP_ID_3. Each will correspond to the following board in the stack,
    from top to bottom, so that the topmost board is MCP_ID_TOP, the second board is MCP_ID_2,
    and the third (i.e. bottom, in this example) board is MCP_ID_3. */
#define MCP_ID_TOP 0b100
#define MCP_ID_2 0b010
#define MCP_ID_3 0b001
//#define MCP_ID_4 0b000
//#define MCP_ID_5 0b011
//#define MCP_ID_6 0b101
//#define MCP_ID_7 0b110
//#define MCP_ID_8 0b111

#define NTP_SERVER "pool.ntp.org"
//#define NTP_SERVER "10.10.0.2"
#define NTP_OFFSET_GMT 3600 // in seconds
#define NTP_OFFSET_DST 3600 // in seconds

// MQTT topic names, make sure this matches Record-a-SLAV
#define TOPIC_PIR "miroslav/pir/" DEVICE_NAME // this resolves to "miroslav/pir/rack_M"
#define TOPIC_ENV "miroslav/env/" DEVICE_NAME
#define TOPIC_STATUS "miroslav/status/" DEVICE_NAME
