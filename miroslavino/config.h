#define DEVICE_ID 1
#define DEVICE_NAME "miroslav_rackM"
#define OTA_PASS "hesoyam"
#define MQTT_USER "sample_user"
#define MQTT_PASS "hesoyam"
// Static IP address that this MIROSLAV will use
//#define LOCAL_IP IPAddress(192, 168, 1, 10+DEVICE_ID)
#define LOCAL_IP IPAddress(192, 168, 1, 11)

#define WIFI_SSID "bhv0"
#define WIFI_PASS "zanlukpicard"

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

// if this is used, environmental monitoring is disabled
#define SKIP_ENV
// if this is used, environment PIR is enabled regardless of SKIP_ENV
// #define DO_ENV_PIR

// ID: "0b", followed by ADDR1, ADDR2, ADDR3:
#define MCP_ID_TOP 0b100
#define MCP_ID_MID 0b010
#define MCP_ID_BOT 0b001

#define NTP_SERVER "pool.ntp.org"
//#define NTP_SERVER "10.10.0.2"
#define NTP_OFFSET_GMT 3600 // in seconds
#define NTP_OFFSET_DST 3600 // in seconds

// MQTT topic names, make sure this matches Record-a-SLAV
#define TOPIC_PIR "miroslav/pir/" DEVICE_NAME // this resolves to "miroslav/pir/miroslav_rackM"
#define TOPIC_ENV "miroslav/env/" DEVICE_NAME
