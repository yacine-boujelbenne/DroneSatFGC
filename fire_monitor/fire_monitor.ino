/*
  ESP32 Fire Monitor
  Captures an ArduCAM JPEG and streams it with available telemetry in one
  TLS-verified multipart/form-data request. No SD card is required.

  Pin map (this board):
    ArduCAM SPI : CS=5, SCK=18, MISO=19, MOSI=23 (VSPI defaults)
    I2C bus     : SDA=21, SCL=22 — shared by ArduCAM, BME280, HM3301 dust
    GPS UART    : RX=16, TX=17 (UART2; keeps USB debug UART 1/3 free)
    SIM         : disabled until it has a dedicated UART/power supply
    MQ gas      : GPIO33 analog
*/
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include <time.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Seeed_HM330X.h"

#define WIFI_SSID        "Xiaomi_11T"
#define WIFI_PASSWORD    "babushkara"
#define RENDER_HOST      "dronesatfgc.onrender.com"
#define SERVER_PORT      443
#define INGEST_PATH      "/ingest"

// GPIO 1/3 are reserved for Serial Monitor at 115200. The former GPS/SIM
// mapping on those pins reconfigured the debug UART to 9600 and caused the
// square/garbled output. Rewire GPS TX->16 and GPS RX->17.
const int CAM_CS = 5;
const int I2C_SDA = 21;
const int I2C_SCL = 22;
const int GPS_RX = 16;      // ESP32 RX  <- GPS TX
const int GPS_TX = 17;      // ESP32 TX  -> GPS RX
const int GAS_ADC_PIN = 33;

const bool ENABLE_CAMERA = true;
const bool ENABLE_BME280 = true;
const bool ENABLE_DUST = true;
const bool ENABLE_GAS = true;
const bool ENABLE_GPS = true;
const bool ENABLE_SIM = false;
const bool AUTO_SYNC_ON_BOOT = true;

const unsigned long CLOUD_SYNC_INTERVAL_MS = 15000;
const unsigned long CAPTURE_TIMEOUT_MS = 5000;
const unsigned long RESPONSE_TIMEOUT_MS = 130000;
const unsigned long GPS_STALE_MS = 30000;
const unsigned long SENSOR_RETRY_INTERVAL_MS = 60000;
const size_t IMAGE_CHUNK_SIZE = 1024;
const uint8_t MAX_UPLOAD_ATTEMPTS = 2;
const char MULTIPART_BOUNDARY[] = "----DroneSatESP32Boundary7MA4YWxk";

#if !(defined(OV5640_MINI_5MP_PLUS) || defined(OV5642_MINI_5MP_PLUS))
#error Enable OV5640_MINI_5MP_PLUS or OV5642_MINI_5MP_PLUS in ArduCAM memorysaver.h
#endif

#if defined(OV5640_MINI_5MP_PLUS)
ArduCAM myCAM(OV5640, CAM_CS);
#else
ArduCAM myCAM(OV5642, CAM_CS);
#endif

Adafruit_BME280 bme;
HM330X dustSensor;
HardwareSerial GpsSerial(2);

// Render currently uses either Let's Encrypt or Google Trust Services.
// These are the ISRG Root X1, GTS Root R1, and GTS Root R4 trust anchors.
const char ROOT_CA[] PROGMEM = R"PEM(-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIFWjCCA0KgAwIBAgIQbkepxUtHDA3sM9CJuRz04TANBgkqhkiG9w0BAQwFADBH
MQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExM
QzEUMBIGA1UEAxMLR1RTIFJvb3QgUjEwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIy
MDAwMDAwWjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNl
cnZpY2VzIExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjEwggIiMA0GCSqGSIb3DQEB
AQUAA4ICDwAwggIKAoICAQC2EQKLHuOhd5s73L+UPreVp0A8of2C+X0yBoJx9vaM
f/vo27xqLpeXo4xL+Sv2sfnOhB2x+cWX3u+58qPpvBKJXqeqUqv4IyfLpLGcY9vX
mX7wCl7raKb0xlpHDU0QM+NOsROjyBhsS+z8CZDfnWQpJSMHobTSPS5g4M/SCYe7
zUjwTcLCeoiKu7rPWRnWr4+wB7CeMfGCwcDfLqZtbBkOtdh+JhpFAz2weaSUKK0P
fyblqAj+lug8aJRT7oM6iCsVlgmy4HqMLnXWnOunVmSPlk9orj2XwoSPwLxAwAtc
vfaHszVsrBhQf4TgTM2S0yDpM7xSma8ytSmzJSq0SPly4cpk9+aCEI3oncKKiPo4
Zor8Y/kB+Xj9e1x3+naH+uzfsQ55lVe0vSbv1gHR6xYKu44LtcXFilWr06zqkUsp
zBmkMiVOKvFlRNACzqrOSbTqn3yDsEB750Orp2yjj32JgfpMpf/VjsPOS+C12LOO
Rc92wO1AK/1TD7Cn1TsNsYqiA94xrcx36m97PtbfkSIS5r762DL8EGMUUXLeXdYW
k70paDPvOmbsB4om3xPXV2V4J95eSRQAogB/mqghtqmxlbCluQ0WEdrHbEg8QOB+
DVrNVjzRlwW5y0vtOUucxD/SVRNuJLDWcfr0wbrM7Rv1/oFB2ACYPTrIrnqYNxgF
lQIDAQABo0IwQDAOBgNVHQ8BAf8EBAMCAQYwDwYDVR0TAQH/BAUwAwEB/zAdBgNV
HQ4EFgQU5K8rJnEaK0gnhS9SZizv8IkTcT4wDQYJKoZIhvcNAQEMBQADggIBADiW
Cu49tJYeX++dnAsznyvgyv3SjgofQXSlfKqE1OXyHuY3UjKcC9FhHb8owbZEKTV1
d5iyfNm9dKyKaOOpMQkpAWBz40d8U6iQSifvS9efk+eCNs6aaAyC58/UEBZvXw6Z
XPYfcX3v73svfuo21pdwCxXu11xWajOl40k4DLh9+42FpLFZXvRq4d2h9mREruZR
gyFmxhE+885H7pwoHyXa/6xmld01D1zvICxi/ZG6qcz8WpyTgYMpl0p8WnK0OdC3
d8t5/Wk6kjftbjhlRn7pYL15iJdfOBL07q9bgsiG1eGZbYwE8na6SfZu6W0eX6Dv
J4J2QPim01hcDyxC2kLGe4g0x8HYRZvBPsVhHdljUEn2NIVq4BjFbkerQUIpm/Zg
DdIx02OYI5NaAIFItO/Nis3Jz5nu2Z6qNuFoS3FJFDYoOj0dzpqPJeaAcWErtXvM
+SUWgeExX6GjfhaknBZqlxi9dnKlC54dNuYvoS++cJEPqOba+MSSQGwlfnuzCdyy
F62ARPBopY+Udf90WuioAnwMCeKpSwughQtiue+hMZL77/ZRBIls6Kl0obsXs7X9
SQ98POyDGCBDTtWTurQ0sR8WNh8M5mQ5Fkzc4P4dyKliPUDqysU0ArSuiYgzNdws
E3PYJ/HQcu51OyLemGhmW/HGY0dVHLqlCFF1pkgl
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIICCjCCAZGgAwIBAgIQbkepyIuUtui7OyrYorLBmTAKBggqhkjOPQQDAzBHMQsw
CQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEU
MBIGA1UEAxMLR1RTIFJvb3QgUjQwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAw
MDAwWjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZp
Y2VzIExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjQwdjAQBgcqhkjOPQIBBgUrgQQA
IgNiAATzdHOnaItgrkO4NcWBMHtLSZ37wWHO5t5GvWvVYRg1rkDdc/eJkTBa6zzu
hXyiQHY7qca4R9gq55KRanPpsXI5nymfopjTX15YhmUPoYRlBtHci8nHc8iMai/l
xKvRHYqjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNVHRMBAf8EBTADAQH/MB0GA1Ud
DgQWBBSATNbrdP9JNqPV2Py1PsVq8JQdjDAKBggqhkjOPQQDAwNnADBkAjBqUFJ0
CMRw3J5QdCHojXohw0+WbhXRIjVhLfoIN+4Zba3bssx9BzT1YBkstTTZbyACMANx
sbqjYAuG7ZoIapVon+Kz4ZNkfF6Tpt95LY2F45TPI11xzPKwTdb+mciUqXWi4w==
-----END CERTIFICATE-----)PEM";

float latestTemperature = NAN;
float latestHumidity = NAN;
float latestPressure = NAN;
float latestPm1 = NAN;
float latestPm25 = NAN;
float latestPm10 = NAN;
float latestGasRaw = NAN;
float latestLatitude = NAN;
float latestLongitude = NAN;
float latestSimSignal = NAN;

bool bmeReady = false;
bool dustReady = false;
bool cameraReady = false;
bool gasReady = false;
bool gpsReady = false;
bool gpsFixValid = false;
bool autoSyncEnabled = AUTO_SYNC_ON_BOOT;
unsigned long lastGpsFixMs = 0;
unsigned long lastCloudSyncTime = 0;
unsigned long lastSensorRetryMs = 0;
uint32_t readingSequence = 0;
uint8_t bmeFailStreak = 0;
uint8_t dustFailStreak = 0;
const uint8_t SENSOR_FAIL_LIMIT = 3;
String gpsLine;
String commandLine;
String cycleLog;

void noteBmeFailure();
void noteDustFailure();
bool initializeCamera();
void initializeComponents();
void retryUnavailableSensors();
void recoverI2cBus();
bool probeI2cAddress(uint8_t address);
bool initBme280();
bool initDustSensor();
bool readBme280();
bool readDustSensor();
void connectWiFi();
void setClock();
void pollGps();
bool parseGpsSentence(const String &sentence);
float nmeaToDegrees(const String &raw, const char hemisphere);
void readSensors();
void handleSerialCommands();
void executeCommand(String command);
void printStatus();
void logEvent(const char *level, const char *component, const String &message);
String componentStatusJson();
String telemetryLogLine();
bool captureAndUpload();
bool sendMultipart(WiFiClientSecure &client, const String &readingId,
                   bool includeImage);
bool streamJpegPart(WiFiClientSecure &client);
bool writeAll(WiFiClientSecure &client, const uint8_t *data, size_t length);
bool writeChunk(WiFiClientSecure &client, const uint8_t *data, size_t length);
bool writeChunk(WiFiClientSecure &client, const String &data);
bool writeTextPart(WiFiClientSecure &client, const char *name, const String &value);
bool writeFloatPart(WiFiClientSecure &client, const char *name, float value);
String makeReadingId();
int readHttpStatus(WiFiClientSecure &client);

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.setDebugOutput(false);
  Serial.println();
  Serial.println(F("=== DroneSat ESP32 diagnostic firmware ==="));
  Serial.println(F("Serial Monitor: 115200 baud, newline ending"));

  pinMode(CAM_CS, OUTPUT);
  digitalWrite(CAM_CS, HIGH);
  pinMode(GAS_ADC_PIN, INPUT);
  analogReadResolution(12);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);  // 100 kHz is safer when ArduCAM + BME280 + HM3301 share the bus
  Wire.setTimeOut(50);
  SPI.begin();

  if (ENABLE_GPS) {
    GpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    logEvent("INFO", "GPS", "UART2 at 9600 on RX16/TX17; no_fix until outdoor sky lock");
  } else {
    logEvent("SKIP", "GPS", "disabled in configuration");
  }
  logEvent("SKIP", "SIM", "disabled; no UART probing or SMS operations");

  initializeComponents();
  connectWiFi();
  setClock();
  printStatus();
  Serial.println(F("Commands: help, status, test wifi|camera|bme|dust|gas|gps, sync, auto on|off"));
  Serial.println(F("GPS tip: no_fix is normal indoors; leave antenna skyward until RMC status becomes A"));
  logEvent("INFO", "SYSTEM", "startup complete; unavailable components will be skipped");
}

bool initializeCamera() {
  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);

  uint8_t temp;
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  temp = myCAM.read_reg(ARDUCHIP_TEST1);
  if (temp != 0x55) {
    Serial.println(F("ArduCAM SPI test failed."));
    return false;
  }

  uint8_t vid, pid;
#if defined (OV5640_MINI_5MP_PLUS)
  myCAM.rdSensorReg16_8(OV5640_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg16_8(OV5640_CHIPID_LOW, &pid);
  if (vid != 0x56 || pid != 0x40) {
    Serial.printf("OV5640 not found (VID=%02X PID=%02X).\n", vid, pid);
    return false;
  }
#else
  myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
  if (vid != 0x56 || pid != 0x42) {
    Serial.printf("OV5642 not found (VID=%02X PID=%02X).\n", vid, pid);
    return false;
  }
#endif

  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
#if defined (OV5640_MINI_5MP_PLUS)
  myCAM.OV5640_set_JPEG_size(OV5640_640x480);
#else
  myCAM.OV5642_set_JPEG_size(OV5642_640x480);
#endif
  delay(1000);
  myCAM.clear_fifo_flag();
  Serial.println(F("ArduCAM initialized in JPEG 640x480 mode."));
  return true;
}

void loop() {
  pollGps();
  handleSerialCommands();
  retryUnavailableSensors();

  unsigned long now = millis();
  if (autoSyncEnabled && (lastCloudSyncTime == 0 ||
      now - lastCloudSyncTime >= CLOUD_SYNC_INTERVAL_MS)) {
    lastCloudSyncTime = now;
    readSensors();
    captureAndUpload();
  }
  delay(10);
}

void recoverI2cBus() {
  // Clear a hung shared I2C bus (common when ArduCAM, BME280, and HM3301 coexist).
  Wire.end();
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, OUTPUT);
  for (uint8_t i = 0; i < 9; ++i) {
    digitalWrite(I2C_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(5);
  }
  pinMode(I2C_SDA, OUTPUT);
  digitalWrite(I2C_SDA, HIGH);
  digitalWrite(I2C_SCL, HIGH);
  delayMicroseconds(5);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  delay(20);
}

bool probeI2cAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool initBme280() {
  recoverI2cBus();
  if (!(probeI2cAddress(0x76) || probeI2cAddress(0x77))) return false;
  bool ok = bme.begin(0x76) || bme.begin(0x77);
  if (ok) bmeFailStreak = 0;
  return ok;
}

bool initDustSensor() {
  recoverI2cBus();
  // HM3301 default I2C address is 0x40.
  if (!probeI2cAddress(0x40)) return false;
  bool ok = (dustSensor.init() == 0);
  if (ok) dustFailStreak = 0;
  return ok;
}

bool readBme280() {
  recoverI2cBus();
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure() / 100.0F;
  if (!(isfinite(t) && isfinite(h) && isfinite(p))) return false;
  // Reject clearly bogus pressure as a hung-bus symptom.
  if (p < 300.0f || p > 1100.0f) return false;
  latestTemperature = t;
  latestHumidity = h;
  latestPressure = p;
  bmeFailStreak = 0;
  return true;
}

bool readDustSensor() {
  recoverI2cBus();
  uint8_t buf[29];
  if (dustSensor.read_sensor_value(buf, 29) != 0) return false;
  // Atmospheric particulate concentrations (µg/m³).
  latestPm1 = (buf[10] << 8) | buf[11];
  latestPm25 = (buf[12] << 8) | buf[13];
  latestPm10 = (buf[14] << 8) | buf[15];
  dustFailStreak = 0;
  return true;
}

void initializeComponents() {
  cameraReady = false;
  if (ENABLE_CAMERA) {
    cameraReady = initializeCamera();
    logEvent(cameraReady ? "OK" : "WARN", "CAMERA",
             cameraReady ? "ArduCAM ready" : "unavailable; telemetry-only sync will continue");
  } else {
    logEvent("SKIP", "CAMERA", "disabled in configuration");
  }

  // Reclaim I2C after ArduCAM sensor register setup before probing env sensors.
  recoverI2cBus();

  bmeReady = ENABLE_BME280 && initBme280();
  if (bmeReady) {
    logEvent("OK", "BME280", "ready on I2C");
  } else {
    logEvent(ENABLE_BME280 ? "WARN" : "SKIP", "BME280",
             ENABLE_BME280 ? "not found at 0x76/0x77; will retry" : "disabled");
  }

  dustReady = ENABLE_DUST && initDustSensor();
  if (dustReady) {
    logEvent("OK", "DUST", "HM3301 ready on I2C");
  } else {
    logEvent(ENABLE_DUST ? "WARN" : "SKIP", "DUST",
             ENABLE_DUST ? "HM3301 unavailable; will retry" : "disabled");
  }

  gasReady = ENABLE_GAS;
  logEvent(gasReady ? "OK" : "SKIP", "GAS",
           gasReady ? "ADC enabled on GPIO33" : "disabled");

  gpsReady = ENABLE_GPS;
  lastSensorRetryMs = millis();
}

void retryUnavailableSensors() {
  if (millis() - lastSensorRetryMs < SENSOR_RETRY_INTERVAL_MS) return;
  lastSensorRetryMs = millis();

  if (ENABLE_CAMERA && !cameraReady) {
    cameraReady = initializeCamera();
    recoverI2cBus();
    logEvent(cameraReady ? "OK" : "WARN", "CAMERA",
             cameraReady ? "recovered" : "retry failed");
  }
  if (ENABLE_BME280 && !bmeReady) {
    bmeReady = initBme280();
    logEvent(bmeReady ? "OK" : "WARN", "BME280",
             bmeReady ? "recovered" : "retry failed");
  }
  if (ENABLE_DUST && !dustReady) {
    dustReady = initDustSensor();
    logEvent(dustReady ? "OK" : "WARN", "DUST",
             dustReady ? "recovered" : "retry failed");
  }
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  logEvent("INFO", "WIFI", "connecting to " + String(WIFI_SSID));
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  wl_status_t previous = WiFi.status();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(250);
    wl_status_t current = WiFi.status();
    if (current != previous) {
      Serial.printf("[%10lu][DEBUG][WIFI] status %d -> %d\n",
                    millis(), previous, current);
      previous = current;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    logEvent("OK", "WIFI", "connected; IP=" + WiFi.localIP().toString() +
             " RSSI=" + String(WiFi.RSSI()) + "dBm");
  } else {
    logEvent("WARN", "WIFI", "connection timed out; status=" +
             String(WiFi.status()) + "; operation skipped until next retry");
    WiFi.disconnect(false);
  }
}

void setClock() {
  if (WiFi.status() != WL_CONNECTED) {
    logEvent("SKIP", "CLOCK", "no WiFi; NTP deferred");
    return;
  }
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  logEvent("INFO", "CLOCK", "synchronizing NTP");
  unsigned long started = millis();
  while (time(nullptr) < 1700000000 && millis() - started < 15000) {
    delay(250);
  }
  if (time(nullptr) < 1700000000) {
    logEvent("WARN", "CLOCK", "NTP failed; TLS upload deferred");
  } else {
    logEvent("OK", "CLOCK", "time synchronized");
  }
}

void pollGps() {
  if (!ENABLE_GPS) return;
  while (GpsSerial.available() > 0) {
    char c = static_cast<char>(GpsSerial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      if (gpsLine.length() > 10) parseGpsSentence(gpsLine);
      gpsLine = "";
    } else if (gpsLine.length() < 120) {
      gpsLine += c;
    } else {
      gpsLine = "";
    }
  }

  if (gpsFixValid && millis() - lastGpsFixMs > GPS_STALE_MS) {
    gpsFixValid = false;
    latestLatitude = NAN;
    latestLongitude = NAN;
  }
}

float nmeaToDegrees(const String &raw, const char hemisphere) {
  if (raw.length() < 4) return NAN;
  // Latitude uses 2 degree digits (ddmm.mmmm); longitude uses 3 (dddmm.mmmm).
  const bool isLon = (hemisphere == 'E' || hemisphere == 'W' ||
                      hemisphere == 'e' || hemisphere == 'w');
  const int degreesDigits = isLon ? 3 : 2;
  if (raw.length() <= degreesDigits) return NAN;
  float degrees = raw.substring(0, degreesDigits).toFloat();
  float minutes = raw.substring(degreesDigits).toFloat();
  float value = degrees + (minutes / 60.0f);
  if (hemisphere == 'S' || hemisphere == 'W' || hemisphere == 's' || hemisphere == 'w') {
    value = -value;
  }
  return value;
}

bool parseGpsSentence(const String &sentence) {
  // Accept GPRMC / GNRMC: ...,status,lat,N/S,lon,E/W,...
  if (!(sentence.startsWith("$GPRMC") || sentence.startsWith("$GNRMC"))) {
    return false;
  }

  String fields[12];
  int fieldCount = 0;
  int start = 0;
  for (int i = 0; i <= sentence.length() && fieldCount < 12; ++i) {
    if (i == sentence.length() || sentence.charAt(i) == ',') {
      fields[fieldCount++] = sentence.substring(start, i);
      start = i + 1;
    }
  }
  if (fieldCount < 7) return false;
  if (fields[2] != "A") return false;  // A = valid fix

  float lat = nmeaToDegrees(fields[3], fields[4].length() ? fields[4].charAt(0) : 'N');
  float lon = nmeaToDegrees(fields[5], fields[6].length() ? fields[6].charAt(0) : 'E');
  if (!isfinite(lat) || !isfinite(lon)) return false;

  latestLatitude = lat;
  latestLongitude = lon;
  gpsReady = true;
  gpsFixValid = true;
  lastGpsFixMs = millis();
  return true;
}

void logEvent(const char *level, const char *component, const String &message) {
  Serial.printf("[%10lu][%s][%s] %s\n", millis(), level, component,
                message.c_str());
  if (cycleLog.length() < 900) {
    if (cycleLog.length()) cycleLog += " | ";
    cycleLog += String(level) + ":" + component + ":" + message;
  }
}

String componentStatusJson() {
  String json = "{";
  json += "\"wifi\":\"";
  json += WiFi.status() == WL_CONNECTED ? "ok" : "unavailable";
  json += "\",\"camera\":\"";
  json += !ENABLE_CAMERA ? "disabled" : (cameraReady ? "ok" : "unavailable");
  json += "\",\"bme280\":\"";
  json += !ENABLE_BME280 ? "disabled" : (bmeReady ? "ok" : "unavailable");
  json += "\",\"dust\":\"";
  json += !ENABLE_DUST ? "disabled" : (dustReady ? "ok" : "unavailable");
  json += "\",\"gas\":\"";
  json += !ENABLE_GAS ? "disabled" : (gasReady ? "ok" : "unavailable");
  json += "\",\"gps\":\"";
  json += !ENABLE_GPS ? "disabled" : (gpsFixValid ? "ok" : "no_fix");
  json += "\",\"sim\":\"";
  json += ENABLE_SIM ? "unavailable" : "disabled";
  json += "\"}";
  return json;
}

String telemetryLogLine() {
  String line = "T=" + String(latestTemperature, 2);
  line += " H=" + String(latestHumidity, 2);
  line += " P=" + String(latestPressure, 2);
  line += " PM1=" + String(latestPm1, 0);
  line += " PM2.5=" + String(latestPm25, 0);
  line += " PM10=" + String(latestPm10, 0);
  line += " GAS=" + String(latestGasRaw, 0);
  line += " LAT=" + String(latestLatitude, 6);
  line += " LON=" + String(latestLongitude, 6);
  line += " RSSI=" + String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
  return line;
}

void printStatus() {
  Serial.println(F("\n--- COMPONENT STATUS ---"));
  Serial.println(componentStatusJson());
  Serial.println(F("------------------------"));
}

void handleSerialCommands() {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      commandLine.trim();
      if (commandLine.length()) executeCommand(commandLine);
      commandLine = "";
    } else if (commandLine.length() < 80) {
      commandLine += c;
    }
  }
}

void executeCommand(String command) {
  command.toLowerCase();
  command.trim();

  if (command == "help") {
    Serial.println(F("status | test wifi | test camera | test bme | test dust"));
    Serial.println(F("test gas | test gps | sync | auto on | auto off"));
  } else if (command == "status") {
    printStatus();
    Serial.println(telemetryLogLine());
  } else if (command == "test wifi") {
    WiFi.disconnect(false);
    connectWiFi();
  } else if (command == "test camera") {
    cameraReady = ENABLE_CAMERA && initializeCamera();
    logEvent(cameraReady ? "OK" : "WARN", "CAMERA",
             cameraReady ? "manual test passed" : "manual test failed");
  } else if (command == "test bme") {
    bmeReady = ENABLE_BME280 && initBme280() && readBme280();
    logEvent(bmeReady ? "OK" : "WARN", "BME280",
             bmeReady ? telemetryLogLine() : "manual test failed");
  } else if (command == "test dust") {
    dustReady = ENABLE_DUST && initDustSensor() && readDustSensor();
    logEvent(dustReady ? "OK" : "WARN", "DUST",
             dustReady ? telemetryLogLine() : "manual test failed");
  } else if (command == "test gas") {
    latestGasRaw = static_cast<float>(analogRead(GAS_ADC_PIN));
    gasReady = ENABLE_GAS;
    logEvent(gasReady ? "OK" : "SKIP", "GAS",
             gasReady ? "raw ADC=" + String(latestGasRaw, 0) : "disabled");
  } else if (command == "test gps") {
    logEvent(gpsFixValid ? "OK" : "WARN", "GPS",
             gpsFixValid
               ? "fix " + String(latestLatitude, 6) + "," + String(latestLongitude, 6)
               : "no_fix: waiting for outdoor sky lock (can take 1-15 min)");
  } else if (command == "sync") {
    readSensors();
    captureAndUpload();
  } else if (command == "auto on") {
    autoSyncEnabled = true;
    logEvent("OK", "SYSTEM", "automatic sync enabled");
  } else if (command == "auto off") {
    autoSyncEnabled = false;
    logEvent("OK", "SYSTEM", "automatic sync disabled");
  } else {
    logEvent("WARN", "COMMAND", "unknown command: " + command);
  }
}

void noteBmeFailure() {
  ++bmeFailStreak;
  logEvent("WARN", "BME280",
           "read failed (" + String(bmeFailStreak) + "/" +
           String(SENSOR_FAIL_LIMIT) + "); recovering I2C");
  if (bmeFailStreak >= SENSOR_FAIL_LIMIT) {
    bmeReady = false;
    latestTemperature = latestHumidity = latestPressure = NAN;
    logEvent("WARN", "BME280", "marked unavailable after repeated failures");
  }
}

void noteDustFailure() {
  ++dustFailStreak;
  logEvent("WARN", "DUST",
           "read failed (" + String(dustFailStreak) + "/" +
           String(SENSOR_FAIL_LIMIT) + "); recovering I2C");
  if (dustFailStreak >= SENSOR_FAIL_LIMIT) {
    dustReady = false;
    latestPm1 = latestPm25 = latestPm10 = NAN;
    logEvent("WARN", "DUST", "marked unavailable after repeated failures");
  }
}

void readSensors() {
  cycleLog = "";

  // Alternate which shared-I2C sensor is touched first so one hung chip
  // cannot permanently starve the other.
  const bool dustFirst = (readingSequence % 2) == 1;

  if (dustFirst) {
    if (dustReady) {
      if (!readDustSensor()) noteDustFailure();
    } else {
      latestPm1 = latestPm25 = latestPm10 = NAN;
    }
    delay(30);
    if (bmeReady) {
      if (!readBme280()) noteBmeFailure();
    } else {
      latestTemperature = latestHumidity = latestPressure = NAN;
    }
  } else {
    if (bmeReady) {
      if (!readBme280()) noteBmeFailure();
    } else {
      latestTemperature = latestHumidity = latestPressure = NAN;
    }
    delay(30);
    if (dustReady) {
      if (!readDustSensor()) noteDustFailure();
    } else {
      latestPm1 = latestPm25 = latestPm10 = NAN;
    }
  }

  if (ENABLE_GAS) {
    uint32_t gasSum = 0;
    for (uint8_t i = 0; i < 16; ++i) {
      gasSum += analogRead(GAS_ADC_PIN);
      delay(2);
    }
    latestGasRaw = gasSum / 16.0f;
    gasReady = true;
  } else {
    latestGasRaw = NAN;
    gasReady = false;
  }

  latestSimSignal = NAN;
  if (ENABLE_GPS && !gpsFixValid) {
    logEvent("INFO", "GPS", "no_fix yet (needs clear sky view)");
  }
  logEvent("DATA", "TELEMETRY", telemetryLogLine());
}

bool captureAndUpload() {
  // Camera SPI sessions can leave the shared I2C bus unsettled.
  recoverI2cBus();

  connectWiFi();
  if (WiFi.status() != WL_CONNECTED) {
    logEvent("SKIP", "SYNC", "no WiFi; data retained in serial log");
    return false;
  }
  if (time(nullptr) < 1700000000) {
    setClock();
    if (time(nullptr) < 1700000000) {
      logEvent("SKIP", "SYNC", "clock invalid; verified TLS unavailable");
      return false;
    }
  }

  bool includeImage = false;
  if (cameraReady) {
    myCAM.flush_fifo();
    myCAM.clear_fifo_flag();
    myCAM.start_capture();

    unsigned long started = millis();
    while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK) &&
           millis() - started <= CAPTURE_TIMEOUT_MS) {
      delay(1);
    }

    if (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
      cameraReady = false;
      myCAM.clear_fifo_flag();
      logEvent("WARN", "CAMERA", "capture timed out; continuing telemetry-only");
    } else {
      uint32_t length = myCAM.read_fifo_length();
      if (length == 0 || length >= MAX_FIFO_SIZE) {
        cameraReady = false;
        myCAM.clear_fifo_flag();
        logEvent("WARN", "CAMERA", "invalid FIFO length; continuing telemetry-only");
      } else {
        includeImage = true;
        logEvent("OK", "CAMERA", "captured " + String(length) + " FIFO bytes");
      }
    }
  } else {
    logEvent("SKIP", "CAMERA", "unavailable; sending telemetry-only reading");
  }

  String readingId = makeReadingId();
  for (uint8_t attempt = 1; attempt <= MAX_UPLOAD_ATTEMPTS; ++attempt) {
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) break;

    WiFiClientSecure client;
    client.setCACert(ROOT_CA);
    client.setTimeout(10000);
    Serial.printf("Opening verified TLS connection to %s (attempt %u)...\n",
                  RENDER_HOST, attempt);
    if (!client.connect(RENDER_HOST, SERVER_PORT)) {
      Serial.println(F("TLS connection failed."));
      if (attempt < MAX_UPLOAD_ATTEMPTS) delay(1000);
      continue;
    }

    bool sent = sendMultipart(client, readingId, includeImage);
    if (!sent) {
      Serial.println(F("Multipart transmission failed."));
      client.stop();
      if (attempt < MAX_UPLOAD_ATTEMPTS) delay(1000);
      continue;
    }

    int status = readHttpStatus(client);
    client.stop();
    if (status >= 200 && status < 300) {
      if (includeImage) myCAM.clear_fifo_flag();
      logEvent("OK", "SYNC", "reading " + readingId +
               " stored; HTTP " + String(status));
      return true;
    }

    Serial.printf("Backend rejected reading %s with HTTP %d.\n",
                  readingId.c_str(), status);
    if (status >= 400 && status < 500) break;
    if (attempt < MAX_UPLOAD_ATTEMPTS) delay(1000);
  }
  if (includeImage) myCAM.clear_fifo_flag();
  return false;
}

bool sendMultipart(WiFiClientSecure &client, const String &readingId,
                   bool includeImage) {
  String headers = "POST " + String(INGEST_PATH) + " HTTP/1.1\r\n";
  headers += "Host: " + String(RENDER_HOST) + "\r\n";
  headers += "Content-Type: multipart/form-data; boundary=";
  headers += String(MULTIPART_BOUNDARY) + "\r\n";
  headers += "Transfer-Encoding: chunked\r\n";
  headers += "Connection: close\r\n\r\n";
  if (!writeAll(client,
                reinterpret_cast<const uint8_t *>(headers.c_str()),
                headers.length())) return false;

  if (!writeTextPart(client, "reading_id", readingId)) return false;
  if (!writeFloatPart(client, "temperature", latestTemperature)) return false;
  if (!writeFloatPart(client, "humidity", latestHumidity)) return false;
  if (!writeFloatPart(client, "pressure", latestPressure)) return false;
  if (!writeFloatPart(client, "pm1", latestPm1)) return false;
  if (!writeFloatPart(client, "pm2_5", latestPm25)) return false;
  if (!writeFloatPart(client, "pm10", latestPm10)) return false;
  if (!writeFloatPart(client, "gas_raw", latestGasRaw)) return false;
  if (!writeFloatPart(client, "latitude", latestLatitude)) return false;
  if (!writeFloatPart(client, "longitude", latestLongitude)) return false;
  if (!writeTextPart(client, "device_status", componentStatusJson())) return false;
  if (!writeTextPart(client, "device_log", cycleLog)) return false;

  if (includeImage) {
    String imageHeader = "--" + String(MULTIPART_BOUNDARY) + "\r\n";
    imageHeader += "Content-Disposition: form-data; name=\"image\"; "
                   "filename=\"capture.jpg\"\r\n";
    imageHeader += "Content-Type: image/jpeg\r\n\r\n";
    if (!writeChunk(client, imageHeader)) return false;
    if (!streamJpegPart(client)) return false;
  }

  String closing = includeImage ? "\r\n--" : "--";
  closing += String(MULTIPART_BOUNDARY) + "--\r\n";
  if (!writeChunk(client, closing)) return false;
  return writeAll(client, reinterpret_cast<const uint8_t *>("0\r\n\r\n"), 5);
}

bool streamJpegPart(WiFiClientSecure &client) {
  myCAM.write_reg(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK);
  uint32_t remaining = myCAM.read_fifo_length();
  uint8_t buffer[IMAGE_CHUNK_SIZE];
  size_t used = 0;
  uint8_t previous = 0;
  bool jpegStarted = false;
  bool jpegEnded = false;
  bool writeOk = true;

  myCAM.CS_LOW();
  myCAM.set_fifo_burst();

  while (remaining-- && !jpegEnded) {
    uint8_t current = SPI.transfer(0x00);
    if (!jpegStarted) {
      if (previous == 0xFF && current == 0xD8) {
        buffer[used++] = 0xFF;
        buffer[used++] = 0xD8;
        jpegStarted = true;
      }
    } else {
      buffer[used++] = current;
      jpegEnded = previous == 0xFF && current == 0xD9;
    }
    previous = current;

    if (used == sizeof(buffer) || jpegEnded) {
      myCAM.CS_HIGH();
      writeOk = writeChunk(client, buffer, used);
      used = 0;
      if (!writeOk || jpegEnded) break;
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();
    }
  }
  myCAM.CS_HIGH();

  if (writeOk && used > 0) writeOk = writeChunk(client, buffer, used);
  if (!jpegStarted || !jpegEnded) {
    Serial.println(F("FIFO did not contain a complete JPEG."));
    return false;
  }
  return writeOk;
}

bool writeAll(WiFiClientSecure &client, const uint8_t *data, size_t length) {
  size_t sent = 0;
  while (sent < length) {
    size_t count = client.write(data + sent, length - sent);
    if (count == 0) return false;
    sent += count;
  }
  return true;
}

bool writeChunk(WiFiClientSecure &client, const uint8_t *data, size_t length) {
  char chunkHeader[16];
  int headerLength = snprintf(chunkHeader, sizeof(chunkHeader), "%X\r\n",
                              (unsigned int)length);
  if (headerLength <= 0) return false;
  if (!writeAll(client, reinterpret_cast<const uint8_t *>(chunkHeader),
                static_cast<size_t>(headerLength))) return false;
  if (!writeAll(client, data, length)) return false;
  return writeAll(client, reinterpret_cast<const uint8_t *>("\r\n"), 2);
}

bool writeChunk(WiFiClientSecure &client, const String &data) {
  return writeChunk(client,
                    reinterpret_cast<const uint8_t *>(data.c_str()),
                    data.length());
}

bool writeTextPart(WiFiClientSecure &client, const char *name,
                   const String &value) {
  String part = "--" + String(MULTIPART_BOUNDARY) + "\r\n";
  part += "Content-Disposition: form-data; name=\"" + String(name) + "\"\r\n\r\n";
  part += value + "\r\n";
  return writeChunk(client, part);
}

bool writeFloatPart(WiFiClientSecure &client, const char *name, float value) {
  if (!isfinite(value)) return true;
  return writeTextPart(client, name, String(value, 4));
}

String makeReadingId() {
  uint64_t chipId = ESP.getEfuseMac();
  char id[64];
  snprintf(id, sizeof(id), "%04X%08X-%lu-%lu",
           static_cast<unsigned int>(chipId >> 32),
           static_cast<unsigned int>(chipId),
           static_cast<unsigned long>(time(nullptr)),
           static_cast<unsigned long>(++readingSequence));
  return String(id);
}

int readHttpStatus(WiFiClientSecure &client) {
  unsigned long started = millis();
  while (!client.available()) {
    if (!client.connected()) return -1;
    if (millis() - started > RESPONSE_TIMEOUT_MS) {
      Serial.println(F("Timed out waiting for inference response."));
      return -1;
    }
    delay(10);
  }

  String statusLine = client.readStringUntil('\n');
  statusLine.trim();
  Serial.println(statusLine);
  if (!statusLine.startsWith("HTTP/")) return -1;
  int firstSpace = statusLine.indexOf(' ');
  if (firstSpace < 0) return -1;
  return statusLine.substring(firstSpace + 1).toInt();
}
