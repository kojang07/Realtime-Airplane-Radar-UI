// Realtime Airplane Radar UI with Vivid Terrain Map & Airplane Icons - Incheon Int'l Airport (RKSI)
// Hardware: ESP32-4848S040 (ESP32-S3 480x480 RGB ST7701S)
// Features: Rotating Airplane Icons, Incheon/Seoul Color Terrain (Sea, Land, Mountains, Han River)

#include <Wire.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <esp_heap_caps.h>
#include <math.h>
#include "time.h"

// ==========================================
// 1. 사용자 설정 (Wi-Fi 및 인천국제공항 기준 위치)
// ==========================================
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// 기준 좌표: 인천국제공항 (RKSI)
const double HOME_LAT     = 37.4602;
const double HOME_LON     = 126.4407;
const char* LOCATION_NAME = "INCHEON";

// 탐지 반경 (100km / 54해리)
const float RADAR_RANGE_KM = 100.0;
const int API_RADIUS_NM    = 54;

// PSRAM을 사용하는 ArduinoJson 커스텀 할당자
struct SpiRamAllocator {
  void* allocate(size_t size) {
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
  }
  void deallocate(void* pointer) {
    heap_caps_free(pointer);
  }
  void* reallocate(void* ptr, size_t new_size) {
    return heap_caps_realloc(ptr, new_size, MALLOC_CAP_SPIRAM);
  }
};
using SpiRamJsonDocument = BasicJsonDocument<SpiRamAllocator>;

// ==========================================
// 2. ST7701S 3-Wire SPI 직접 초기화
// ==========================================
#define SPI_CS_PIN   39
#define SPI_SCK_PIN  48
#define SPI_SDA_PIN  47

void st7701_send(uint8_t data, bool is_cmd) {
  digitalWrite(SPI_CS_PIN, LOW);
  digitalWrite(SPI_SCK_PIN, LOW);
  digitalWrite(SPI_SDA_PIN, is_cmd ? LOW : HIGH);
  digitalWrite(SPI_SCK_PIN, HIGH);

  for (int i = 7; i >= 0; i--) {
    digitalWrite(SPI_SCK_PIN, LOW);
    digitalWrite(SPI_SDA_PIN, (data >> i) & 0x01);
    digitalWrite(SPI_SCK_PIN, HIGH);
  }
  digitalWrite(SPI_CS_PIN, HIGH);
}

void st7701_cmd(uint8_t cmd) { st7701_send(cmd, true); }
void st7701_dat(uint8_t dat) { st7701_send(dat, false); }

void st7701_init_panel() {
  pinMode(SPI_CS_PIN, OUTPUT);
  pinMode(SPI_SCK_PIN, OUTPUT);
  pinMode(SPI_SDA_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  digitalWrite(SPI_SCK_PIN, HIGH);
  delay(20);

  st7701_cmd(0xFF); st7701_dat(0x77); st7701_dat(0x01); st7701_dat(0x00); st7701_dat(0x00); st7701_dat(0x10);
  st7701_cmd(0xC0); st7701_dat(0x3B); st7701_dat(0x00);
  st7701_cmd(0xC1); st7701_dat(0x0D); st7701_dat(0x02);
  st7701_cmd(0xC2); st7701_dat(0x31); st7701_dat(0x05);
  st7701_cmd(0xCD); st7701_dat(0x00);

  st7701_cmd(0xB0);
  st7701_dat(0x00); st7701_dat(0x11); st7701_dat(0x18); st7701_dat(0x0E);
  st7701_dat(0x11); st7701_dat(0x06); st7701_dat(0x07); st7701_dat(0x08);
  st7701_dat(0x07); st7701_dat(0x22); st7701_dat(0x04); st7701_dat(0x12);
  st7701_dat(0x0F); st7701_dat(0xAA); st7701_dat(0x31); st7701_dat(0x18);

  st7701_cmd(0xB1);
  st7701_dat(0x00); st7701_dat(0x11); st7701_dat(0x19); st7701_dat(0x0E);
  st7701_dat(0x12); st7701_dat(0x07); st7701_dat(0x08); st7701_dat(0x08);
  st7701_dat(0x08); st7701_dat(0x22); st7701_dat(0x04); st7701_dat(0x11);
  st7701_dat(0x11); st7701_dat(0xA9); st7701_dat(0x32); st7701_dat(0x18);

  st7701_cmd(0xFF); st7701_dat(0x77); st7701_dat(0x01); st7701_dat(0x00); st7701_dat(0x00); st7701_dat(0x11);
  st7701_cmd(0xB0); st7701_dat(0x60);
  st7701_cmd(0xB1); st7701_dat(0x32);
  st7701_cmd(0xB2); st7701_dat(0x07);
  st7701_cmd(0xB3); st7701_dat(0x80);
  st7701_cmd(0xB5); st7701_dat(0x49);
  st7701_cmd(0xB7); st7701_dat(0x85);
  st7701_cmd(0xB8); st7701_dat(0x21);
  st7701_cmd(0xC1); st7701_dat(0x78);
  st7701_cmd(0xC2); st7701_dat(0x78);
  delay(20);

  st7701_cmd(0xE0); st7701_dat(0x00); st7701_dat(0x1B); st7701_dat(0x02);
  st7701_cmd(0xE1);
  st7701_dat(0x08); st7701_dat(0xA0); st7701_dat(0x00); st7701_dat(0x00);
  st7701_dat(0x07); st7701_dat(0xA0); st7701_dat(0x00); st7701_dat(0x00);
  st7701_dat(0x00); st7701_dat(0x44); st7701_dat(0x44);

  st7701_cmd(0xE2);
  st7701_dat(0x11); st7701_dat(0x11); st7701_dat(0x44); st7701_dat(0x44);
  st7701_dat(0xED); st7701_dat(0xA0); st7701_dat(0x00); st7701_dat(0x00);
  st7701_dat(0xEC); st7701_dat(0xA0); st7701_dat(0x00); st7701_dat(0x00);

  st7701_cmd(0xE3); st7701_dat(0x00); st7701_dat(0x00); st7701_dat(0x11); st7701_dat(0x11);
  st7701_cmd(0xE4); st7701_dat(0x44); st7701_dat(0x44);

  st7701_cmd(0xE5);
  st7701_dat(0x0A); st7701_dat(0xE9); st7701_dat(0xD8); st7701_dat(0xA0);
  st7701_dat(0x0C); st7701_dat(0xEB); st7701_dat(0xD8); st7701_dat(0xA0);
  st7701_dat(0x0E); st7701_dat(0xED); st7701_dat(0xD8); st7701_dat(0xA0);
  st7701_dat(0x10); st7701_dat(0xEF); st7701_dat(0xD8); st7701_dat(0xA0);

  st7701_cmd(0xE6); st7701_dat(0x00); st7701_dat(0x00); st7701_dat(0x11); st7701_dat(0x11);
  st7701_cmd(0xE7); st7701_dat(0x44); st7701_dat(0x44);

  st7701_cmd(0xE8);
  st7701_dat(0x09); st7701_dat(0xE8); st7701_dat(0xD8); st7701_dat(0xA0);
  st7701_dat(0x0B); st7701_dat(0xEA); st7701_dat(0xD8); st7701_dat(0xA0);
  st7701_dat(0x0D); st7701_dat(0xEC); st7701_dat(0xD8); st7701_dat(0xA0);
  st7701_dat(0x0F); st7701_dat(0xEE); st7701_dat(0xD8); st7701_dat(0xA0);

  st7701_cmd(0xEB);
  st7701_dat(0x02); st7701_dat(0x00); st7701_dat(0xE4); st7701_dat(0xE4);
  st7701_dat(0x88); st7701_dat(0x00); st7701_dat(0x40);

  st7701_cmd(0xEC); st7701_dat(0x3C); st7701_dat(0x00);

  st7701_cmd(0xED);
  st7701_dat(0xAB); st7701_dat(0x89); st7701_dat(0x76); st7701_dat(0x54);
  st7701_dat(0x02); st7701_dat(0xFF); st7701_dat(0xFF); st7701_dat(0xFF);
  st7701_dat(0xFF); st7701_dat(0xFF); st7701_dat(0xFF); st7701_dat(0x20);
  st7701_dat(0x45); st7701_dat(0x67); st7701_dat(0x98); st7701_dat(0xBA);

  st7701_cmd(0xFF); st7701_dat(0x77); st7701_dat(0x01); st7701_dat(0x00); st7701_dat(0x00); st7701_dat(0x00);
  st7701_cmd(0x3A); st7701_dat(0x60);
  st7701_cmd(0x36); st7701_dat(0x00);
  st7701_cmd(0x20);

  st7701_cmd(0x11);
  delay(120);
  st7701_cmd(0x29);
  delay(20);
}

// ==========================================
// 3. LovyanGFX Panel_RGB 구성
// ==========================================
class LGFX : public lgfx::LGFX_Device {
public:
  lgfx::Bus_RGB   _bus;
  lgfx::Panel_RGB _panel;
  lgfx::Light_PWM _light;

  LGFX(void) {
    {
      auto cfg = _panel.config();
      cfg.memory_width  = 480;
      cfg.memory_height = 480;
      cfg.panel_width   = 480;
      cfg.panel_height  = 480;
      cfg.offset_x      = 0;
      cfg.offset_y      = 0;
      _panel.config(cfg);
    }

    {
      auto cfg = _bus.config();
      cfg.panel = &_panel;

      cfg.pin_d0  = GPIO_NUM_4;
      cfg.pin_d1  = GPIO_NUM_5;
      cfg.pin_d2  = GPIO_NUM_6;
      cfg.pin_d3  = GPIO_NUM_7;
      cfg.pin_d4  = GPIO_NUM_15;

      cfg.pin_d5  = GPIO_NUM_8;
      cfg.pin_d6  = GPIO_NUM_20;
      cfg.pin_d7  = GPIO_NUM_3;
      cfg.pin_d8  = GPIO_NUM_46;
      cfg.pin_d9  = GPIO_NUM_9;
      cfg.pin_d10 = GPIO_NUM_10;

      cfg.pin_d11 = GPIO_NUM_11;
      cfg.pin_d12 = GPIO_NUM_12;
      cfg.pin_d13 = GPIO_NUM_13;
      cfg.pin_d14 = GPIO_NUM_14;
      cfg.pin_d15 = GPIO_NUM_0;

      cfg.pin_henable = GPIO_NUM_18;
      cfg.pin_vsync   = GPIO_NUM_17;
      cfg.pin_hsync   = GPIO_NUM_16;
      cfg.pin_pclk    = GPIO_NUM_21;

      cfg.freq_write = 8000000;

      cfg.hsync_polarity    = 1;
      cfg.hsync_front_porch = 10;
      cfg.hsync_pulse_width = 8;
      cfg.hsync_back_porch  = 50;

      cfg.vsync_polarity    = 1;
      cfg.vsync_front_porch = 10;
      cfg.vsync_pulse_width = 8;
      cfg.vsync_back_porch  = 20;

      cfg.pclk_active_neg = 0;
      cfg.de_idle_high    = 0;
      cfg.pclk_idle_high  = 0;

      _bus.config(cfg);
      _panel.setBus(&_bus);
    }

    {
      auto cfg = _light.config();
      cfg.pin_bl = GPIO_NUM_38;
      cfg.freq = 44100;
      cfg.pwm_channel = 7;
      _light.config(cfg);
      _panel.setLight(&_light);
    }

    setPanel(&_panel);
  }
};

LGFX lcd;
LGFX_Sprite radarCanvas(&lcd);

// ---------- 레이아웃 상수 ----------
const int SCREEN_W = 480;
const int SCREEN_H = 480;
const int CX = 240;
const int CY = 240;
const int R  = 222;

const int OUTER_GAP = 3;
const int OUTER_RING_THICKNESS = 2;

// ---------- 스윕 및 타이머 ----------
float sweepAngle = 0;
unsigned long lastSweepUpdate = 0;
unsigned long lastApiFetchTime = 0;

const int TRAIL_WIDTH = 45;
const int SWEEP_SPEED = 40;
const unsigned long API_FETCH_INTERVAL = 12000;

// ---------- 컬러 팔레트 ----------
uint16_t greenBright;
uint16_t green;
uint16_t greenDim;
uint16_t greenDark;

uint16_t colSeaBg;
uint16_t colLandFill;
uint16_t colCoastLine;
uint16_t colMountainFill;
uint16_t colMountainRidge;
uint16_t colRiver;
uint16_t colCityDot;
uint16_t colCityText;
uint16_t colAirport;

// ==========================================
// 4. 수도권 · 서해안 상세 지형 벡터 데이터
// ==========================================
const float POLY_MAINLAND[][2] = {
  {38.45f, 125.50f}, {38.45f, 127.80f}, {36.45f, 127.80f},
  {36.45f, 126.35f}, {36.75f, 126.15f}, {36.95f, 126.35f},
  {36.98f, 126.75f}, {37.02f, 126.82f}, {37.18f, 126.68f},
  {37.30f, 126.68f}, {37.38f, 126.62f}, {37.48f, 126.61f},
  {37.58f, 126.60f}, {37.72f, 126.55f}, {37.75f, 126.68f},
  {37.82f, 126.66f}, {37.88f, 126.42f}, {37.95f, 126.18f},
  {37.85f, 125.75f}, {38.15f, 125.50f}
};
const int POLY_MAINLAND_LEN = sizeof(POLY_MAINLAND) / sizeof(POLY_MAINLAND[0]);

const float POLY_YEONGJONG[][2] = {
  {37.51f, 126.42f}, {37.50f, 126.56f}, {37.43f, 126.55f}, {37.42f, 126.40f},
  {37.47f, 126.38f}
};
const int POLY_YEONGJONG_LEN = sizeof(POLY_YEONGJONG) / sizeof(POLY_YEONGJONG[0]);

const float POLY_GANGHWA[][2] = {
  {37.81f, 126.43f}, {37.78f, 126.52f}, {37.62f, 126.51f}, {37.61f, 126.38f},
  {37.74f, 126.35f}
};
const int POLY_GANGHWA_LEN = sizeof(POLY_GANGHWA) / sizeof(POLY_GANGHWA[0]);

const float POLY_DAEBU[][2] = {
  {37.29f, 126.55f}, {37.26f, 126.62f}, {37.21f, 126.58f}, {37.23f, 126.48f}
};
const int POLY_DAEBU_LEN = sizeof(POLY_DAEBU) / sizeof(POLY_DAEBU[0]);

const float POLY_DEOKJEOK[][2] = {
  {37.26f, 126.12f}, {37.23f, 126.18f}, {37.20f, 126.13f}, {37.23f, 126.08f}
};
const int POLY_DEOKJEOK_LEN = sizeof(POLY_DEOKJEOK) / sizeof(POLY_DEOKJEOK[0]);

const float POLY_MTN_BUKHAN[][2] = {
  {38.05f, 126.95f}, {38.02f, 127.35f}, {37.75f, 127.38f}, {37.61f, 127.05f},
  {37.63f, 126.93f}, {37.85f, 126.90f}
};
const int POLY_MTN_BUKHAN_LEN = sizeof(POLY_MTN_BUKHAN) / sizeof(POLY_MTN_BUKHAN[0]);

const float POLY_MTN_GWANAK[][2] = {
  {37.48f, 126.92f}, {37.48f, 127.25f}, {37.30f, 127.22f}, {37.32f, 126.95f}
};
const int POLY_MTN_GWANAK_LEN = sizeof(POLY_MTN_GWANAK) / sizeof(POLY_MTN_GWANAK[0]);

const float POLY_MTN_MANI[][2] = {
  {37.66f, 126.41f}, {37.65f, 126.47f}, {37.61f, 126.45f}, {37.62f, 126.40f}
};
const int POLY_MTN_MANI_LEN = sizeof(POLY_MTN_MANI) / sizeof(POLY_MTN_MANI[0]);

const float POLY_MTN_CHARYEONG[][2] = {
  {36.85f, 126.55f}, {36.82f, 127.25f}, {36.60f, 127.20f}, {36.65f, 126.55f}
};
const int POLY_MTN_CHARYEONG_LEN = sizeof(POLY_MTN_CHARYEONG) / sizeof(POLY_MTN_CHARYEONG[0]);

const char* PEAK_NAMES[] = {"BUKHAN 836m", "GWANAK 632m", "MANI 472m", "GAYA 678m"};
const double PEAK_COORDS[][2] = {
  {37.6584, 126.9780},
  {37.4453, 126.9642},
  {37.6122, 126.4342},
  {36.7064, 126.6119}
};
const int PEAKS_LEN = 4;

const float RIVER_HAN[][2] = {
  {37.52f, 127.45f}, {37.55f, 127.22f}, {37.53f, 127.05f}, {37.52f, 126.92f},
  {37.58f, 126.80f}, {37.68f, 126.68f}, {37.76f, 126.58f}
};
const int RIVER_HAN_LEN = sizeof(RIVER_HAN) / sizeof(RIVER_HAN[0]);

const float RIVER_IMJIN[][2] = {
  {38.10f, 127.05f}, {37.95f, 126.85f}, {37.88f, 126.68f}, {37.78f, 126.62f}
};
const int RIVER_IMJIN_LEN = sizeof(RIVER_IMJIN) / sizeof(RIVER_IMJIN[0]);

const float RIVER_BUKHAN[][2] = {
  {37.95f, 127.55f}, {37.75f, 127.42f}, {37.54f, 127.32f}
};
const int RIVER_BUKHAN_LEN = sizeof(RIVER_BUKHAN) / sizeof(RIVER_BUKHAN[0]);

const char* CITY_NAMES[] = {"SEOUL", "INCHEON", "SUWON", "GOYANG", "SEONGNAM", "PYEONGTAEK", "CHEONAN", "PAJU"};
const double CITY_COORDS[][2] = {
  {37.5665, 126.9780},
  {37.4563, 126.7052},
  {37.2636, 127.0286},
  {37.6584, 126.8320},
  {37.4200, 127.1267},
  {36.9921, 127.1129},
  {36.8151, 127.1139},
  {37.7599, 126.7802}
};
const int CITIES_LEN = 8;

const char* AIRPORT_CODES[] = {"RKSI", "RKSS", "RKSM", "RKSO", "RKSW"};
const double AIRPORT_COORDS[][2] = {
  {37.4602, 126.4407},
  {37.5583, 126.7906},
  {37.4458, 127.1139},
  {37.0906, 127.0297},
  {37.2394, 127.0069}
};
const int AIRPORTS_LEN = 5;

// ---------- AIRCRAFT DATA ----------
struct AircraftPoint {
  bool valid;
  float distanceKm;
  float bearingDeg;
  int altitudeFt;
  float speedKt;
  float trackDeg;
  char flight[12];
  char category[5];
  char typeCode[8];
};

const int MAX_PLANES = 40;

AircraftPoint planes[MAX_PLANES];
AircraftPoint pendingPlanes[MAX_PLANES];

int planeCount = 0;
int pendingPlaneCount = 0;

bool apiOk = false;
bool pendingApiOk = false;
bool fetchInProgress = false;
bool pendingDataReady = false;
String apiStatusMsg = "WAIT...";

// ---------- MATH & GEO HELPERS ----------
double deg2radD(double deg) {
  return deg * M_PI / 180.0;
}

double rad2degD(double rad) {
  return rad * 180.0 / M_PI;
}

float angleDiffBehindSweep(float sweep, float targetBearing) {
  float d = sweep - targetBearing;
  while (d < 0) d += 360;
  while (d >= 360) d -= 360;
  return d;
}

float distanceKm(double lat1, double lon1, double lat2, double lon2) {
  const double Rearth = 6371.0;

  double dLat = deg2radD(lat2 - lat1);
  double dLon = deg2radD(lon2 - lon1);

  double a =
    sin(dLat / 2) * sin(dLat / 2) +
    cos(deg2radD(lat1)) * cos(deg2radD(lat2)) *
    sin(dLon / 2) * sin(dLon / 2);

  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return Rearth * c;
}

float bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double y = sin(deg2radD(lon2 - lon1)) * cos(deg2radD(lat2));
  double x =
    cos(deg2radD(lat1)) * sin(deg2radD(lat2)) -
    sin(deg2radD(lat1)) * cos(deg2radD(lat2)) * cos(deg2radD(lon2 - lon1));

  double brng = rad2degD(atan2(y, x));
  if (brng < 0) brng += 360.0;
  return brng;
}

void geoToScreen(double lat, double lon, int &outX, int &outY) {
  float d = distanceKm(HOME_LAT, HOME_LON, lat, lon);
  float brng = bearingDeg(HOME_LAT, HOME_LON, lat, lon);
  float rr = (d / RADAR_RANGE_KM) * R;
  float rad = (brng - 90.0) * DEG_TO_RAD;
  outX = CX + cos(rad) * rr;
  outY = CY + sin(rad) * rr;
}

void canvasCorrectedRadialLine(int angleDeg, int r1, int r2, uint16_t col) {
  float rad = (angleDeg - 90) * DEG_TO_RAD;

  int x1 = CX + cos(rad) * r1;
  int y1 = CY + sin(rad) * r1;
  int x2 = CX + cos(rad) * r2;
  int y2 = CY + sin(rad) * r2;

  radarCanvas.drawLine(x1, y1, x2, y2, col);
}

// ---------- API FETCH ----------
bool tryFetchFromUrl(const String &url, const char* serverTag) {
  Serial.printf("\n[ADS-B] Trying %s -> %s\n", serverTag, url.c_str());

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(15);

  HTTPClient http;
  http.setConnectTimeout(10000);
  http.setTimeout(15000);
  http.setReuse(false);
  http.addHeader("User-Agent", "ESP32-S3-Radar/1.0");
  http.addHeader("Accept", "application/json");
  http.addHeader("Accept-Encoding", "identity");

  if (!http.begin(client, url)) {
    apiStatusMsg = "BEGIN ERR";
    return false;
  }

  int code = http.GET();
  Serial.printf("[ADS-B] HTTP Response Code: %d\n", code);

  if (code != 200) {
    apiStatusMsg = "HTTP " + String(code);
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  Serial.printf("[ADS-B] Payload length: %d bytes\n", payload.length());
  if (payload.length() < 20) {
    apiStatusMsg = "EMPTY DATA";
    return false;
  }

  StaticJsonDocument<512> filter;
  JsonObject acFilter = filter["ac"].createNestedObject();
  acFilter["lat"] = true;
  acFilter["lon"] = true;
  acFilter["alt_baro"] = true;
  acFilter["alt_geom"] = true;
  acFilter["gs"] = true;
  acFilter["track"] = true;
  acFilter["flight"] = true;
  acFilter["category"] = true;
  acFilter["t"] = true;

  JsonObject aircraftFilter = filter["aircraft"].createNestedObject();
  aircraftFilter["lat"] = true;
  aircraftFilter["lon"] = true;
  aircraftFilter["alt_baro"] = true;
  aircraftFilter["alt_geom"] = true;
  aircraftFilter["gs"] = true;
  aircraftFilter["track"] = true;
  aircraftFilter["flight"] = true;
  aircraftFilter["category"] = true;
  aircraftFilter["t"] = true;

  SpiRamJsonDocument doc(131072);
  DeserializationError err = deserializeJson(doc, payload, DeserializationOption::Filter(filter));

  if (err) {
    Serial.printf("[ADS-B] JSON Parse Error: %s\n", err.c_str());
    apiStatusMsg = "JSON ERR";
    return false;
  }

  AircraftPoint tempPlanes[MAX_PLANES];
  int tempCount = 0;

  JsonArray arr = doc["ac"].as<JsonArray>();
  if (arr.isNull() || arr.size() == 0) {
    arr = doc["aircraft"].as<JsonArray>();
  }

  Serial.printf("[ADS-B] Parsed array size: %d\n", arr.size());

  for (JsonObject ac : arr) {
    if (tempCount >= MAX_PLANES) break;
    if (!ac["lat"].is<float>() || !ac["lon"].is<float>()) continue;

    double lat = ac["lat"];
    double lon = ac["lon"];

    float d = distanceKm(HOME_LAT, HOME_LON, lat, lon);
    if (d > RADAR_RANGE_KM) continue;

    AircraftPoint &p = tempPlanes[tempCount];
    p.valid = true;
    p.distanceKm = d;
    p.bearingDeg = bearingDeg(HOME_LAT, HOME_LON, lat, lon);

    if (ac["alt_baro"].is<int>()) p.altitudeFt = ac["alt_baro"];
    else if (ac["alt_geom"].is<int>()) p.altitudeFt = ac["alt_geom"];
    else p.altitudeFt = -1;

    p.speedKt = ac["gs"] | 0.0;
    p.trackDeg = ac["track"] | 0.0;

    const char* fl = ac["flight"] | "";
    strncpy(p.flight, fl, sizeof(p.flight) - 1);
    p.flight[sizeof(p.flight) - 1] = '\0';

    const char* cat = ac["category"] | "";
    strncpy(p.category, cat, sizeof(p.category) - 1);
    p.category[sizeof(p.category) - 1] = '\0';

    const char* typ = ac["t"] | "";
    strncpy(p.typeCode, typ, sizeof(p.typeCode) - 1);
    p.typeCode[sizeof(p.typeCode) - 1] = '\0';

    tempCount++;
  }

  noInterrupts();
  pendingPlaneCount = tempCount;
  for (int i = 0; i < tempCount; i++) {
    pendingPlanes[i] = tempPlanes[i];
  }
  pendingApiOk = true;
  pendingDataReady = true;
  interrupts();

  Serial.printf("[ADS-B] Success (%s)! Aircraft within 100km: %d\n", serverTag, tempCount);
  return true;
}

void fetchAircraftData() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[ADS-B] WiFi not connected!");
    apiStatusMsg = "NO WIFI";
    pendingApiOk = false;
    pendingPlaneCount = 0;
    pendingDataReady = true;
    return;
  }

  String path = String(HOME_LAT, 4) + "/" + String(HOME_LON, 4) + "/" + String(API_RADIUS_NM);

  String url1 = "https://api.adsb.lol/v2/point/" + path;
  if (tryFetchFromUrl(url1, "adsb.lol")) return;

  delay(300);

  String url2 = "https://opendata.adsb.fi/api/v2/lat/" + String(HOME_LAT, 4) +
                "/lon/" + String(HOME_LON, 4) + "/dist/" + String(API_RADIUS_NM);
  if (tryFetchFromUrl(url2, "adsb.fi")) return;

  delay(300);

  String url3 = "https://api.airplanes.live/v2/point/" + path;
  if (tryFetchFromUrl(url3, "airplanes.live")) return;

  pendingApiOk = false;
  pendingDataReady = true;
}

void aircraftFetchTask(void *parameter) {
  fetchAircraftData();
  fetchInProgress = false;
  vTaskDelete(NULL);
}

void startAircraftFetchBackground() {
  if (fetchInProgress) return;
  fetchInProgress = true;

  xTaskCreatePinnedToCore(
    aircraftFetchTask,
    "aircraftFetchTask",
    24576,
    NULL,
    1,
    NULL,
    0
  );
}

void applyPendingAircraftData() {
  if (!pendingDataReady) return;

  noInterrupts();
  planeCount = pendingPlaneCount;
  for (int i = 0; i < planeCount; i++) {
    planes[i] = pendingPlanes[i];
  }
  apiOk = pendingApiOk;
  pendingDataReady = false;
  interrupts();
}

// ---------- 지형(바다 · 육지 · 산 · 강) 그리기 함수 ----------
void drawFilledGeoPolygon(const float pts[][2], int len, uint16_t fillCol, uint16_t borderCol) {
  int vx[32];
  int vy[32];
  if (len > 32) len = 32;

  int minY = SCREEN_H, maxY = 0;
  for (int i = 0; i < len; i++) {
    geoToScreen(pts[i][0], pts[i][1], vx[i], vy[i]);
    if (vy[i] < minY) minY = vy[i];
    if (vy[i] > maxY) maxY = vy[i];
  }

  if (minY < 2) minY = 2;
  if (maxY > SCREEN_H - 3) maxY = SCREEN_H - 3;

  int nodes[32];
  for (int y = minY; y <= maxY; y++) {
    int nodeCount = 0;
    int j = len - 1;
    for (int i = 0; i < len; i++) {
      if ((vy[i] < y && vy[j] >= y) || (vy[j] < y && vy[i] >= y)) {
        nodes[nodeCount++] = vx[i] + (float)(y - vy[i]) / (vy[j] - vy[i]) * (vx[j] - vx[i]);
      }
      j = i;
    }

    for (int i = 0; i < nodeCount - 1; i++) {
      for (int k = i + 1; k < nodeCount; k++) {
        if (nodes[i] > nodes[k]) {
          int tmp = nodes[i];
          nodes[i] = nodes[k];
          nodes[k] = tmp;
        }
      }
    }

    for (int i = 0; i < nodeCount; i += 2) {
      if (i + 1 >= nodeCount) break;
      int xStart = max(2, nodes[i]);
      int xEnd   = min(SCREEN_W - 3, nodes[i + 1]);
      if (xEnd > xStart) {
        radarCanvas.drawFastHLine(xStart, y, xEnd - xStart + 1, fillCol);
      }
    }
  }

  for (int i = 0; i < len; i++) {
    int next = (i + 1) % len;
    radarCanvas.drawLine(vx[i], vy[i], vx[next], vy[next], borderCol);
  }
}

void drawThickPolyline(const float pts[][2], int len, uint16_t col, bool thick) {
  for (int i = 0; i < len - 1; i++) {
    int x1, y1, x2, y2;
    geoToScreen(pts[i][0], pts[i][1], x1, y1);
    geoToScreen(pts[i + 1][0], pts[i + 1][1], x2, y2);

    radarCanvas.drawLine(x1, y1, x2, y2, col);
    if (thick) {
      radarCanvas.drawLine(x1 + 1, y1, x2 + 1, y2, col);
      radarCanvas.drawLine(x1, y1 + 1, x2, y2 + 1, col);
    }
  }
}

void drawVividTerrainMap() {
  radarCanvas.fillRect(2, 2, SCREEN_W - 4, SCREEN_H - 4, colSeaBg);

  drawFilledGeoPolygon(POLY_MAINLAND, POLY_MAINLAND_LEN, colLandFill, colCoastLine);
  drawFilledGeoPolygon(POLY_YEONGJONG, POLY_YEONGJONG_LEN, colLandFill, colCoastLine);
  drawFilledGeoPolygon(POLY_GANGHWA, POLY_GANGHWA_LEN, colLandFill, colCoastLine);
  drawFilledGeoPolygon(POLY_DAEBU, POLY_DAEBU_LEN, colLandFill, colCoastLine);
  drawFilledGeoPolygon(POLY_DEOKJEOK, POLY_DEOKJEOK_LEN, colLandFill, colCoastLine);

  drawFilledGeoPolygon(POLY_MTN_BUKHAN, POLY_MTN_BUKHAN_LEN, colMountainFill, colMountainRidge);
  drawFilledGeoPolygon(POLY_MTN_GWANAK, POLY_MTN_GWANAK_LEN, colMountainFill, colMountainRidge);
  drawFilledGeoPolygon(POLY_MTN_MANI, POLY_MTN_MANI_LEN, colMountainFill, colMountainRidge);
  drawFilledGeoPolygon(POLY_MTN_CHARYEONG, POLY_MTN_CHARYEONG_LEN, colMountainFill, colMountainRidge);

  radarCanvas.setTextDatum(top_left);
  radarCanvas.setTextSize(1);
  radarCanvas.setTextColor(colMountainRidge);
  for (int i = 0; i < PEAKS_LEN; i++) {
    int px, py;
    geoToScreen(PEAK_COORDS[i][0], PEAK_COORDS[i][1], px, py);
    if (hypot(px - CX, py - CY) <= R - 18) {
      radarCanvas.fillTriangle(px, py - 5, px - 4, py + 3, px + 4, py + 3, colMountainRidge);
      radarCanvas.drawString(PEAK_NAMES[i], px + 6, py - 3);
    }
  }

  drawThickPolyline(RIVER_HAN, RIVER_HAN_LEN, colRiver, true);
  drawThickPolyline(RIVER_IMJIN, RIVER_IMJIN_LEN, colRiver, true);
  drawThickPolyline(RIVER_BUKHAN, RIVER_BUKHAN_LEN, colRiver, false);

  radarCanvas.setTextColor(colCityText);
  for (int i = 0; i < CITIES_LEN; i++) {
    int cx, cy;
    geoToScreen(CITY_COORDS[i][0], CITY_COORDS[i][1], cx, cy);
    if (hypot(cx - CX, cy - CY) <= R - 15) {
      radarCanvas.fillCircle(cx, cy, 3, colCityDot);
      radarCanvas.drawCircle(cx, cy, 5, TFT_WHITE);
      radarCanvas.drawString(CITY_NAMES[i], cx + 7, cy - 3);
    }
  }

  radarCanvas.setTextColor(colAirport);
  for (int i = 0; i < AIRPORTS_LEN; i++) {
    int ax, ay;
    geoToScreen(AIRPORT_COORDS[i][0], AIRPORT_COORDS[i][1], ax, ay);
    if (hypot(ax - CX, ay - CY) <= R - 10) {
      radarCanvas.drawRect(ax - 4, ay - 4, 9, 9, colAirport);
      radarCanvas.drawLine(ax - 4, ay + 4, ax + 4, ay - 4, colAirport);
      radarCanvas.drawString(AIRPORT_CODES[i], ax + 7, ay - 4);
    }
  }
}

void drawSweepLine() {
  for (float i = TRAIL_WIDTH; i >= 0; i -= 1.0) {
    float a = sweepAngle - i;
    if (a < 0) a += 360;

    float intensity = (float)(TRAIL_WIDTH - i) / TRAIL_WIDTH;
    uint8_t g = 40 + intensity * 180;
    uint16_t col = radarCanvas.color565(0, g, 40);

    float rad = (a - 90) * DEG_TO_RAD;
    int x = CX + cos(rad) * R;
    int y = CY + sin(rad) * R;

    if ((int)i % 3 == 0) {
      radarCanvas.drawLine(CX, CY, x, y, col);
    }
  }

  float rad = (sweepAngle - 90) * DEG_TO_RAD;
  int sx = CX + cos(rad) * R;
  int sy = CY + sin(rad) * R;

  radarCanvas.drawLine(CX, CY, sx, sy, greenBright);
  radarCanvas.drawLine(CX + 1, CY, sx + 1, sy, greenBright);
}

void drawRadarCirclesToCanvas() {
  int step = R / 5;
  for (int i = 1; i <= 4; i++) {
    radarCanvas.drawCircle(CX, CY, step * i, greenDim);
  }
  radarCanvas.drawCircle(CX, CY, R, greenBright);

  for (int i = 0; i < OUTER_RING_THICKNESS; i++) {
    radarCanvas.drawCircle(CX, CY, R + OUTER_GAP + 1 + i, greenBright);
  }
}

void drawCrossLinesToCanvas() {
  radarCanvas.drawFastHLine(CX - R, CY, R * 2, greenDim);
  radarCanvas.drawFastVLine(CX, CY - R, R * 2, greenDim);

  for (int d = 22; d < R; d += 44) {
    radarCanvas.drawFastVLine(CX + d, CY - 4, 8, green);
    radarCanvas.drawFastVLine(CX - d, CY - 4, 8, green);

    radarCanvas.drawFastHLine(CX - 4, CY + d, 8, green);
    radarCanvas.drawFastHLine(CX - 4, CY - d, 8, green);
  }
}

void drawDistanceLabelsToCanvas() {
  radarCanvas.setTextDatum(middle_center);
  radarCanvas.setTextSize(1);
  radarCanvas.setTextColor(greenBright);

  int step = R / 5;
  int km[] = {20, 40, 60, 80};

  for (int i = 0; i < 4; i++) {
    int rPos = step * (i + 1);
    radarCanvas.drawString(String(km[i]) + "km", CX + 20, CY - rPos - 8);
    radarCanvas.drawString(String(km[i]) + "km", CX + 20, CY + rPos + 8);
  }
}

void drawAngleScaleToCanvas() {
  for (int a = 0; a < 360; a++) {
    int len = 4;
    if (a % 10 == 0) len = 12;
    else if (a % 5 == 0) len = 7;

    canvasCorrectedRadialLine(a, R - len, R, greenBright);
  }

  radarCanvas.setTextColor(greenBright);
  radarCanvas.setTextSize(1);
  radarCanvas.setTextDatum(middle_center);

  for (int a = 0; a < 360; a += 30) {
    float rad = (a - 90) * DEG_TO_RAD;

    int tx = CX + cos(rad) * (R - 22);
    int ty = CY + sin(rad) * (R - 22);

    radarCanvas.drawNumber(a, tx, ty);
  }
}

// ★ 진행 방위각(headingDeg)에 맞춰 회전하는 항공기 실루엣 아이콘 그리기
void drawAirplaneIcon(int cx, int cy, float headingDeg, float scale, uint16_t color) {
  float rad = headingDeg * DEG_TO_RAD;
  float sinA = sin(rad);
  float cosA = cos(rad);

  // 로컬 좌표 (기수 위쪽 0도 기준: -Y가 전방) -> 화면 회전 변환 람다
  auto rotX = [&](float lx, float ly) -> int {
    return cx + (int)roundf((lx * cosA - ly * sinA) * scale);
  };
  auto rotY = [&](float lx, float ly) -> int {
    return cy + (int)roundf((lx * sinA + ly * cosA) * scale);
  };

  // 동체 (기수 -> 꼬리)
  int noseX = rotX(0, -10.0f);
  int noseY = rotY(0, -10.0f);
  int tailX = rotX(0,  8.5f);
  int tailY = rotY(0,  8.5f);

  // 주익 (좌/우 날개 끝 및 동체 접합부)
  int wingFrontX = rotX(0, -2.5f);
  int wingFrontY = rotY(0, -2.5f);
  int wingBackX  = rotX(0,  2.5f);
  int wingBackY  = rotY(0,  2.5f);
  int wingLX     = rotX(-9.5f, 3.0f);
  int wingLY     = rotY(-9.5f, 3.0f);
  int wingRX     = rotX( 9.5f, 3.0f);
  int wingRY     = rotY( 9.5f, 3.0f);

  // 꼬리날개 (수평미익)
  int tailRootX  = rotX(0, 6.0f);
  int tailRootY  = rotY(0, 6.0f);
  int tailWingLX = rotX(-4.2f, 9.2f);
  int tailWingLY = rotY(-4.2f, 9.2f);
  int tailWingRX = rotX( 4.2f, 9.2f);
  int tailWingRY = rotY( 4.2f, 9.2f);

  // 1. 시인성을 위한 검정색 외곽 테두리(그림자)
  radarCanvas.fillTriangle(wingFrontX, wingFrontY - 1, wingLX - 1, wingLY + 1, wingRX + 1, wingRY + 1, TFT_BLACK);
  radarCanvas.fillTriangle(tailRootX, tailRootY - 1, tailWingLX - 1, tailWingLY + 1, tailWingRX + 1, tailWingRY + 1, TFT_BLACK);

  // 2. 항공기 컬러 실루엣 채우기 (주익 + 꼬리날개 + 동체)
  radarCanvas.fillTriangle(wingFrontX, wingFrontY, wingLX, wingLY, wingBackX, wingBackY, color);
  radarCanvas.fillTriangle(wingFrontX, wingFrontY, wingRX, wingRY, wingBackX, wingBackY, color);
  radarCanvas.fillTriangle(tailRootX, tailRootY, tailWingLX, tailWingLY, tailX, tailY, color);
  radarCanvas.fillTriangle(tailRootX, tailRootY, tailWingRX, tailWingRY, tailX, tailY, color);

  // 굵은 동체 중심선
  radarCanvas.drawLine(noseX, noseY, tailX, tailY, color);
  radarCanvas.drawLine(noseX + 1, noseY, tailX + 1, tailY, color);
  radarCanvas.drawLine(noseX, noseY + 1, tailX, tailY + 1, color);
}

void drawAircraftTargetsToCanvas() {
  for (int i = 0; i < planeCount; i++) {
    if (!planes[i].valid) continue;

    float ageAngle = angleDiffBehindSweep(sweepAngle, planes[i].bearingDeg);
    if (ageAngle > 320) continue;

    float fade = 1.0;
    if (ageAngle > 240) {
      fade = 1.0 - ((ageAngle - 240.0) / 80.0);
      if (fade < 0.40) fade = 0.40;
    }

    float rr = (planes[i].distanceKm / RADAR_RANGE_KM) * R;
    float rad = (planes[i].bearingDeg - 90) * DEG_TO_RAD;

    int x = CX + cos(rad) * rr;
    int y = CY + sin(rad) * rr;

    uint8_t r = 0;
    uint8_t g = 255 * fade;
    uint8_t b = 50 * fade;

    if (planes[i].altitudeFt >= 30000) {
      r = 0; g = 230 * fade; b = 255 * fade;
    } else if (planes[i].altitudeFt >= 10000) {
      r = 50 * fade; g = 255 * fade; b = 50 * fade;
    } else if (planes[i].altitudeFt >= 0) {
      r = 255 * fade; g = 235 * fade; b = 0;
    } else {
      r = 255 * fade; g = 60 * fade; b = 60 * fade;
    }

    uint16_t col = radarCanvas.color565(r, g, b);

    float iconScale = 1.0f;
    if (planes[i].altitudeFt > 30000) iconScale = 1.15f;
    if (planes[i].distanceKm < 25) iconScale = 1.25f;

    float heading = (planes[i].trackDeg >= 0) ? planes[i].trackDeg : 0.0f;

    // 진행 방향 전방 속도 벡터 점선
    if (planes[i].trackDeg >= 0) {
      float tr = (heading - 90) * DEG_TO_RAD;
      int x2 = x + cos(tr) * 18;
      int y2 = y + sin(tr) * 18;
      radarCanvas.drawLine(x, y, x2, y2, col);
    }

    // 회전형 항공기 아이콘 출력
    drawAirplaneIcon(x, y, heading, iconScale, col);

    if (ageAngle >= 0 && ageAngle < 120) {
      radarCanvas.setTextDatum(top_left);
      radarCanvas.setTextSize(1);
      radarCanvas.setTextColor(TFT_WHITE, TFT_BLACK);

      String label = String(planes[i].flight);
      label.trim();
      if (label.length() == 0) label = "UNKNOWN";

      radarCanvas.drawString(label, x + 11, y - 10);

      if (planes[i].altitudeFt >= 0) {
        radarCanvas.setTextColor(TFT_YELLOW, TFT_BLACK);
        radarCanvas.drawString(String(planes[i].altitudeFt / 100) + "FL", x + 11, y + 1);
      }
    }
  }
}

void drawCornerOverlaysToCanvas() {
  // 좌측 상단 상태창
  radarCanvas.fillRect(6, 6, 108, 42, TFT_BLACK);
  radarCanvas.drawRect(6, 6, 108, 42, greenBright);

  radarCanvas.setTextDatum(top_left);
  radarCanvas.setTextSize(2);

  if (apiOk) {
    radarCanvas.setTextColor(green);
    radarCanvas.drawString("ADS-B", 10, 8);

    radarCanvas.setTextColor(greenBright);
    radarCanvas.drawString("AC:" + String(planeCount), 10, 28);
  } else {
    radarCanvas.setTextColor(radarCanvas.color565(255, 80, 80));
    radarCanvas.drawString(apiStatusMsg, 10, 16);
  }

  // 우측 상단 시각 박스
  radarCanvas.fillRect(SCREEN_W - 110, 6, 104, 34, TFT_BLACK);
  radarCanvas.drawRect(SCREEN_W - 110, 6, 104, 34, greenBright);

  radarCanvas.setTextDatum(top_right);
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char timeStr[12];
    sprintf(timeStr, "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    radarCanvas.setTextSize(2);
    radarCanvas.setTextColor(TFT_YELLOW);
    radarCanvas.drawString(timeStr, SCREEN_W - 10, 8);
  } else {
    radarCanvas.setTextSize(2);
    radarCanvas.setTextColor(TFT_YELLOW);
    radarCanvas.drawString("--:--:--", SCREEN_W - 10, 8);
  }
  radarCanvas.setTextSize(1);
  radarCanvas.setTextColor(green);
  radarCanvas.drawString("KST (UTC+9)", SCREEN_W - 10, 28);

  // 좌측 하단 미니 나침반
  int ccx = 36;
  int ccy = 444;
  radarCanvas.fillCircle(ccx, ccy, 25, TFT_BLACK);
  radarCanvas.drawCircle(ccx, ccy, 25, greenDim);
  radarCanvas.setTextDatum(middle_center);
  radarCanvas.drawCircle(ccx, ccy, 14, greenDim);
  radarCanvas.fillTriangle(ccx, ccy - 13, ccx - 4, ccy - 3, ccx + 4, ccy - 3, greenBright);
  radarCanvas.fillTriangle(ccx, ccy + 13, ccx - 4, ccy + 3, ccx + 4, ccy + 3, greenDim);
  radarCanvas.fillCircle(ccx, ccy, 2, greenBright);

  radarCanvas.setTextSize(1);
  radarCanvas.setTextColor(greenBright);
  radarCanvas.drawString("N", ccx, ccy - 20);
  radarCanvas.setTextColor(greenDim);
  radarCanvas.drawString("S", ccx, ccy + 20);
  radarCanvas.drawString("W", ccx - 20, ccy);
  radarCanvas.drawString("E", ccx + 20, ccy);

  // 우측 하단 공항 및 스윕 각도
  radarCanvas.fillRect(SCREEN_W - 114, SCREEN_H - 42, 108, 36, TFT_BLACK);
  radarCanvas.drawRect(SCREEN_W - 114, SCREEN_H - 42, 108, 36, greenBright);

  radarCanvas.setTextDatum(bottom_right);
  radarCanvas.setTextSize(2);
  radarCanvas.setTextColor(greenBright);
  radarCanvas.drawString(LOCATION_NAME, SCREEN_W - 10, SCREEN_H - 22);

  radarCanvas.setTextSize(1);
  radarCanvas.setTextColor(TFT_YELLOW);
  char buf[24];
  sprintf(buf, "SWEEP %03d DEG", (int)sweepAngle);
  radarCanvas.drawString(buf, SCREEN_W - 10, SCREEN_H - 8);
}

void drawRadarFrameToCanvas() {
  drawVividTerrainMap();
  drawSweepLine();
  drawRadarCirclesToCanvas();
  drawCrossLinesToCanvas();
  drawDistanceLabelsToCanvas();
  drawAngleScaleToCanvas();
  drawAircraftTargetsToCanvas();
  drawCornerOverlaysToCanvas();
  radarCanvas.drawRect(1, 1, SCREEN_W - 2, SCREEN_H - 2, greenBright);
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32-4848S040 Incheon (RKSI) ADS-B Radar Boot ===");

  st7701_init_panel();

  lcd.init();
  lcd.setBrightness(220);

  greenBright = lcd.color565(0, 255, 60);
  green       = lcd.color565(0, 200, 50);
  greenDim    = lcd.color565(20, 120, 55);
  greenDark   = lcd.color565(15, 60, 40);

  colSeaBg         = lcd.color565(12, 55, 115);   // 서해 바다
  colLandFill      = lcd.color565(38, 82, 42);    // 수도권 육지
  colCoastLine     = lcd.color565(70, 220, 200);  // 서해안 해안선
  colMountainFill  = lcd.color565(135, 92, 45);   // 산악 지대
  colMountainRidge = lcd.color565(215, 165, 95);  // 산맥 능선 및 봉우리
  colRiver         = lcd.color565(60, 190, 255);  // 한강/임진강/북한강
  colCityDot       = lcd.color565(255, 70, 170);  // 주요 도시 마커
  colCityText      = lcd.color565(245, 245, 255); // 도시 이름
  colAirport       = lcd.color565(255, 210, 0);   // 공항 마커

  radarCanvas.setPsram(true);
  radarCanvas.setColorDepth(16);
  radarCanvas.setSwapBytes(false);

  if (!radarCanvas.createSprite(480, 480)) {
    Serial.println("Radar sprite create failed!");
  }

  lcd.fillScreen(TFT_BLACK);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.setTextDatum(middle_center);
  lcd.drawString("Connecting WiFi...", 240, 240);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  lcd.fillScreen(TFT_BLACK);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected! IP: ");
    Serial.println(WiFi.localIP());

    configTzTime("KST-9", "pool.ntp.org", "time.nist.gov");

    fetchAircraftData();
    applyPendingAircraftData();
    lastApiFetchTime = millis();
  } else {
    Serial.println("WiFi failed!");
    apiStatusMsg = "NO WIFI";
    apiOk = false;
  }

  drawRadarFrameToCanvas();
  radarCanvas.pushSprite(0, 0);
}

// ---------- LOOP ----------
void loop() {
  if (millis() - lastSweepUpdate > SWEEP_SPEED) {
    lastSweepUpdate = millis();

    drawRadarFrameToCanvas();
    radarCanvas.pushSprite(0, 0);

    sweepAngle += 2;

    if (sweepAngle >= 360) {
      sweepAngle -= 360;

      applyPendingAircraftData();

      if (millis() - lastApiFetchTime > API_FETCH_INTERVAL) {
        lastApiFetchTime = millis();
        startAircraftFetchBackground();
      }
    }
  }
}