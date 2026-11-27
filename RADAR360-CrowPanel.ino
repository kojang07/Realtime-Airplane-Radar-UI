/*
 * ============================================================================
 *  RADAR 360 - CrowPanel 7.0" HMI ESP32-S3 (800x480 RGB + LovyanGFX GT911)
 *  Interactive Region Picker + Aircraft Visual Detail Card + Vector Map
 * ============================================================================
 *  [Arduino IDE 권장 설정]
 *   - Board            : ESP32S3 Dev Module
 *   - USB CDC On Boot  : Disabled (UART0 포트 사용 시 시리얼 모니터 출력)
 *   - Flash Size       : 4MB (32Mb) 또는 16MB (128Mb)
 *   - Flash Mode       : QIO 80MHz
 *   - PSRAM            : OPI PSRAM (필수 활성화)
 *   - Partition Scheme : Huge APP (3MB No OTA / 1MB SPIFFS)
 * ============================================================================
 */

#include <Wire.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <esp_heap_caps.h>
#include <math.h>
#include "time.h"

// ==========================================
// 0. 열거형(Enum) 및 구조체(Struct) 최상단 정의
// ==========================================
enum AcVisualType {
  AC_VIS_JET2 = 0,   // 일반 쌍발 제트 여객기 (B737, A320 등)
  AC_VIS_WIDE2,      // 광동체 대형 쌍발기 (B777, B787, A350, A330 등)
  AC_VIS_HEAVY4,     // 4발 초대형기 (B747, A380 등)
  AC_VIS_PROP,       // 터보프롭 / 경비행기 (DH8D, AT76, C172 등)
  AC_VIS_HELI        // 헬리콥터
};

struct RadarConfig {
  // Network
  String ssid       = "";
  String pass       = "";
  String hostname   = "radar";

  // Radar & Vector Map
  String presetName = "BUSAN";
  double lat        = 35.1795;
  double lon        = 128.9382;
  float  radiusKm   = 100.0f;
  int    refreshSec = 10;
  int    minAltFt   = 0;
  int    maxAltFt   = 60000;
  int    maxAc      = 24;
  int    mapStyle   = 0;      // 0: Vivid Color Vector, 1: Tactical Green HUD, 2: Midnight Dark

  int    unitDist   = 0;      // 0: km, 1: nm
  int    unitAlt    = 1;      // 0: m,  1: ft
  int    unitSpeed  = 1;      // 0: km/h, 1: kt
  int    viewMode   = 0;      // 0: Flight radar, 1: Weather view, 2: Clock view

  bool   showMapBg     = true;
  bool   showScanLine  = true;
  bool   showCallsign  = true;
  bool   showAltitude  = true;
  bool   showHeading   = false;
  bool   showSpeed     = true;
  bool   showDistance  = false;
  bool   showAcType    = true;
  bool   showVectors   = true;

  // Weather
  bool   wxUseRadarLoc = true;
  double wxLat         = 35.1795;
  double wxLon         = 128.9382;
  int    unitTemp      = 0;   // 0: Celsius, 1: Fahrenheit
  int    unitPrecip    = 0;   // 0: mm, 1: in
  bool   wxShowCards   = true;
  bool   wxFeelsLike   = true;
  bool   wxPrecip      = true;
  bool   wxHumidity    = true;
  bool   wxWind        = true;
  bool   wxPressure    = true;

  // Clock
  int    clockStyle    = 0;   // 0:Rolex, 1:Longines, 2:Breitling, 3:Omega, 4:DigModern, 5:DigNeon, 6:DigFlip
  String tzPosix       = "KST-9";
  String tzLabel       = "SEOUL (UTC+9)";

  // System
  int    brightness    = 80;
};

struct WeatherData {
  bool   valid       = false;
  float  tempC       = 0.0f;
  float  feelsLikeC  = 0.0f;
  int    humidity    = 0;
  int    cloudCover  = 0;
  float  precipMm    = 0.0f;
  float  windKmh     = 0.0f;
  float  gustKmh     = 0.0f;
  int    windDir     = 0;
  float  pressureHpa = 1013.0f;
  int    wmoCode     = 0;
  int    dayCode[3]  = {0, 0, 0};
  float  dayMaxC[3]  = {0, 0, 0};
  float  dayMinC[3]  = {0, 0, 0};
  int    dayPop[3]   = {0, 0, 0};
};

struct AirportInfo {
  const char* key;
  const char* title;
  const char* code;
  double lat;
  double lon;
  const char* elev;
  const char* freq;
};

struct GeoPoint {
  float lat;
  float lon;
};

struct WaypointFix {
  float lat;
  float lon;
  const char* code;
};

struct MapLabel {
  float lat;
  float lon;
  const char* name;
  bool isSea;
};

struct AircraftPoint {
  bool valid;
  double lat;
  double lon;
  float distanceKm;
  float bearingDeg;
  int altitudeFt;
  float speedKt;
  float trackDeg;
  char hex[8];
  char flight[12];
  char category[5];
  char typeCode[8];
};

// ==========================================
// 1. CrowPanel 7.0" (800x480) LovyanGFX RGB + GT911 터치 설정
// ==========================================
class LGFX : public lgfx::LGFX_Device {
public:
  lgfx::Bus_RGB     _bus;
  lgfx::Panel_RGB   _panel;
  lgfx::Light_PWM   _light;
  lgfx::Touch_GT911 _touch;

  LGFX(void) {
    {
      auto c = _panel.config();
      c.memory_width  = 800;
      c.memory_height = 480;
      c.panel_width   = 800;
      c.panel_height  = 480;
      c.offset_x      = 0;
      c.offset_y      = 0;
      _panel.config(c);
    }

    {
      auto c = _bus.config();
      c.panel = &_panel;

      c.pin_d0  = GPIO_NUM_15;
      c.pin_d1  = GPIO_NUM_7;
      c.pin_d2  = GPIO_NUM_6;
      c.pin_d3  = GPIO_NUM_5;
      c.pin_d4  = GPIO_NUM_4;
      c.pin_d5  = GPIO_NUM_9;
      c.pin_d6  = GPIO_NUM_46;
      c.pin_d7  = GPIO_NUM_3;
      c.pin_d8  = GPIO_NUM_8;
      c.pin_d9  = GPIO_NUM_16;
      c.pin_d10 = GPIO_NUM_1;
      c.pin_d11 = GPIO_NUM_14;
      c.pin_d12 = GPIO_NUM_21;
      c.pin_d13 = GPIO_NUM_47;
      c.pin_d14 = GPIO_NUM_48;
      c.pin_d15 = GPIO_NUM_45;

      c.pin_henable = GPIO_NUM_41;
      c.pin_vsync   = GPIO_NUM_40;
      c.pin_hsync   = GPIO_NUM_39;
      c.pin_pclk    = GPIO_NUM_0;

      // 화면 흔들림 방지를 위한 10MHz PCLK 및 동기 극성 고정
      c.freq_write = 10000000;

      c.hsync_polarity    = 0;
      c.hsync_front_porch = 40;
      c.hsync_pulse_width = 48;
      c.hsync_back_porch  = 40;

      c.vsync_polarity    = 0;
      c.vsync_front_porch = 1;
      c.vsync_pulse_width = 31;
      c.vsync_back_porch  = 13;

      c.pclk_active_neg = 1;
      c.de_idle_high    = 0;
      c.pclk_idle_high  = 1;

      _bus.config(c);
      _panel.setBus(&_bus);
    }

    {
      auto c = _light.config();
      c.pin_bl      = GPIO_NUM_2;
      c.freq        = 44100;
      c.pwm_channel = 7;
      _light.config(c);
      _panel.setLight(&_light);
    }

    {
      auto c = _touch.config();
      c.x_min           = 0;
      c.x_max           = 799;
      c.y_min           = 0;
      c.y_max           = 479;
      c.pin_int         = -1;
      c.pin_rst         = -1;
      c.bus_shared      = true;
      c.offset_rotation = 0;
      c.i2c_port        = 0;
      c.i2c_addr        = 0x14;
      c.pin_sda         = GPIO_NUM_19;
      c.pin_scl         = GPIO_NUM_20;
      c.freq            = 100000;
      _touch.config(c);
      _panel.setTouch(&_touch);
    }

    setPanel(&_panel);
  }
};

// ==========================================
// 2. 전역 객체 및 상수 정의
// ==========================================
const char* AP_SSID      = "RADAR-360-Setup";
const char* DEFAULT_SSID = "";   // 비워두면 웹 설정 모드(AP) 자동 실행, 직접 입력 시 자동 접속
const char* DEFAULT_PASS = "";

const int MAP_SIZE = 420;
const int MAP_HALF = MAP_SIZE / 2; // 210

LGFX lcd;
LGFX_Sprite wideCanvas(&lcd);    // 800x480 전체 화면 더블 버퍼
LGFX_Sprite mapSprite(&lcd);     // 420x420 사전 렌더링 벡터 지도 버퍼

RadarConfig cfg;
WeatherData wxData, pendingWxData;
bool pendingWxReady = false, wxFetchInProgress = false;
unsigned long lastWxFetchTime = 0;
const unsigned long WX_FETCH_INTERVAL = 600000UL;

Preferences preferences;
WebServer server(80);
DNSServer dnsServer;
bool isApMode = false;

volatile bool netBusy = false;

#if ARDUINOJSON_VERSION_MAJOR >= 7
struct SpiRamAllocator : ArduinoJson::Allocator {
  void* allocate(size_t size) override { return heap_caps_malloc(size, MALLOC_CAP_SPIRAM); }
  void deallocate(void* pointer) override { heap_caps_free(pointer); }
  void* reallocate(void* ptr, size_t new_size) override { return heap_caps_realloc(ptr, new_size, MALLOC_CAP_SPIRAM); }
};
#else
struct SpiRamAllocator {
  void* allocate(size_t size) { return heap_caps_malloc(size, MALLOC_CAP_SPIRAM); }
  void deallocate(void* pointer) { heap_caps_free(pointer); }
  void* reallocate(void* ptr, size_t new_size) { return heap_caps_realloc(ptr, new_size, MALLOC_CAP_SPIRAM); }
};
using SpiRamJsonDocument = BasicJsonDocument<SpiRamAllocator>;
#endif

// ---------- LAYOUT ----------
const int SCREEN_W = 800;
const int SCREEN_H = 480;

const int INFO_X = 482;
const int INFO_Y = 6;
const int INFO_W = 312;
const int INFO_H = 468;

const int CX = 238;   // 좌측 원형 레이더 중심 X
const int CY = 240;   // 좌측 원형 레이더 중심 Y
const int R  = 204;   // 좌측 원형 레이더 반지름

// ---------- SWEEP ----------
float sweepAngle = 0;
unsigned long lastSweepUpdate = 0;
unsigned long lastApiFetchTime = 0;
const int TRAIL_WIDTH = 50;
const int SWEEP_SPEED = 70;

// ---------- COLORS ----------
uint16_t greenBright, green, greenDim, greenDark;
uint16_t colSea, colLand, colCoast, colRiver, colRoad;
uint16_t colCityDot, colCityText, colAirway, colWaypoint;
uint16_t colAirport, colCardBg, colWxPanel, colWxBorder, colHudCyan, colSelBg;

// ---------- 터치 및 팝업 상태 관리 ----------
bool wasTouched = false;
unsigned long lastTouchPoll = 0;
unsigned long lastTouchAction = 0;
int lastTouchX = -1, lastTouchY = -1;

int selectedPlaneIdx = -1;
String selectedPlaneHex = "";
bool regionModalOpen = false;

// ==========================================
// 3. 모든 함수 선행 선언 (Arduino IDE 컴파일 안전 보장)
// ==========================================
const AirportInfo* getCurrentAirportInfo();
void loadConfig();
void saveConfig();
double deg2radD(double d);
double rad2degD(double r);
float angleDiffBehindSweep(float s, float b);
float distanceKm(double lat1, double lon1, double lat2, double lon2);
float bearingDeg(double lat1, double lon1, double lat2, double lon2);
void geoToRadarXY(double lat, double lon, int &outX, int &outY);
void geoToMapSpriteXY(double lat, double lon, int &outX, int &outY);
void applyVectorMapThemeColors();
void fillGeoPolygonOnSprite(LGFX_Sprite &spr, const GeoPoint* pts, int len, uint16_t fillCol, uint16_t strokeCol, int strokeCount);
void drawPolylineOnSprite(LGFX_Sprite &spr, const GeoPoint* pts, int len, uint16_t col, int thickness = 1);
void drawDashedAirwayOnSprite(LGFX_Sprite &spr, const GeoPoint* pts, int len, uint16_t col);
void buildVectorMapSprite();
void setupWebServer();
void runApSetupPortal();
const char* degToCompass(int deg);
void fetchWeatherData();
void weatherFetchTask(void *parameter);
void startWeatherFetchBackground();
void applyPendingWeatherData();
bool tryFetchFromUrl(const String &url);
void fetchAircraftData();
void aircraftFetchTask(void *parameter);
void startAircraftFetchBackground();
void applyPendingAircraftData();
const char* getAirlineName(const char* flight);
AcVisualType classifyAircraftVisual(const char* typeCode, const char* category);
void canvasCorrectedRadialLine(int angleDeg, int r1, int r2, uint16_t col);
void rotPt(int cx, int cy, float cosA, float sinA, float lx, float ly, int &ox, int &oy);
void drawAirplaneIcon(int cx, int cy, float headingDeg, float scale, uint16_t fillCol);
void drawAircraftIllustrationBox(int bx, int by, int bw, int bh, const AircraftPoint &p);
void drawBottomRightSelectedAircraftPanel();
void drawRightPanelsToCanvas();
void drawRegionPickerModal();
void drawFlightRadarToCanvas();
void drawThickHand(int cx, int cy, float angleDeg, int len, int tailLen, int halfW, uint16_t col);
void drawClockViewToCanvas();
void drawWeatherViewToCanvas();
void drawMainFrame();
void switchViewMode(int newMode);
void selectAirportByIndex(int idx);
void handleTouchTap(int tx, int ty);

// ==========================================
// 4. 내장 벡터 지형도 + 공항/항로 데이터
// ==========================================
const AirportInfo AIRPORTS[] = {
  {"BUSAN",     "GIMHAE (RKPK)",    "RKPK", 35.1795, 128.9382, "4m / 13ft",    "118.10 MHz"},
  {"INCHEON",   "INCHEON (RKSI)",   "RKSI", 37.4602, 126.4407, "7m / 23ft",    "118.20 MHz"},
  {"JEJU",      "JEJU (RKPC)",      "RKPC", 33.5113, 126.4930, "36m / 118ft",  "118.20 MHz"},
  {"NARITA",    "NARITA (RJAA)",    "RJAA", 35.7720, 140.3929, "41m / 135ft",  "118.35 MHz"},
  {"HANEDA",    "HANEDA (RJTT)",    "RJTT", 35.5494, 139.7798, "6m / 21ft",    "118.10 MHz"},
  {"SINGAPORE", "CHANGI (WSSS)",    "WSSS",  1.3644, 103.9915, "7m / 22ft",    "118.60 MHz"},
  {"HONGKONG",  "HONG KONG (VHHH)", "VHHH", 22.3080, 113.9185, "9m / 28ft",    "118.40 MHz"},
  {"BANGKOK",   "BANGKOK (VTBS)",   "VTBS", 13.6900, 100.7501, "2m / 5ft",     "119.00 MHz"},
  {"DUBAI",     "DUBAI (OMDB)",     "OMDB", 25.2532,  55.3657, "19m / 62ft",   "118.75 MHz"},
  {"SYDNEY",    "SYDNEY (YSSY)",    "YSSY",-33.9399, 151.1753, "6m / 21ft",    "120.50 MHz"},
  {"LONDON",    "HEATHROW (EGLL)",  "EGLL", 51.4700,  -0.4543, "25m / 83ft",   "118.50 MHz"},
  {"PARIS",     "CDG PARIS (LFPG)", "LFPG", 49.0097,   2.5479, "119m / 392ft", "119.25 MHz"},
  {"FRANKFURT", "FRANKFURT (EDDF)", "EDDF", 50.0379,   8.5622, "111m / 364ft", "119.90 MHz"},
  {"NEWYORK",   "JFK NY (KJFK)",    "KJFK", 40.6413, -73.7781, "4m / 13ft",    "119.10 MHz"},
  {"LA",        "LAX (KLAX)",       "KLAX", 33.9416,-118.4085, "38m / 125ft",  "120.95 MHz"}
};
const int AIRPORTS_LEN = sizeof(AIRPORTS) / sizeof(AIRPORTS[0]);

const AirportInfo* getCurrentAirportInfo() {
  for (int i = 0; i < AIRPORTS_LEN; i++) {
    if (cfg.presetName.equalsIgnoreCase(AIRPORTS[i].key)) return &AIRPORTS[i];
  }
  return nullptr;
}

const GeoPoint POLY_MAINLAND[] = {
  {34.45, 126.30}, {34.74, 127.74}, {34.85, 127.88}, {34.93, 128.07}, {34.84, 128.20},
  {34.83, 128.42}, {34.96, 128.45}, {35.02, 128.57}, {35.18, 128.57},
  {35.10, 128.75}, {35.02, 128.83}, {35.08, 128.96}, {35.05, 129.03},
  {35.10, 129.04}, {35.15, 129.12}, {35.16, 129.18}, {35.25, 129.22},
  {35.36, 129.32}, {35.48, 129.42}, {35.53, 129.37}, {35.62, 129.44},
  {35.80, 129.51}, {36.02, 129.58}, {37.20, 129.35}, {38.30, 128.60},
  {38.30, 126.10}, {37.75, 126.15}, {37.60, 126.55}, {37.35, 126.65},
  {37.00, 126.40}, {36.50, 126.30}, {35.60, 126.45}, {34.45, 126.30}
};
const int POLY_MAINLAND_LEN = sizeof(POLY_MAINLAND) / sizeof(GeoPoint);

const GeoPoint POLY_GEOJE[] = {
  {34.99, 128.63}, {34.93, 128.72}, {34.80, 128.70}, {34.73, 128.62},
  {34.82, 128.52}, {34.94, 128.55}
};
const int POLY_GEOJE_LEN = sizeof(POLY_GEOJE) / sizeof(GeoPoint);

const GeoPoint POLY_NAMHAE[] = {
  {34.88, 127.88}, {34.84, 128.02}, {34.72, 128.00}, {34.74, 127.85}
};
const int POLY_NAMHAE_LEN = sizeof(POLY_NAMHAE) / sizeof(GeoPoint);

const GeoPoint POLY_TSUSHIMA[] = {
  {34.70, 129.45}, {34.55, 129.42}, {34.38, 129.36}, {34.25, 129.28},
  {34.10, 129.22}, {34.14, 129.16}, {34.33, 129.22}, {34.48, 129.31},
  {34.67, 129.36}
};
const int POLY_TSUSHIMA_LEN = sizeof(POLY_TSUSHIMA) / sizeof(GeoPoint);

const GeoPoint POLY_JEJU[] = {
  {33.56, 126.53}, {33.52, 126.88}, {33.38, 126.91}, {33.24, 126.56},
  {33.28, 126.18}, {33.44, 126.24}
};
const int POLY_JEJU_LEN = sizeof(POLY_JEJU) / sizeof(GeoPoint);

const GeoPoint POLY_YEONGJONG[] = {
  {37.52, 126.42}, {37.50, 126.56}, {37.43, 126.52}, {37.43, 126.39}
};
const int POLY_YEONGJONG_LEN = sizeof(POLY_YEONGJONG) / sizeof(GeoPoint);

const GeoPoint RIVER_NAKDONG[] = {
  {36.05, 128.40}, {35.80, 128.45}, {35.55, 128.50}, {35.40, 128.65},
  {35.38, 128.88}, {35.28, 128.98}, {35.18, 128.96}, {35.06, 128.94}
};
const int RIVER_NAKDONG_LEN = sizeof(RIVER_NAKDONG) / sizeof(GeoPoint);

const GeoPoint RIVER_HAN[] = {
  {37.52, 127.30}, {37.55, 127.05}, {37.53, 126.92}, {37.62, 126.72}, {37.72, 126.58}
};
const int RIVER_HAN_LEN = sizeof(RIVER_HAN) / sizeof(GeoPoint);

const GeoPoint ROAD_GYEONGBU[] = {
  {35.12, 129.04}, {35.34, 129.04}, {35.54, 129.26}, {35.84, 129.21}, {35.87, 128.60}
};
const int ROAD_GYEONGBU_LEN = sizeof(ROAD_GYEONGBU) / sizeof(GeoPoint);

const GeoPoint ROAD_NAMHAE[] = {
  {35.18, 128.96}, {35.23, 128.68}, {35.18, 128.11}
};
const int ROAD_NAMHAE_LEN = sizeof(ROAD_NAMHAE) / sizeof(GeoPoint);

const GeoPoint ROUTE_KADOL[] = {{34.55, 128.25}, {34.82, 128.62}, {35.01, 128.88}, {35.1795, 128.9382}};
const GeoPoint ROUTE_DOTOL[] = {{35.85, 128.32}, {35.54, 128.62}, {35.34, 128.88}, {35.1795, 128.9382}};
const GeoPoint ROUTE_OLMEN[] = {{34.58, 129.62}, {34.88, 129.28}, {35.06, 129.05}, {35.1795, 128.9382}};
const GeoPoint ROUTE_EAST[]  = {{35.98, 129.42}, {35.59, 129.35}, {35.34, 129.12}, {35.1795, 128.9382}};

const WaypointFix RKPK_WAYPOINTS[] = {
  {34.55, 128.25, "KADOL"}, {34.82, 128.62, "TOPAX"},
  {35.85, 128.32, "DOTOL"}, {35.54, 128.62, "DALSU"},
  {34.58, 129.62, "OLMEN"}, {34.88, 129.28, "KAMIT"},
  {35.59, 129.35, "USN"}
};
const int RKPK_WAYPOINTS_LEN = sizeof(RKPK_WAYPOINTS) / sizeof(WaypointFix);

const MapLabel MAP_LABELS[] = {
  {35.16, 129.06, "BUSAN", false},
  {35.54, 129.31, "ULSAN", false},
  {35.22, 128.68, "CHANGWON", false},
  {34.88, 128.62, "GEOJE", false},
  {35.87, 128.60, "DAEGU", false},
  {35.84, 129.21, "GYEONGJU", false},
  {35.18, 128.11, "JINJU", false},
  {34.42, 129.33, "TSUSHIMA", false},
  {34.65, 129.05, "KOREA STRAIT", true},
  {37.56, 126.98, "SEOUL", false},
  {37.45, 126.70, "INCHEON", false},
  {37.26, 127.02, "SUWON", false},
  {33.49, 126.53, "JEJU CITY", false},
  {33.25, 126.56, "SEOGWIPO", false}
};
const int MAP_LABELS_LEN = sizeof(MAP_LABELS) / sizeof(MapLabel);

// ==========================================
// 5. NVS 설정 불러오기 / 저장
// ==========================================
void loadConfig() {
  preferences.begin("radar360", true);
  cfg.ssid          = preferences.getString("ssid", DEFAULT_SSID);
  cfg.pass          = preferences.getString("pass", DEFAULT_PASS);
  cfg.hostname      = preferences.getString("host", "radar");
  cfg.presetName    = preferences.getString("preset", "BUSAN");
  cfg.lat           = preferences.getDouble("lat", 35.1795);
  cfg.lon           = preferences.getDouble("lon", 128.9382);
  cfg.radiusKm      = preferences.getFloat("radKm", 100.0f);
  cfg.refreshSec    = preferences.getInt("refSec", 10);
  cfg.minAltFt      = preferences.getInt("minAlt", 0);
  cfg.maxAltFt      = preferences.getInt("maxAlt", 60000);
  cfg.maxAc         = preferences.getInt("maxAc", 24);
  cfg.mapStyle      = preferences.getInt("mapSty", 0);
  cfg.unitDist      = preferences.getInt("uDist", 0);
  cfg.unitAlt       = preferences.getInt("uAlt", 1);
  cfg.unitSpeed     = preferences.getInt("uSpd", 1);
  cfg.viewMode      = preferences.getInt("viewM", 0);
  cfg.showMapBg     = preferences.getBool("sMap", true);
  cfg.showScanLine  = preferences.getBool("sScan", true);
  cfg.showCallsign  = preferences.getBool("sCall", true);
  cfg.showAltitude  = preferences.getBool("sAlt", true);
  cfg.showHeading   = preferences.getBool("sHdg", false);
  cfg.showSpeed     = preferences.getBool("sSpd", true);
  cfg.showDistance  = preferences.getBool("sDst", false);
  cfg.showAcType    = preferences.getBool("sTyp", true);
  cfg.showVectors   = preferences.getBool("sVec", true);

  cfg.wxUseRadarLoc = preferences.getBool("wxUseR", true);
  cfg.wxLat         = preferences.getDouble("wxLat", 35.1795);
  cfg.wxLon         = preferences.getDouble("wxLon", 128.9382);
  cfg.unitTemp      = preferences.getInt("uTmp", 0);
  cfg.unitPrecip    = preferences.getInt("uPrc", 0);
  cfg.wxShowCards   = preferences.getBool("wxCard", true);
  cfg.wxFeelsLike   = preferences.getBool("wxFeel", true);
  cfg.wxPrecip      = preferences.getBool("wxPrc", true);
  cfg.wxHumidity    = preferences.getBool("wxHum", true);
  cfg.wxWind        = preferences.getBool("wxWnd", true);
  cfg.wxPressure    = preferences.getBool("wxPrs", true);

  cfg.clockStyle    = preferences.getInt("clkSty", 0);
  cfg.tzPosix       = preferences.getString("tzPos", "KST-9");
  cfg.tzLabel       = preferences.getString("tzLbl", "SEOUL (UTC+9)");

  cfg.brightness    = preferences.getInt("brt", 80);
  preferences.end();
}

void saveConfig() {
  preferences.begin("radar360", false);
  preferences.putString("ssid", cfg.ssid);
  preferences.putString("pass", cfg.pass);
  preferences.putString("host", cfg.hostname);
  preferences.putString("preset", cfg.presetName);
  preferences.putDouble("lat", cfg.lat);
  preferences.putDouble("lon", cfg.lon);
  preferences.putFloat("radKm", cfg.radiusKm);
  preferences.putInt("refSec", cfg.refreshSec);
  preferences.putInt("minAlt", cfg.minAltFt);
  preferences.putInt("maxAlt", cfg.maxAltFt);
  preferences.putInt("maxAc", cfg.maxAc);
  preferences.putInt("mapSty", cfg.mapStyle);
  preferences.putInt("uDist", cfg.unitDist);
  preferences.putInt("uAlt", cfg.unitAlt);
  preferences.putInt("uSpd", cfg.unitSpeed);
  preferences.putInt("viewM", cfg.viewMode);
  preferences.putBool("sMap", cfg.showMapBg);
  preferences.putBool("sScan", cfg.showScanLine);
  preferences.putBool("sCall", cfg.showCallsign);
  preferences.putBool("sAlt", cfg.showAltitude);
  preferences.putBool("sHdg", cfg.showHeading);
  preferences.putBool("sSpd", cfg.showSpeed);
  preferences.putBool("sDst", cfg.showDistance);
  preferences.putBool("sTyp", cfg.showAcType);
  preferences.putBool("sVec", cfg.showVectors);

  preferences.putBool("wxUseR", cfg.wxUseRadarLoc);
  preferences.putDouble("wxLat", cfg.wxLat);
  preferences.putDouble("wxLon", cfg.wxLon);
  preferences.putInt("uTmp", cfg.unitTemp);
  preferences.putInt("uPrc", cfg.unitPrecip);
  preferences.putBool("wxCard", cfg.wxShowCards);
  preferences.putBool("wxFeel", cfg.wxFeelsLike);
  preferences.putBool("wxPrc", cfg.wxPrecip);
  preferences.putBool("wxHum", cfg.wxHumidity);
  preferences.putBool("wxWnd", cfg.wxWind);
  preferences.putBool("wxPrs", cfg.wxPressure);

  preferences.putInt("clkSty", cfg.clockStyle);
  preferences.putString("tzPos", cfg.tzPosix);
  preferences.putString("tzLbl", cfg.tzLabel);

  preferences.putInt("brt", cfg.brightness);
  preferences.end();
}

// ==========================================
// 6. 좌표 변환 및 초경량 내장 벡터 지도 렌더링 (420x420)
// ==========================================
double deg2radD(double d) { return d * M_PI / 180.0; }
double rad2degD(double r) { return r * 180.0 / M_PI; }

float angleDiffBehindSweep(float s, float b) {
  float d = s - b; while (d < 0) d += 360; while (d >= 360) d -= 360; return d;
}

float distanceKm(double lat1, double lon1, double lat2, double lon2) {
  double dLat = deg2radD(lat2 - lat1), dLon = deg2radD(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(deg2radD(lat1)) * cos(deg2radD(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  return 6371.0 * (2 * atan2(sqrt(a), sqrt(1 - a)));
}

float bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double y = sin(deg2radD(lon2 - lon1)) * cos(deg2radD(lat2));
  double x = cos(deg2radD(lat1)) * sin(deg2radD(lat2)) -
             sin(deg2radD(lat1)) * cos(deg2radD(lat2)) * cos(deg2radD(lon2 - lon1));
  double b = rad2degD(atan2(y, x));
  return (b < 0) ? b + 360.0 : b;
}

void geoToRadarXY(double lat, double lon, int &outX, int &outY) {
  float d = distanceKm(cfg.lat, cfg.lon, lat, lon);
  float brng = bearingDeg(cfg.lat, cfg.lon, lat, lon);
  float rr = (d / cfg.radiusKm) * R;
  float rad = (brng - 90.0f) * DEG_TO_RAD;
  outX = CX + (int)roundf(cosf(rad) * rr);
  outY = CY + (int)roundf(sinf(rad) * rr);
}

void geoToMapSpriteXY(double lat, double lon, int &outX, int &outY) {
  int rx, ry;
  geoToRadarXY(lat, lon, rx, ry);
  outX = rx - (CX - MAP_HALF);
  outY = ry - (CY - MAP_HALF);
}

void applyVectorMapThemeColors() {
  if (cfg.mapStyle == 1) {
    colSea      = lcd.color565(4, 18, 12);
    colLand     = lcd.color565(12, 38, 22);
    colCoast    = lcd.color565(0, 210, 80);
    colRiver    = lcd.color565(0, 140, 90);
    colRoad     = lcd.color565(25, 85, 45);
    colCityDot  = lcd.color565(0, 255, 80);
    colCityText = lcd.color565(160, 255, 190);
    colAirway   = lcd.color565(0, 190, 130);
    colWaypoint = lcd.color565(120, 255, 200);
  } else if (cfg.mapStyle == 2) {
    colSea      = lcd.color565(6, 10, 14);
    colLand     = lcd.color565(22, 28, 34);
    colCoast    = lcd.color565(75, 115, 140);
    colRiver    = lcd.color565(45, 85, 115);
    colRoad     = lcd.color565(55, 62, 70);
    colCityDot  = lcd.color565(180, 200, 215);
    colCityText = lcd.color565(175, 195, 210);
    colAirway   = lcd.color565(80, 150, 190);
    colWaypoint = lcd.color565(130, 190, 230);
  } else {
    colSea      = lcd.color565(10, 38, 78);
    colLand     = lcd.color565(28, 62, 38);
    colCoast    = lcd.color565(65, 205, 175);
    colRiver    = lcd.color565(35, 145, 235);
    colRoad     = lcd.color565(145, 110, 55);
    colCityDot  = lcd.color565(255, 195, 65);
    colCityText = lcd.color565(235, 245, 220);
    colAirway   = lcd.color565(110, 215, 255);
    colWaypoint = lcd.color565(180, 240, 255);
  }
}

void fillGeoPolygonOnSprite(LGFX_Sprite &spr, const GeoPoint* pts, int len, uint16_t fillCol, uint16_t strokeCol, int strokeCount) {
  int px[40], py[40];
  if (len > 40) len = 40;

  int minY = MAP_SIZE, maxY = 0;
  for (int i = 0; i < len; i++) {
    geoToMapSpriteXY(pts[i].lat, pts[i].lon, px[i], py[i]);
    if (py[i] < minY) minY = py[i];
    if (py[i] > maxY) maxY = py[i];
  }
  minY = max(0, minY);
  maxY = min(MAP_SIZE - 1, maxY);

  int nodeX[40];
  for (int y = minY; y <= maxY; y++) {
    int nodes = 0;
    int j = len - 1;
    for (int i = 0; i < len; i++) {
      if ((py[i] < y && py[j] >= y) || (py[j] < y && py[i] >= y)) {
        nodeX[nodes++] = (int)(px[i] + (float)(y - py[i]) / (py[j] - py[i]) * (px[j] - px[i]));
      }
      j = i;
    }
    for (int i = 0; i < nodes - 1; i++) {
      for (int k = i + 1; k < nodes; k++) {
        if (nodeX[i] > nodeX[k]) {
          int tmp = nodeX[i]; nodeX[i] = nodeX[k]; nodeX[k] = tmp;
        }
      }
    }
    for (int i = 0; i < nodes; i += 2) {
      if (i + 1 >= nodes) break;
      int xStart = max(0, nodeX[i]);
      int xEnd   = min(MAP_SIZE - 1, nodeX[i + 1]);
      if (xEnd > xStart) spr.drawFastHLine(xStart, y, xEnd - xStart + 1, fillCol);
    }
  }
  for (int i = 0; i < strokeCount; i++) {
    spr.drawLine(px[i], py[i], px[i + 1], py[i + 1], strokeCol);
  }
}

void drawPolylineOnSprite(LGFX_Sprite &spr, const GeoPoint* pts, int len, uint16_t col, int thickness) {
  for (int i = 0; i < len - 1; i++) {
    int x1, y1, x2, y2;
    geoToMapSpriteXY(pts[i].lat, pts[i].lon, x1, y1);
    geoToMapSpriteXY(pts[i + 1].lat, pts[i + 1].lon, x2, y2);
    spr.drawLine(x1, y1, x2, y2, col);
    if (thickness > 1) spr.drawLine(x1 + 1, y1, x2 + 1, y2, col);
  }
}

void drawDashedAirwayOnSprite(LGFX_Sprite &spr, const GeoPoint* pts, int len, uint16_t col) {
  for (int i = 0; i < len - 1; i++) {
    int x1, y1, x2, y2;
    geoToMapSpriteXY(pts[i].lat, pts[i].lon, x1, y1);
    geoToMapSpriteXY(pts[i + 1].lat, pts[i + 1].lon, x2, y2);
    float dx = x2 - x1, dy = y2 - y1;
    float totalLen = sqrtf(dx * dx + dy * dy);
    if (totalLen < 1.0f) continue;
    float stepX = dx / totalLen, stepY = dy / totalLen;
    for (float d = 0; d < totalLen; d += 8.0f) {
      float dEnd = min(d + 4.0f, totalLen);
      spr.drawLine(x1 + (int)(stepX * d), y1 + (int)(stepY * d),
                   x1 + (int)(stepX * dEnd), y1 + (int)(stepY * dEnd), col);
    }
  }
}

void buildVectorMapSprite() {
  applyVectorMapThemeColors();
  mapSprite.fillSprite(TFT_BLACK);

  for (int y = MAP_HALF - R; y <= MAP_HALF + R; y++) {
    int dy = y - MAP_HALF;
    int dx = (int)sqrtf((float)(R * R - dy * dy));
    if (dx > 0) mapSprite.drawFastHLine(MAP_HALF - dx, y, dx * 2, colSea);
  }

  if (distanceKm(cfg.lat, cfg.lon, 35.8, 127.8) < 450.0f) {
    fillGeoPolygonOnSprite(mapSprite, POLY_MAINLAND, POLY_MAINLAND_LEN, colLand, colCoast, POLY_MAINLAND_LEN - 1);
    fillGeoPolygonOnSprite(mapSprite, POLY_GEOJE, POLY_GEOJE_LEN, colLand, colCoast, POLY_GEOJE_LEN - 1);
    fillGeoPolygonOnSprite(mapSprite, POLY_NAMHAE, POLY_NAMHAE_LEN, colLand, colCoast, POLY_NAMHAE_LEN - 1);
    fillGeoPolygonOnSprite(mapSprite, POLY_TSUSHIMA, POLY_TSUSHIMA_LEN, colLand, colCoast, POLY_TSUSHIMA_LEN - 1);
    fillGeoPolygonOnSprite(mapSprite, POLY_JEJU, POLY_JEJU_LEN, colLand, colCoast, POLY_JEJU_LEN - 1);
    fillGeoPolygonOnSprite(mapSprite, POLY_YEONGJONG, POLY_YEONGJONG_LEN, colLand, colCoast, POLY_YEONGJONG_LEN - 1);

    drawPolylineOnSprite(mapSprite, ROAD_GYEONGBU, ROAD_GYEONGBU_LEN, colRoad, 1);
    drawPolylineOnSprite(mapSprite, ROAD_NAMHAE, ROAD_NAMHAE_LEN, colRoad, 1);
    drawPolylineOnSprite(mapSprite, RIVER_NAKDONG, RIVER_NAKDONG_LEN, colRiver, 2);
    drawPolylineOnSprite(mapSprite, RIVER_HAN, RIVER_HAN_LEN, colRiver, 2);
  } else {
    for (double dLat = -1.5; dLat <= 1.5; dLat += 0.5) {
      int x1, y1, x2, y2;
      geoToMapSpriteXY(floor(cfg.lat) + dLat, cfg.lon - 2.0, x1, y1);
      geoToMapSpriteXY(floor(cfg.lat) + dLat, cfg.lon + 2.0, x2, y2);
      mapSprite.drawLine(x1, y1, x2, y2, colRoad);
    }
    for (double dLon = -1.5; dLon <= 1.5; dLon += 0.5) {
      int x1, y1, x2, y2;
      geoToMapSpriteXY(cfg.lat - 2.0, floor(cfg.lon) + dLon, x1, y1);
      geoToMapSpriteXY(cfg.lat + 2.0, floor(cfg.lon) + dLon, x2, y2);
      mapSprite.drawLine(x1, y1, x2, y2, colRoad);
    }
  }

  if (distanceKm(cfg.lat, cfg.lon, 35.1795, 128.9382) < 120.0f) {
    drawDashedAirwayOnSprite(mapSprite, ROUTE_KADOL, 4, colAirway);
    drawDashedAirwayOnSprite(mapSprite, ROUTE_DOTOL, 4, colAirway);
    drawDashedAirwayOnSprite(mapSprite, ROUTE_OLMEN, 4, colAirway);
    drawDashedAirwayOnSprite(mapSprite, ROUTE_EAST,  4, colAirway);

    mapSprite.setTextDatum(top_center);
    mapSprite.setTextSize(1);
    mapSprite.setTextColor(colWaypoint);
    for (int i = 0; i < RKPK_WAYPOINTS_LEN; i++) {
      int wx, wy;
      geoToMapSpriteXY(RKPK_WAYPOINTS[i].lat, RKPK_WAYPOINTS[i].lon, wx, wy);
      if (hypot(wx - MAP_HALF, wy - MAP_HALF) <= R - 20) {
        mapSprite.drawTriangle(wx, wy - 4, wx - 4, wy + 3, wx + 4, wy + 3, colWaypoint);
        mapSprite.drawString(RKPK_WAYPOINTS[i].code, wx, wy + 5);
      }
    }
  }

  mapSprite.setTextDatum(middle_left);
  mapSprite.setTextSize(1);
  for (int i = 0; i < MAP_LABELS_LEN; i++) {
    int lx, ly;
    geoToMapSpriteXY(MAP_LABELS[i].lat, MAP_LABELS[i].lon, lx, ly);
    if (hypot(lx - MAP_HALF, ly - MAP_HALF) <= R - 30) {
      if (MAP_LABELS[i].isSea) {
        mapSprite.setTextColor(colRiver);
        mapSprite.drawString(MAP_LABELS[i].name, lx, ly);
      } else {
        mapSprite.fillCircle(lx, ly, 2, colCityDot);
        mapSprite.setTextColor(colCityText);
        mapSprite.drawString(MAP_LABELS[i].name, lx + 5, ly);
      }
    }
  }

  mapSprite.setTextDatum(top_left);
  mapSprite.setTextSize(1);
  mapSprite.setTextColor(colAirport);
  for (int i = 0; i < AIRPORTS_LEN; i++) {
    int ax, ay;
    geoToMapSpriteXY(AIRPORTS[i].lat, AIRPORTS[i].lon, ax, ay);
    if (hypot(ax - MAP_HALF, ay - MAP_HALF) <= R - 14) {
      mapSprite.drawRect(ax - 3, ay - 3, 7, 7, colAirport);
      mapSprite.drawString(AIRPORTS[i].code, ax + 6, ay - 4);
    }
  }

  for (int y = 0; y < MAP_SIZE; y++) {
    int dy = y - MAP_HALF;
    if (abs(dy) >= R) {
      mapSprite.drawFastHLine(0, y, MAP_SIZE, TFT_BLACK);
    } else {
      int dx = (int)sqrtf((float)(R * R - dy * dy));
      if (MAP_HALF - dx > 0) mapSprite.drawFastHLine(0, y, MAP_HALF - dx, TFT_BLACK);
      if (MAP_HALF + dx < MAP_SIZE) mapSprite.drawFastHLine(MAP_HALF + dx, y, MAP_SIZE - (MAP_HALF + dx), TFT_BLACK);
    }
  }

  mapSprite.drawFastVLine(MAP_HALF - 1, MAP_HALF - 6, 12, TFT_WHITE);
  mapSprite.drawFastVLine(MAP_HALF + 1, MAP_HALF - 6, 12, TFT_WHITE);
}

// ==========================================
// 7. RADAR 360 웹 콘솔 HTML
// ==========================================
const char WEB_HTML[] PROGMEM = R"RADAR360_HTML(
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RADAR 360 Console (Vector Edition)</title>
<style>
  :root { --bg:#081210; --card:#0d1c19; --border:#1f3d36; --accent:#5effc3; --text:#e2f5ef; --muted:#7ea398; }
  * { box-sizing:border-box; }
  body { margin:0; padding:16px; background:var(--bg); color:var(--text); font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; }
  .wrap { max-width:860px; margin:0 auto; }
  .header { display:flex; justify-content:space-between; align-items:center; margin-bottom:14px; }
  .brand { display:flex; align-items:center; gap:12px; }
  .logo { width:40px; height:40px; border-radius:50%; border:2px solid var(--accent); display:flex; align-items:center; justify-content:center; color:var(--accent); font-weight:bold; }
  .brand h1 { margin:0; font-size:20px; letter-spacing:1px; }
  .brand p { margin:2px 0 0; font-size:12px; color:var(--muted); }
  .status { text-align:right; font-size:12px; }
  .dot { display:inline-block; width:8px; height:8px; border-radius:50%; background:#ffd54f; margin-right:5px; }

  .preview-box { background:var(--card); border:1px solid var(--border); border-radius:12px; padding:14px; margin-bottom:14px; display:flex; flex-direction:column; align-items:center; }
  .preview-head { width:100%; display:flex; justify-content:space-between; align-items:center; margin-bottom:10px; font-size:12px; color:var(--muted); }
  .preview-badge { background:#16332c; color:var(--accent); padding:3px 9px; border-radius:20px; font-weight:600; font-size:11px; }
  #previewCanvas { width:100%; max-width:520px; aspect-ratio:800/480; border-radius:10px; border:2px solid #264d44; background:#000; box-shadow:0 8px 24px rgba(0,0,0,0.5); }

  .tabs { display:flex; background:var(--card); border:1px solid var(--border); border-radius:10px; padding:4px; margin-bottom:14px; gap:4px; flex-wrap:wrap; }
  .tab { flex:1; min-width:70px; text-align:center; padding:9px 4px; border-radius:7px; cursor:pointer; font-size:13px; color:var(--muted); border:none; background:transparent; }
  .tab.active { background:#16332c; color:var(--accent); font-weight:600; }
  .panel { display:none; background:var(--card); border:1px solid var(--border); border-radius:12px; padding:18px; margin-bottom:16px; }
  .panel.active { display:block; }
  h2 { margin:0 0 14px; font-size:15px; color:var(--accent); }
  .grid { display:grid; grid-template-columns:1fr 1fr; gap:12px; }
  @media(max-width:560px){ .grid { grid-template-columns:1fr; } }
  .field { margin-bottom:10px; }
  label { display:block; font-size:12px; color:var(--muted); margin-bottom:5px; }
  input[type=text], input[type=password], input[type=number], select {
    width:100%; padding:10px; border-radius:8px; border:1px solid var(--border);
    background:#07100e; color:var(--text); font-size:14px;
  }
  .chk-grid { display:grid; grid-template-columns:1fr 1fr; gap:8px; margin-top:10px; }
  .chk { display:flex; align-items:center; gap:8px; background:#0a1613; border:1px solid var(--border); padding:9px 10px; border-radius:8px; font-size:13px; cursor:pointer; }
  .chk input { accent-color:var(--accent); width:16px; height:16px; }
  .btn { padding:10px 16px; border-radius:8px; border:1px solid var(--border); background:#112621; color:var(--text); cursor:pointer; font-size:13px; }
  .btn-primary { background:var(--accent); color:#052117; border:none; font-weight:700; padding:12px 22px; font-size:14px; }
  .btn-danger { background:#3d1616; border-color:#7a2929; color:#ff9e9e; }
  .scan-list { margin-top:10px; max-height:150px; overflow-y:auto; border:1px solid var(--border); border-radius:8px; display:none; }
  .scan-item { padding:8px 12px; border-bottom:1px solid var(--border); cursor:pointer; font-size:13px; display:flex; justify-content:space-between; }
  .scan-item:hover { background:#16332c; }
  .hint { font-size:11px; color:var(--muted); margin-top:4px; }
  .toast { position:fixed; bottom:20px; right:20px; background:var(--accent); color:#052117; padding:10px 16px; border-radius:8px; font-weight:700; display:none; }
</style>
</head>
<body>
<div class="wrap">
  <div class="header">
    <div class="brand">
      <div class="logo">360</div>
      <div>
        <h1>RADAR 360 - Vector Edition</h1>
        <p>Interactive Region Picker / Aircraft Visual Card / Live ADS-B</p>
      </div>
    </div>
    <div class="status">
      <div><span class="dot" id="stDot"></span><span id="stMode">Loading...</span></div>
      <div id="stIp" style="color:var(--muted)"></div>
    </div>
  </div>

  <div class="preview-box">
    <div class="preview-head">
      <span>LCD Live Preview (800 x 480)</span>
      <span class="preview-badge" id="pvModeLabel">FLIGHT RADAR</span>
    </div>
    <canvas id="previewCanvas" width="800" height="480"></canvas>
  </div>

  <div class="tabs">
    <button class="tab active" onclick="openTab('net',this)">Network</button>
    <button class="tab" onclick="openTab('radar',this)">Radar</button>
    <button class="tab" onclick="openTab('wx',this)">Weather</button>
    <button class="tab" onclick="openTab('clk',this)">Clock</button>
    <button class="tab" onclick="openTab('sys',this)">System</button>
  </div>

  <form id="cfgForm" onsubmit="saveAll(event)">
    <div id="tab-net" class="panel active">
      <h2>WI-FI Setup</h2>
      <div class="grid">
        <div class="field"><label>Network name (SSID)</label><input type="text" id="ssid" name="ssid"></div>
        <div class="field"><label>Password</label><input type="password" id="pass" name="pass" placeholder="Leave blank to keep current password"></div>
      </div>
      <div class="field">
        <label>Device hostname</label>
        <input type="text" id="hostname" name="hostname" value="radar">
        <div class="hint">접속 주소: http://<span id="hostPreview">radar</span>.local</div>
      </div>
      <button type="button" class="btn" onclick="scanWifi()">Scan networks</button>
      <span class="hint" style="margin-left:8px;">Wi-Fi 또는 호스트명 변경 시 재부팅됩니다.</span>
      <div id="scanList" class="scan-list"></div>
    </div>

    <div id="tab-radar" class="panel">
      <h2>Location & Built-in Vector Map</h2>
      <div class="grid">
        <div class="field">
          <label>Airport Presets</label>
          <select id="preset" name="preset" onchange="applyPreset()">
            <optgroup label="대한민국 (Korea)">
              <option value="BUSAN">부산 김해국제공항 (RKPK)</option>
              <option value="INCHEON">인천국제공항 (RKSI)</option>
              <option value="JEJU">제주국제공항 (RKPC)</option>
            </optgroup>
            <optgroup label="아시아 · 오세아니아">
              <option value="NARITA">도쿄 나리타 (RJAA)</option>
              <option value="HANEDA">도쿄 하네다 (RJTT)</option>
              <option value="SINGAPORE">싱가포르 창이 (WSSS)</option>
              <option value="HONGKONG">홍콩 (VHHH)</option>
              <option value="BANGKOK">방콕 수완나품 (VTBS)</option>
              <option value="DUBAI">두바이 (OMDB)</option>
              <option value="SYDNEY">시드니 (YSSY)</option>
            </optgroup>
            <optgroup label="유럽 · 미주">
              <option value="LONDON">런던 히드로 (EGLL)</option>
              <option value="PARIS">파리 샤를드골 (LFPG)</option>
              <option value="FRANKFURT">프랑크푸르트 (EDDF)</option>
              <option value="NEWYORK">뉴욕 JFK (KJFK)</option>
              <option value="LA">로스앤젤레스 (KLAX)</option>
            </optgroup>
            <option value="CUSTOM">직접 입력 (Custom)</option>
          </select>
        </div>
        <div class="field">
          <label>Vector Map Theme</label>
          <select id="mapStyle" name="mapStyle">
            <option value="0">Vivid Color Vector + Airways (컬러 지형도)</option>
            <option value="1">Tactical Green HUD (전술 그린)</option>
            <option value="2">Midnight Dark (다크 모노크롬)</option>
          </select>
        </div>
        <div class="field"><label>Latitude (WGS-84)</label><input type="number" id="lat" name="lat" step="0.0001"></div>
        <div class="field"><label>Longitude (WGS-84)</label><input type="number" id="lon" name="lon" step="0.0001"></div>
        <div class="field"><label>Radius (km, 10~200)</label><input type="number" id="radiusKm" name="radiusKm" min="10" max="200"></div>
        <div class="field"><label>Refresh interval (sec)</label><input type="number" id="refreshSec" name="refreshSec" min="5" max="60"></div>
        <div class="field"><label>Minimum altitude (ft)</label><input type="number" id="minAltFt" name="minAltFt" min="0" max="60000" step="500"></div>
        <div class="field"><label>Maximum altitude (ft)</label><input type="number" id="maxAltFt" name="maxAltFt" min="1000" max="60000" step="500"></div>
        <div class="field"><label>Maximum aircraft (1~40)</label><input type="number" id="maxAc" name="maxAc" min="1" max="40"></div>
      </div>

      <h2 style="margin-top:18px;">Display & Units</h2>
      <div class="grid">
        <div class="field"><label>Distance</label><select id="unitDist" name="unitDist"><option value="0">Kilometers (km)</option><option value="1">Nautical miles (nm)</option></select></div>
        <div class="field"><label>Altitude</label><select id="unitAlt" name="unitAlt"><option value="1">Feet (ft)</option><option value="0">Meters (m)</option></select></div>
        <div class="field"><label>Speed</label><select id="unitSpeed" name="unitSpeed"><option value="1">Knots (kt)</option><option value="0">km/h</option></select></div>
        <div class="field">
          <label>Startup / Active View</label>
          <select id="viewMode" name="viewMode" onchange="syncViewMode(this.value)">
            <option value="0">Flight radar (항공기 레이더)</option>
            <option value="1">Weather view (실시간 날씨)</option>
            <option value="2">Clock view (시계 화면)</option>
          </select>
        </div>
      </div>

      <div class="chk-grid">
        <label class="chk"><input type="checkbox" id="showMapBg" name="showMapBg"> Vector map background</label>
        <label class="chk"><input type="checkbox" id="showScanLine" name="showScanLine"> Scan line</label>
        <label class="chk"><input type="checkbox" id="showCallsign" name="showCallsign"> Callsign</label>
        <label class="chk"><input type="checkbox" id="showAltitude" name="showAltitude"> Altitude</label>
        <label class="chk"><input type="checkbox" id="showHeading" name="showHeading"> Heading</label>
        <label class="chk"><input type="checkbox" id="showSpeed" name="showSpeed"> Speed</label>
        <label class="chk"><input type="checkbox" id="showDistance" name="showDistance"> Distance</label>
        <label class="chk"><input type="checkbox" id="showAcType" name="showAcType"> Aircraft type</label>
        <label class="chk"><input type="checkbox" id="showVectors" name="showVectors"> Heading vectors</label>
      </div>
    </div>

    <div id="tab-wx" class="panel">
      <h2>Display Mode</h2>
      <div class="field">
        <label>화면 표시 모드 선택</label>
        <select id="wxViewMode" onchange="syncViewMode(this.value)">
          <option value="0">Flight radar (항공기 레이더)</option>
          <option value="1">Weather view (실시간 날씨)</option>
          <option value="2">Clock view (시계 화면)</option>
        </select>
      </div>

      <h2 style="margin-top:18px;">Weather Location</h2>
      <label class="chk" style="margin-bottom:12px;"><input type="checkbox" id="wxUseRadarLoc" name="wxUseRadarLoc" onchange="toggleWxLoc()"> Use radar location</label>
      <div class="grid">
        <div class="field"><label>Weather latitude (WGS-84)</label><input type="number" id="wxLat" name="wxLat" step="0.0001"></div>
        <div class="field"><label>Weather longitude (WGS-84)</label><input type="number" id="wxLon" name="wxLon" step="0.0001"></div>
      </div>

      <h2 style="margin-top:18px;">Weather Units</h2>
      <div class="grid">
        <div class="field"><label>Temperature unit</label><select id="unitTemp" name="unitTemp"><option value="0">Celsius C</option><option value="1">Fahrenheit F</option></select></div>
        <div class="field"><label>Precipitation unit</label><select id="unitPrecip" name="unitPrecip"><option value="0">Millimeters mm</option><option value="1">Inches in</option></select></div>
      </div>
    </div>

    <div id="tab-clk" class="panel">
      <h2>Clock Display Mode</h2>
      <div class="field">
        <label>화면 표시 모드 선택</label>
        <select id="clkViewMode" onchange="syncViewMode(this.value)">
          <option value="0">Flight radar (항공기 레이더)</option>
          <option value="1">Weather view (실시간 날씨)</option>
          <option value="2">Clock view (시계 화면)</option>
        </select>
      </div>

      <h2 style="margin-top:18px;">Watch Face & Timezone</h2>
      <div class="grid">
        <div class="field">
          <label>시계 화면 스타일 선택</label>
          <select id="clockStyle" name="clockStyle">
            <optgroup label="아날로그 명품 시계 (Analog)">
              <option value="0">ROLEX (Submariner Black)</option>
              <option value="1">LONGINES (Master Classic White)</option>
              <option value="2">BREITLING (Navitimer Chrono)</option>
              <option value="3">OMEGA (Seamaster Deep Blue)</option>
            </optgroup>
            <optgroup label="디지털 시계 (Digital)">
              <option value="4">Digital Modern (시간 + 날짜 + 기온)</option>
              <option value="5">Digital Neon Ring (원형 초침 게이지)</option>
              <option value="6">Digital Retro Flip (플립 시계 스타일)</option>
            </optgroup>
          </select>
        </div>
        <div class="field">
          <label>지역별 시간대 (Timezone)</label>
          <select id="tzPosix" name="tzPosix" onchange="updateTzLabel()">
            <option value="KST-9"   data-lbl="SEOUL (UTC+9)"      data-off="9">대한민국 서울/부산 (UTC+9)</option>
            <option value="JST-9"   data-lbl="TOKYO (UTC+9)"      data-off="9">일본 도쿄 (UTC+9)</option>
            <option value="SGT-8"   data-lbl="SINGAPORE (UTC+8)"  data-off="8">싱가포르 / 홍콩 (UTC+8)</option>
            <option value="ICT-7"   data-lbl="BANGKOK (UTC+7)"    data-off="7">태국 방콕 (UTC+7)</option>
            <option value="GST-4"   data-lbl="DUBAI (UTC+4)"      data-off="4">아랍에미리트 두바이 (UTC+4)</option>
            <option value="GMT0"    data-lbl="LONDON (UTC+0)"     data-off="0">영국 런던 (UTC+0)</option>
            <option value="CET-1"   data-lbl="PARIS (UTC+1)"      data-off="1">프랑스 파리 / 독일 (UTC+1)</option>
            <option value="EST5"    data-lbl="NEW YORK (UTC-5)"   data-off="-5">미국 뉴욕 (UTC-5)</option>
            <option value="PST8"    data-lbl="LOS ANGELES (UTC-8)" data-off="-8">미국 LA (UTC-8)</option>
            <option value="AEST-10" data-lbl="SYDNEY (UTC+10)"    data-off="10">호주 시드니 (UTC+10)</option>
          </select>
          <input type="hidden" id="tzLabel" name="tzLabel" value="SEOUL (UTC+9)">
        </div>
      </div>
    </div>

    <div id="tab-sys" class="panel">
      <h2>Display</h2>
      <div class="field">
        <label>Day brightness: <span id="brtVal">80</span>%</label>
        <input type="range" id="brightness" name="brightness" min="10" max="100" style="width:100%;accent-color:var(--accent);"
               oninput="document.getElementById('brtVal').innerText=this.value">
      </div>
      <h2 style="margin-top:20px;">Device</h2>
      <p class="hint" style="margin-bottom:12px;">Factory reset은 저장된 Wi-Fi 및 모든 설정을 초기화하고 설정 모드로 재부팅합니다.</p>
      <button type="button" class="btn" onclick="deviceAction('/api/restart','재부팅 하시겠습니까?')">Restart</button>
      <button type="button" class="btn btn-danger" style="margin-left:8px;" onclick="deviceAction('/api/reset','모든 설정을 초기화하시겠습니까?')">Factory reset</button>
    </div>

    <button type="submit" class="btn btn-primary">Save all settings</button>
  </form>
</div>
<div id="toast" class="toast">Saved!</div>

<script>
var PRESETS = {
  BUSAN:[35.1795,128.9382], INCHEON:[37.4602,126.4407], JEJU:[33.5113,126.4930],
  NARITA:[35.7720,140.3929], HANEDA:[35.5494,139.7798], SINGAPORE:[1.3644,103.9915],
  HONGKONG:[22.3080,113.9185], BANGKOK:[13.6900,100.7501], DUBAI:[25.2532,55.3657],
  SYDNEY:[-33.9399,151.1753], LONDON:[51.4700,-0.4543], PARIS:[49.0097,2.5479],
  FRANKFURT:[50.0379,8.5622], NEWYORK:[40.6413,-73.7781], LA:[33.9416,-118.4085]
};
var CHKS = ['showMapBg','showScanLine','showCallsign','showAltitude','showHeading','showSpeed','showDistance','showAcType','showVectors','wxUseRadarLoc'];
var pvSweep = 45;

var openTab = function(id, el) {
  document.querySelectorAll('.panel').forEach(function(p) { p.classList.remove('active'); });
  document.querySelectorAll('.tab').forEach(function(t) { t.classList.remove('active'); });
  document.getElementById('tab-' + id).classList.add('active');
  el.classList.add('active');
  if (id === 'radar') syncViewMode('0');
  else if (id === 'wx') syncViewMode('1');
  else if (id === 'clk') syncViewMode('2');
};

var syncViewMode = function(v) {
  document.getElementById('viewMode').value = v;
  document.getElementById('wxViewMode').value = v;
  document.getElementById('clkViewMode').value = v;
  drawPreview();
};

var updateTzLabel = function() {
  var sel = document.getElementById('tzPosix');
  var opt = sel.options[sel.selectedIndex];
  document.getElementById('tzLabel').value = opt.getAttribute('data-lbl') || 'LOCAL';
  drawPreview();
};

var applyPreset = function() {
  var k = document.getElementById('preset').value;
  if (PRESETS[k]) {
    document.getElementById('lat').value = PRESETS[k][0];
    document.getElementById('lon').value = PRESETS[k][1];
    if (document.getElementById('wxUseRadarLoc').checked) {
      document.getElementById('wxLat').value = PRESETS[k][0];
      document.getElementById('wxLon').value = PRESETS[k][1];
    }
  }
  drawPreview();
};

var toggleWxLoc = function() {
  var u = document.getElementById('wxUseRadarLoc').checked;
  document.getElementById('wxLat').disabled = u;
  document.getElementById('wxLon').disabled = u;
  if (u) {
    document.getElementById('wxLat').value = document.getElementById('lat').value;
    document.getElementById('wxLon').value = document.getElementById('lon').value;
  }
  drawPreview();
};

var getTzDate = function() {
  var sel = document.getElementById('tzPosix');
  var off = parseFloat((sel.options[sel.selectedIndex] && sel.options[sel.selectedIndex].getAttribute('data-off')) || '9');
  var now = new Date();
  var utc = now.getTime() + now.getTimezoneOffset() * 60000;
  return new Date(utc + off * 3600000);
};

var pad2 = function(n) { return String(n).padStart(2, '0'); };

var drawRadarPreview = function(ctx, preset, timeStr) {
  var cx = 238, cy = 240, r = 204;
  var showMap = document.getElementById('showMapBg').checked;
  var mapSty = parseInt(document.getElementById('mapStyle').value || '0');
  var host = document.getElementById('hostname').value || 'radar';

  ctx.save();
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.clip();
  ctx.fillStyle = !showMap ? '#000' : (mapSty === 1 ? '#04120c' : (mapSty === 2 ? '#060a0e' : '#0a264e'));
  ctx.fillRect(cx - r, cy - r, r * 2, r * 2);

  if (showMap) {
    ctx.fillStyle = mapSty === 1 ? '#0c2616' : (mapSty === 2 ? '#161c22' : '#1c3e26');
    ctx.strokeStyle = mapSty === 1 ? '#00d250' : (mapSty === 2 ? '#4b738c' : '#41cdaf');
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(170, 200, 145, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.strokeStyle = 'rgba(110,215,255,0.55)'; ctx.lineWidth = 2; ctx.setLineDash([5, 5]);
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(360, 175); ctx.moveTo(cx, cy); ctx.lineTo(295, 325); ctx.stroke();
    ctx.setLineDash([]);
  }

  if (document.getElementById('showScanLine').checked) {
    var rad = (pvSweep - 90) * Math.PI / 180;
    var grd = ctx.createRadialGradient(cx, cy, 10, cx, cy, r);
    grd.addColorStop(0, 'rgba(0,255,80,0.05)'); grd.addColorStop(1, 'rgba(0,255,80,0.35)');
    ctx.fillStyle = grd;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.arc(cx, cy, r, rad - 0.6, rad); ctx.closePath(); ctx.fill();
    ctx.strokeStyle = '#00ff3c'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + Math.cos(rad) * r, cy + Math.sin(rad) * r); ctx.stroke();
  }
  ctx.restore();

  ctx.strokeStyle = '#147837'; ctx.lineWidth = 1;
  for (var i = 1; i <= 4; i++) { ctx.beginPath(); ctx.arc(cx, cy, (r / 5) * i, 0, Math.PI * 2); ctx.stroke(); }
  ctx.strokeStyle = '#00ff3c'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.stroke();
  ctx.strokeStyle = '#147837';
  ctx.beginPath(); ctx.moveTo(cx - r, cy); ctx.lineTo(cx + r, cy); ctx.moveTo(cx, cy - r); ctx.lineTo(cx, cy + r); ctx.stroke();

  var acs = [
    { x: 175, y: 165, hdg: 125, col: '#00e6ff', fl: 'KAL123' },
    { x: 315, y: 205, hdg: 245, col: '#32ff32', fl: 'KE1402' },
    { x: 275, y: 315, hdg: 35,  col: '#ffeb00', fl: 'ABL332' }
  ];
  acs.forEach(function(a) {
    ctx.save(); ctx.translate(a.x, a.y); ctx.rotate(a.hdg * Math.PI / 180);
    ctx.fillStyle = a.col;
    ctx.beginPath(); ctx.moveTo(0,-10); ctx.lineTo(3,-3); ctx.lineTo(10,3); ctx.lineTo(10,5); ctx.lineTo(2,2);
    ctx.lineTo(2,7); ctx.lineTo(5,10); ctx.lineTo(-5,10); ctx.lineTo(-2,7); ctx.lineTo(-2,2);
    ctx.lineTo(-10,5); ctx.lineTo(-10,3); ctx.lineTo(-3,-3); ctx.closePath(); ctx.fill();
    ctx.restore();
    ctx.fillStyle = '#fff'; ctx.font = 'bold 11px monospace'; ctx.fillText(a.fl, a.x + 12, a.y - 4);
  });

  ctx.fillStyle = '#00ff3c'; ctx.font = 'bold 16px monospace';
  ctx.fillText('REAL ADS-B', 14, 26); ctx.fillText('AC: 12', 14, 48);

  ctx.fillStyle = '#040c08';
  ctx.fillRect(482, 6, 312, 276);
  ctx.fillRect(482, 288, 312, 110);
  ctx.fillRect(482, 404, 312, 70);
  ctx.strokeStyle = '#1f6e43'; ctx.lineWidth = 2;
  ctx.strokeRect(482, 6, 312, 276);
  ctx.strokeRect(482, 288, 312, 110);
  ctx.strokeRect(482, 404, 312, 70);

  ctx.fillStyle = '#5effc3'; ctx.font = 'bold 15px monospace';
  ctx.fillText('DETECTED AIRCRAFTS', 494, 28);
  ctx.fillStyle = '#7ea398'; ctx.font = '12px monospace';
  ctx.fillText('CALLSIGN  DIST   ALT     SPD   TYPE', 494, 52);
  ctx.fillStyle = '#00e6ff';
  ctx.fillText('KAL123    24km  12,000  340kt  B738', 494, 78);
  ctx.fillStyle = '#32ff32';
  ctx.fillText('AAR456    38km   8,000  280kt  A321', 494, 102);

  ctx.fillStyle = '#5effc3'; ctx.font = 'bold 16px monospace';
  ctx.fillText(preset + ' [SELECT]', 494, 310);
  ctx.fillStyle = '#e2f5ef'; ctx.font = '12px monospace';
  ctx.fillText(document.getElementById('lat').value + ' / ' + document.getElementById('lon').value, 494, 334);
  ctx.fillStyle = '#ffeb3b'; ctx.font = 'bold 12px monospace';
  ctx.fillText('WEB: http://' + host + '.local', 494, 386);

  ctx.fillStyle = '#5effc3'; ctx.font = 'bold 16px monospace';
  ctx.fillText('TIME:', 494, 445);
  ctx.fillStyle = '#ffeb3b'; ctx.font = 'bold 24px monospace';
  ctx.fillText(timeStr, 554, 447);
  ctx.fillStyle = '#00ff3c'; ctx.font = '12px monospace';
  ctx.fillText('MODE >', 735, 444);
};

var drawWeatherPreview = function(ctx, preset, hmStr) {
  ctx.fillStyle = '#0c2036'; ctx.fillRect(0, 0, 800, 480);
  ctx.fillStyle = '#fff'; ctx.font = 'bold 22px sans-serif'; ctx.textAlign = 'center';
  ctx.fillText('WEATHER - ' + preset + '   ' + hmStr, 400, 38);
  ctx.fillStyle = 'rgba(16,26,36,0.92)'; ctx.strokeStyle = '#3a5c74';
  ctx.strokeRect(60, 80, 320, 140); ctx.strokeRect(420, 80, 320, 140);
  ctx.strokeRect(60, 290, 680, 110);
  ctx.fillStyle = '#5effc3'; ctx.font = 'bold 34px sans-serif'; ctx.fillText('16 C  (H:68%)', 220, 160);
  ctx.fillStyle = '#ffeb3b'; ctx.font = 'bold 22px sans-serif'; ctx.fillText('WND: 040/08KT - 1014hPa', 580, 160);
  ctx.fillStyle = '#fff'; ctx.font = '18px sans-serif'; ctx.fillText('Today 18/10   |   Tmrw 13/6   |   Day 3 17/11', 400, 352);
  ctx.textAlign = 'left';
};

var drawClockPreview = function(ctx, hr, mn, sc, day, yr, mo, tzLbl, preset) {
  var sty = parseInt(document.getElementById('clockStyle').value || '0');
  ctx.textAlign = 'center';
  ctx.fillStyle = '#070b0a'; ctx.fillRect(0, 0, 800, 480);
  ctx.strokeStyle = '#1f6e43'; ctx.strokeRect(18, 40, 175, 400); ctx.strokeRect(607, 40, 175, 400);
  ctx.fillStyle = '#5effc3'; ctx.font = 'bold 14px monospace';
  ctx.fillText('STATION', 105, 80); ctx.fillStyle = '#fff'; ctx.fillText(preset, 105, 110);
  ctx.fillStyle = '#5effc3'; ctx.fillText('TIMEZONE', 695, 80); ctx.fillStyle = '#ffeb3b'; ctx.fillText(tzLbl, 695, 110);

  if (sty <= 3) {
    ctx.fillStyle = '#161a1c'; ctx.beginPath(); ctx.arc(400, 240, 224, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = sty === 1 ? '#f2f0e8' : (sty === 3 ? '#0a2656' : '#000');
    ctx.beginPath(); ctx.arc(400, 240, 190, 0, Math.PI * 2); ctx.fill();
    var brands = ['ROLEX', 'LONGINES', 'BREITLING', 'OMEGA'];
    ctx.fillStyle = sty === 1 ? '#111' : '#dcbe5a'; ctx.font = 'bold 26px serif'; ctx.fillText(brands[sty], 400, 165);
    ctx.fillStyle = sty === 1 ? '#111' : '#fff'; ctx.font = 'bold 28px monospace'; ctx.fillText(pad2(hr) + ':' + pad2(mn) + ':' + pad2(sc), 400, 255);
  } else {
    ctx.fillStyle = '#00ff3c'; ctx.font = 'bold 64px monospace';
    ctx.fillText(pad2(hr) + ':' + pad2(mn) + ':' + pad2(sc), 400, 245);
    ctx.fillStyle = '#ffeb3b'; ctx.font = '22px monospace';
    ctx.fillText(yr + '-' + pad2(mo) + '-' + pad2(day), 400, 305);
  }
  ctx.textAlign = 'left';
};

var drawPreview = function() {
  var cv = document.getElementById('previewCanvas');
  if (!cv) return;
  var ctx = cv.getContext('2d');
  var brt = parseInt(document.getElementById('brightness').value || '80');
  cv.style.filter = 'brightness(' + (0.35 + (brt / 100) * 0.65) + ')';

  var vm = parseInt(document.getElementById('viewMode').value || '0');
  var preset = document.getElementById('preset').value || 'BUSAN';
  var tzLbl = document.getElementById('tzLabel').value || 'SEOUL (UTC+9)';
  var d = getTzDate();
  var hr = d.getHours(), mn = d.getMinutes(), sc = d.getSeconds();

  ctx.fillStyle = '#040806';
  ctx.fillRect(0, 0, 800, 480);

  if (vm === 0) {
    document.getElementById('pvModeLabel').innerText = 'FLIGHT RADAR (VECTOR MAP)';
    drawRadarPreview(ctx, preset, pad2(hr) + ':' + pad2(mn) + ':' + pad2(sc));
  } else if (vm === 1) {
    document.getElementById('pvModeLabel').innerText = 'WEATHER VIEW (800x480)';
    drawWeatherPreview(ctx, preset, pad2(hr) + ':' + pad2(mn));
  } else {
    document.getElementById('pvModeLabel').innerText = 'CLOCK VIEW (800x480)';
    drawClockPreview(ctx, hr, mn, sc, d.getDate(), d.getFullYear(), d.getMonth() + 1, tzLbl, preset);
  }
};

document.getElementById('cfgForm').addEventListener('input', drawPreview);
document.getElementById('cfgForm').addEventListener('change', drawPreview);
document.getElementById('hostname').addEventListener('input', function(e) {
  document.getElementById('hostPreview').innerText = e.target.value || 'radar';
});

var loadState = function() {
  fetch('/api/config').then(function(r) { return r.json(); }).then(function(d) {
    document.getElementById('stMode').innerText = d.apMode ? 'Wi-Fi setup' : 'Connected';
    document.getElementById('stDot').style.background = d.apMode ? '#ffd54f' : '#5effc3';
    document.getElementById('stIp').innerText = d.ip;

    var keys = ['ssid','hostname','preset','mapStyle','lat','lon','radiusKm','refreshSec','minAltFt','maxAltFt','maxAc',
                'unitDist','unitAlt','unitSpeed','viewMode','wxLat','wxLon','unitTemp','unitPrecip','clockStyle','tzPosix','tzLabel','brightness'];
    keys.forEach(function(k) { if (document.getElementById(k)) document.getElementById(k).value = d[k]; });
    syncViewMode(String(d.viewMode));
    document.getElementById('hostPreview').innerText = d.hostname;
    document.getElementById('brtVal').innerText = d.brightness;
    CHKS.forEach(function(k) { if (document.getElementById(k)) document.getElementById(k).checked = d[k]; });
    toggleWxLoc();
    drawPreview();
  });
};

var scanWifi = function() {
  var list = document.getElementById('scanList');
  list.style.display = 'block';
  list.innerHTML = '<div class="scan-item">스캔 중...</div>';
  fetch('/api/scan').then(function(r) { return r.json(); }).then(function(arr) {
    if (!arr.length) { list.innerHTML = '<div class="scan-item">검색된 Wi-Fi 없음</div>'; return; }
    list.innerHTML = arr.map(function(n) {
      return '<div class="scan-item" onclick="document.getElementById(\'ssid\').value=\'' + n.ssid + '\'"><span>' + n.ssid + '</span><span>' + n.rssi + ' dBm</span></div>';
    }).join('');
  });
};

var saveAll = function(e) {
  e.preventDefault();
  document.getElementById('wxLat').disabled = false;
  document.getElementById('wxLon').disabled = false;
  updateTzLabel();
  var params = new URLSearchParams();
  var fields = ['ssid','pass','hostname','preset','mapStyle','lat','lon','radiusKm','refreshSec','minAltFt','maxAltFt','maxAc',
                'unitDist','unitAlt','unitSpeed','viewMode','wxLat','wxLon','unitTemp','unitPrecip','clockStyle','tzPosix','tzLabel','brightness'];
  fields.forEach(function(k) {
    var el = document.getElementById(k);
    if (el) params.append(k, el.value);
  });
  CHKS.forEach(function(k) {
    var el = document.getElementById(k);
    if (el) params.append(k, el.checked ? '1' : '0');
  });
  toggleWxLoc();

  fetch('/api/save', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: params.toString()
  }).then(function(r) { return r.json(); }).then(function(res) {
    var t = document.getElementById('toast');
    t.innerText = res.reboot ? '저장 완료! 재부팅 중...' : '설정이 즉시 적용되었습니다!';
    t.style.display = 'block';
    setTimeout(function() { t.style.display = 'none'; }, 3000);
  });
};

var deviceAction = function(url, msg) {
  if (!confirm(msg)) return;
  fetch(url, { method: 'POST' }).then(function() {
    alert('요청을 처리했습니다. 기기가 재부팅됩니다.');
  });
};

setInterval(function() { pvSweep = (pvSweep + 6) % 360; drawPreview(); }, 250);
loadState();
</script>
</body>
</html>
)RADAR360_HTML";

void setupWebServer() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", WEB_HTML); });

  server.on("/api/config", HTTP_GET, []() {
    String j = "{";
    j += "\"apMode\":" + String(isApMode ? "true" : "false") + ",";
    j += "\"ip\":\"" + (isApMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + "\",";
    j += "\"ssid\":\"" + cfg.ssid + "\",\"hostname\":\"" + cfg.hostname + "\",\"preset\":\"" + cfg.presetName + "\",";
    j += "\"mapStyle\":" + String(cfg.mapStyle) + ",\"lat\":" + String(cfg.lat, 4) + ",\"lon\":" + String(cfg.lon, 4) + ",\"radiusKm\":" + String(cfg.radiusKm, 0) + ",";
    j += "\"refreshSec\":" + String(cfg.refreshSec) + ",\"minAltFt\":" + String(cfg.minAltFt) + ",\"maxAltFt\":" + String(cfg.maxAltFt) + ",\"maxAc\":" + String(cfg.maxAc) + ",";
    j += "\"unitDist\":" + String(cfg.unitDist) + ",\"unitAlt\":" + String(cfg.unitAlt) + ",\"unitSpeed\":" + String(cfg.unitSpeed) + ",\"viewMode\":" + String(cfg.viewMode) + ",";
    j += "\"showMapBg\":" + String(cfg.showMapBg ? "true" : "false") + ",\"showScanLine\":" + String(cfg.showScanLine ? "true" : "false") + ",";
    j += "\"showCallsign\":" + String(cfg.showCallsign ? "true" : "false") + ",\"showAltitude\":" + String(cfg.showAltitude ? "true" : "false") + ",";
    j += "\"showHeading\":" + String(cfg.showHeading ? "true" : "false") + ",\"showSpeed\":" + String(cfg.showSpeed ? "true" : "false") + ",";
    j += "\"showDistance\":" + String(cfg.showDistance ? "true" : "false") + ",\"showAcType\":" + String(cfg.showAcType ? "true" : "false") + ",";
    j += "\"showVectors\":" + String(cfg.showVectors ? "true" : "false") + ",";
    j += "\"wxUseRadarLoc\":" + String(cfg.wxUseRadarLoc ? "true" : "false") + ",\"wxLat\":" + String(cfg.wxLat, 4) + ",\"wxLon\":" + String(cfg.wxLon, 4) + ",";
    j += "\"unitTemp\":" + String(cfg.unitTemp) + ",\"unitPrecip\":" + String(cfg.unitPrecip) + ",";
    j += "\"clockStyle\":" + String(cfg.clockStyle) + ",\"tzPosix\":\"" + cfg.tzPosix + "\",\"tzLabel\":\"" + cfg.tzLabel + "\",";
    j += "\"brightness\":" + String(cfg.brightness) + "}";
    server.send(200, "application/json", j);
  });

  server.on("/api/scan", HTTP_GET, []() {
    int n = WiFi.scanNetworks();
    String j = "[";
    bool first = true;
    for (int i = 0; i < n; i++) {
      String s = WiFi.SSID(i);
      if (s.length() == 0) continue;
      if (!first) j += ",";
      s.replace("\"", "");
      j += "{\"ssid\":\"" + s + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
      first = false;
    }
    j += "]";
    server.send(200, "application/json", j);
  });

  server.on("/api/save", HTTP_POST, []() {
    bool needReboot = false;
    bool mapChanged = false;
    double oldLat = cfg.lat, oldLon = cfg.lon;
    float  oldRad = cfg.radiusKm;
    int    oldSty = cfg.mapStyle;

    String newSsid = server.arg("ssid"); newSsid.trim();
    String newPass = server.arg("pass");
    String newHost = server.arg("hostname"); newHost.trim();
    if (newHost.length() == 0) newHost = "radar";

    if (newSsid != cfg.ssid || newHost != cfg.hostname || newPass.length() > 0) needReboot = true;
    if (newSsid.length() > 0) cfg.ssid = newSsid;
    if (newPass.length() > 0) cfg.pass = newPass;
    cfg.hostname = newHost;

    if (server.hasArg("preset"))     cfg.presetName = server.arg("preset");
    if (server.hasArg("mapStyle"))   cfg.mapStyle   = server.arg("mapStyle").toInt();
    if (server.hasArg("lat"))        cfg.lat        = server.arg("lat").toDouble();
    if (server.hasArg("lon"))        cfg.lon        = server.arg("lon").toDouble();
    if (server.hasArg("radiusKm"))   cfg.radiusKm   = constrain(server.arg("radiusKm").toFloat(), 10.0f, 200.0f);
    if (server.hasArg("refreshSec")) cfg.refreshSec = constrain(server.arg("refreshSec").toInt(), 5, 60);
    if (server.hasArg("minAltFt"))   cfg.minAltFt   = server.arg("minAltFt").toInt();
    if (server.hasArg("maxAltFt"))   cfg.maxAltFt   = server.arg("maxAltFt").toInt();
    if (server.hasArg("maxAc"))      cfg.maxAc      = constrain(server.arg("maxAc").toInt(), 1, 40);
    if (server.hasArg("unitDist"))   cfg.unitDist   = server.arg("unitDist").toInt();
    if (server.hasArg("unitAlt"))    cfg.unitAlt    = server.arg("unitAlt").toInt();
    if (server.hasArg("unitSpeed"))  cfg.unitSpeed  = server.arg("unitSpeed").toInt();
    if (server.hasArg("viewMode"))   cfg.viewMode   = server.arg("viewMode").toInt();

    cfg.showMapBg     = (server.arg("showMapBg") == "1");
    cfg.showScanLine  = (server.arg("showScanLine") == "1");
    cfg.showCallsign  = (server.arg("showCallsign") == "1");
    cfg.showAltitude  = (server.arg("showAltitude") == "1");
    cfg.showHeading   = (server.arg("showHeading") == "1");
    cfg.showSpeed     = (server.arg("showSpeed") == "1");
    cfg.showDistance  = (server.arg("showDistance") == "1");
    cfg.showAcType    = (server.arg("showAcType") == "1");
    cfg.showVectors   = (server.arg("showVectors") == "1");

    cfg.wxUseRadarLoc = (server.arg("wxUseRadarLoc") == "1");
    cfg.wxLat         = cfg.wxUseRadarLoc ? cfg.lat : server.arg("wxLat").toDouble();
    cfg.wxLon         = cfg.wxUseRadarLoc ? cfg.lon : server.arg("wxLon").toDouble();
    if (server.hasArg("unitTemp"))   cfg.unitTemp   = server.arg("unitTemp").toInt();
    if (server.hasArg("unitPrecip")) cfg.unitPrecip = server.arg("unitPrecip").toInt();

    if (server.hasArg("clockStyle")) cfg.clockStyle = server.arg("clockStyle").toInt();
    if (server.hasArg("tzPosix"))    cfg.tzPosix    = server.arg("tzPosix");
    if (server.hasArg("tzLabel"))    cfg.tzLabel    = server.arg("tzLabel");
    if (server.hasArg("brightness")) cfg.brightness = constrain(server.arg("brightness").toInt(), 10, 100);

    if (fabs(oldLat - cfg.lat) > 0.0001 || fabs(oldLon - cfg.lon) > 0.0001 ||
        fabs(oldRad - cfg.radiusKm) > 1.0f || oldSty != cfg.mapStyle) {
      mapChanged = true;
    }

    saveConfig();
    lcd.setBrightness(map(cfg.brightness, 10, 100, 40, 255));
    configTzTime(cfg.tzPosix.c_str(), "pool.ntp.org", "time.nist.gov");

    if (mapChanged) {
      buildVectorMapSprite();
    }
    if (!isApMode) {
      if (cfg.viewMode == 0) startAircraftFetchBackground();
      else startWeatherFetchBackground();
    }

    server.send(200, "application/json", needReboot ? "{\"reboot\":true}" : "{\"reboot\":false}");
    if (needReboot || isApMode) { delay(1200); ESP.restart(); }
  });

  server.on("/api/restart", HTTP_POST, []() { server.send(200, "text/plain", "OK"); delay(500); ESP.restart(); });
  server.on("/api/reset", HTTP_POST, []() {
    preferences.begin("radar360", false); preferences.clear(); preferences.end();
    server.send(200, "text/plain", "OK"); delay(500); ESP.restart();
  });
  server.onNotFound([]() {
    if (isApMode) { server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true); server.send(302, "text/plain", ""); }
    else server.send(404, "text/plain", "Not Found");
  });
  server.begin();
}

void runApSetupPortal() {
  isApMode = true;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID);
  delay(200);
  IPAddress apIP = WiFi.softAPIP();

  Serial.println("[AP] Setup portal active at http://" + apIP.toString());

  lcd.fillScreen(TFT_BLACK);
  lcd.drawRoundRect(16, 16, SCREEN_W - 32, SCREEN_H - 32, 12, greenBright);
  lcd.setTextDatum(middle_center);
  lcd.setTextSize(3); lcd.setTextColor(TFT_YELLOW, TFT_BLACK);    lcd.drawString("RADAR 360 SETUP (7.0\" HMI)", SCREEN_W / 2, 85);
  lcd.setTextSize(2); lcd.setTextColor(TFT_WHITE, TFT_BLACK);     lcd.drawString("1. Connect Wi-Fi to:", SCREEN_W / 2, 160);
  lcd.setTextSize(3); lcd.setTextColor(greenBright, TFT_BLACK);   lcd.drawString(AP_SSID, SCREEN_W / 2, 205);
  lcd.setTextSize(2); lcd.setTextColor(TFT_WHITE, TFT_BLACK);     lcd.drawString("2. Open Browser:", SCREEN_W / 2, 280);
  lcd.setTextSize(3); lcd.setTextColor(colRiver, TFT_BLACK);      lcd.drawString("http://" + apIP.toString(), SCREEN_W / 2, 325);
  lcd.setTextSize(2); lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK); lcd.drawString("Waiting for Web Config...", SCREEN_W / 2, 405);

  dnsServer.start(53, "*", apIP);
  setupWebServer();
  while (true) { dnsServer.processNextRequest(); server.handleClient(); delay(5); }
}

// ==========================================
// 8. 실시간 날씨 API (Open-Meteo)
// ==========================================
const char* degToCompass(int deg) {
  const char* dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
  return dirs[((deg + 22) % 360) / 45];
}

void fetchWeatherData() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (ESP.getFreeHeap() < 42000) return;

  double lat = cfg.wxUseRadarLoc ? cfg.lat : cfg.wxLat;
  double lon = cfg.wxUseRadarLoc ? cfg.lon : cfg.wxLon;

  String url = "https://api.open-meteo.com/v1/forecast?latitude=" + String(lat, 4) +
               "&longitude=" + String(lon, 4) +
               "&current=temperature_2m,relative_humidity_2m,apparent_temperature,precipitation,weather_code,cloud_cover,surface_pressure,wind_speed_10m,wind_direction_10m,wind_gusts_10m"
               "&daily=weather_code,temperature_2m_max,temperature_2m_min,precipitation_probability_max&forecast_days=3&timezone=auto";

  WiFiClientSecure client; client.setInsecure(); client.setTimeout(10);
  HTTPClient http;
  http.useHTTP10(true);
  http.setConnectTimeout(6000);
  http.setTimeout(10000);
  if (!http.begin(client, url)) return;

  if (http.GET() == 200) {
    String payload = http.getString();
#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    StaticJsonDocument<2560> doc;
#endif
    if (!deserializeJson(doc, payload)) {
      JsonObject cur = doc["current"];
      JsonObject dly = doc["daily"];
      WeatherData tmp;
      tmp.valid       = true;
      tmp.tempC       = cur["temperature_2m"] | 0.0f;
      tmp.feelsLikeC  = cur["apparent_temperature"] | 0.0f;
      tmp.humidity    = cur["relative_humidity_2m"] | 0;
      tmp.cloudCover  = cur["cloud_cover"] | 0;
      tmp.precipMm    = cur["precipitation"] | 0.0f;
      tmp.windKmh     = cur["wind_speed_10m"] | 0.0f;
      tmp.gustKmh     = cur["wind_gusts_10m"] | 0.0f;
      tmp.windDir     = cur["wind_direction_10m"] | 0;
      tmp.pressureHpa = cur["surface_pressure"] | 1013.0f;
      tmp.wmoCode     = cur["weather_code"] | 0;

      for (int i = 0; i < 3; i++) {
        tmp.dayCode[i] = dly["weather_code"][i] | 0;
        tmp.dayMaxC[i] = dly["temperature_2m_max"][i] | 0.0f;
        tmp.dayMinC[i] = dly["temperature_2m_min"][i] | 0.0f;
        tmp.dayPop[i]  = dly["precipitation_probability_max"][i] | 0;
      }
      noInterrupts(); pendingWxData = tmp; pendingWxReady = true; interrupts();
    }
  }
  http.end();
}

void weatherFetchTask(void *parameter) {
  fetchWeatherData();
  wxFetchInProgress = false;
  netBusy = false;
  vTaskDelete(NULL);
}

void startWeatherFetchBackground() {
  if (wxFetchInProgress || netBusy) return;
  netBusy = true;
  wxFetchInProgress = true;
  xTaskCreatePinnedToCore(weatherFetchTask, "weatherFetchTask", 20480, NULL, 1, NULL, 0);
}

void applyPendingWeatherData() {
  if (!pendingWxReady) return;
  noInterrupts(); wxData = pendingWxData; pendingWxReady = false; interrupts();
}

// ==========================================
// 9. ADS-B 실시간 항공기 데이터 수신
// ==========================================
const int MAX_PLANES = 40;
AircraftPoint planes[MAX_PLANES], pendingPlanes[MAX_PLANES];
int planeCount = 0, pendingPlaneCount = 0;
bool apiOk = false, pendingApiOk = false, fetchInProgress = false, pendingDataReady = false;
String apiStatusMsg = "WAIT...";

bool tryFetchFromUrl(const String &url) {
  if (ESP.getFreeHeap() < 42000) {
    apiStatusMsg = "LOW MEM";
    return false;
  }

  WiFiClientSecure client; client.setInsecure(); client.setTimeout(12);
  HTTPClient http;
  http.useHTTP10(true);
  http.setConnectTimeout(8000); http.setTimeout(12000); http.setReuse(false);
  http.addHeader("User-Agent", "ESP32-AirRadar/1.0");
  http.addHeader("Accept", "application/json");
  http.addHeader("Accept-Encoding", "identity");

  if (!http.begin(client, url)) { apiStatusMsg = "BEGIN ERR"; return false; }
  int code = http.GET();
  if (code != 200) { apiStatusMsg = "HTTP " + String(code); http.end(); return false; }
  String payload = http.getString(); http.end();
  if (payload.length() < 20) { apiStatusMsg = "EMPTY"; return false; }

#if ARDUINOJSON_VERSION_MAJOR >= 7
  JsonDocument filter;
  JsonObject acF = filter["ac"][0].to<JsonObject>();
  acF["hex"]=true; acF["lat"]=true; acF["lon"]=true; acF["alt_baro"]=true; acF["alt_geom"]=true; acF["gs"]=true; acF["track"]=true; acF["flight"]=true; acF["t"]=true; acF["category"]=true;
  JsonObject airF = filter["aircraft"][0].to<JsonObject>();
  airF["hex"]=true; airF["lat"]=true; airF["lon"]=true; airF["alt_baro"]=true; airF["alt_geom"]=true; airF["gs"]=true; airF["track"]=true; airF["flight"]=true; airF["t"]=true; airF["category"]=true;
  SpiRamAllocator allocator; JsonDocument doc(&allocator);
#else
  StaticJsonDocument<512> filter;
  JsonObject acF = filter["ac"].createNestedObject();
  acF["hex"]=true; acF["lat"]=true; acF["lon"]=true; acF["alt_baro"]=true; acF["alt_geom"]=true; acF["gs"]=true; acF["track"]=true; acF["flight"]=true; acF["t"]=true; acF["category"]=true;
  JsonObject airF = filter["aircraft"].createNestedObject();
  airF["hex"]=true; airF["lat"]=true; airF["lon"]=true; airF["alt_baro"]=true; airF["alt_geom"]=true; airF["gs"]=true; airF["track"]=true; airF["flight"]=true; airF["t"]=true; airF["category"]=true;
  SpiRamJsonDocument doc(131072);
#endif

  if (deserializeJson(doc, payload, DeserializationOption::Filter(filter))) { apiStatusMsg = "JSON ERR"; return false; }

  AircraftPoint tempPlanes[MAX_PLANES];
  int tempCount = 0, limitCount = min(cfg.maxAc, MAX_PLANES);

  JsonArray arr = doc["ac"].as<JsonArray>();
  if (arr.isNull() || arr.size() == 0) arr = doc["aircraft"].as<JsonArray>();

  for (JsonObject ac : arr) {
    if (tempCount >= limitCount) break;
    if (!ac["lat"].is<float>() || !ac["lon"].is<float>()) continue;
    double lat = ac["lat"], lon = ac["lon"];
    float d = distanceKm(cfg.lat, cfg.lon, lat, lon);
    if (d > cfg.radiusKm) continue;

    int alt = ac["alt_baro"].is<int>() ? (int)ac["alt_baro"] : (ac["alt_geom"].is<int>() ? (int)ac["alt_geom"] : -1);
    if (alt >= 0 && (alt < cfg.minAltFt || alt > cfg.maxAltFt)) continue;

    AircraftPoint &p = tempPlanes[tempCount++];
    p.valid = true; p.lat = lat; p.lon = lon;
    p.distanceKm = d; p.bearingDeg = bearingDeg(cfg.lat, cfg.lon, lat, lon);
    p.altitudeFt = alt; p.speedKt = ac["gs"] | 0.0f; p.trackDeg = ac["track"] | 0.0f;
    strncpy(p.hex, ac["hex"] | "", sizeof(p.hex) - 1); p.hex[sizeof(p.hex) - 1] = '\0';
    strncpy(p.flight, ac["flight"] | "", sizeof(p.flight) - 1); p.flight[sizeof(p.flight) - 1] = '\0';
    strncpy(p.category, ac["category"] | "", sizeof(p.category) - 1); p.category[sizeof(p.category) - 1] = '\0';
    strncpy(p.typeCode, ac["t"] | "", sizeof(p.typeCode) - 1); p.typeCode[sizeof(p.typeCode) - 1] = '\0';
  }

  for (int i = 0; i < tempCount - 1; i++) {
    for (int j = i + 1; j < tempCount; j++) {
      if (tempPlanes[j].distanceKm < tempPlanes[i].distanceKm) {
        AircraftPoint swapTmp = tempPlanes[i];
        tempPlanes[i] = tempPlanes[j];
        tempPlanes[j] = swapTmp;
      }
    }
  }

  noInterrupts();
  pendingPlaneCount = tempCount;
  for (int i = 0; i < tempCount; i++) pendingPlanes[i] = tempPlanes[i];
  pendingApiOk = true; pendingDataReady = true;
  interrupts();
  Serial.printf("[ADS-B] Aircraft found: %d\n", tempCount);
  return true;
}

void fetchAircraftData() {
  if (WiFi.status() != WL_CONNECTED) { apiStatusMsg = "NO WIFI"; pendingApiOk = false; pendingPlaneCount = 0; pendingDataReady = true; return; }
  int apiRadiusNm = max(10, (int)roundf(cfg.radiusKm / 1.852f));
  String path = String(cfg.lat, 4) + "/" + String(cfg.lon, 4) + "/" + String(apiRadiusNm);
  if (tryFetchFromUrl("https://api.adsb.lol/v2/point/" + path)) return;
  delay(250);
  if (tryFetchFromUrl("https://opendata.adsb.fi/api/v2/lat/" + String(cfg.lat, 4) + "/lon/" + String(cfg.lon, 4) + "/dist/" + String(apiRadiusNm))) return;
  delay(250);
  if (tryFetchFromUrl("https://api.airplanes.live/v2/point/" + path)) return;
  pendingApiOk = false; pendingDataReady = true;
}

void aircraftFetchTask(void *parameter) {
  fetchAircraftData();
  fetchInProgress = false;
  netBusy = false;
  vTaskDelete(NULL);
}

void startAircraftFetchBackground() {
  if (fetchInProgress || netBusy) return;
  netBusy = true;
  fetchInProgress = true;
  xTaskCreatePinnedToCore(aircraftFetchTask, "aircraftFetchTask", 28672, NULL, 1, NULL, 0);
}

void applyPendingAircraftData() {
  if (!pendingDataReady) return;
  noInterrupts();
  planeCount = pendingPlaneCount;
  for (int i = 0; i < planeCount; i++) planes[i] = pendingPlanes[i];
  apiOk = pendingApiOk; pendingDataReady = false;
  interrupts();

  if (selectedPlaneHex.length() > 0) {
    int foundIdx = -1;
    for (int i = 0; i < planeCount; i++) {
      if (selectedPlaneHex.equalsIgnoreCase(String(planes[i].hex))) {
        foundIdx = i;
        break;
      }
    }
    selectedPlaneIdx = foundIdx;
    if (foundIdx == -1) selectedPlaneHex = "";
  } else if (selectedPlaneIdx >= planeCount) {
    selectedPlaneIdx = -1;
  }
}

// ==========================================
// 10. 레이더 방위 눈금 / 비행기 아이콘 / 기종별 상세 일러스트 렌더링
// ==========================================
void canvasCorrectedRadialLine(int angleDeg, int r1, int r2, uint16_t col) {
  float rad = (angleDeg - 90) * DEG_TO_RAD;
  wideCanvas.drawLine(CX + cos(rad) * r1, CY + sin(rad) * r1,
                      CX + cos(rad) * r2, CY + sin(rad) * r2, col);
}

void rotPt(int cx, int cy, float cosA, float sinA, float lx, float ly, int &ox, int &oy) {
  ox = cx + (int)roundf(lx * cosA - ly * sinA);
  oy = cy + (int)roundf(lx * sinA + ly * cosA);
}

void drawAirplaneIcon(int cx, int cy, float headingDeg, float scale, uint16_t fillCol) {
  float rad = (headingDeg - 90.0f) * DEG_TO_RAD;
  float cosA = cosf(rad), sinA = sinf(rad);

  int noseX, noseY, tailX, tailY, bodyFX, bodyFY, bodyRX, bodyRY;
  int wingLX, wingLY, wingRX, wingRY, stabRootX, stabRootY, stabLX, stabLY, stabRX, stabRY;

  rotPt(cx, cy, cosA, sinA,  8.5f * scale,  0.0f,         noseX, noseY);
  rotPt(cx, cy, cosA, sinA, -7.5f * scale,  0.0f,         tailX, tailY);
  rotPt(cx, cy, cosA, sinA,  2.5f * scale,  0.0f,         bodyFX, bodyFY);
  rotPt(cx, cy, cosA, sinA, -2.2f * scale,  0.0f,         bodyRX, bodyRY);
  rotPt(cx, cy, cosA, sinA, -2.8f * scale, -8.5f * scale, wingLX, wingLY);
  rotPt(cx, cy, cosA, sinA, -2.8f * scale,  8.5f * scale, wingRX, wingRY);
  rotPt(cx, cy, cosA, sinA, -4.8f * scale,  0.0f,         stabRootX, stabRootY);
  rotPt(cx, cy, cosA, sinA, -8.0f * scale, -3.8f * scale, stabLX, stabLY);
  rotPt(cx, cy, cosA, sinA, -8.0f * scale,  3.8f * scale, stabRX, stabRY);

  wideCanvas.drawLine(noseX + 1, noseY + 1, tailX + 1, tailY + 1, TFT_BLACK);
  wideCanvas.fillTriangle(bodyFX + 1, bodyFY + 1, wingLX + 1, wingLY + 1, bodyRX + 1, bodyRY + 1, TFT_BLACK);
  wideCanvas.fillTriangle(bodyFX + 1, bodyFY + 1, wingRX + 1, wingRY + 1, bodyRX + 1, bodyRY + 1, TFT_BLACK);
  wideCanvas.fillTriangle(stabRootX + 1, stabRootY + 1, stabLX + 1, stabLY + 1, tailX + 1, tailY + 1, TFT_BLACK);
  wideCanvas.fillTriangle(stabRootX + 1, stabRootY + 1, stabRX + 1, stabRY + 1, tailX + 1, tailY + 1, TFT_BLACK);

  wideCanvas.fillTriangle(bodyFX, bodyFY, wingLX, wingLY, bodyRX, bodyRY, fillCol);
  wideCanvas.fillTriangle(bodyFX, bodyFY, wingRX, wingRY, bodyRX, bodyRY, fillCol);
  wideCanvas.fillTriangle(stabRootX, stabRootY, stabLX, stabLY, tailX, tailY, fillCol);
  wideCanvas.fillTriangle(stabRootX, stabRootY, stabRX, stabRY, tailX, tailY, fillCol);

  int fL_x, fL_y, fR_x, fR_y, tL_x, tL_y, tR_x, tR_y;
  rotPt(cx, cy, cosA, sinA,  5.5f * scale, -0.9f * scale, fL_x, fL_y);
  rotPt(cx, cy, cosA, sinA,  5.5f * scale,  0.9f * scale, fR_x, fR_y);
  rotPt(cx, cy, cosA, sinA, -7.5f * scale, -0.7f * scale, tL_x, tL_y);
  rotPt(cx, cy, cosA, sinA, -7.5f * scale,  0.7f * scale, tR_x, tR_y);

  wideCanvas.fillTriangle(noseX, noseY, fL_x, fL_y, fR_x, fR_y, fillCol);
  wideCanvas.fillTriangle(fL_x, fL_y, fR_x, fR_y, tL_x, tL_y, fillCol);
  wideCanvas.fillTriangle(fR_x, fR_y, tL_x, tL_y, tR_x, tR_y, fillCol);
}

const char* getAirlineName(const char* flight) {
  if (!flight || strlen(flight) < 3) return "PRIVATE / GA";
  if (strncmp(flight, "KAL", 3) == 0) return "KOREAN AIR";
  if (strncmp(flight, "AAR", 3) == 0) return "ASIANA AIRLINES";
  if (strncmp(flight, "JJA", 3) == 0) return "JEJU AIR";
  if (strncmp(flight, "JNA", 3) == 0) return "JIN AIR";
  if (strncmp(flight, "TWB", 3) == 0) return "T'WAY AIR";
  if (strncmp(flight, "ABL", 3) == 0) return "AIR BUSAN";
  if (strncmp(flight, "ASV", 3) == 0) return "AIR SEOUL";
  if (strncmp(flight, "EOK", 3) == 0) return "EASTAR JET";
  if (strncmp(flight, "APZ", 3) == 0) return "AIR PREMIA";
  if (strncmp(flight, "ANA", 3) == 0) return "ALL NIPPON";
  if (strncmp(flight, "JAL", 3) == 0) return "JAPAN AIRLINES";
  if (strncmp(flight, "APJ", 3) == 0) return "PEACH AVIATION";
  if (strncmp(flight, "CPA", 3) == 0) return "CATHAY PACIFIC";
  if (strncmp(flight, "SIA", 3) == 0) return "SINGAPORE AIR";
  if (strncmp(flight, "THA", 3) == 0) return "THAI AIRWAYS";
  if (strncmp(flight, "EVA", 3) == 0) return "EVA AIR";
  if (strncmp(flight, "CAL", 3) == 0) return "CHINA AIRLINES";
  if (strncmp(flight, "CCA", 3) == 0) return "AIR CHINA";
  if (strncmp(flight, "CES", 3) == 0) return "CHINA EASTERN";
  if (strncmp(flight, "CSN", 3) == 0) return "CHINA SOUTHERN";
  if (strncmp(flight, "UAE", 3) == 0) return "EMIRATES";
  if (strncmp(flight, "QTR", 3) == 0) return "QATAR AIRWAYS";
  if (strncmp(flight, "DAL", 3) == 0) return "DELTA AIR LINES";
  if (strncmp(flight, "UAL", 3) == 0) return "UNITED AIRLINES";
  if (strncmp(flight, "AAL", 3) == 0) return "AMERICAN AIR";
  if (strncmp(flight, "DLH", 3) == 0) return "LUFTHANSA";
  if (strncmp(flight, "BAW", 3) == 0) return "BRITISH AIRWAYS";
  if (strncmp(flight, "AFR", 3) == 0) return "AIR FRANCE";
  if (strncmp(flight, "FDX", 3) == 0) return "FEDEX EXPRESS";
  if (strncmp(flight, "UPS", 3) == 0) return "UPS AIRLINES";
  return "COMMERCIAL / INTL";
}

AcVisualType classifyAircraftVisual(const char* typeCode, const char* category) {
  String t = String(typeCode);
  t.toUpperCase();
  String c = String(category);
  c.toUpperCase();

  if (c == "A7" || t.startsWith("H60") || t.startsWith("EC") || t.startsWith("B407") || t.startsWith("S76") || t.startsWith("R44")) {
    return AC_VIS_HELI;
  }
  if (t.startsWith("B74") || t.startsWith("A38") || t.startsWith("A34") || t.startsWith("B70") || t.startsWith("C17")) {
    return AC_VIS_HEAVY4;
  }
  if (t.startsWith("B77") || t.startsWith("B78") || t.startsWith("A35") || t.startsWith("A33") || t.startsWith("B76") || c == "A5") {
    return AC_VIS_WIDE2;
  }
  if (t.startsWith("DH8") || t.startsWith("AT7") || t.startsWith("AT4") || t.startsWith("C172") || t.startsWith("C208") ||
      t.startsWith("BE2") || t.startsWith("PC12") || t.startsWith("PA28") || c == "A1") {
    return AC_VIS_PROP;
  }
  return AC_VIS_JET2;
}

void drawAircraftIllustrationBox(int bx, int by, int bw, int bh, const AircraftPoint &p) {
  uint16_t boxBg   = wideCanvas.color565(8, 26, 38);
  uint16_t gridCol = wideCanvas.color565(18, 52, 68);
  uint16_t bodyCol = wideCanvas.color565(225, 245, 255);
  uint16_t wingCol = wideCanvas.color565(110, 210, 245);
  uint16_t engCol  = wideCanvas.color565(255, 210, 60);

  wideCanvas.fillRoundRect(bx, by, bw, bh, 6, boxBg);
  wideCanvas.drawRoundRect(bx, by, bw, bh, 6, greenDim);

  for (int gx = bx + 16; gx < bx + bw; gx += 16) wideCanvas.drawFastVLine(gx, by + 2, bh - 4, gridCol);
  for (int gy = by + 16; gy < by + bh - 16; gy += 16) wideCanvas.drawFastHLine(bx + 2, gy, bw - 4, gridCol);

  int cx = bx + bw / 2;
  int cy = by + (bh - 16) / 2 + 2;

  AcVisualType vis = classifyAircraftVisual(p.typeCode, p.category);

  if (vis == AC_VIS_HELI) {
    wideCanvas.fillRoundRect(cx - 22, cy - 8, 34, 18, 8, bodyCol);
    wideCanvas.fillRoundRect(cx + 2, cy - 5, 10, 9, 3, colHudCyan);
    wideCanvas.fillRect(cx - 44, cy - 3, 24, 5, wingCol);
    wideCanvas.fillTriangle(cx - 46, cy - 12, cx - 46, cy + 5, cx - 38, cy - 2, engCol);
    wideCanvas.drawFastHLine(cx - 46, cy - 13, 84, TFT_WHITE);
    wideCanvas.drawFastHLine(cx - 44, cy - 12, 80, colHudCyan);
    wideCanvas.fillRect(cx - 6, cy - 12, 4, 4, engCol);
    wideCanvas.drawFastHLine(cx - 20, cy + 13, 30, wingCol);
    wideCanvas.drawLine(cx - 14, cy + 9, cx - 16, cy + 13, wingCol);
    wideCanvas.drawLine(cx + 4, cy + 9, cx + 6, cy + 13, wingCol);
  }
  else if (vis == AC_VIS_PROP) {
    wideCanvas.fillTriangle(cx, cy - 34, cx - 4, cy - 18, cx + 4, cy - 18, bodyCol);
    wideCanvas.fillRect(cx - 4, cy - 18, 8, 42, bodyCol);
    wideCanvas.fillTriangle(cx - 4, cy + 24, cx + 4, cy + 24, cx, cy + 34, bodyCol);
    wideCanvas.fillRect(cx - 42, cy - 8, 84, 9, wingCol);
    wideCanvas.fillRect(cx - 15, cy + 24, 30, 5, wingCol);
    wideCanvas.fillRect(cx - 19, cy - 13, 5, 12, engCol);
    wideCanvas.fillRect(cx + 14, cy - 13, 5, 12, engCol);
    wideCanvas.drawFastHLine(cx - 26, cy - 14, 19, TFT_WHITE);
    wideCanvas.drawFastHLine(cx + 7,  cy - 14, 19, TFT_WHITE);
    wideCanvas.drawFastVLine(cx, cy + 22, 12, engCol);
  }
  else {
    int fuseW  = (vis == AC_VIS_JET2) ? 5 : 7;
    int span   = (vis == AC_VIS_JET2) ? 38 : ((vis == AC_VIS_WIDE2) ? 45 : 49);
    int noseY  = cy - 35;
    int tailY  = cy + 34;

    wideCanvas.fillTriangle(cx - fuseW, cy - 12, cx - fuseW, cy + 4,  cx - span, cy + 14, wingCol);
    wideCanvas.fillTriangle(cx - fuseW, cy + 4,  cx - span, cy + 14, cx - span + 3, cy + 19, wingCol);
    wideCanvas.fillTriangle(cx + fuseW, cy - 12, cx + fuseW, cy + 4,  cx + span, cy + 14, wingCol);
    wideCanvas.fillTriangle(cx + fuseW, cy + 4,  cx + span, cy + 14, cx + span - 3, cy + 19, wingCol);

    int stabSpan = (vis == AC_VIS_JET2) ? 15 : 19;
    wideCanvas.fillTriangle(cx - 2, cy + 21, cx - 2, cy + 30, cx - stabSpan, cy + 34, wingCol);
    wideCanvas.fillTriangle(cx + 2, cy + 21, cx + 2, cy + 30, cx + stabSpan, cy + 34, wingCol);

    wideCanvas.fillTriangle(cx, noseY, cx - fuseW, cy - 21, cx + fuseW, cy - 21, bodyCol);
    wideCanvas.fillRect(cx - fuseW, cy - 21, fuseW * 2 + 1, 44, bodyCol);
    wideCanvas.fillTriangle(cx - fuseW, cy + 23, cx + fuseW, cy + 23, cx, tailY, bodyCol);

    wideCanvas.drawFastHLine(cx - 2, cy - 25, 5, boxBg);
    wideCanvas.drawFastHLine(cx - 3, cy - 24, 7, boxBg);

    wideCanvas.fillRoundRect(cx - 18, cy - 8, 5, 11, 2, engCol);
    wideCanvas.fillRoundRect(cx + 14, cy - 8, 5, 11, 2, engCol);
    if (vis == AC_VIS_HEAVY4) {
      wideCanvas.fillRoundRect(cx - 32, cy + 1, 5, 10, 2, engCol);
      wideCanvas.fillRoundRect(cx + 28, cy + 1, 5, 10, 2, engCol);
    }

    wideCanvas.drawFastVLine(cx, cy + 19, 15, TFT_RED);
  }

  String typStr = String(p.typeCode); typStr.trim();
  if (typStr.length() == 0) typStr = "AIRCRAFT";
  const char* classTag = (vis == AC_VIS_HEAVY4) ? "HEAVY 4J" :
                         ((vis == AC_VIS_WIDE2) ? "WIDEBODY" :
                         ((vis == AC_VIS_PROP)  ? "PROP/GA"  :
                         ((vis == AC_VIS_HELI)  ? "ROTOR"    : "JET")));

  wideCanvas.fillRect(bx + 2, by + bh - 16, bw - 4, 14, wideCanvas.color565(12, 38, 48));
  wideCanvas.setTextDatum(middle_center);
  wideCanvas.setTextSize(1);
  wideCanvas.setTextColor(TFT_YELLOW);
  wideCanvas.drawString(typStr + " · " + String(classTag), cx, by + bh - 9);
}

void drawBottomRightSelectedAircraftPanel() {
  AircraftPoint &p = planes[selectedPlaneIdx];
  const int bx = INFO_X;   // 482
  const int by = 288;
  const int bw = INFO_W;   // 312
  const int bh = 186;      // 288 ~ 474

  wideCanvas.fillRoundRect(bx, by, bw, bh, 8, wideCanvas.color565(6, 20, 24));
  wideCanvas.drawRoundRect(bx, by, bw, bh, 8, colHudCyan);

  wideCanvas.fillRoundRect(bx + bw - 64, by + 6, 56, 22, 5, wideCanvas.color565(90, 22, 22));
  wideCanvas.drawRoundRect(bx + bw - 64, by + 6, 56, 22, 5, TFT_RED);
  wideCanvas.setTextDatum(middle_center);
  wideCanvas.setTextSize(1);
  wideCanvas.setTextColor(TFT_WHITE);
  wideCanvas.drawString("CLOSE", bx + bw - 36, by + 17);

  String fl = String(p.flight); fl.trim();
  if (fl.length() == 0) fl = "UNKNOWN";
  const char* airline = getAirlineName(fl.c_str());

  wideCanvas.setTextDatum(top_left);
  wideCanvas.setTextSize(2);
  wideCanvas.setTextColor(colHudCyan);
  wideCanvas.drawString(fl, bx + 10, by + 8);

  wideCanvas.setTextSize(1);
  wideCanvas.setTextColor(TFT_LIGHTGREY);
  wideCanvas.drawString(airline, bx + 105, by + 13);

  wideCanvas.drawFastHLine(bx + 8, by + 32, bw - 16, greenDim);

  drawAircraftIllustrationBox(bx + 8, by + 37, 112, 104, p);

  int tx = bx + 126;
  String hexStr = String(p.hex); hexStr.toUpperCase();

  String altStr = "---";
  if (p.altitudeFt >= 0) {
    int altVal = (cfg.unitAlt == 1) ? p.altitudeFt : (int)(p.altitudeFt * 0.3048f);
    altStr = String(altVal) + (cfg.unitAlt == 1 ? "ft" : "m") + " FL" + String(p.altitudeFt / 100);
  }

  String spdStr = (cfg.unitSpeed == 1)
                  ? String((int)roundf(p.speedKt)) + "kt (" + String((int)roundf(p.speedKt * 1.852f)) + "k)"
                  : String((int)roundf(p.speedKt * 1.852f)) + "km/h";

  String hdgStr = String((int)roundf(p.trackDeg)) + "deg (" + String(degToCompass((int)roundf(p.trackDeg))) + ")";

  String dstStr = (cfg.unitDist == 1)
                  ? String(p.distanceKm / 1.852f, 1) + "nm / " + String((int)roundf(p.bearingDeg)) + "deg"
                  : String(p.distanceKm, 1) + "km / " + String((int)roundf(p.bearingDeg)) + "deg";

  wideCanvas.setTextSize(1);
  wideCanvas.setTextColor(green);        wideCanvas.drawString("HEX:", tx,      by + 40);
  wideCanvas.setTextColor(TFT_WHITE);    wideCanvas.drawString(hexStr,   tx + 30, by + 40);

  wideCanvas.setTextColor(green);        wideCanvas.drawString("ALT:", tx,      by + 58);
  wideCanvas.setTextColor(greenBright);  wideCanvas.drawString(altStr,   tx + 30, by + 58);

  wideCanvas.setTextColor(green);        wideCanvas.drawString("SPD:", tx,      by + 76);
  wideCanvas.setTextColor(TFT_YELLOW);   wideCanvas.drawString(spdStr,   tx + 30, by + 76);

  wideCanvas.setTextColor(green);        wideCanvas.drawString("HDG:", tx,      by + 94);
  wideCanvas.setTextColor(colHudCyan);   wideCanvas.drawString(hdgStr,   tx + 30, by + 94);

  wideCanvas.setTextColor(green);        wideCanvas.drawString("DST:", tx,      by + 112);
  wideCanvas.setTextColor(TFT_WHITE);    wideCanvas.drawString(dstStr,   tx + 30, by + 112);

  wideCanvas.setTextColor(green);        wideCanvas.drawString("POS:", tx,      by + 130);
  wideCanvas.setTextColor(TFT_LIGHTGREY);wideCanvas.drawString(String(p.lat, 2) + "," + String(p.lon, 2), tx + 30, by + 130);

  wideCanvas.drawFastHLine(bx + 8, by + 146, bw - 16, greenDark);

  struct tm timeinfo;
  char timeStr[12] = "--:--:--";
  if (getLocalTime(&timeinfo)) sprintf(timeStr, "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

  wideCanvas.setTextDatum(middle_left);
  wideCanvas.setTextSize(2);
  wideCanvas.setTextColor(TFT_YELLOW);
  wideCanvas.drawString(timeStr, bx + 10, by + 166);

  wideCanvas.fillRoundRect(bx + 150, by + 153, 76, 24, 4, wideCanvas.color565(18, 44, 62));
  wideCanvas.drawRoundRect(bx + 150, by + 153, 76, 24, 4, colHudCyan);
  wideCanvas.setTextDatum(middle_center);
  wideCanvas.setTextSize(1);
  wideCanvas.setTextColor(colHudCyan);
  wideCanvas.drawString("REGION", bx + 188, by + 165);

  wideCanvas.fillRoundRect(bx + 232, by + 153, 72, 24, 4, wideCanvas.color565(16, 48, 34));
  wideCanvas.drawRoundRect(bx + 232, by + 153, 72, 24, 4, greenBright);
  wideCanvas.setTextColor(greenBright);
  wideCanvas.drawString("MODE >", bx + 268, by + 165);
}

void drawRightPanelsToCanvas() {
  int topY = INFO_Y;
  int topH = 276;
  wideCanvas.fillRoundRect(INFO_X, topY, INFO_W, topH, 8, colCardBg);
  wideCanvas.drawRoundRect(INFO_X, topY, INFO_W, topH, 8, greenDim);

  wideCanvas.setTextDatum(top_left);
  wideCanvas.setTextSize(2);
  wideCanvas.setTextColor(greenBright);
  wideCanvas.drawString("DETECTED AIRCRAFTS", INFO_X + 14, topY + 10);

  int hdrY = topY + 36;
  wideCanvas.setTextSize(1);
  wideCanvas.setTextColor(green);
  wideCanvas.drawString("CALLSIGN", INFO_X + 12,  hdrY);
  wideCanvas.drawString("DIST",     INFO_X + 86,  hdrY);
  wideCanvas.drawString("ALT",      INFO_X + 140, hdrY);
  wideCanvas.drawString("SPD",      INFO_X + 204, hdrY);
  wideCanvas.drawString("TYPE",     INFO_X + 260, hdrY);
  wideCanvas.drawFastHLine(INFO_X + 10, hdrY + 13, INFO_W - 20, greenDim);

  int maxRows = min(planeCount, 8);
  for (int i = 0; i < maxRows; i++) {
    int ry = topY + 55 + i * 27;
    if (i > 0) wideCanvas.drawFastHLine(INFO_X + 10, ry - 4, INFO_W - 20, greenDark);

    if (i == selectedPlaneIdx) {
      wideCanvas.fillRoundRect(INFO_X + 6, ry - 2, INFO_W - 12, 23, 4, colSelBg);
      wideCanvas.drawRoundRect(INFO_X + 6, ry - 2, INFO_W - 12, 23, 4, colHudCyan);
    }

    uint16_t rowCol = greenBright;
    if (planes[i].altitudeFt >= 30000)      rowCol = colHudCyan;
    else if (planes[i].altitudeFt >= 10000) rowCol = greenBright;
    else if (planes[i].altitudeFt >= 0)     rowCol = TFT_YELLOW;
    else                                    rowCol = wideCanvas.color565(255, 80, 80);

    String fl = String(planes[i].flight); fl.trim();
    if (fl.length() == 0) fl = "UNK";
    if (fl.length() > 7) fl = fl.substring(0, 7);

    String dStr = (cfg.unitDist == 1)
                  ? String((int)roundf(planes[i].distanceKm / 1.852f)) + "nm"
                  : String((int)roundf(planes[i].distanceKm)) + "km";

    String aStr = "---";
    if (planes[i].altitudeFt >= 0) {
      int altVal = (cfg.unitAlt == 1) ? planes[i].altitudeFt : (int)(planes[i].altitudeFt * 0.3048f);
      char altBuf[12];
      if (altVal >= 1000) sprintf(altBuf, "%d,%03d", altVal / 1000, altVal % 1000);
      else sprintf(altBuf, "%d", altVal);
      aStr = String(altBuf);
    }

    String sStr = (cfg.unitSpeed == 1)
                  ? String((int)roundf(planes[i].speedKt)) + "kt"
                  : String((int)roundf(planes[i].speedKt * 1.852f)) + "k";

    String tStr = String(planes[i].typeCode); tStr.trim();
    if (tStr.length() == 0) tStr = String(planes[i].category);
    if (tStr.length() == 0) tStr = "----";

    wideCanvas.setTextDatum(top_left);
    wideCanvas.setTextSize(1);
    wideCanvas.setTextColor(rowCol);
    wideCanvas.drawString(fl,   INFO_X + 12,  ry + 3);
    wideCanvas.setTextColor(TFT_WHITE);
    wideCanvas.drawString(dStr, INFO_X + 86,  ry + 3);
    wideCanvas.drawString(aStr, INFO_X + 140, ry + 3);
    wideCanvas.drawString(sStr, INFO_X + 204, ry + 3);
    wideCanvas.setTextColor(TFT_LIGHTGREY);
    wideCanvas.drawString(tStr.substring(0, 5), INFO_X + 260, ry + 3);
  }

  if (planeCount == 0) {
    wideCanvas.setTextDatum(middle_center);
    wideCanvas.setTextSize(1);
    wideCanvas.setTextColor(greenDim);
    wideCanvas.drawString("SCANNING AIRSPACE...", INFO_X + INFO_W / 2, topY + 148);
  }

  if (selectedPlaneIdx >= 0 && selectedPlaneIdx < planeCount && planes[selectedPlaneIdx].valid) {
    drawBottomRightSelectedAircraftPanel();
    return;
  }

  int midY = 288;
  int midH = 110;
  wideCanvas.fillRoundRect(INFO_X, midY, INFO_W, midH, 8, colCardBg);
  wideCanvas.drawRoundRect(INFO_X, midY, INFO_W, midH, 8, greenDim);

  const AirportInfo* ap = getCurrentAirportInfo();
  String apTitle = ap ? String(ap->title) : (cfg.presetName + " RADAR");
  String elevStr = ap ? String(ap->elev)  : "N/A";
  String freqStr = ap ? String(ap->freq)  : "121.50 MHz";

  wideCanvas.setTextDatum(top_left);
  wideCanvas.setTextSize(2);
  wideCanvas.setTextColor(greenBright);
  wideCanvas.drawString(apTitle, INFO_X + 12, midY + 8);

  wideCanvas.fillRoundRect(INFO_X + INFO_W - 68, midY + 6, 58, 20, 4, wideCanvas.color565(18, 44, 62));
  wideCanvas.drawRoundRect(INFO_X + INFO_W - 68, midY + 6, 58, 20, 4, colHudCyan);
  wideCanvas.setTextDatum(middle_center);
  wideCanvas.setTextSize(1);
  wideCanvas.setTextColor(colHudCyan);
  wideCanvas.drawString("SELECT", INFO_X + INFO_W - 39, midY + 16);

  wideCanvas.setTextDatum(top_left);
  wideCanvas.setTextColor(TFT_WHITE);
  wideCanvas.drawString(String(cfg.lat, 4) + " / " + String(cfg.lon, 4), INFO_X + 12, midY + 33);
  wideCanvas.setTextColor(TFT_LIGHTGREY);
  wideCanvas.drawString("ELEV: " + elevStr, INFO_X + 174, midY + 33);

  wideCanvas.setTextColor(colRiver);
  wideCanvas.drawString("FREQ: " + freqStr, INFO_X + 12, midY + 51);

  String wndStr = "WND: ---/--KT";
  if (wxData.valid) {
    char wBuf[24];
    float wSpd = (cfg.unitSpeed == 1) ? (wxData.windKmh / 1.852f) : wxData.windKmh;
    sprintf(wBuf, "WND: %03d/%02d%s", wxData.windDir, (int)roundf(wSpd), (cfg.unitSpeed == 1) ? "KT" : "K");
    wndStr = String(wBuf);
  }
  wideCanvas.setTextColor(greenBright);
  wideCanvas.drawString(wndStr, INFO_X + 174, midY + 51);

  wideCanvas.drawFastHLine(INFO_X + 10, midY + 67, INFO_W - 20, greenDark);
  wideCanvas.setTextColor(TFT_YELLOW);
  wideCanvas.drawString("WEB: http://" + cfg.hostname + ".local", INFO_X + 12, midY + 75);
  String ipStr = (WiFi.status() == WL_CONNECTED) ? ("IP : http://" + WiFi.localIP().toString()) : "IP : AP Mode";
  wideCanvas.setTextColor(colHudCyan);
  wideCanvas.drawString(ipStr, INFO_X + 12, midY + 91);

  int botY = 404;
  int botH = 70;
  wideCanvas.fillRoundRect(INFO_X, botY, INFO_W, botH, 8, colCardBg);
  wideCanvas.drawRoundRect(INFO_X, botY, INFO_W, botH, 8, greenDim);

  wideCanvas.setTextDatum(middle_left);
  wideCanvas.setTextSize(2);
  wideCanvas.setTextColor(greenBright);
  wideCanvas.drawString("TIME:", INFO_X + 12, botY + 35);

  struct tm timeinfo;
  char timeStr[12] = "--:--:--";
  bool ntpOk = getLocalTime(&timeinfo);
  if (ntpOk) sprintf(timeStr, "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

  wideCanvas.setTextSize(3);
  wideCanvas.setTextColor(TFT_YELLOW);
  wideCanvas.drawString(timeStr, INFO_X + 76, botY + 36);

  uint16_t fixCol = (WiFi.status() == WL_CONNECTED) ? greenBright : TFT_RED;
  wideCanvas.fillCircle(INFO_X + 232, botY + 23, 4, fixCol);
  wideCanvas.setTextSize(1);
  wideCanvas.setTextColor(fixCol);
  wideCanvas.drawString(ntpOk ? "GPS: FIX" : "SYNC...", INFO_X + 241, botY + 23);

  wideCanvas.fillRoundRect(INFO_X + 230, botY + 37, 72, 22, 4, wideCanvas.color565(16, 48, 34));
  wideCanvas.drawRoundRect(INFO_X + 230, botY + 37, 72, 22, 4, greenBright);
  wideCanvas.setTextDatum(middle_center);
  wideCanvas.setTextColor(greenBright);
  wideCanvas.drawString("MODE >", INFO_X + 266, botY + 48);
}

// ==========================================
// 11. 메인 화면 지역(공항) 선택 팝업 오버레이
// ==========================================
void drawRegionPickerModal() {
  const int mx = 56;
  const int my = 32;
  const int mw = 688;
  const int mh = 416;

  wideCanvas.fillRoundRect(mx, my, mw, mh, 12, wideCanvas.color565(5, 16, 14));
  wideCanvas.drawRoundRect(mx, my, mw, mh, 12, greenBright);
  wideCanvas.drawRoundRect(mx + 1, my + 1, mw - 2, mh - 2, 11, colHudCyan);

  wideCanvas.setTextDatum(top_left);
  wideCanvas.setTextSize(2);
  wideCanvas.setTextColor(greenBright);
  wideCanvas.drawString("SELECT RADAR REGION / AIRPORT", mx + 18, my + 14);

  wideCanvas.fillRoundRect(mx + mw - 84, my + 10, 68, 28, 6, wideCanvas.color565(90, 22, 22));
  wideCanvas.drawRoundRect(mx + mw - 84, my + 10, 68, 28, 6, TFT_RED);
  wideCanvas.setTextDatum(middle_center);
  wideCanvas.setTextSize(1);
  wideCanvas.setTextColor(TFT_WHITE);
  wideCanvas.drawString("CLOSE", mx + mw - 50, my + 24);

  wideCanvas.drawFastHLine(mx + 14, my + 46, mw - 28, greenDim);

  const int cols = 3;
  const int gridX = mx + 16;
  const int gridY = my + 56;
  const int cellW = 212;
  const int cellH = 62;
  const int gapX  = 10;
  const int gapY  = 8;

  for (int i = 0; i < AIRPORTS_LEN; i++) {
    int col = i % cols;
    int row = i / cols;
    int cx = gridX + col * (cellW + gapX);
    int cy = gridY + row * (cellH + gapY);

    bool isCurrent = cfg.presetName.equalsIgnoreCase(AIRPORTS[i].key);

    wideCanvas.fillRoundRect(cx, cy, cellW, cellH, 8, isCurrent ? colSelBg : wideCanvas.color565(10, 24, 20));
    wideCanvas.drawRoundRect(cx, cy, cellW, cellH, 8, isCurrent ? colHudCyan : greenDim);

    wideCanvas.setTextDatum(top_left);
    wideCanvas.setTextSize(2);
    wideCanvas.setTextColor(isCurrent ? colHudCyan : greenBright);
    wideCanvas.drawString(String(AIRPORTS[i].code) + " " + String(AIRPORTS[i].key), cx + 10, cy + 10);

    wideCanvas.setTextSize(1);
    wideCanvas.setTextColor(isCurrent ? TFT_YELLOW : TFT_LIGHTGREY);
    char coordBuf[32];
    sprintf(coordBuf, "%.2f / %.2f  (%s)", AIRPORTS[i].lat, AIRPORTS[i].lon, AIRPORTS[i].freq);
    wideCanvas.drawString(coordBuf, cx + 10, cy + 38);
  }
}

void drawFlightRadarToCanvas() {
  wideCanvas.fillScreen(TFT_BLACK);

  if (cfg.showMapBg) {
    mapSprite.pushSprite(&wideCanvas, CX - MAP_HALF, CY - MAP_HALF);
  }

  if (cfg.showScanLine) {
    for (float i = TRAIL_WIDTH; i >= 0; i -= 2.5f) {
      float a = sweepAngle - i;
      if (a < 0) a += 360;
      float intensity = (float)(TRAIL_WIDTH - i) / TRAIL_WIDTH;
      uint8_t g = 40 + intensity * 175;
      float rad = (a - 90) * DEG_TO_RAD;
      wideCanvas.drawLine(CX, CY, CX + cos(rad) * R, CY + sin(rad) * R, wideCanvas.color565(0, g, 45));
    }
    float rad = (sweepAngle - 90) * DEG_TO_RAD;
    int sx = CX + cos(rad) * R;
    int sy = CY + sin(rad) * R;
    wideCanvas.drawLine(CX, CY, sx, sy, greenBright);
    wideCanvas.drawLine(CX + 1, CY, sx + 1, sy, greenBright);
  }

  int step = R / 5;
  for (int i = 1; i <= 4; i++) wideCanvas.drawCircle(CX, CY, step * i, greenDim);
  wideCanvas.drawCircle(CX, CY, R, greenBright);
  wideCanvas.drawCircle(CX, CY, R + 2, greenBright);

  wideCanvas.drawFastHLine(CX - R, CY, R * 2, greenDim);
  wideCanvas.drawFastVLine(CX, CY - R, R * 2, greenDim);
  for (int a = 30; a < 360; a += 30) {
    if (a == 90 || a == 180 || a == 270) continue;
    canvasCorrectedRadialLine(a, 0, R, greenDark);
  }

  wideCanvas.setTextDatum(middle_left);
  wideCanvas.setTextSize(1);
  wideCanvas.setTextColor(greenBright);
  for (int i = 1; i <= 5; i++) {
    float val = (cfg.radiusKm / 5.0f) * i;
    String dLbl = (cfg.unitDist == 1) ? String((int)roundf(val / 1.852f)) + "nm" : String((int)roundf(val)) + "km";
    wideCanvas.drawString(dLbl, CX + 5, CY - step * i - 7);
  }

  for (int a = 0; a < 360; a += 2) {
    int len = (a % 30 == 0) ? 10 : ((a % 10 == 0) ? 7 : 4);
    canvasCorrectedRadialLine(a, R - len, R, (a % 10 == 0) ? greenBright : greenDim);
  }
  wideCanvas.setTextDatum(middle_center);
  for (int a = 0; a < 360; a += 30) {
    float rad = (a - 90) * DEG_TO_RAD;
    int tx = CX + cos(rad) * (R + 16);
    int ty = CY + sin(rad) * (R + 16);
    if (a % 90 == 0) {
      wideCanvas.setTextSize(2);
      wideCanvas.setTextColor(greenBright);
      const char* card = (a == 0) ? "N" : ((a == 90) ? "E" : ((a == 180) ? "S" : "W"));
      wideCanvas.drawString(card, tx, ty);
    } else {
      wideCanvas.setTextSize(1);
      wideCanvas.setTextColor(green);
      wideCanvas.drawNumber(a, tx, ty);
    }
  }

  for (int i = 0; i < planeCount; i++) {
    if (!planes[i].valid) continue;
    bool isSelected = (i == selectedPlaneIdx);
    float ageAngle = cfg.showScanLine ? angleDiffBehindSweep(sweepAngle, planes[i].bearingDeg) : 0;
    if (!isSelected && cfg.showScanLine && ageAngle > 300) continue;

    float fade = 1.0f;
    if (!isSelected && cfg.showScanLine && ageAngle > 220) {
      fade = max(0.35f, 1.0f - ((ageAngle - 220.0f) / 80.0f));
    }

    int x, y;
    geoToRadarXY(planes[i].lat, planes[i].lon, x, y);
    if (hypot(x - CX, y - CY) > R - 6) continue;

    uint8_t r = 0, g = 255 * fade, b = 35 * fade;
    if (planes[i].altitudeFt >= 30000)      { r = 60 * fade;  g = 210 * fade; b = 255 * fade; }
    else if (planes[i].altitudeFt >= 10000) { r = 40 * fade;  g = 255 * fade; b = 70 * fade; }
    else if (planes[i].altitudeFt >= 0)     { r = 255 * fade; g = 220 * fade; b = 30 * fade; }
    else                                    { r = 255 * fade; g = 60 * fade;  b = 60 * fade; }

    uint16_t col = wideCanvas.color565(r, g, b);
    float scale = isSelected ? 1.45f : ((planes[i].distanceKm < 25) ? 1.25f : 1.05f);
    float heading = (planes[i].trackDeg >= 0) ? planes[i].trackDeg : 0.0f;

    if (isSelected) {
      wideCanvas.drawCircle(x, y, 16, colHudCyan);
      wideCanvas.drawCircle(x, y, 18, TFT_YELLOW);
    }

    if (cfg.showVectors && planes[i].trackDeg >= 0) {
      float tr = (heading - 90.0f) * DEG_TO_RAD;
      wideCanvas.drawLine(x + cos(tr) * 8, y + sin(tr) * 8, x + cos(tr) * 18, y + sin(tr) * 18, col);
    }

    drawAirplaneIcon(x, y, heading, scale, col);

    if (isSelected || !cfg.showScanLine || ageAngle < 180) {
      wideCanvas.setTextDatum(top_left);
      wideCanvas.setTextSize(1);
      int ly = y - 12;

      String l1 = "";
      if (cfg.showCallsign) {
        String fl = String(planes[i].flight); fl.trim();
        l1 += (fl.length() > 0 ? fl : "UNK");
      }
      if (cfg.showAcType && strlen(planes[i].typeCode) > 0) {
        if (l1.length() > 0) l1 += " ";
        l1 += "[" + String(planes[i].typeCode) + "]";
      }
      if (l1.length() > 0) {
        wideCanvas.setTextColor(isSelected ? colHudCyan : TFT_WHITE, TFT_BLACK);
        wideCanvas.drawString(l1, x + 14, ly);
        ly += 10;
      }

      if (cfg.showAltitude && planes[i].altitudeFt >= 0) {
        char flBuf[12];
        if (cfg.unitAlt == 1) sprintf(flBuf, "FL%03d", planes[i].altitudeFt / 100);
        else sprintf(flBuf, "%dm", (int)(planes[i].altitudeFt * 0.3048f));
        wideCanvas.setTextColor(greenBright, TFT_BLACK);
        wideCanvas.drawString(flBuf, x + 14, ly);
        ly += 10;
      }

      String l3 = "";
      if (cfg.showSpeed && planes[i].speedKt > 0) {
        l3 += (cfg.unitSpeed == 1) ? String((int)planes[i].speedKt) + "KT" : String((int)(planes[i].speedKt * 1.852f)) + "K";
      }
      if (cfg.showDistance) {
        if (l3.length() > 0) l3 += " ";
        l3 += (cfg.unitDist == 1) ? String(planes[i].distanceKm / 1.852f, 1) + "nm" : String(planes[i].distanceKm, 0) + "km";
      }
      if (l3.length() > 0) {
        wideCanvas.setTextColor(TFT_YELLOW, TFT_BLACK);
        wideCanvas.drawString(l3, x + 14, ly);
      }
    }
  }

  wideCanvas.setTextDatum(top_left);
  wideCanvas.setTextSize(2);
  if (apiOk) {
    wideCanvas.setTextColor(green, TFT_BLACK);
    wideCanvas.drawString("REAL ADS-B", 12, 10);
    wideCanvas.setTextColor(greenBright, TFT_BLACK);
    wideCanvas.drawString("AC: " + String(planeCount), 12, 32);
  } else {
    wideCanvas.setTextColor(wideCanvas.color565(255, 80, 80), TFT_BLACK);
    wideCanvas.drawString(apiStatusMsg, 12, 14);
  }

  int counts[5] = {0};
  for (int i = 0; i < planeCount; i++) {
    if (!planes[i].valid) continue;
    int bin = constrain((int)(planes[i].distanceKm / (cfg.radiusKm / 5.0f)), 0, 4);
    counts[bin]++;
  }
  const int startX = 398, baseY = 56, barW = 8, barGap = 12;
  wideCanvas.setTextDatum(top_left); wideCanvas.setTextSize(1); wideCanvas.setTextColor(green, TFT_BLACK);
  wideCanvas.drawString("RANGE AC", startX, 10);
  for (int b = 0; b < 5; b++) {
    int x = startX + b * barGap;
    int c = min(counts[b], 8);
    for (int t = 0; t < c; t++) wideCanvas.fillRect(x, baseY - t * 5, barW, 3, greenBright);
    wideCanvas.drawFastHLine(x, baseY + 6, barW, greenDark);
  }

  int ccx = 46, ccy = 432;
  wideCanvas.setTextDatum(middle_center);
  wideCanvas.drawCircle(ccx, ccy, 16, greenDim);
  wideCanvas.fillTriangle(ccx, ccy - 15, ccx - 4, ccy - 4, ccx + 4, ccy - 4, greenBright);
  wideCanvas.fillTriangle(ccx, ccy + 15, ccx - 4, ccy + 4, ccx + 4, ccy + 4, greenDim);
  wideCanvas.fillTriangle(ccx - 15, ccy, ccx - 4, ccy - 4, ccx - 4, ccy + 4, greenDim);
  wideCanvas.fillTriangle(ccx + 15, ccy, ccx + 4, ccy - 4, ccx + 4, ccy + 4, greenDim);
  wideCanvas.setTextSize(1); wideCanvas.setTextColor(greenBright, TFT_BLACK); wideCanvas.drawString("N", ccx, ccy - 25);
  wideCanvas.setTextColor(greenDim, TFT_BLACK);
  wideCanvas.drawString("S", ccx, ccy + 25);
  wideCanvas.drawString("W", ccx - 25, ccy);
  wideCanvas.drawString("E", ccx + 25, ccy);

  int hx = 438, hy = 434;
  wideCanvas.drawCircle(hx, hy, 20, greenDim);
  float sa = (sweepAngle - 90) * DEG_TO_RAD;
  wideCanvas.drawLine(hx, hy, hx + cos(sa) * 16, hy + sin(sa) * 16, greenBright);
  wideCanvas.fillCircle(hx, hy, 2, greenBright);
  wideCanvas.setTextSize(1); wideCanvas.setTextColor(green, TFT_BLACK);
  wideCanvas.drawString("SWEEP", hx, hy - 28);

  drawRightPanelsToCanvas();

  if (regionModalOpen) {
    drawRegionPickerModal();
  }
}

// ==========================================
// 12. 시계 뷰 & 날씨 뷰 렌더링
// ==========================================
void drawThickHand(int cx, int cy, float angleDeg, int len, int tailLen, int halfW, uint16_t col) {
  float rad = (angleDeg - 90.0f) * DEG_TO_RAD;
  float perp = rad + M_PI / 2.0f;
  int tx = cx + cos(rad) * len, ty = cy + sin(rad) * len;
  int bx = cx - cos(rad) * tailLen, by = cy - sin(rad) * tailLen;
  int lx = bx + cos(perp) * halfW, ly = by + sin(perp) * halfW;
  int rx = bx - cos(perp) * halfW, ry = by - sin(perp) * halfW;
  wideCanvas.fillTriangle(tx, ty, lx, ly, rx, ry, col);
}

void drawClockViewToCanvas() {
  const int ccx = 400, ccy = 240;
  struct tm ti;
  bool hasTime = getLocalTime(&ti);
  int hr = hasTime ? ti.tm_hour : 10;
  int mn = hasTime ? ti.tm_min  : 10;
  int sc = hasTime ? ti.tm_sec  : 30;

  float secAng = sc * 6.0f, minAng = mn * 6.0f + sc * 0.1f, hourAng = (hr % 12) * 30.0f + mn * 0.5f;
  wideCanvas.fillScreen(TFT_BLACK);

  String webAddr = "http://" + cfg.hostname + ".local";
  String ipAddr  = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "192.168.4.1";

  if (cfg.clockStyle <= 3) {
    wideCanvas.drawRoundRect(14, 24, 142, 432, 10, greenDim);
    wideCanvas.drawRoundRect(644, 24, 142, 432, 10, greenDim);
    wideCanvas.setTextDatum(middle_center);
    wideCanvas.setTextSize(1); wideCanvas.setTextColor(green); wideCanvas.drawString("STATION", 85, 54);
    wideCanvas.setTextSize(2); wideCanvas.setTextColor(greenBright); wideCanvas.drawString(cfg.presetName, 85, 82);

    if (hasTime) {
      char dtStr[16];
      sprintf(dtStr, "%04d-%02d-%02d", ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday);
      wideCanvas.setTextSize(1); wideCanvas.setTextColor(TFT_LIGHTGREY); wideCanvas.drawString("DATE", 85, 145);
      wideCanvas.setTextSize(2); wideCanvas.setTextColor(TFT_YELLOW); wideCanvas.drawString(dtStr, 85, 172);
    }

    wideCanvas.fillRoundRect(24, 396, 122, 44, 8, wideCanvas.color565(16, 48, 34));
    wideCanvas.drawRoundRect(24, 396, 122, 44, 8, greenBright);
    wideCanvas.setTextSize(2); wideCanvas.setTextColor(greenBright);
    wideCanvas.drawString("< WEATHER", 85, 418);

    wideCanvas.setTextSize(1); wideCanvas.setTextColor(green); wideCanvas.drawString("TIMEZONE", 715, 54);
    wideCanvas.setTextColor(TFT_YELLOW); wideCanvas.drawString(cfg.tzLabel, 715, 82);
    wideCanvas.setTextSize(2); wideCanvas.setTextColor(greenBright); wideCanvas.drawString("AC: " + String(planeCount), 715, 165);

    wideCanvas.setTextSize(1); wideCanvas.setTextColor(TFT_LIGHTGREY); wideCanvas.drawString("WEB SETUP", 715, 315);
    wideCanvas.setTextColor(colHudCyan); wideCanvas.drawString(webAddr, 715, 336);
    wideCanvas.setTextColor(TFT_YELLOW); wideCanvas.drawString(ipAddr, 715, 356);

    wideCanvas.fillRoundRect(654, 396, 122, 44, 8, wideCanvas.color565(16, 48, 34));
    wideCanvas.drawRoundRect(654, 396, 122, 44, 8, greenBright);
    wideCanvas.setTextSize(2); wideCanvas.setTextColor(greenBright);
    wideCanvas.drawString("RADAR >", 715, 418);

    uint16_t dialCol = (cfg.clockStyle == 1) ? wideCanvas.color565(242, 240, 232) : ((cfg.clockStyle == 3) ? wideCanvas.color565(10, 38, 86) : TFT_BLACK);
    uint16_t rimCol  = (cfg.clockStyle == 2) ? TFT_WHITE : wideCanvas.color565(90, 96, 102);
    wideCanvas.fillCircle(ccx, ccy, 230, rimCol);
    wideCanvas.fillCircle(ccx, ccy, 192, dialCol);

    for (int i = 0; i < 12; i++) {
      float rad = (i * 30 - 90) * DEG_TO_RAD;
      wideCanvas.fillCircle(ccx + cos(rad) * 164, ccy + sin(rad) * 164, (i % 3 == 0) ? 8 : 6, (cfg.clockStyle == 1) ? TFT_BLACK : TFT_WHITE);
    }
    const char* brands[] = {"ROLEX", "LONGINES", "BREITLING", "OMEGA"};
    wideCanvas.setTextSize(3);
    wideCanvas.setTextColor((cfg.clockStyle == 1) ? TFT_BLACK : wideCanvas.color565(220, 190, 90));
    wideCanvas.drawString(brands[cfg.clockStyle], ccx, ccy - 76);

    wideCanvas.setTextSize(1);
    wideCanvas.setTextColor((cfg.clockStyle == 1) ? TFT_DARKGREY : TFT_LIGHTGREY);
    wideCanvas.drawString("TAP DIAL TO CHANGE STYLE", ccx, ccy + 85);

    uint16_t hCol = (cfg.clockStyle == 1) ? wideCanvas.color565(18, 65, 165) : TFT_WHITE;
    uint16_t sCol = (cfg.clockStyle == 0) ? greenBright : TFT_RED;
    drawThickHand(ccx, ccy, hourAng, 108, 22, 7, hCol);
    drawThickHand(ccx, ccy, minAng,  156, 26, 5, hCol);
    drawThickHand(ccx, ccy, secAng,  172, 34, 2, sCol);
    wideCanvas.fillCircle(ccx, ccy, 8, hCol);
  } else {
    wideCanvas.drawRoundRect(24, 24, SCREEN_W - 48, SCREEN_H - 48, 18, colRiver);
    char hms[12]; sprintf(hms, "%02d:%02d:%02d", hr, mn, sc);
    wideCanvas.setTextDatum(middle_center);
    wideCanvas.setTextSize(7); wideCanvas.setTextColor(greenBright);
    wideCanvas.drawString(hms, ccx, ccy - 26);
    wideCanvas.setTextSize(2); wideCanvas.setTextColor(TFT_YELLOW);
    wideCanvas.drawString(cfg.tzLabel + "  |  " + cfg.presetName, ccx, ccy + 48);
    wideCanvas.setTextColor(colHudCyan);
    wideCanvas.drawString("WEB: " + webAddr + " (" + ipAddr + ")", ccx, ccy + 88);

    wideCanvas.fillRoundRect(44, 392, 160, 46, 8, wideCanvas.color565(16, 48, 34));
    wideCanvas.drawRoundRect(44, 392, 160, 46, 8, greenBright);
    wideCanvas.setTextColor(greenBright);
    wideCanvas.drawString("< WEATHER", 124, 415);

    wideCanvas.fillRoundRect(310, 392, 180, 46, 8, wideCanvas.color565(20, 40, 60));
    wideCanvas.drawRoundRect(310, 392, 180, 46, 8, colHudCyan);
    wideCanvas.setTextColor(colHudCyan);
    wideCanvas.drawString("WATCH FACE", 400, 415);

    wideCanvas.fillRoundRect(596, 392, 160, 46, 8, wideCanvas.color565(16, 48, 34));
    wideCanvas.drawRoundRect(596, 392, 160, 46, 8, greenBright);
    wideCanvas.setTextColor(greenBright);
    wideCanvas.drawString("RADAR >", 676, 415);
  }
}

void drawWeatherViewToCanvas() {
  wideCanvas.fillScreen(colSea);

  wideCanvas.setTextDatum(middle_center);
  wideCanvas.setTextSize(3); wideCanvas.setTextColor(greenBright, colWxPanel);
  wideCanvas.drawString(" WEATHER · " + cfg.presetName + " ", 400, 36);

  if (!wxData.valid) {
    wideCanvas.setTextSize(2); wideCanvas.setTextColor(TFT_WHITE, colWxPanel);
    wideCanvas.drawString("Loading Open-Meteo Weather...", 400, 240);
  } else {
    float tVal = (cfg.unitTemp == 1) ? (wxData.tempC * 9.0f / 5.0f + 32.0f) : wxData.tempC;
    float fVal = (cfg.unitTemp == 1) ? (wxData.feelsLikeC * 9.0f / 5.0f + 32.0f) : wxData.feelsLikeC;
    String tUnit = (cfg.unitTemp == 1) ? "F" : "C";

    wideCanvas.fillRoundRect(60, 74, 320, 140, 12, colWxPanel);
    wideCanvas.drawRoundRect(60, 74, 320, 140, 12, colWxBorder);
    wideCanvas.setTextDatum(top_left);
    wideCanvas.setTextSize(4); wideCanvas.setTextColor(TFT_WHITE);
    wideCanvas.drawString(String(tVal, 1) + tUnit, 85, 92);
    wideCanvas.setTextSize(2); wideCanvas.setTextColor(TFT_LIGHTGREY);
    wideCanvas.drawString("Feels like: " + String(fVal, 1) + tUnit, 85, 146);
    wideCanvas.setTextColor(colRiver);
    wideCanvas.drawString("Humidity: " + String(wxData.humidity) + "%", 85, 178);

    wideCanvas.fillRoundRect(420, 74, 320, 140, 12, colWxPanel);
    wideCanvas.drawRoundRect(420, 74, 320, 140, 12, colWxBorder);
    float wSpd = (cfg.unitSpeed == 1) ? (wxData.windKmh / 1.852f) : wxData.windKmh;
    wideCanvas.setTextSize(2); wideCanvas.setTextColor(greenBright);
    wideCanvas.drawString("Wind: " + String(degToCompass(wxData.windDir)) + " " + String((int)roundf(wSpd)) + (cfg.unitSpeed == 1 ? "kt" : "km/h"), 445, 96);
    wideCanvas.setTextColor(TFT_YELLOW);
    wideCanvas.drawString("Pressure: " + String((int)wxData.pressureHpa) + " hPa", 445, 136);
    wideCanvas.setTextColor(colRiver);
    wideCanvas.drawString("Precip: " + String(wxData.precipMm, 1) + " mm", 445, 176);

    wideCanvas.fillRoundRect(60, 236, 680, 140, 12, colWxPanel);
    wideCanvas.drawRoundRect(60, 236, 680, 140, 12, colWxBorder);
    const char* dNames[3] = {"TODAY", "TOMORROW", "DAY 3"};
    for (int i = 0; i < 3; i++) {
      int bx = 173 + i * 226;
      float dMax = (cfg.unitTemp == 1) ? (wxData.dayMaxC[i] * 9.0f / 5.0f + 32.0f) : wxData.dayMaxC[i];
      float dMin = (cfg.unitTemp == 1) ? (wxData.dayMinC[i] * 9.0f / 5.0f + 32.0f) : wxData.dayMinC[i];
      wideCanvas.setTextDatum(middle_center);
      wideCanvas.setTextSize(2); wideCanvas.setTextColor(TFT_LIGHTGREY); wideCanvas.drawString(dNames[i], bx, 266);
      wideCanvas.setTextSize(3); wideCanvas.setTextColor(TFT_WHITE); wideCanvas.drawString(String((int)roundf(dMax)) + "/" + String((int)roundf(dMin)) + tUnit, bx, 312);
      wideCanvas.setTextSize(2); wideCanvas.setTextColor(colRiver); wideCanvas.drawString("POP " + String(wxData.dayPop[i]) + "%", bx, 354);
    }
  }

  String ipAddr = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "192.168.4.1";
  wideCanvas.setTextDatum(middle_center);
  wideCanvas.setTextSize(1); wideCanvas.setTextColor(TFT_YELLOW, colWxPanel);
  wideCanvas.drawString(" WEB: http://" + cfg.hostname + ".local  (" + ipAddr + ") ", 400, 400);

  wideCanvas.fillRoundRect(60, 418, 170, 46, 8, wideCanvas.color565(16, 48, 34));
  wideCanvas.drawRoundRect(60, 418, 170, 46, 8, greenBright);
  wideCanvas.setTextSize(2); wideCanvas.setTextColor(greenBright);
  wideCanvas.drawString("< RADAR", 145, 441);

  wideCanvas.fillRoundRect(570, 418, 170, 46, 8, wideCanvas.color565(16, 48, 34));
  wideCanvas.drawRoundRect(570, 418, 170, 46, 8, greenBright);
  wideCanvas.setTextColor(greenBright);
  wideCanvas.drawString("CLOCK >", 655, 441);
}

void drawMainFrame() {
  if (cfg.viewMode == 1) drawWeatherViewToCanvas();
  else if (cfg.viewMode == 2) drawClockViewToCanvas();
  else drawFlightRadarToCanvas();

  wideCanvas.pushSprite(0, 0);
}

// ==========================================
// 13. 터치 이벤트 처리 핸들러 (지역 선택 팝업 & 항공기/콜사인 터치)
// ==========================================
void switchViewMode(int newMode) {
  cfg.viewMode = newMode;
  regionModalOpen = false;
  saveConfig();
  if (cfg.viewMode == 0) startAircraftFetchBackground();
  else if (cfg.viewMode == 1 && !wxData.valid) startWeatherFetchBackground();
  drawMainFrame();
}

void selectAirportByIndex(int idx) {
  if (idx < 0 || idx >= AIRPORTS_LEN) return;
  cfg.presetName = AIRPORTS[idx].key;
  cfg.lat        = AIRPORTS[idx].lat;
  cfg.lon        = AIRPORTS[idx].lon;
  if (cfg.wxUseRadarLoc) {
    cfg.wxLat = cfg.lat;
    cfg.wxLon = cfg.lon;
  }
  selectedPlaneIdx = -1;
  selectedPlaneHex = "";
  regionModalOpen = false;
  saveConfig();

  buildVectorMapSprite();
  startAircraftFetchBackground();
  drawMainFrame();
}

void handleTouchTap(int tx, int ty) {
  Serial.printf("[TOUCH] X: %3d   Y: %3d (viewMode=%d, modal=%d)\n", tx, ty, cfg.viewMode, regionModalOpen);

  if (cfg.viewMode == 0) {
    // [A] 지역 선택 팝업이 열려 있을 때의 터치 처리
    if (regionModalOpen) {
      const int mx = 56, my = 32, mw = 688, mh = 416;
      if ((tx >= mx + mw - 90 && tx <= mx + mw && ty >= my && ty <= my + 48) ||
          tx < mx || tx > mx + mw || ty < my || ty > my + mh) {
        regionModalOpen = false;
        drawMainFrame();
        return;
      }

      const int cols = 3;
      const int gridX = mx + 16;
      const int gridY = my + 56;
      const int cellW = 212;
      const int cellH = 62;
      const int gapX  = 10;
      const int gapY  = 8;

      for (int i = 0; i < AIRPORTS_LEN; i++) {
        int col = i % cols;
        int row = i / cols;
        int cx = gridX + col * (cellW + gapX);
        int cy = gridY + row * (cellH + gapY);
        if (tx >= cx && tx <= cx + cellW && ty >= cy && ty <= cy + cellH) {
          selectAirportByIndex(i);
          return;
        }
      }
      return;
    }

    // [B] 우측 상단 DETECTED AIRCRAFTS 목록의 콜사인/행 터치 (X: 482..794, Y: 54..276)
    if (tx >= INFO_X && tx <= INFO_X + INFO_W && ty >= INFO_Y + 48 && ty <= INFO_Y + 275) {
      int row = (ty - (INFO_Y + 55)) / 27;
      if (row >= 0 && row < min(planeCount, 8)) {
        if (selectedPlaneIdx == row) {
          selectedPlaneIdx = -1;
          selectedPlaneHex = "";
        } else {
          selectedPlaneIdx = row;
          selectedPlaneHex = String(planes[row].hex);
        }
        drawMainFrame();
      }
      return;
    }

    // [C] 좌측 레이더 지도 위의 비행기 아이콘 또는 콜사인 라벨 터치 (X < 476)
    if (tx < 476 && planeCount > 0) {
      int bestIdx = -1;
      float bestDist = 30.0f;
      for (int i = 0; i < planeCount; i++) {
        if (!planes[i].valid) continue;
        int px, py;
        geoToRadarXY(planes[i].lat, planes[i].lon, px, py);
        if (hypot(px - CX, py - CY) > R - 6) continue;

        float dIcon = hypot(tx - px, ty - py);
        bool inLabelBox = (tx >= px + 8 && tx <= px + 76 && ty >= py - 16 && ty <= py + 18);

        if (inLabelBox) {
          bestIdx = i;
          break;
        }
        if (dIcon < bestDist) {
          bestDist = dIcon;
          bestIdx = i;
        }
      }
      if (bestIdx != -1) {
        if (selectedPlaneIdx == bestIdx) {
          selectedPlaneIdx = -1;
          selectedPlaneHex = "";
        } else {
          selectedPlaneIdx = bestIdx;
          selectedPlaneHex = String(planes[bestIdx].hex);
        }
        drawMainFrame();
        return;
      }
    }

    // [D] 우측 하단 영역 터치 (X: 482..794, Y: 288..474)
    if (tx >= INFO_X && tx <= INFO_X + INFO_W && ty >= 288 && ty <= 474) {
      if (selectedPlaneIdx != -1) {
        if (tx >= INFO_X + INFO_W - 76 && ty <= 326) {
          selectedPlaneIdx = -1;
          selectedPlaneHex = "";
          drawMainFrame();
          return;
        }
        if (tx >= INFO_X + 145 && tx <= INFO_X + 228 && ty >= 435) {
          regionModalOpen = true;
          drawMainFrame();
          return;
        }
        if (tx >= INFO_X + 230 && ty >= 435) {
          switchViewMode(1);
          return;
        }
      } else {
        if (ty <= 398) {
          regionModalOpen = true;
          drawMainFrame();
          return;
        }
        if (ty >= 404) {
          switchViewMode(1);
          return;
        }
      }
    }
  }
  else if (cfg.viewMode == 1) {
    if (ty >= 405) {
      if (tx < 400) switchViewMode(0);
      else          switchViewMode(2);
    }
  }
  else if (cfg.viewMode == 2) {
    if (tx < 180) {
      switchViewMode(1);
    } else if (tx > 620) {
      switchViewMode(0);
    } else {
      cfg.clockStyle = (cfg.clockStyle + 1) % 7;
      saveConfig();
      drawMainFrame();
    }
  }
}

// ==========================================
// 14. SETUP & LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== RADAR 360 CrowPanel 7.0\" (Interactive Touch UI) ===");

  loadConfig();

  // CrowPanel 7.0" 전용 I2C 초기화 및 GT911 주소 자동 감지
  Wire.begin(19, 20);
  Wire.beginTransmission(0x18);
  Wire.write(0x01);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.beginTransmission(0x30);
  Wire.write(0x10);
  Wire.endTransmission();

  uint8_t detectedTouchAddr = 0x14;
  Wire.beginTransmission(0x5D);
  if (Wire.endTransmission() == 0) {
    detectedTouchAddr = 0x5D;
  } else {
    Wire.beginTransmission(0x14);
    if (Wire.endTransmission() == 0) detectedTouchAddr = 0x14;
  }
  Wire.end();

  auto tCfg = lcd._touch.config();
  tCfg.i2c_addr = detectedTouchAddr;
  lcd._touch.config(tCfg);
  Serial.printf("[TOUCH] GT911 configured at 0x%02X\n", detectedTouchAddr);

  lcd.init();
  lcd.setBrightness(map(cfg.brightness, 10, 100, 40, 255));
  Serial.println("[LCD & GT911] Initialized 800x480 RGB + Touch");

  greenBright = lcd.color565(0, 255, 35);
  green       = lcd.color565(0, 180, 25);
  greenDim    = lcd.color565(0, 100, 25);
  greenDark   = lcd.color565(0, 45, 12);
  colAirport  = lcd.color565(255, 210, 0);
  colCardBg   = lcd.color565(4, 12, 8);
  colWxPanel  = lcd.color565(16, 26, 36);
  colWxBorder = lcd.color565(58, 92, 116);
  colHudCyan  = lcd.color565(0, 230, 255);
  colSelBg    = lcd.color565(14, 48, 54);

  wideCanvas.setPsram(true);
  wideCanvas.setColorDepth(16);
  if (!wideCanvas.createSprite(SCREEN_W, SCREEN_H)) {
    Serial.println("[ERR] wideCanvas PSRAM allocation failed! Enable OPI PSRAM!");
  }

  mapSprite.setPsram(true);
  mapSprite.setColorDepth(16);
  if (!mapSprite.createSprite(MAP_SIZE, MAP_SIZE)) {
    Serial.println("[ERR] mapSprite PSRAM allocation failed!");
  } else {
    buildVectorMapSprite();
    Serial.println("[MAP] Built-in 420x420 Vector Map rendered!");
  }

  if (cfg.ssid.length() == 0) {
    runApSetupPortal();
  }

  lcd.fillScreen(TFT_BLACK);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.setTextDatum(middle_center);
  lcd.drawString("Connecting WiFi...", SCREEN_W / 2, 215);
  lcd.setTextColor(greenBright, TFT_BLACK);
  lcd.drawString(cfg.ssid, SCREEN_W / 2, 250);

  Serial.println("[WiFi] Connecting to " + cfg.ssid);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Connection failed -> Starting AP Setup Portal");
    runApSetupPortal();
  }

  Serial.printf("[WiFi] Connected! IP: %s | Free Heap: %u bytes\n", WiFi.localIP().toString().c_str(), ESP.getFreeHeap());
  MDNS.begin(cfg.hostname.c_str());
  setupWebServer();
  configTzTime(cfg.tzPosix.c_str(), "pool.ntp.org", "time.nist.gov");

  drawMainFrame();
  lastApiFetchTime = millis();
  lastWxFetchTime  = millis();
  startAircraftFetchBackground();
}

void loop() {
  server.handleClient();

  if (millis() - lastTouchPoll > 50) {
    lastTouchPoll = millis();
    uint16_t x = 0, y = 0;
    bool isTouched = lcd.getTouch(&x, &y);

    if (isTouched) {
      if (!wasTouched && (millis() - lastTouchAction > 280)) {
        lastTouchAction = millis();
        lastTouchX = (int)x;
        lastTouchY = (int)y;
        handleTouchTap(lastTouchX, lastTouchY);
      }
    }
    wasTouched = isTouched;
  }

  unsigned long interval = (cfg.viewMode == 0) ? SWEEP_SPEED : 250;
  if (millis() - lastSweepUpdate > interval) {
    lastSweepUpdate = millis();

    applyPendingWeatherData();

    if (!regionModalOpen) {
      drawMainFrame();

      if (cfg.viewMode == 0) {
        sweepAngle += 4;
        if (sweepAngle >= 360) {
          sweepAngle -= 360;
          applyPendingAircraftData();

          if (!netBusy && (millis() - lastApiFetchTime > (unsigned long)cfg.refreshSec * 1000UL)) {
            lastApiFetchTime = millis();
            startAircraftFetchBackground();
          }
        }
      }
    }

    if (!netBusy && (millis() - lastWxFetchTime > WX_FETCH_INTERVAL)) {
      lastWxFetchTime = millis();
      startWeatherFetchBackground();
    }
  }
}