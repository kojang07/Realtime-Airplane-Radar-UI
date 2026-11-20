// Realtime Airplane Radar UI - STEP 6.3 (Vivid Color Map + RKPK STAR/SID + Airplane Icons)
// CrowPanel 7.0" ESP32-S3 800x480 RGB
// Real aircraft data + background API update + sweep reveal effect + Pre-rendered Color Map & Airways

#include <Wire.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <math.h>
#include "time.h"

// ==========================================
// 1. 사용자 설정 (Wi-Fi 및 기준 위치)
// ==========================================
const char* ssid     = "bj_home";
const char* password = "";

// 기준 좌표: 김해국제공항 (RKPK)
const double HOME_LAT = 35.1795;
const double HOME_LON = 128.9382;
const char* LOCATION_NAME = "GIMHAE (RKPK)";

// Radar range
const float RADAR_RANGE_KM = 100.0;
const int API_RADIUS_NM = 54;   // about 100 km

class LGFX : public lgfx::LGFX_Device {
public:
  lgfx::Bus_RGB   _bus;
  lgfx::Panel_RGB _panel;
  lgfx::Light_PWM _light;

  LGFX(void) {
    {
      auto cfg = _panel.config();
      cfg.memory_width  = 800;
      cfg.memory_height = 480;
      cfg.panel_width   = 800;
      cfg.panel_height  = 480;
      _panel.config(cfg);
    }

    {
      auto cfg = _bus.config();
      cfg.panel = &_panel;

      cfg.pin_d0  = GPIO_NUM_15;
      cfg.pin_d1  = GPIO_NUM_7;
      cfg.pin_d2  = GPIO_NUM_6;
      cfg.pin_d3  = GPIO_NUM_5;
      cfg.pin_d4  = GPIO_NUM_4;
      cfg.pin_d5  = GPIO_NUM_9;
      cfg.pin_d6  = GPIO_NUM_46;
      cfg.pin_d7  = GPIO_NUM_3;
      cfg.pin_d8  = GPIO_NUM_8;
      cfg.pin_d9  = GPIO_NUM_16;
      cfg.pin_d10 = GPIO_NUM_1;
      cfg.pin_d11 = GPIO_NUM_14;
      cfg.pin_d12 = GPIO_NUM_21;
      cfg.pin_d13 = GPIO_NUM_47;
      cfg.pin_d14 = GPIO_NUM_48;
      cfg.pin_d15 = GPIO_NUM_45;

      cfg.pin_henable = GPIO_NUM_41;
      cfg.pin_vsync   = GPIO_NUM_40;
      cfg.pin_hsync   = GPIO_NUM_39;
      cfg.pin_pclk    = GPIO_NUM_0;

      cfg.freq_write = 12000000;

      cfg.hsync_front_porch = 40;
      cfg.hsync_pulse_width = 48;
      cfg.hsync_back_porch  = 40;
      cfg.vsync_front_porch = 1;
      cfg.vsync_pulse_width = 31;
      cfg.vsync_back_porch  = 13;

      cfg.pclk_active_neg = 1;
      cfg.de_idle_high    = 0;
      cfg.pclk_idle_high  = 0;

      _bus.config(cfg);
      _panel.setBus(&_bus);
    }

    {
      auto cfg = _light.config();
      cfg.pin_bl = GPIO_NUM_2;
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
LGFX_Sprite mapSprite(&lcd);     // 컬러 지도 + 항로선을 미리 그려둘 오프스크린 버퍼

// ---------- LAYOUT ----------
const int MARGIN = 5;

const int RADAR_X = MARGIN;
const int RADAR_Y = MARGIN;
const int RADAR_W = 470;
const int RADAR_H = 470;

const int INFO_X = 485;
const int INFO_Y = MARGIN;
const int INFO_W = 310;
const int INFO_H = 470;

const int CX = 240;
const int CY = 240;

const float X_CORR = 0.93;
const int R = 212;

const int OUTER_GAP = 3;
const int OUTER_RING_THICKNESS = 3;

// ---------- SWEEP ----------
float sweepAngle = 0;
unsigned long lastSweepUpdate = 0;

const int TRAIL_WIDTH = 60;
const int SWEEP_SPEED = 25;

unsigned long lastClockUpdate = 0;

// ---------- COLORS ----------
uint16_t greenBright;
uint16_t green;
uint16_t greenDim;
uint16_t greenDark;
uint16_t panelBg;

// 컬러 지도 및 항로 전용 팔레트
uint16_t colSea;         // 바다색
uint16_t colLand;        // 육지색
uint16_t colCoast;       // 해안선 경계선
uint16_t colRiver;       // 강/하천
uint16_t colRoad;        // 주요 간선축
uint16_t colCityDot;     // 도시 마커
uint16_t colCityText;    // 도시 이름
uint16_t colAirway;      // STAR/SID 점선 항로 색상
uint16_t colWaypoint;    // 웨이포인트 삼각형 및 라벨 색상

// ---------- VECTOR MAP DATA ----------
struct GeoPoint {
  float lat;
  float lon;
};

// 한반도 내륙 폴리곤 (남해안~동해안 해안선 + 북서쪽 내륙 외곽 연결)
const GeoPoint POLY_MAINLAND[] = {
  {34.74, 127.74}, {34.85, 127.88}, {34.93, 128.07}, {34.84, 128.20},
  {34.83, 128.42}, {34.96, 128.45}, {35.02, 128.57}, {35.18, 128.57},
  {35.10, 128.75}, {35.02, 128.83}, {35.08, 128.96}, {35.05, 129.03},
  {35.10, 129.04}, {35.15, 129.12}, {35.16, 129.18}, {35.25, 129.22},
  {35.36, 129.32}, {35.48, 129.42}, {35.53, 129.37}, {35.62, 129.44},
  {35.80, 129.51}, {36.02, 129.58}, {36.35, 129.60}, {36.35, 127.50},
  {34.74, 127.50}
};
const int POLY_MAINLAND_LEN = sizeof(POLY_MAINLAND) / sizeof(GeoPoint);

// 거제도
const GeoPoint POLY_GEOJE[] = {
  {34.99, 128.63}, {34.93, 128.72}, {34.80, 128.70}, {34.73, 128.62},
  {34.82, 128.52}, {34.94, 128.55}
};
const int POLY_GEOJE_LEN = sizeof(POLY_GEOJE) / sizeof(GeoPoint);

// 남해도
const GeoPoint POLY_NAMHAE[] = {
  {34.88, 127.88}, {34.84, 128.02}, {34.72, 128.00}, {34.74, 127.85}
};
const int POLY_NAMHAE_LEN = sizeof(POLY_NAMHAE) / sizeof(GeoPoint);

// 대마도 (Tsushima)
const GeoPoint POLY_TSUSHIMA[] = {
  {34.70, 129.45}, {34.55, 129.42}, {34.38, 129.36}, {34.25, 129.28},
  {34.10, 129.22}, {34.14, 129.16}, {34.33, 129.22}, {34.48, 129.31},
  {34.67, 129.36}
};
const int POLY_TSUSHIMA_LEN = sizeof(POLY_TSUSHIMA) / sizeof(GeoPoint);

// 낙동강 본류
const GeoPoint RIVER_NAKDONG[] = {
  {36.05, 128.40}, {35.80, 128.45}, {35.55, 128.50}, {35.40, 128.65},
  {35.38, 128.88}, {35.28, 128.98}, {35.18, 128.96}, {35.06, 128.94}
};
const int RIVER_NAKDONG_LEN = sizeof(RIVER_NAKDONG) / sizeof(GeoPoint);

// 주요 간선 교통축 (부산-울산-경주-대구 / 부산-창원-진주)
const GeoPoint ROAD_GYEONGBU[] = {
  {35.12, 129.04}, {35.34, 129.04}, {35.54, 129.26}, {35.84, 129.21}, {35.87, 128.60}
};
const int ROAD_GYEONGBU_LEN = sizeof(ROAD_GYEONGBU) / sizeof(GeoPoint);

const GeoPoint ROAD_NAMHAE[] = {
  {35.18, 128.96}, {35.23, 128.68}, {35.18, 128.11}
};
const int ROAD_NAMHAE_LEN = sizeof(ROAD_NAMHAE) / sizeof(GeoPoint);

// ---------- RKPK STAR / SID AIRWAYS & WAYPOINTS ----------
const GeoPoint ROUTE_KADOL[] = {
  {34.55, 128.25}, {34.82, 128.62}, {35.01, 128.88}, {35.1795, 128.9382}
};
const int ROUTE_KADOL_LEN = sizeof(ROUTE_KADOL) / sizeof(GeoPoint);

const GeoPoint ROUTE_DOTOL[] = {
  {35.85, 128.32}, {35.54, 128.62}, {35.34, 128.88}, {35.1795, 128.9382}
};
const int ROUTE_DOTOL_LEN = sizeof(ROUTE_DOTOL) / sizeof(GeoPoint);

const GeoPoint ROUTE_OLMEN[] = {
  {34.58, 129.62}, {34.88, 129.28}, {35.06, 129.05}, {35.1795, 128.9382}
};
const int ROUTE_OLMEN_LEN = sizeof(ROUTE_OLMEN) / sizeof(GeoPoint);

const GeoPoint ROUTE_EAST[] = {
  {35.98, 129.42}, {35.59, 129.35}, {35.34, 129.12}, {35.1795, 128.9382}
};
const int ROUTE_EAST_LEN = sizeof(ROUTE_EAST) / sizeof(GeoPoint);

struct WaypointFix {
  float lat;
  float lon;
  const char* code;
};

const WaypointFix RKPK_WAYPOINTS[] = {
  {34.55, 128.25, "KADOL"},
  {34.82, 128.62, "TOPAX"},
  {35.85, 128.32, "DOTOL"},
  {35.54, 128.62, "DALSU"},
  {34.58, 129.62, "OLMEN"},
  {34.88, 129.28, "KAMIT"},
  {35.59, 129.35, "USN"}
};
const int RKPK_WAYPOINTS_LEN = sizeof(RKPK_WAYPOINTS) / sizeof(WaypointFix);

struct MapLabel {
  float lat;
  float lon;
  const char* name;
  bool isSea;
};

const MapLabel MAP_LABELS[] = {
  {35.16, 129.06, "BUSAN", false},
  {35.54, 129.31, "ULSAN", false},
  {35.22, 128.68, "CHANGWON", false},
  {34.88, 128.62, "GEOJE", false},
  {35.87, 128.60, "DAEGU", false},
  {35.84, 129.21, "GYEONGJU", false},
  {35.18, 128.11, "JINJU", false},
  {34.42, 129.33, "TSUSHIMA", false},
  {34.65, 129.05, "KOREA STRAIT", true}
};
const int MAP_LABELS_LEN = sizeof(MAP_LABELS) / sizeof(MapLabel);

// ---------- AIRCRAFT ----------
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

// ---------- HELPERS ----------
int rxCorr(int radius) {
  return (int)(radius * X_CORR);
}

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

  double c = 2 * atan2(sqrt(a), sqrt(1.0 - a));
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

void geoToRadarXY(double lat, double lon, int &outX, int &outY) {
  float d = distanceKm(HOME_LAT, HOME_LON, lat, lon);
  float brng = bearingDeg(HOME_LAT, HOME_LON, lat, lon);

  float rr = (d / RADAR_RANGE_KM) * R;
  float rad = (brng - 90.0) * DEG_TO_RAD;

  outX = CX + cos(rad) * rxCorr(rr);
  outY = CY + sin(rad) * rr;
}

// 폴리곤 내부를 육지 색상으로 채우고 외곽선(해안선) 그리기
void fillGeoPolygonOnSprite(LGFX_Sprite &spr, const GeoPoint* pts, int len, uint16_t fillCol, uint16_t strokeCol, int strokeCount) {
  int px[32];
  int py[32];
  if (len > 32) len = 32;

  int minY = 480, maxY = 0;
  for (int i = 0; i < len; i++) {
    geoToRadarXY(pts[i].lat, pts[i].lon, px[i], py[i]);
    if (py[i] < minY) minY = py[i];
    if (py[i] > maxY) maxY = py[i];
  }
  if (minY < 0) minY = 0;
  if (maxY > 479) maxY = 479;

  int nodeX[32];
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
          int tmp = nodeX[i];
          nodeX[i] = nodeX[k];
          nodeX[k] = tmp;
        }
      }
    }

    for (int i = 0; i < nodes; i += 2) {
      if (i + 1 >= nodes) break;
      int xStart = max(0, nodeX[i]);
      int xEnd   = min(479, nodeX[i + 1]);
      if (xEnd > xStart) {
        spr.drawFastHLine(xStart, y, xEnd - xStart + 1, fillCol);
      }
    }
  }

  for (int i = 0; i < strokeCount; i++) {
    spr.drawLine(px[i], py[i], px[i + 1], py[i + 1], strokeCol);
  }
}

void drawPolylineOnSprite(LGFX_Sprite &spr, const GeoPoint* pts, int len, uint16_t col, int thickness = 1) {
  for (int i = 0; i < len - 1; i++) {
    int x1, y1, x2, y2;
    geoToRadarXY(pts[i].lat, pts[i].lon, x1, y1);
    geoToRadarXY(pts[i + 1].lat, pts[i + 1].lon, x2, y2);
    spr.drawLine(x1, y1, x2, y2, col);
    if (thickness > 1) {
      spr.drawLine(x1 + 1, y1, x2 + 1, y2, col);
    }
  }
}

void drawDashedLineOnSprite(LGFX_Sprite &spr, int x1, int y1, int x2, int y2, uint16_t col, int dashLen = 4, int gapLen = 4) {
  float dx = x2 - x1;
  float dy = y2 - y1;
  float totalLen = sqrt(dx * dx + dy * dy);
  if (totalLen < 1.0f) return;

  float stepX = dx / totalLen;
  float stepY = dy / totalLen;

  int cycle = dashLen + gapLen;
  for (float d = 0; d < totalLen; d += cycle) {
    float dEnd = min(d + dashLen, totalLen);
    int sx = x1 + (int)(stepX * d);
    int sy = y1 + (int)(stepY * d);
    int ex = x1 + (int)(stepX * dEnd);
    int ey = y1 + (int)(stepY * dEnd);
    spr.drawLine(sx, sy, ex, ey, col);
  }
}

void drawDashedAirwayOnSprite(LGFX_Sprite &spr, const GeoPoint* pts, int len, uint16_t col) {
  for (int i = 0; i < len - 1; i++) {
    int x1, y1, x2, y2;
    geoToRadarXY(pts[i].lat, pts[i].lon, x1, y1);
    geoToRadarXY(pts[i + 1].lat, pts[i + 1].lon, x2, y2);
    drawDashedLineOnSprite(spr, x1, y1, x2, y2, col, 4, 4);
  }
}

// setup()에서 한 번만 호출되어 컬러 지도 + STAR/SID 항로를 mapSprite에 사전 렌더링
void buildColorMapSprite() {
  mapSprite.fillSprite(TFT_BLACK);

  // 1. 레이더 원 내부를 바다색(colSea)으로 채우기
  for (int y = CY - R; y <= CY + R; y++) {
    int dy = y - CY;
    float dx = sqrt((float)(R * R - dy * dy)) * X_CORR;
    int xStart = CX - (int)dx;
    int width  = (int)(dx * 2);
    if (width > 0) {
      mapSprite.drawFastHLine(xStart, y, width, colSea);
    }
  }

  // 2. 육지 폴리곤 채우기 + 해안선 그리기
  fillGeoPolygonOnSprite(mapSprite, POLY_MAINLAND, POLY_MAINLAND_LEN, colLand, colCoast, 21);
  fillGeoPolygonOnSprite(mapSprite, POLY_GEOJE, POLY_GEOJE_LEN, colLand, colCoast, POLY_GEOJE_LEN - 1);
  fillGeoPolygonOnSprite(mapSprite, POLY_NAMHAE, POLY_NAMHAE_LEN, colLand, colCoast, POLY_NAMHAE_LEN - 1);
  fillGeoPolygonOnSprite(mapSprite, POLY_TSUSHIMA, POLY_TSUSHIMA_LEN, colLand, colCoast, POLY_TSUSHIMA_LEN - 1);

  // 3. 주요 도로망 및 낙동강 그리기
  drawPolylineOnSprite(mapSprite, ROAD_GYEONGBU, ROAD_GYEONGBU_LEN, colRoad, 1);
  drawPolylineOnSprite(mapSprite, ROAD_NAMHAE, ROAD_NAMHAE_LEN, colRoad, 1);
  drawPolylineOnSprite(mapSprite, RIVER_NAKDONG, RIVER_NAKDONG_LEN, colRiver, 2);

  // 4. 김해공항(RKPK) 주요 접근·출발 점선 항로 그리기
  drawDashedAirwayOnSprite(mapSprite, ROUTE_KADOL, ROUTE_KADOL_LEN, colAirway);
  drawDashedAirwayOnSprite(mapSprite, ROUTE_DOTOL, ROUTE_DOTOL_LEN, colAirway);
  drawDashedAirwayOnSprite(mapSprite, ROUTE_OLMEN, ROUTE_OLMEN_LEN, colAirway);
  drawDashedAirwayOnSprite(mapSprite, ROUTE_EAST, ROUTE_EAST_LEN, colAirway);

  // 5. 레이더 원 밖으로 삐져나온 영역을 검은색으로 마스킹
  for (int y = 0; y < 480; y++) {
    for (int x = 0; x < 480; x++) {
      float dx = (x - CX) / X_CORR;
      float dy = (y - CY);
      if (dx * dx + dy * dy > (R - 1) * (R - 1)) {
        mapSprite.drawPixel(x, y, TFT_BLACK);
      }
    }
  }

  // 6. 주요 도시 및 해협 컬러 라벨
  mapSprite.setTextDatum(middle_left);
  mapSprite.setTextSize(1);

  for (int i = 0; i < MAP_LABELS_LEN; i++) {
    int lx, ly;
    geoToRadarXY(MAP_LABELS[i].lat, MAP_LABELS[i].lon, lx, ly);

    float dx = (lx - CX) / X_CORR;
    float dy = (ly - CY);
    if (sqrt(dx * dx + dy * dy) <= R - 34) {
      if (MAP_LABELS[i].isSea) {
        mapSprite.setTextColor(mapSprite.color565(90, 165, 225));
        mapSprite.drawString(MAP_LABELS[i].name, lx, ly);
      } else {
        mapSprite.fillCircle(lx, ly, 2, colCityDot);
        mapSprite.setTextColor(colCityText);
        mapSprite.drawString(MAP_LABELS[i].name, lx + 5, ly);
      }
    }
  }

  // 7. STAR/SID 웨이포인트 삼각형 심볼 및 코드 라벨
  mapSprite.setTextDatum(top_center);
  mapSprite.setTextSize(1);
  mapSprite.setTextColor(colWaypoint);

  for (int i = 0; i < RKPK_WAYPOINTS_LEN; i++) {
    int wx, wy;
    geoToRadarXY(RKPK_WAYPOINTS[i].lat, RKPK_WAYPOINTS[i].lon, wx, wy);

    float dx = (wx - CX) / X_CORR;
    float dy = (wy - CY);
    if (sqrt(dx * dx + dy * dy) <= R - 22) {
      mapSprite.drawTriangle(wx, wy - 4, wx - 4, wy + 3, wx + 4, wy + 3, colWaypoint);
      mapSprite.drawString(RKPK_WAYPOINTS[i].code, wx, wy + 5);
    }
  }

  // 8. 김해공항(RKPK) 활주로 심볼
  mapSprite.drawFastVLine(CX - 1, CY - 6, 12, TFT_WHITE);
  mapSprite.drawFastVLine(CX + 1, CY - 6, 12, TFT_WHITE);
}

void canvasCorrectedCircle(int cx, int cy, int r, uint16_t col) {
  int lastX = cx + rxCorr(r);
  int lastY = cy;

  for (int a = 1; a <= 360; a++) {
    float rad = a * DEG_TO_RAD;
    int x = cx + cos(rad) * rxCorr(r);
    int y = cy + sin(rad) * r;

    radarCanvas.drawLine(lastX, lastY, x, y, col);

    lastX = x;
    lastY = y;
  }
}

void canvasCorrectedRadialLine(int angleDeg, int r1, int r2, uint16_t col) {
  float rad = (angleDeg - 90) * DEG_TO_RAD;

  int x1 = CX + cos(rad) * rxCorr(r1);
  int y1 = CY + sin(rad) * r1;
  int x2 = CX + cos(rad) * rxCorr(r2);
  int y2 = CY + sin(rad) * r2;

  radarCanvas.drawLine(x1, y1, x2, y2, col);
}

// ---------- 회전형 비행기 아이콘 렌더링 ----------
// 로컬 좌표 (lx: 기수 전방 +, ly: 우현 +)를 방위각(headingDeg)만큼 회전하여 출력
void rotPt(int cx, int cy, float cosA, float sinA, float lx, float ly, int &ox, int &oy) {
  ox = cx + (int)roundf(lx * cosA - ly * sinA);
  oy = cy + (int)roundf(lx * sinA + ly * cosA);
}

void drawAirplaneIcon(int cx, int cy, float headingDeg, float scale, uint16_t fillCol) {
  float rad = (headingDeg - 90.0f) * DEG_TO_RAD;
  float cosA = cosf(rad);
  float sinA = sinf(rad);

  // 비행기 주요 꼭짓점 정의 (단위: 픽셀 * scale)
  int noseX, noseY;         // 기수 끝
  int tailX, tailY;         // 동체 후미 끝
  int bodyFX, bodyFY;       // 동체 전방 날개 뿌리
  int bodyRX, bodyRY;       // 동체 후방 날개 뿌리
  int wingLX, wingLY;       // 좌측 주익 끝
  int wingRX, wingRY;       // 우측 주익 끝
  int stabRootX, stabRootY; // 꼬리날개 뿌리
  int stabLX, stabLY;       // 좌측 수평꼬리날개 끝
  int stabRX, stabRY;       // 우측 수평꼬리날개 끝

  rotPt(cx, cy, cosA, sinA,  8.5f * scale,  0.0f,         noseX, noseY);
  rotPt(cx, cy, cosA, sinA, -7.5f * scale,  0.0f,         tailX, tailY);
  rotPt(cx, cy, cosA, sinA,  2.5f * scale,  0.0f,         bodyFX, bodyFY);
  rotPt(cx, cy, cosA, sinA, -2.2f * scale,  0.0f,         bodyRX, bodyRY);
  rotPt(cx, cy, cosA, sinA, -2.8f * scale, -8.5f * scale, wingLX, wingLY);
  rotPt(cx, cy, cosA, sinA, -2.8f * scale,  8.5f * scale, wingRX, wingRY);
  rotPt(cx, cy, cosA, sinA, -4.8f * scale,  0.0f,         stabRootX, stabRootY);
  rotPt(cx, cy, cosA, sinA, -8.0f * scale, -3.8f * scale, stabLX, stabLY);
  rotPt(cx, cy, cosA, sinA, -8.0f * scale,  3.8f * scale, stabRX, stabRY);

  // 1. 컬러 지도 위 시인성을 위한 검은색 그림자(외곽선) 레이어
  radarCanvas.drawLine(noseX + 1, noseY + 1, tailX + 1, tailY + 1, TFT_BLACK);
  radarCanvas.fillTriangle(bodyFX + 1, bodyFY + 1, wingLX + 1, wingLY + 1, bodyRX + 1, bodyRY + 1, TFT_BLACK);
  radarCanvas.fillTriangle(bodyFX + 1, bodyFY + 1, wingRX + 1, wingRY + 1, bodyRX + 1, bodyRY + 1, TFT_BLACK);
  radarCanvas.fillTriangle(stabRootX + 1, stabRootY + 1, stabLX + 1, stabLY + 1, tailX + 1, tailY + 1, TFT_BLACK);
  radarCanvas.fillTriangle(stabRootX + 1, stabRootY + 1, stabRX + 1, stabRY + 1, tailX + 1, tailY + 1, TFT_BLACK);

  // 2. 주익(좌/우 후퇴익) 채우기
  radarCanvas.fillTriangle(bodyFX, bodyFY, wingLX, wingLY, bodyRX, bodyRY, fillCol);
  radarCanvas.fillTriangle(bodyFX, bodyFY, wingRX, wingRY, bodyRX, bodyRY, fillCol);

  // 3. 수평꼬리날개 채우기
  radarCanvas.fillTriangle(stabRootX, stabRootY, stabLX, stabLY, tailX, tailY, fillCol);
  radarCanvas.fillTriangle(stabRootX, stabRootY, stabRX, stabRY, tailX, tailY, fillCol);

  // 4. 동체(Fuselage) 2px 두께 중심선
  int fL_x, fL_y, fR_x, fR_y, tL_x, tL_y, tR_x, tR_y;
  rotPt(cx, cy, cosA, sinA,  5.5f * scale, -0.9f * scale, fL_x, fL_y);
  rotPt(cx, cy, cosA, sinA,  5.5f * scale,  0.9f * scale, fR_x, fR_y);
  rotPt(cx, cy, cosA, sinA, -7.5f * scale, -0.7f * scale, tL_x, tL_y);
  rotPt(cx, cy, cosA, sinA, -7.5f * scale,  0.7f * scale, tR_x, tR_y);

  radarCanvas.fillTriangle(noseX, noseY, fL_x, fL_y, fR_x, fR_y, fillCol);
  radarCanvas.fillTriangle(fL_x, fL_y, fR_x, fR_y, tL_x, tL_y, fillCol);
  radarCanvas.fillTriangle(fR_x, fR_y, tL_x, tL_y, tR_x, tR_y, fillCol);
}

// ---------- API ----------
void fetchAircraftData() {
  if (WiFi.status() != WL_CONNECTED) {
    pendingApiOk = false;
    pendingPlaneCount = 0;
    pendingDataReady = true;
    return;
  }

  String url = "https://api.adsb.lol/v2/point/";
  url += String(HOME_LAT, 4);
  url += "/";
  url += String(HOME_LON, 4);
  url += "/";
  url += String(API_RADIUS_NM);

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setTimeout(12000);
  http.addHeader("User-Agent", "ESP32-AirRadar/1.0");

  if (!http.begin(client, url)) {
    pendingApiOk = false;
    pendingDataReady = true;
    return;
  }

  int code = http.GET();

  if (code != 200) {
    Serial.printf("API HTTP error: %d\n", code);
    http.end();
    pendingApiOk = false;
    pendingDataReady = true;
    return;
  }

  String payload = http.getString();
  http.end();

  DynamicJsonDocument doc(65536);
  DeserializationError err = deserializeJson(doc, payload);

  if (err) {
    Serial.print("JSON error: ");
    Serial.println(err.c_str());
    pendingApiOk = false;
    pendingDataReady = true;
    return;
  }

  AircraftPoint tempPlanes[MAX_PLANES];
  int tempCount = 0;

  JsonArray arr = doc["ac"].as<JsonArray>();
  if (arr.isNull()) {
    arr = doc["aircraft"].as<JsonArray>();
  }

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

  Serial.printf("Background aircraft found: %d\n", tempCount);
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
    12000,
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

  Serial.printf("Applied aircraft data: %d\n", planeCount);
}

// ---------- RADAR CANVAS DRAW ----------
void drawGridToCanvas() {
  mapSprite.pushSprite(&radarCanvas, 0, 0);

  for (int d = -240; d <= 240; d += 40) {
    int x = CX + rxCorr(d);
    if (x >= RADAR_X && x <= RADAR_X + RADAR_W) {
      radarCanvas.drawFastVLine(x, RADAR_Y, RADAR_H, greenDark);
    }
  }

  for (int d = -240; d <= 240; d += 40) {
    int y = CY + d;
    if (y >= RADAR_Y && y <= RADAR_Y + RADAR_H) {
      radarCanvas.drawFastHLine(RADAR_X, y, RADAR_W, greenDark);
    }
  }

  mapSprite.pushSprite(&radarCanvas, 0, 0, TFT_BLACK);

  radarCanvas.drawRect(RADAR_X, RADAR_Y, RADAR_W, RADAR_H, green);
}

void drawSweepToCanvas() {
  for (float i = TRAIL_WIDTH; i >= 0; i -= 1.5) {
    float a = sweepAngle - i;
    if (a < 0) a += 360;

    float intensity = (float)(TRAIL_WIDTH - i) / TRAIL_WIDTH;

    uint8_t g = 45 + intensity * 165;
    uint16_t col = radarCanvas.color565(0, g, 45);

    float rad = (a - 90) * DEG_TO_RAD;

    int x = CX + cos(rad) * rxCorr(R);
    int y = CY + sin(rad) * R;

    radarCanvas.drawLine(CX, CY, x, y, col);
  }

  float rad = (sweepAngle - 90) * DEG_TO_RAD;
  int sx = CX + cos(rad) * rxCorr(R);
  int sy = CY + sin(rad) * R;

  radarCanvas.drawLine(CX, CY, sx, sy, greenBright);
  radarCanvas.drawLine(CX + 1, CY, sx + 1, sy, greenBright);
}

void drawRadarCirclesToCanvas() {
  canvasCorrectedCircle(CX, CY, 35, green);
  canvasCorrectedCircle(CX, CY, 70, green);
  canvasCorrectedCircle(CX, CY, 110, green);
  canvasCorrectedCircle(CX, CY, 150, green);
  canvasCorrectedCircle(CX, CY, 190, green);
  canvasCorrectedCircle(CX, CY, R, greenBright);

  for (int i = 0; i < OUTER_RING_THICKNESS; i++) {
    canvasCorrectedCircle(CX, CY, R + OUTER_GAP + 1 + i, greenBright);
  }
}

void drawCrossLinesToCanvas() {
  radarCanvas.drawFastHLine(CX - rxCorr(R), CY, rxCorr(R) * 2, greenBright);
  radarCanvas.drawFastVLine(CX, CY - R, R * 2, greenBright);

  for (int d = 20; d < R; d += 40) {
    radarCanvas.drawFastVLine(CX + rxCorr(d), CY - 7, 14, greenBright);
    radarCanvas.drawFastVLine(CX - rxCorr(d), CY - 7, 14, greenBright);

    radarCanvas.drawFastHLine(CX - 7, CY + d, 14, greenBright);
    radarCanvas.drawFastHLine(CX - 7, CY - d, 14, greenBright);
  }

  for (int a = 30; a < 360; a += 30) {
    if (a == 90 || a == 180 || a == 270) continue;
    canvasCorrectedRadialLine(a, 0, R, greenDim);
  }
}

void drawDistanceLabelsToCanvas() {
  radarCanvas.setTextDatum(middle_center);
  radarCanvas.setTextSize(1);
  radarCanvas.setTextColor(greenBright);

  int radii[] = {35, 70, 110, 150, 190};
  int km[]    = {20, 40, 60, 80, 100};

  const int LABEL_OFFSET = 9;

  for (int i = 0; i < 5; i++) {
    radarCanvas.drawString(String(km[i]) + "km", CX + 18, CY - radii[i] - LABEL_OFFSET);
    radarCanvas.drawString(String(km[i]) + "km", CX + 18, CY + radii[i] + LABEL_OFFSET);
  }

  radarCanvas.drawString("0", CX + 15, CY + 12);
}

void drawAngleScaleToCanvas() {
  for (int a = 0; a < 360; a++) {
    int len = 4;
    if (a % 10 == 0) len = 13;
    else if (a % 5 == 0) len = 8;

    canvasCorrectedRadialLine(a, R - len, R, greenBright);
  }

  radarCanvas.setTextColor(greenBright);
  radarCanvas.setTextSize(2);
  radarCanvas.setTextDatum(middle_center);

  for (int a = 0; a < 360; a += 30) {
    float rad = (a - 90) * DEG_TO_RAD;

    int tx = CX + cos(rad) * rxCorr(R - 28);
    int ty = CY + sin(rad) * (R - 28);

    radarCanvas.drawNumber(a, tx, ty);
  }
}

void drawAircraftTargetsToCanvas() {
  for (int i = 0; i < planeCount; i++) {
    if (!planes[i].valid) continue;

    float ageAngle = angleDiffBehindSweep(sweepAngle, planes[i].bearingDeg);

    if (ageAngle > 260) continue;

    float fade = 1.0;
    if (ageAngle > 200) {
      fade = 1.0 - ((ageAngle - 200.0) / 60.0);
      if (fade < 0.25) fade = 0.25;
    }

    float rr = (planes[i].distanceKm / RADAR_RANGE_KM) * R;
    float rad = (planes[i].bearingDeg - 90) * DEG_TO_RAD;

    int x = CX + cos(rad) * rxCorr(rr);
    int y = CY + sin(rad) * rr;

    uint8_t r = 0;
    uint8_t g = 255 * fade;
    uint8_t b = 35 * fade;

    if (planes[i].altitudeFt >= 30000) {
      r = 60 * fade; g = 210 * fade; b = 255 * fade;
    } else if (planes[i].altitudeFt >= 10000) {
      r = 40 * fade; g = 255 * fade; b = 70 * fade;
    } else if (planes[i].altitudeFt >= 0) {
      r = 255 * fade; g = 220 * fade; b = 30 * fade;
    } else {
      r = 255 * fade; g = 60 * fade; b = 60 * fade;
    }

    uint16_t col = radarCanvas.color565(r, g, b);

    // 거리·고도별 비행기 아이콘 크기 스케일
    float scale = 1.0f;
    if (planes[i].altitudeFt > 30000) scale = 1.15f;
    if (planes[i].distanceKm < 20) scale = 1.30f;

    float heading = (planes[i].trackDeg >= 0) ? planes[i].trackDeg : 0.0f;

    // 기수 전방 속도 벡터선
    if (planes[i].trackDeg >= 0) {
      float tr = (heading - 90.0f) * DEG_TO_RAD;
      int x1 = x + cos(tr) * (int)(8 * scale);
      int y1 = y + sin(tr) * (int)(8 * scale);
      int x2 = x + cos(tr) * (int)(17 * scale);
      int y2 = y + sin(tr) * (int)(17 * scale);
      radarCanvas.drawLine(x1, y1, x2, y2, col);
    }

    // 회전형 비행기 아이콘 그리기
    drawAirplaneIcon(x, y, heading, scale, col);

    // 스윕 직후 편명(Callsign) 표시
    if (ageAngle >= 0 && ageAngle < 45) {
      radarCanvas.setTextDatum(top_left);
      radarCanvas.setTextSize(1);
      radarCanvas.setTextColor(TFT_WHITE, TFT_BLACK);

      String label = String(planes[i].flight);
      label.trim();
      if (label.length() == 0) label = "UNKNOWN";

      radarCanvas.drawString(label, x + 11, y - 13);
    }
  }

  // Top-left status
  radarCanvas.setTextDatum(top_left);
  radarCanvas.setTextSize(2);

  if (apiOk) {
    radarCanvas.setTextColor(green, TFT_BLACK);
    radarCanvas.drawString("REAL ADS-B", 12, 10);

    radarCanvas.setTextColor(greenBright, TFT_BLACK); 
    radarCanvas.setTextSize(2);
    radarCanvas.drawString("AC: " + String(planeCount), 12, 32);
  } else {
    radarCanvas.setTextColor(radarCanvas.color565(255, 80, 80), TFT_BLACK);
    radarCanvas.drawString("NO API", 14, 14);
  }

  if (fetchInProgress) {
    radarCanvas.setTextSize(2);
    radarCanvas.setTextColor(greenDim, TFT_BLACK);
    radarCanvas.drawString("UPDATING", 10, 54);
  }

  // 3D-like compass bottom-left
  int ccx = 50;
  int ccy = 427;
  uint16_t compassBright = greenBright;
  uint16_t compassDim = greenDim;
  uint16_t compassDark = greenDark;

  radarCanvas.setTextDatum(middle_center);

  radarCanvas.drawCircle(ccx, ccy, 17, compassDim);
  radarCanvas.drawCircle(ccx, ccy, 18, compassDark);

  radarCanvas.fillTriangle(ccx, ccy - 16, ccx - 5, ccy - 4, ccx + 5, ccy - 4, compassBright);
  radarCanvas.drawTriangle(ccx, ccy - 16, ccx - 5, ccy - 4, ccx + 5, ccy - 4, TFT_WHITE);

  radarCanvas.fillTriangle(ccx, ccy + 16, ccx - 5, ccy + 4, ccx + 5, ccy + 4, compassDim);
  radarCanvas.fillTriangle(ccx - 16, ccy, ccx - 4, ccy - 5, ccx - 4, ccy + 5, compassDim);
  radarCanvas.fillTriangle(ccx + 16, ccy, ccx + 4, ccy - 5, ccx + 4, ccy + 5, compassDim);

  radarCanvas.fillCircle(ccx, ccy, 2, compassBright);
  radarCanvas.drawCircle(ccx, ccy, 4, compassDim);

  radarCanvas.setTextSize(1);
  radarCanvas.setTextColor(compassBright, TFT_BLACK);
  radarCanvas.drawString("N", ccx, ccy - 29);

  radarCanvas.setTextColor(compassDim, TFT_BLACK);
  radarCanvas.drawString("S", ccx, ccy + 29);
  radarCanvas.drawString("W", ccx - 29, ccy - 3);
  radarCanvas.drawString("E", ccx + 29, ccy - 3);
}

void drawRangeBarsToCanvas() {
  const int bins = 5;
  int counts[bins] = {0};

  for (int i = 0; i < planeCount; i++) {
    if (!planes[i].valid) continue;

    int bin = (int)(planes[i].distanceKm / 20.0);
    if (bin < 0) bin = 0;
    if (bin >= bins) bin = bins - 1;

    counts[bin]++;
  }

  const int startX = 395;
  const int baseY  = 62;
  const int barW   = 10;
  const int barGap = 14;

  const int tickH = 3;
  const int tickGap = 2;
  const int maxTicks = 8;

  radarCanvas.setTextDatum(top_left);
  radarCanvas.setTextSize(1);
  radarCanvas.setTextColor(green, TFT_BLACK);
  radarCanvas.drawString("RANGE AC", startX, 12);

  for (int b = 0; b < bins; b++) {
    int x = startX + b * barGap;
    int c = counts[b];
    if (c > maxTicks) c = maxTicks;

    for (int t = 0; t < c; t++) {
      int y = baseY - t * (tickH + tickGap);
      radarCanvas.fillRect(x, y, barW, tickH, greenBright);
    }

    radarCanvas.drawFastHLine(x, baseY + 7, barW, greenDark);
  }

  radarCanvas.setTextColor(green, TFT_BLACK);
  radarCanvas.drawString("20",  startX - 1,              baseY + 14);
  radarCanvas.drawString("60",  startX + 2 * barGap - 1, baseY + 14);
  radarCanvas.drawString("100", startX + 4 * barGap - 4, baseY + 14);
}

void drawSweepHeadingIndicatorToCanvas() {
  int hx = 430;
  int hy = 420;

  int angleInt = (int)sweepAngle;
  if (angleInt < 0) angleInt += 360;
  if (angleInt >= 360) angleInt -= 360;

  radarCanvas.setTextDatum(middle_center);

  radarCanvas.drawCircle(hx, hy, 28, greenDark);
  radarCanvas.drawCircle(hx, hy, 29, greenDim);

  float a = (sweepAngle - 90) * DEG_TO_RAD;

  int tipX = hx + cos(a) * 23;
  int tipY = hy + sin(a) * 23;

  float leftA  = a + 2.45;
  float rightA = a - 2.45;

  int leftX = hx + cos(leftA) * 10;
  int leftY = hy + sin(leftA) * 10;
  int rightX = hx + cos(rightA) * 10;
  int rightY = hy + sin(rightA) * 10;

  radarCanvas.fillTriangle(tipX, tipY, leftX, leftY, rightX, rightY, greenBright);
  radarCanvas.drawLine(hx, hy, tipX, tipY, TFT_WHITE);

  radarCanvas.fillCircle(hx, hy, 3, greenBright);

  radarCanvas.setTextSize(2);
  radarCanvas.setTextColor(greenBright, TFT_BLACK);

  char buf[8];
  sprintf(buf, "%03d", angleInt);
  radarCanvas.drawString(buf, hx, hy + 42);

  radarCanvas.setTextSize(1);
  radarCanvas.setTextColor(green, TFT_BLACK);
  radarCanvas.drawString("SWEEP", hx, hy - 38);
}

void drawRadarFrameToCanvas() {
  drawGridToCanvas();
  drawRadarCirclesToCanvas();
  drawCrossLinesToCanvas();
  drawSweepToCanvas();
  drawDistanceLabelsToCanvas();
  drawAngleScaleToCanvas();
  drawAircraftTargetsToCanvas();
  drawRangeBarsToCanvas();
  drawSweepHeadingIndicatorToCanvas();
}

void drawTimePanel() {
  int boxX = INFO_X + 18;
  int boxY = INFO_Y + 392;
  int boxW = INFO_W - 36;
  int boxH = 68;

  lcd.fillRect(boxX, boxY, boxW, boxH, TFT_BLACK);

  lcd.drawFastHLine(INFO_X + 10, boxY - 17, INFO_W - 20, greenDark);

  lcd.setTextDatum(top_left);
  lcd.setTextSize(2);
  lcd.setTextColor(greenBright, TFT_BLACK);
  lcd.drawString("TIME:", boxX + 110, boxY - 7);

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char timeStr[12];
    sprintf(timeStr, "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

    lcd.setTextSize(5);
    lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    lcd.drawString(timeStr, boxX + 20, boxY + 22);
  } else {
    lcd.setTextSize(4);
    lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    lcd.drawString("--:--:--", boxX + 20, boxY + 22);
  }
}

void drawLocationPanel() {
  int boxX = INFO_X + 18;
  int boxY = INFO_Y + 305;
  int boxW = INFO_W - 36;
  int boxH = 70;

  lcd.fillRect(boxX, boxY, boxW, boxH, TFT_BLACK);
  lcd.drawFastHLine(INFO_X + 10, boxY - 10, INFO_W - 20, greenDark);

  lcd.setTextDatum(top_center);
  lcd.setTextSize(2);
  lcd.setTextColor(greenBright, TFT_BLACK);
  lcd.drawString(LOCATION_NAME, boxX + boxW / 2, boxY);

  lcd.setTextDatum(top_center);
  lcd.setTextSize(2);
  lcd.setTextColor(TFT_YELLOW, TFT_BLACK);

  String coord = String(HOME_LAT, 4) + " / " + String(HOME_LON, 4);
  lcd.drawString(coord, boxX + boxW / 2, boxY + 32);
}

void drawDetectedAircraftPanel() {
  int topY = INFO_Y + 10;
  int bottomY = INFO_Y + 295;
  int boxX = INFO_X + 10;
  int boxW = INFO_W - 20;
  int boxH = bottomY - topY;

  lcd.fillRect(boxX, topY, boxW, boxH, TFT_BLACK);

  lcd.setTextDatum(top_center);
  lcd.setTextSize(2);
  lcd.setTextColor(greenBright, TFT_BLACK);
  lcd.drawString("DETECTED AIRCRAFTS", INFO_X + INFO_W / 2, topY + 2);

  const int MAX_ROWS = 10;
  int idx[MAX_ROWS];
  for (int i = 0; i < MAX_ROWS; i++) idx[i] = -1;

  for (int i = 0; i < planeCount; i++) {
    if (!planes[i].valid) continue;

    for (int j = 0; j < MAX_ROWS; j++) {
      if (idx[j] == -1 || planes[i].distanceKm < planes[idx[j]].distanceKm) {
        for (int k = MAX_ROWS - 1; k > j; k--) idx[k] = idx[k - 1];
        idx[j] = i;
        break;
      }
    }
  }

  lcd.setTextDatum(top_left);
  lcd.setTextSize(1);

  int y = topY + 34;

  for (int row = 0; row < MAX_ROWS; row++) {
    if (idx[row] == -1) break;

    AircraftPoint &p = planes[idx[row]];

    String call = String(p.flight);
    call.trim();
    if (call.length() == 0) call = "UNKNOWN";

    uint16_t dotCol = greenBright;
    if (p.altitudeFt >= 30000) dotCol = lcd.color565(0, 180, 255);
    else if (p.altitudeFt >= 10000) dotCol = lcd.color565(0, 255, 35);
    else if (p.altitudeFt >= 0) dotCol = lcd.color565(255, 220, 0);
    else dotCol = lcd.color565(255, 40, 40);

    lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    lcd.drawString(call.substring(0, 7), boxX + 2, y);

    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.drawString(String((int)p.distanceKm) + "km", boxX + 76, y);

    if (p.altitudeFt >= 0) lcd.drawString(String(p.altitudeFt) + "ft", boxX + 118, y);
    else lcd.drawString("---ft", boxX + 118, y);

    lcd.drawString(String((int)p.speedKt) + "kt", boxX + 188, y);

    lcd.setTextColor(green, TFT_BLACK);
    String typ = String(p.typeCode);
    typ.trim();
    if (typ.length() == 0) typ = String(p.category);

    lcd.drawString(typ.substring(0, 5), boxX + 235, y);

    lcd.fillCircle(boxX + 272, y + 5, 4, dotCol);

    lcd.drawFastHLine(boxX, y + 17, boxW, greenDark);

    y += 26;
  }

  if (planeCount == 0) {
    lcd.setTextDatum(top_center);
    lcd.setTextSize(1);
    lcd.setTextColor(greenDim, TFT_BLACK);
    lcd.drawString("NO AIRCRAFT IN RANGE", INFO_X + INFO_W / 2, topY + 52);
  }
}

// ---------- RIGHT PANEL ----------
void drawRightInfoPanel() {
  lcd.fillRect(INFO_X, INFO_Y, INFO_W, INFO_H, TFT_BLACK);
  lcd.drawRect(INFO_X, INFO_Y, INFO_W, INFO_H, green);

  lcd.setTextDatum(top_left);

  drawDetectedAircraftPanel();
  drawLocationPanel();
  drawTimePanel();
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);

  Wire.begin(19, 20);
  Wire.beginTransmission(0x18);
  Wire.write(0x01);
  Wire.write(0x3B);
  Wire.endTransmission();

  lcd.init();
  lcd.setBrightness(180);

  greenBright = lcd.color565(0, 255, 35);
  green       = lcd.color565(0, 180, 25);
  greenDim    = lcd.color565(0, 90, 18);
  greenDark   = lcd.color565(0, 45, 10);
  panelBg     = lcd.color565(0, 20, 0);

  // 선명한 컬러 지도 및 항공 항로 팔레트
  colSea      = lcd.color565(10, 38, 78);     // 진한 해양 블루
  colLand     = lcd.color565(28, 62, 38);     // 뚜렷한 포레스트 그린 육지
  colCoast    = lcd.color565(65, 205, 175);   // 선명한 민트/시안 해안선
  colRiver    = lcd.color565(35, 145, 235);   // 선명한 낙동강 블루
  colRoad     = lcd.color565(145, 110, 55);   // 따뜻한 앰버 주요 간선축
  colCityDot  = lcd.color565(255, 195, 65);   // 밝은 오렌지 옐로우 도시 마커
  colCityText = lcd.color565(235, 245, 220);  // 고대비 아이보리 지명
  colAirway   = lcd.color565(110, 215, 255);  // 점선 항로선 (스카이 시안)
  colWaypoint = lcd.color565(180, 240, 255);  // 웨이포인트 삼각형 및 이름

  radarCanvas.setPsram(true);
  radarCanvas.setColorDepth(16);
  if (!radarCanvas.createSprite(480, 480)) {
    Serial.println("Radar sprite create failed!");
  }

  mapSprite.setPsram(true);
  mapSprite.setColorDepth(16);
  if (!mapSprite.createSprite(480, 480)) {
    Serial.println("Map sprite create failed!");
  } else {
    buildColorMapSprite();   // 컬러 지형도 + STAR/SID 점선 항로 사전 렌더링
  }

  lcd.fillScreen(TFT_BLACK);

  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.setTextDatum(middle_center);
  lcd.drawString("Connecting WiFi...", 240, 240);

  WiFi.begin(ssid, password);

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(300);
  }

  lcd.fillScreen(TFT_BLACK);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");

    // 한국 표준시 (KST, UTC+9) 설정
    configTzTime("KST-9", "pool.ntp.org", "time.nist.gov");

    fetchAircraftData();
    applyPendingAircraftData();
    startAircraftFetchBackground();

  } else {
    Serial.println("WiFi failed");
    apiOk = false;
  }

  drawRadarFrameToCanvas();
  radarCanvas.pushSprite(0, 0);

  drawRightInfoPanel();
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
      drawDetectedAircraftPanel();
      startAircraftFetchBackground();
    }
  }

  if (millis() - lastClockUpdate > 1000) {
    lastClockUpdate = millis();
    drawTimePanel();
  }
}