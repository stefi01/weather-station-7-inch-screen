/*
  Sky Weather — 7 inch desk.
  Own prefs name skywx. Does not clear the plane tracker.
  Sun arc, pressure, air, running high/low until midnight,
  forecast high/low, alerts, tides, moon, rain, satellite / clouds / storm.
  No road map. No seconds. Clock corner updates once a minute.
*/

#include <Arduino_GFX_Library.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <FFat.h>
#include <FS.h>
#include <Wire.h>
#include <TJpg_Decoder.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "iss_world.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define GFX_BL 2
#define PW 800
#define PH 480
#define TOUCH_SDA 19
#define TOUCH_SCL 20
#define TOUCH_RST 38
#define GT911_ADDR 0x5D
#define KW 80

enum {
  PG_HOME = 0,
  PG_WIFI,
  PG_PLACE,
  PG_RADAR,
  PG_RAIN,
  PG_SKY,
  PG_NIGHT,
  PG_LAUNCH,
  PG_ISS,
  PG_CLOCK,
  PG_SET,
  PG_STATION,
  PG_WARN,
  PG_DAYS,
  PG_HOURS
};
enum { NET_NONE = 0, NET_PACK, NET_MAP, NET_FIND, NET_LAUNCH, NET_ISS };
enum { MAP_SAT = 0, MAP_CLOUD, MAP_STORM };

struct PlaceHit {
  char label[40];
  float lat, lon;
};
struct AlertParse {
  char event[48];
  char sev[16];
  char sender[36];
  char ends[40];
  char bestEvent[48];
  char bestSender[36];
  char bestEnds[40];
  char headline[80];
  char bestHeadline[80];
  char keepEvent[3][48];
  char keepEnds[3][40];
  char keepHead[3][80];
  char keepSev[3][16];
  int nKeep;
  int bestRank;
  int count;
};
struct ZoneRow {
  const char *iana;
  const char *posix;
};
struct Launch {
  char day[12];
  char when[8];
  char rocket[28];
  char pad[28];
  char agency[16];
};
struct SkyObj {
  char name[16];
  char note[20];
  char look[4];
  char hours[36];
  int az, el;
};
struct SkyCand {
  char name[16];
  char note[20];
  char hours[36];
  int az, el, score;
};
struct IssTle {
  double epochJd, n, e, inc, raan, argp, m0;
  bool ok;
};

static const ZoneRow ZONES[] = {
    {"America/New_York", "EST5EDT,M3.2.0,M11.1.0"},
    {"America/Detroit", "EST5EDT,M3.2.0,M11.1.0"},
    {"America/Kentucky/Louisville", "EST5EDT,M3.2.0,M11.1.0"},
    {"America/Kentucky/Monticello", "EST5EDT,M3.2.0,M11.1.0"},
    {"America/Indiana/Indianapolis", "EST5EDT,M3.2.0,M11.1.0"},
    {"America/Indiana/Marengo", "EST5EDT,M3.2.0,M11.1.0"},
    {"America/Indiana/Petersburg", "EST5EDT,M3.2.0,M11.1.0"},
    {"America/Indiana/Vevay", "EST5EDT,M3.2.0,M11.1.0"},
    {"America/Indiana/Vincennes", "EST5EDT,M3.2.0,M11.1.0"},
    {"America/Indiana/Winamac", "EST5EDT,M3.2.0,M11.1.0"},
    {"America/Chicago", "CST6CDT,M3.2.0,M11.1.0"},
    {"America/Indiana/Knox", "CST6CDT,M3.2.0,M11.1.0"},
    {"America/Indiana/Tell_City", "CST6CDT,M3.2.0,M11.1.0"},
    {"America/Menominee", "CST6CDT,M3.2.0,M11.1.0"},
    {"America/North_Dakota/Beulah", "CST6CDT,M3.2.0,M11.1.0"},
    {"America/North_Dakota/Center", "CST6CDT,M3.2.0,M11.1.0"},
    {"America/North_Dakota/New_Salem", "CST6CDT,M3.2.0,M11.1.0"},
    {"America/Denver", "MST7MDT,M3.2.0,M11.1.0"},
    {"America/Boise", "MST7MDT,M3.2.0,M11.1.0"},
    {"America/Phoenix", "MST7"},
    {"America/Los_Angeles", "PST8PDT,M3.2.0,M11.1.0"},
    {"America/Anchorage", "AKST9AKDT,M3.2.0,M11.1.0"},
    {"America/Juneau", "AKST9AKDT,M3.2.0,M11.1.0"},
    {"America/Sitka", "AKST9AKDT,M3.2.0,M11.1.0"},
    {"America/Yakutat", "AKST9AKDT,M3.2.0,M11.1.0"},
    {"America/Nome", "AKST9AKDT,M3.2.0,M11.1.0"},
    {"America/Metlakatla", "AKST9AKDT,M3.2.0,M11.1.0"},
    {"America/Adak", "HST10HDT,M3.2.0,M11.1.0"},
    {"Pacific/Honolulu", "HST10"},
    {"America/Puerto_Rico", "AST4"},
    {"America/Virgin", "AST4"},
    {"America/St_Thomas", "AST4"},
    {"Pacific/Guam", "ChST-10"},
    {"Pacific/Saipan", "ChST-10"},
    {"Pacific/Pago_Pago", "SST11"},
};
static const char *MON[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                            "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
static const char *WDAY[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
static const uint16_t INK = RGB565(80, 220, 255);
static const uint16_t SUN_C = RGB565(255, 210, 40);
static const uint16_t WELL = RGB565(0, 0, 0);

Arduino_ESP32RGBPanel *rgbBus = nullptr;
Arduino_RGB_Display *gfx = nullptr;
Preferences prefs;

uint16_t COL_BG, COL_DIM, COL_MID, COL_HOT, COL_TXT, COL_UP, COL_DN, COL_AMB;

int page = PG_HOME;
int backTo = PG_HOME;
int blLevel = 2;
bool dayMode = false;
bool useC = false;
bool wifiOk = false;
bool placeSet = false;
float placeLat = 0, placeLon = 0;
char placeName[40] = "";
char placeTz[56] = "";
char placeQuery[42] = "";
char placeMsg[28] = "";
PlaceHit hits[4];
int nHits = 0, placeTop = 0;
bool placeShift = false, placeSym = false;

bool wxOk = false;
int wxCode = 0, wxDay = 1, wxWind = 0, wxDir = 0, wxTemp = 0, wxFeel = 0, wxRh = 0, wxUv = 0, wxAqi = -1;
float wxRad = 0, pressHpa = 0, pressBase = 0, pressLo = 0, pressHi = 0;
int pressTrend = 0;
int dayHot = 0, dayCold = 0, extremeYmd = 0;
int fcHigh = 0, fcLow = 0;
char wxUp[16] = "";
char wxDown[16] = "";
float rainPop[12];
float rainAmt[12];
int nRain = 0;
int dayHi[7], dayLo[7], dayPop[7], dayCode[7];
int nDays = 0;
int slotT[12], slotP[12], slotC[12];
int nSlot = 0;
struct WxAlert {
  char event[48];
  char ends[40];
  char headline[80];
  char sev[16];
};
WxAlert warns[3];
int nWarn = 0;
int auroraKp = -1;
float rainToday = 0, rainYday = 0, rainMonth = 0;
bool rainTotalsOk = false;

bool alertLive = false, alertOn = false;
char alertEvent[48] = "";
char alertEnds[40] = "";
char alertDetail[80] = "";
char alertNote[24] = "set a place";

char tideId[12] = "";
char tideName[24] = "";
char tideHigh[28] = "";
char tideLow[28] = "";
bool tideKnown = false;

Launch launches[4];
int nLaunch = 0;
bool launchOk = false;

SkyObj skyVis[18];
int nSkyVis = 0;
int nightPage = 0;
uint32_t lastAstro = 0;

IssTle issTle;
bool issOk = false, issVisNow = false;
int issAz = 0, issEl = 0;
float issAltKm = 0;
float issLat = 0, issLon = 0;
char issLook[4] = "";
char issWhen[28] = "";
uint32_t lastIssPass = 0;
float issTrackLat[72];
float issTrackLon[72];
int nIssTrack = 0;
struct IssPass {
  char line[48];
  char from[4];
  time_t when;
  int durSec;
};
IssPass issPasses[5];
int nIssPass = 0;
time_t issNextPass = 0;
uint32_t lastPassAt = 0;
float sunLat = 0, sunLon = 0;
bool sunOk = false;
const int ISS_Y0 = 64;
const int ISS_H = 200;
const int ISS_SRC_W = 320;
const int ISS_SRC_H = 119;
const int ISS_JPG_H = 176;
uint16_t *issFb = nullptr;
uint16_t *issShade = nullptr;
uint16_t *issJpg = nullptr;
bool issMapOk = false;
uint32_t lastIssMapTry = 0;

int mapKind = MAP_SAT;
int jpgOx = 0, jpgOy = 0, jpgClipY = 470;
uint8_t *mapHold = nullptr;
uint8_t *mapRadar = nullptr;
int mapHoldLen = 0;
int mapRadarLen = 0;
bool mapFail = false;
struct MapTown {
  char name[24];
  float lat, lon;
};
MapTown mapTowns[18];
int nMapTowns = 0;
float tideBestKm = 99999.0f;

char wifiSsid[64] = "";
char wifiPass[64] = "";
char wifiScan[16][33];
char wifiMsg[28] = "";
int nWifiScan = 0, wifiScanTop = 0, wifiFocus = 0;
bool wifiShift = false, wifiSym = false, wifiScanBusy = false;

char *keyDest = nullptr;
int keyCap = 0;
bool *keyShift = nullptr;
bool *keySym = nullptr;

volatile int netJob = NET_NONE;
volatile int netPend = NET_NONE;
volatile bool netBusy = false;
volatile bool netDone = false;
volatile int netDoneJob = NET_NONE;

uint32_t lastPack = 0, lastMap = 0, lastIss = 0, lastLaunch = 0, lastTap = 0;
int shownMin = -1;
char packBuf[14336];

void cols() {
  if (dayMode) {
    COL_BG = RGB565(232, 226, 214);
    COL_DIM = RGB565(210, 202, 186);
    COL_MID = RGB565(90, 94, 102);
    COL_HOT = RGB565(196, 132, 8);
    COL_TXT = RGB565(28, 30, 34);
    COL_UP = RGB565(20, 120, 60);
    COL_DN = RGB565(170, 40, 40);
    COL_AMB = RGB565(176, 90, 16);
  } else {
    COL_BG = RGB565(8, 12, 18);
    COL_DIM = RGB565(16, 28, 40);
    COL_MID = RGB565(150, 170, 186);
    COL_HOT = RGB565(255, 210, 40);
    COL_TXT = RGB565(236, 242, 246);
    COL_UP = RGB565(40, 210, 120);
    COL_DN = RGB565(230, 60, 70);
    COL_AMB = RGB565(255, 170, 40);
  }
}

void centre(const char *s, int cx, int y) {
  int16_t x1, y1;
  uint16_t w, h;
  gfx->getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  gfx->setCursor(cx - (int)w / 2, y);
  gfx->print(s);
}

void rightText(const char *s, int right, int y) {
  int16_t x1, y1;
  uint16_t w, h;
  gfx->getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  gfx->setCursor(right - (int)w, y);
  gfx->print(s);
}

bool hit(int x, int y, int x0, int y0, int x1, int y1) {
  return x >= x0 && x <= x1 && y >= y0 && y <= y1;
}

void chip(int x, int y, int w, int h, const char *t, bool on) {
  gfx->fillRoundRect(x, y, w, h, 6, on ? COL_HOT : COL_DIM);
  gfx->setTextColor(on ? COL_BG : COL_TXT, on ? COL_HOT : COL_DIM);
  gfx->setTextSize(2);
  centre(t, x + w / 2, y + h / 2 - 8);
}

void ticks(int x, int y, int w, int h, uint16_t c) {
  const int n = 14;
  gfx->drawFastHLine(x, y, n, c);
  gfx->drawFastVLine(x, y, n, c);
  gfx->drawFastHLine(x + w - n, y, n, c);
  gfx->drawFastVLine(x + w - 1, y, n, c);
  gfx->drawFastHLine(x, y + h - 1, n, c);
  gfx->drawFastVLine(x, y + h - n, n, c);
  gfx->drawFastHLine(x + w - n, y + h - 1, n, c);
  gfx->drawFastVLine(x + w - 1, y + h - n, n, c);
}

void applyPlaceTz() {
  if (!placeTz[0]) return;
  setenv("TZ", placeTz, 1);
  tzset();
}

void applyBl() {
  pinMode(GFX_BL, OUTPUT);
  if (blLevel >= 2) {
    digitalWrite(GFX_BL, HIGH);
    return;
  }
  analogWrite(GFX_BL, blLevel == 0 ? 70 : 150);
}

int showDeg(int f) {
  if (!useC) return f;
  float c = (f - 32) * 5.0f / 9.0f;
  if (c >= 0) return (int)(c + 0.5f);
  return (int)(c - 0.5f);
}

void savePrefs() {
  prefs.begin("skywx", false);
  prefs.putString("ssid", wifiSsid);
  prefs.putString("pass", wifiPass);
  prefs.putBool("set", placeSet);
  prefs.putFloat("lat", placeLat);
  prefs.putFloat("lon", placeLon);
  prefs.putString("name", placeName);
  prefs.putString("tzp", placeTz);
  prefs.putInt("bl", blLevel);
  prefs.putBool("day", dayMode);
  prefs.putBool("c", useC);
  prefs.putInt("hot", dayHot);
  prefs.putInt("cold", dayCold);
  prefs.putInt("ymd", extremeYmd);
  prefs.putFloat("pbase", pressBase);
  prefs.putString("tide", tideId);
  prefs.putString("tname", tideName);
  prefs.end();
}

void loadPrefs() {
  prefs.begin("skywx", true);
  strlcpy(wifiSsid, prefs.getString("ssid", "").c_str(), sizeof(wifiSsid));
  strlcpy(wifiPass, prefs.getString("pass", "").c_str(), sizeof(wifiPass));
  placeSet = prefs.getBool("set", false);
  placeLat = prefs.getFloat("lat", 0);
  placeLon = prefs.getFloat("lon", 0);
  strlcpy(placeName, prefs.getString("name", "").c_str(), sizeof(placeName));
  strlcpy(placeTz, prefs.getString("tzp", "").c_str(), sizeof(placeTz));
  blLevel = prefs.getInt("bl", 2);
  dayMode = prefs.getBool("day", false);
  useC = prefs.getBool("c", false);
  dayHot = prefs.getInt("hot", 0);
  dayCold = prefs.getInt("cold", 0);
  extremeYmd = prefs.getInt("ymd", 0);
  pressBase = prefs.getFloat("pbase", 0);
  strlcpy(tideId, prefs.getString("tide", "").c_str(), sizeof(tideId));
  strlcpy(tideName, prefs.getString("tname", "").c_str(), sizeof(tideName));
  prefs.end();
  if (!placeSet || !placeName[0]) placeSet = false;
  if (blLevel < 0 || blLevel > 2) blLevel = 2;
  tideKnown = tideId[0] != 0;
  applyPlaceTz();
  cols();
}

void factoryReset() {
  prefs.begin("skywx", false);
  prefs.clear();
  prefs.end();
}

bool liveClock(time_t *out) {
  time_t now = time(nullptr);
  if (now < 1700000000) return false;
  if (out) *out = now;
  return true;
}

int localYmd() {
  time_t now;
  if (!liveClock(&now)) return 0;
  struct tm t;
  localtime_r(&now, &t);
  return (t.tm_year + 1900) * 10000 + (t.tm_mon + 1) * 100 + t.tm_mday;
}

void fmtHm(int h24, int mi, char *out, int n) {
  int h = h24 % 12;
  if (h == 0) h = 12;
  snprintf(out, n, "%d:%02d %s", h, mi, h24 >= 12 ? "PM" : "AM");
}

void footLabel(const char *hm, char *out, int n) {
  int h = 0, mi = 0;
  char ap = 'A';
  if (sscanf(hm, "%d:%d %c", &h, &mi, &ap) < 2) {
    strlcpy(out, hm, n);
    return;
  }
  snprintf(out, n, "%d:%02d %s", h, mi, (ap == 'P' || ap == 'p') ? "pm" : "am");
}

const char *clockFaceWord() {
  int c = wxCode;
  if (c >= 95) return "thunder";
  if ((c >= 71 && c < 80) || (c >= 85 && c < 95)) return "snow";
  if (c >= 51) return "rain";
  if (c >= 45) return "fog";
  if (c >= 3) return "cloudy";
  if (wxWind >= 25) return "windy";
  if (c == 0) return wxDay ? "sunny" : "clear";
  return "partly cloudy";
}

const char *skyWord() {
  int c = wxCode;
  if (c == 96 || c == 99) return "HAIL";
  if (c >= 95) return "THUNDER";
  if ((c >= 71 && c <= 77) || c == 85 || c == 86) return "SNOW";
  if ((c >= 51 && c <= 67) || (c >= 80 && c <= 82)) return "RAIN";
  if (wxWind >= 25 && c <= 3) return "WIND";
  if (c == 0) return wxDay ? "SUN" : "NIGHT";
  if (c == 1) return wxDay ? "CLEAR" : "NIGHT";
  if (c == 2) return "PARTLY CLOUDY";
  return "CLOUDY";
}

uint16_t skyColor(const char *w) {
  if (!strcmp(w, "SUN") || !strcmp(w, "CLEAR")) return SUN_C;
  if (!strcmp(w, "THUNDER") || !strcmp(w, "HAIL")) return COL_DN;
  if (!strcmp(w, "SNOW")) return COL_TXT;
  if (!strcmp(w, "WIND")) return COL_AMB;
  return COL_TXT;
}

const char *aqiWord(int aqi) {
  if (aqi < 0) return "--";
  if (aqi <= 50) return "good";
  if (aqi <= 100) return "moderate";
  if (aqi <= 150) return "poor";
  return "bad";
}

uint16_t aqiColor(int aqi) {
  if (aqi < 0) return COL_MID;
  if (aqi <= 50) return COL_UP;
  if (aqi <= 100) return COL_AMB;
  return COL_DN;
}

bool stayIn() {
  if (!wxOk) return false;
  if (alertOn && strstr(alertEvent, "Warning")) return true;
  if (wxCode >= 95 || wxWind >= 30 || wxAqi > 150) return true;
  if (wxDay && wxUv >= 8) return true;
  float wet = 0;
  int pop = 0;
  for (int i = 0; i < 3 && i < nRain; i++) {
    if ((int)rainPop[i] > pop) pop = (int)rainPop[i];
    wet += rainAmt[i];
  }
  return pop >= 60 && wet >= 0.10f;
}

void drawClockCorner() {
  gfx->fillRect(470, 0, 330, 44, COL_BG);
  if (!placeTz[0]) {
    gfx->setTextColor(COL_MID, COL_BG);
    gfx->setTextSize(2);
    rightText("place clock", 788, 12);
    return;
  }
  if (!liveClock(nullptr)) {
    gfx->setTextColor(COL_MID, COL_BG);
    gfx->setTextSize(2);
    rightText("internet clock", 788, 12);
    return;
  }
  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);
  char line[24];
  fmtHm(t.tm_hour, t.tm_min, line, sizeof(line));
  gfx->setTextColor(COL_TXT, COL_BG);
  gfx->setTextSize(3);
  rightText(line, 788, 0);
  snprintf(line, sizeof(line), "%s %s %d", WDAY[t.tm_wday], MON[t.tm_mon], t.tm_mday);
  gfx->setTextColor(INK, COL_BG);
  gfx->setTextSize(2);
  rightText(line, 788, 26);
  shownMin = t.tm_hour * 60 + t.tm_min;
}

void drawSunArc(int x, int y, int w, int h) {
  gfx->fillRoundRect(x, y, w, h, 8, WELL);
  ticks(x + 6, y + 6, w - 12, h - 12, INK);
  int cx = x + w / 2;
  int base = y + h - 22;
  int rad = w / 2 - 28;
  int room = h - 70;
  if (rad > room) rad = room;
  if (rad < 36) rad = 36;
  for (int deg = 0; deg <= 180; deg += 5) {
    float a = (180 - deg) * (float)M_PI / 180.0f;
    int px = cx + (int)(cosf(a) * rad);
    int py = base - (int)(sinf(a) * rad);
    gfx->fillCircle(px, py, (deg % 30 == 0) ? 3 : 2, INK);
  }
  float frac = 0.5f;
  int rh = 0, rm = 0, sh = 0, sm = 0;
  char rap = 'A', sap = 'A';
  bool haveRise = sscanf(wxUp, "%d:%d %c", &rh, &rm, &rap) >= 2;
  bool haveSet = sscanf(wxDown, "%d:%d %c", &sh, &sm, &sap) >= 2;
  int rise = haveRise ? ((rap == 'P' || rap == 'p') ? (rh % 12 + 12) : (rh % 12)) * 60 + rm : -1;
  int setm = haveSet ? ((sap == 'P' || sap == 'p') ? (sh % 12 + 12) : (sh % 12)) * 60 + sm : -1;
  time_t now;
  int nowMin = 12 * 60;
  if (liveClock(&now)) {
    struct tm t;
    localtime_r(&now, &t);
    nowMin = t.tm_hour * 60 + t.tm_min;
  }
  if (rise >= 0 && setm > rise) {
    if (nowMin <= rise) frac = 0;
    else if (nowMin >= setm) frac = 1;
    else frac = (float)(nowMin - rise) / (float)(setm - rise);
  }
  float sa = (180.0f - frac * 180.0f) * (float)M_PI / 180.0f;
  int sx = cx + (int)(cosf(sa) * rad);
  int sy = base - (int)(sinf(sa) * rad);
  bool up = frac > 0.02f && frac < 0.98f;
  bool sunOnArc = true;
  if (rise >= 0 && setm > rise) sunOnArc = nowMin >= rise && nowMin < setm;
  if (sunOnArc) {
    gfx->fillCircle(sx, sy, up ? 11 : 7, SUN_C);
    if (up) {
      for (int i = 0; i < 8; i++) {
        float r = i * 0.785398f;
        gfx->drawLine(sx + (int)(cosf(r) * 14), sy + (int)(sinf(r) * 14),
                      sx + (int)(cosf(r) * 20), sy + (int)(sinf(r) * 20), SUN_C);
      }
    }
  }
  char line[24];
  gfx->setTextColor(INK, WELL);
  gfx->setTextSize(2);
  if (wxOk) snprintf(line, sizeof(line), "%.0f W/m2", wxRad);
  else strlcpy(line, "-- W/m2", sizeof(line));
  gfx->setCursor(x + 14, y + 10);
  gfx->print(line);
  if (wxOk) snprintf(line, sizeof(line), "%d UV INDEX", wxUv);
  else strlcpy(line, "-- UV INDEX", sizeof(line));
  rightText(line, x + w - 14, y + 10);
  char foot[16];
  footLabel(wxUp, foot, sizeof(foot));
  gfx->setCursor(x + 16, base + 2);
  gfx->print(wxUp[0] ? foot : "-- am");
  footLabel(wxDown, foot, sizeof(foot));
  rightText(wxDown[0] ? foot : "-- pm", x + w - 16, base + 2);
}

void drawPressure(int x, int y, int w, int h) {
  gfx->fillRoundRect(x, y, w, h, 6, WELL);
  ticks(x + 4, y + 4, w - 8, h - 8, INK);
  char line[40];
  if (pressHpa > 0)
    snprintf(line, sizeof(line), "air pressure  %.2f in", pressHpa / 33.8639f);
  else
    strlcpy(line, "air pressure  --", sizeof(line));
  gfx->setTextColor(INK, WELL);
  gfx->setTextSize(2);
  gfx->setCursor(x + 14, y + h / 2 - 8);
  gfx->print(line);
  if (pressBase > 0 && pressHpa > 0 && pressTrend >= 0) {
    const char *tr = "steady";
    uint16_t tc = COL_MID;
    if (pressTrend > 0) {
      tr = "rising";
      tc = COL_UP;
    }
    gfx->setTextColor(tc, WELL);
    rightText(tr, x + w - 12, y + h / 2 - 8);
  }
}

void drawFoot() {
  const char *lab[] = {"PLACE", "RADAR", "RAIN", "SKY", "WIFI", "SET"};
  for (int i = 0; i < 6; i++) chip(8 + i * 132, 420, 124, 52, lab[i], false);
}

int footAt(int x, int y) {
  if (y < 420 || y > 472) return -1;
  int i = (x - 8) / 132;
  if (i < 0 || i > 5) return -1;
  if (x > 8 + i * 132 + 124) return -1;
  return i;
}

void drawMoonPhase(int cx, int cy, int r, float phase) {
  const uint16_t lit = RGB565(236, 232, 214);
  const uint16_t dark = RGB565(28, 32, 42);
  gfx->fillCircle(cx, cy, r, dark);
  if (phase >= 0 && phase <= 1) {
    float k = cosf(phase * 2.0f * (float)M_PI);
    for (int dy = -r; dy <= r; dy++) {
      int span = r * r - dy * dy;
      if (span <= 0) continue;
      int xe = (int)sqrtf((float)span);
      int x1, x2;
      if (phase < 0.5f) {
        x1 = (int)(k * (float)xe);
        x2 = xe;
      } else {
        x1 = -xe;
        x2 = (int)(-k * (float)xe);
      }
      if (x2 > x1) gfx->drawFastHLine(cx + x1, cy + dy, x2 - x1, lit);
    }
  }
  gfx->drawCircle(cx, cy, r, COL_MID);
}

void drawAlertBar() {
  gfx->fillRoundRect(8, 46, 784, 32, 6, COL_DIM);
  gfx->setTextSize(2);
  if (!placeSet) {
    gfx->setTextColor(COL_MID, COL_DIM);
    centre("type a US city or ZIP", 400, 54);
    return;
  }
  if (!wifiOk) {
    gfx->setTextColor(COL_DN, COL_DIM);
    centre("join wifi", 400, 54);
    return;
  }
  if (alertOn && alertEvent[0]) {
    char until[40], line[72];
    formatUntil(alertEnds, until, sizeof(until));
    snprintf(line, sizeof(line), "%s  %s", alertEvent, until);
    gfx->setTextColor(strstr(alertEvent, "Warning") ? COL_DN : COL_AMB, COL_DIM);
    centre(line, 400, 54);
    return;
  }
  if (alertLive) {
    const char *sky = wxOk ? skyWord() : "";
    bool openSky = !strcmp(sky, "CLEAR") || !strcmp(sky, "SUN");
    gfx->setTextColor(COL_UP, COL_DIM);
    centre(openSky ? "ALL CLEAR" : "NO WARNINGS", 400, 54);
    return;
  }
  gfx->setTextColor(COL_MID, COL_DIM);
  centre(alertNote[0] ? alertNote : "waiting", 400, 54);
}

void drawWxCloud(int cx, int cy, uint16_t c) {
  gfx->fillCircle(cx - 12, cy, 12, c);
  gfx->fillCircle(cx + 2, cy - 6, 14, c);
  gfx->fillCircle(cx + 16, cy + 1, 11, c);
}

void drawHomeWx(int cx, int cy) {
  if (!wxOk) return;
  const char *w = skyWord();
  if (!strcmp(w, "THUNDER") || !strcmp(w, "HAIL")) {
    drawWxCloud(cx, cy - 8, COL_MID);
    gfx->fillTriangle(cx - 4, cy + 4, cx + 10, cy + 4, cx - 2, cy + 24, SUN_C);
    gfx->fillTriangle(cx + 2, cy + 14, cx + 14, cy + 14, cx + 4, cy + 32, SUN_C);
  } else if (!strcmp(w, "RAIN") || !strcmp(w, "SNOW")) {
    drawWxCloud(cx, cy - 8, COL_MID);
    uint16_t drop = !strcmp(w, "SNOW") ? COL_TXT : INK;
    for (int i = -16; i <= 16; i += 8) gfx->fillCircle(cx + i, cy + 16, 3, drop);
    if (wxWind >= 25) {
      gfx->drawLine(cx - 20, cy + 28, cx + 18, cy + 28, INK);
      gfx->drawLine(cx - 16, cy + 36, cx + 22, cy + 36, INK);
    }
  } else if (!strcmp(w, "WIND")) {
    gfx->drawLine(cx - 22, cy - 10, cx + 22, cy - 10, INK);
    gfx->drawLine(cx - 18, cy, cx + 26, cy, INK);
    gfx->drawLine(cx - 22, cy + 10, cx + 18, cy + 10, INK);
  } else if (!strcmp(w, "CLOUDY")) {
    drawWxCloud(cx, cy, COL_MID);
  } else if (!strcmp(w, "PARTLY CLOUDY")) {
    gfx->fillCircle(cx - 10, cy - 8, 12, SUN_C);
    drawWxCloud(cx + 8, cy + 6, COL_MID);
  } else if (!strcmp(w, "NIGHT")) {
    int pct = 0;
    float phase = 0;
    if (moonName(&phase, &pct)) drawMoonPhase(cx, cy, 16, phase);
    else gfx->drawCircle(cx, cy, 16, COL_TXT);
  } else if (!strcmp(w, "CLEAR")) {
    gfx->drawCircle(cx, cy, 16, SUN_C);
    gfx->fillCircle(cx, cy, 7, SUN_C);
  } else {
    gfx->fillCircle(cx, cy, 14, SUN_C);
    for (int a = 0; a < 8; a++) {
      float r = a * 0.785398f;
      gfx->drawLine(cx + (int)(cosf(r) * 18), cy + (int)(sinf(r) * 18), cx + (int)(cosf(r) * 28),
                    cy + (int)(sinf(r) * 28), SUN_C);
    }
  }
}

void drawHome() {
  page = PG_HOME;
  gfx->fillScreen(COL_BG);
  ticks(2, 2, PW - 4, PH - 4, COL_DIM);
  gfx->setTextColor(SUN_C, COL_BG);
  gfx->setTextSize(2);
  gfx->setCursor(12, 12);
  gfx->print(placeSet ? placeName : "SET PLACE");
  drawClockCorner();
  drawAlertBar();

  gfx->fillRoundRect(8, 84, 284, 206, 8, COL_DIM);
  ticks(14, 90, 272, 194, INK);
  const char *word = wxOk ? skyWord() : "SKY";
  gfx->setTextColor(skyColor(word), COL_DIM);
  gfx->setTextSize(!strcmp(word, "PARTLY CLOUDY") ? 2 : 3);
  gfx->setCursor(22, 98);
  gfx->print(word);
  gfx->setTextSize(2);
  gfx->setTextColor(stayIn() ? COL_AMB : COL_UP, COL_DIM);
  gfx->setCursor(22, 128);
  gfx->print(wxOk ? (stayIn() ? "STAY IN" : "OUT") : "");
  gfx->setTextColor(COL_TXT, COL_DIM);
  gfx->setTextSize(4);
  gfx->setCursor(22, 150);
  if (wxOk) {
    gfx->print(showDeg(wxTemp));
    gfx->print(useC ? "C" : "F");
  } else
    gfx->print("--");
  gfx->setTextSize(2);
  char line[40];
  if (extremeYmd && wxOk)
    snprintf(line, sizeof(line), "HIGH %d%s", showDeg(dayHot), useC ? "C" : "F");
  else
    strlcpy(line, "HIGH --", sizeof(line));
  gfx->setTextColor(COL_DN, COL_DIM);
  gfx->setCursor(22, 188);
  gfx->print(line);
  if (extremeYmd && wxOk)
    snprintf(line, sizeof(line), "LOW %d%s", showDeg(dayCold), useC ? "C" : "F");
  else
    strlcpy(line, "LOW --", sizeof(line));
  gfx->setTextColor(INK, COL_DIM);
  gfx->setCursor(22, 210);
  gfx->print(line);
  if (wxOk) snprintf(line, sizeof(line), "high %d   low %d", showDeg(fcHigh), showDeg(fcLow));
  else strlcpy(line, "high --   low --", sizeof(line));
  gfx->setTextColor(COL_TXT, COL_DIM);
  gfx->setCursor(22, 232);
  gfx->print(line);
  if (wxOk) snprintf(line, sizeof(line), "wind %d   hum %d%%", wxWind, wxRh);
  else strlcpy(line, "wind --", sizeof(line));
  gfx->setCursor(22, 252);
  gfx->print(line);
  gfx->setTextColor(aqiColor(wxAqi), COL_DIM);
  if (wxAqi >= 0) snprintf(line, sizeof(line), "air %s %d", aqiWord(wxAqi), wxAqi);
  else strlcpy(line, "air --", sizeof(line));
  gfx->setCursor(22, 272);
  gfx->print(line);
  drawHomeWx(236, 118);

  drawSunArc(300, 84, 492, 156);
  drawPressure(300, 246, 492, 44);

  gfx->fillRoundRect(8, 298, 252, 114, 8, COL_DIM);
  gfx->fillRoundRect(272, 298, 252, 114, 8, COL_DIM);
  gfx->fillRoundRect(536, 298, 256, 114, 8, COL_DIM);
  ticks(16, 306, 236, 98, INK);
  ticks(280, 306, 236, 98, INK);
  ticks(544, 306, 240, 98, INK);

  int pct = 0;
  float phase = 0;
  const char *mn = moonName(&phase, &pct);
  gfx->setTextColor(SUN_C, COL_DIM);
  gfx->setTextSize(2);
  gfx->setCursor(24, 314);
  gfx->print("MOON");
  gfx->setTextColor(COL_TXT, COL_DIM);
  gfx->setCursor(24, 342);
  gfx->print(mn ? mn : "waiting");
  if (mn) {
    snprintf(line, sizeof(line), "%d%%", pct);
    gfx->setTextColor(INK, COL_DIM);
    gfx->setCursor(24, 368);
    gfx->print(line);
    drawMoonPhase(228, 360, 20, phase);
  }

  gfx->setTextColor(SUN_C, COL_DIM);
  gfx->setCursor(288, 314);
  gfx->print("TIDE");
  gfx->setTextColor(COL_TXT, COL_DIM);
  gfx->setCursor(288, 342);
  gfx->print(tideHigh[0] ? tideHigh : (tideKnown ? "tide waiting" : "no coast tide"));
  gfx->setCursor(288, 368);
  if (tideLow[0]) gfx->print(tideLow);

  gfx->setTextColor(SUN_C, COL_DIM);
  gfx->setCursor(552, 314);
  gfx->print("RAIN");
  int pop = 0;
  float wet = 0;
  int bars = nRain < 6 ? nRain : 6;
  for (int i = 0; i < bars; i++) {
    if ((int)rainPop[i] > pop) pop = (int)rainPop[i];
    wet += rainAmt[i];
    int bh = (int)(rainPop[i] * 36 / 100.0f);
    if (bh < 2 && rainPop[i] > 0) bh = 2;
    uint16_t c = rainPop[i] >= 60 ? COL_AMB : INK;
    gfx->fillRect(560 + i * 28, 392 - bh, 16, bh, c);
  }
  gfx->setTextColor(COL_TXT, COL_DIM);
  gfx->setCursor(552, 342);
  if (nRain) snprintf(line, sizeof(line), "6h  %d%%  %.2f in", pop, wet);
  else strlcpy(line, "waiting", sizeof(line));
  gfx->print(line);
  drawFoot();
}

void drawBack(const char *title) {
  chip(8, 8, 140, 36, "< BACK", false);
  chip(652, 8, 140, 36, "HOME", true);
  gfx->setTextColor(SUN_C, COL_BG);
  gfx->setTextSize(2);
  centre(title, 400, 16);
}

void drawRainPage() {
  page = PG_RAIN;
  gfx->fillScreen(COL_BG);
  drawBack("RAIN NEXT 12 HOURS");
  char line[48];
  float sum = 0;
  int peak = 0;
  for (int i = 0; i < nRain; i++) {
    sum += rainAmt[i];
    if ((int)rainPop[i] > peak) peak = (int)rainPop[i];
  }
  gfx->setTextColor(INK, COL_BG);
  gfx->setTextSize(2);
  if (nRain) snprintf(line, sizeof(line), "total %.2f in    peak %d%%    today %.2f in", sum, peak, rainToday);
  else strlcpy(line, "waiting for the place", sizeof(line));
  centre(line, 400, 56);
  int base = 360;
  time_t now;
  int hour0 = 0;
  if (liveClock(&now)) {
    struct tm t;
    localtime_r(&now, &t);
    hour0 = t.tm_hour;
  }
  for (int i = 0; i < 12; i++) {
    int x = 24 + i * 64;
    int pop = (i < nRain) ? (int)rainPop[i] : 0;
    int bh = pop * 180 / 100;
    uint16_t c = pop >= 70 ? COL_DN : (pop >= 40 ? COL_AMB : INK);
    if (bh > 0) gfx->fillRect(x, base - bh, 36, bh, c);
    gfx->drawFastHLine(x, base, 36, COL_MID);
    gfx->setTextColor(COL_TXT, COL_BG);
    gfx->setTextSize(2);
    int h = (hour0 + i) % 24;
    char hb[8];
    snprintf(hb, sizeof(hb), "%d", h % 12 == 0 ? 12 : h % 12);
    centre(hb, x + 18, base + 8);
    if (i < nRain && rainAmt[i] >= 0.01f) {
      snprintf(hb, sizeof(hb), "%.2f", rainAmt[i]);
      gfx->setTextColor(INK, COL_BG);
      centre(hb, x + 18, base - bh - 22);
    }
  }
}

const char *codeWord(int c) {
  if (c >= 95) return "thunder";
  if ((c >= 71 && c <= 77) || c == 85 || c == 86) return "snow";
  if ((c >= 51 && c <= 67) || (c >= 80 && c <= 82)) return "rain";
  if (c >= 45 && c <= 48) return "fog";
  if (c == 3) return "cloud";
  if (c == 2) return "partly cloudy";
  return "clear";
}

void dayTitle(int ahead, char *out, int n) {
  if (ahead == 0) {
    strlcpy(out, "Today", n);
    return;
  }
  time_t now;
  if (!liveClock(&now)) {
    strlcpy(out, "Day", n);
    return;
  }
  now += (time_t)ahead * 86400;
  struct tm t;
  localtime_r(&now, &t);
  strlcpy(out, WDAY[t.tm_wday], n);
}

void drawStation() {
  page = PG_STATION;
  gfx->fillScreen(COL_BG);
  drawBack("WIND AND AIR");
  gfx->fillRoundRect(12, 64, 380, 390, 8, COL_DIM);
  gfx->fillRoundRect(408, 64, 380, 390, 8, COL_DIM);
  int cx = 200, cy = 230;
  gfx->drawCircle(cx, cy, 78, INK);
  gfx->drawCircle(cx, cy, 4, SUN_C);
  float r = wxDir * 0.0174533f;
  gfx->drawLine(cx, cy, cx + (int)(sinf(r) * 68), cy - (int)(cosf(r) * 68), SUN_C);
  gfx->setTextColor(SUN_C, COL_DIM);
  gfx->setTextSize(2);
  centre(lookWord(wxDir), cx, 120);
  char line[32];
  gfx->setTextSize(3);
  gfx->setTextColor(COL_TXT, COL_DIM);
  snprintf(line, sizeof(line), "%d mph", wxWind);
  centre(line, cx, 330);
  gfx->setTextSize(2);
  snprintf(line, sizeof(line), "%d deg", wxDir);
  centre(line, cx, 370);
  float inHg = pressHpa > 0 ? pressHpa / 33.8639f : 0;
  gfx->setTextColor(SUN_C, COL_DIM);
  centre("AIR PRESSURE", 598, 84);
  gfx->setTextSize(3);
  gfx->setTextColor(COL_TXT, COL_DIM);
  if (inHg > 0) snprintf(line, sizeof(line), "%.2f in", inHg);
  else strlcpy(line, "-- in", sizeof(line));
  centre(line, 598, 140);
  gfx->setTextSize(2);
  if (pressLo > 0) snprintf(line, sizeof(line), "low  %.2f", pressLo / 33.8639f);
  else strlcpy(line, "low  --", sizeof(line));
  gfx->setCursor(430, 220);
  gfx->print(line);
  if (pressHi > 0) snprintf(line, sizeof(line), "high  %.2f", pressHi / 33.8639f);
  else strlcpy(line, "high  --", sizeof(line));
  gfx->setCursor(430, 258);
  gfx->print(line);
  if (pressTrend > 0) {
    gfx->setTextColor(COL_UP, COL_DIM);
    gfx->setCursor(430, 310);
    gfx->print("rising");
  } else if (pressTrend == 0 && pressHpa > 0) {
    gfx->setTextColor(COL_MID, COL_DIM);
    gfx->setCursor(430, 310);
    gfx->print("steady");
  }
  int gauge = 430;
  if (inHg > 0) {
    float t = (inHg - 28.0f) / 3.0f;
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    gauge = 430 + (int)(t * 280);
  }
  gfx->drawFastHLine(430, 380, 280, INK);
  gfx->fillCircle(gauge, 380, 8, SUN_C);
  gfx->setTextColor(COL_MID, COL_DIM);
  gfx->setCursor(430, 400);
  gfx->print("28");
  gfx->setCursor(680, 400);
  gfx->print("31");
}

void drawDays() {
  page = PG_DAYS;
  gfx->fillScreen(COL_BG);
  drawBack("7 DAY FORECAST");
  if (!nDays) {
    gfx->setTextColor(COL_MID, COL_BG);
    gfx->setTextSize(2);
    centre(wxOk ? "forecast waiting" : "waiting for the sky", 400, 220);
    return;
  }
  int rows = nDays > 7 ? 7 : nDays;
  for (int i = 0; i < rows; i++) {
    int y = 60 + i * 56;
    uint16_t row = (i % 2) ? COL_BG : COL_DIM;
    gfx->fillRect(12, y, 776, 52, row);
    char name[12];
    dayTitle(i, name, sizeof(name));
    gfx->setTextSize(2);
    gfx->setTextColor(SUN_C, row);
    gfx->setCursor(24, y + 16);
    gfx->print(name);
    gfx->setTextColor(COL_TXT, row);
    gfx->setCursor(180, y + 16);
    gfx->print(codeWord(dayCode[i]));
    char line[24];
    snprintf(line, sizeof(line), "%d  %d", showDeg(dayHi[i]), showDeg(dayLo[i]));
    gfx->setTextColor(COL_DN, row);
    gfx->setCursor(420, y + 16);
    gfx->print(line);
    snprintf(line, sizeof(line), "%d%%", dayPop[i]);
    gfx->setTextColor(INK, row);
    gfx->setCursor(620, y + 16);
    gfx->print(line);
  }
}

void drawHours() {
  page = PG_HOURS;
  gfx->fillScreen(COL_BG);
  drawBack("NEXT 24 HOURS");
  if (!nSlot) {
    gfx->setTextColor(COL_MID, COL_BG);
    gfx->setTextSize(2);
    centre("waiting for the sky", 400, 220);
    return;
  }
  int n = nSlot > 12 ? 12 : nSlot;
  bool anyRain = false;
  for (int i = 0; i < n; i++) {
    if (slotP[i] >= 10) anyRain = true;
  }
  time_t now = 0;
  bool clock = liveClock(&now);
  struct tm ti;
  if (clock) localtime_r(&now, &ti);
  int baseH = clock ? ti.tm_hour : -1;
  gfx->drawFastHLine(24, 400, 752, INK);
  int prevX = -1, prevY = -1;
  for (int i = 0; i < n; i++) {
    int x = 40 + i * (720 / (n > 1 ? n - 1 : 1));
    char lab[12];
    if (baseH >= 0) {
      int h = (baseH + i * 2) % 24;
      int hr = h % 12;
      if (!hr) hr = 12;
      snprintf(lab, sizeof(lab), "%d%s", hr, h >= 12 ? "p" : "a");
    } else
      strlcpy(lab, "--", sizeof(lab));
    gfx->setTextSize(2);
    gfx->setTextColor(COL_TXT, COL_BG);
    centre(lab, x, 64);
    snprintf(lab, sizeof(lab), "%d%s", showDeg(slotT[i]), useC ? "C" : "F");
    gfx->setTextColor(SUN_C, COL_BG);
    centre(lab, x, 96);
    snprintf(lab, sizeof(lab), "%d%%", slotP[i]);
    gfx->setTextColor(slotP[i] >= 40 ? COL_AMB : INK, COL_BG);
    centre(lab, x, 128);
    int gy = 400 - slotP[i] * 160 / 100;
    if (gy < 220) gy = 220;
    if (prevX >= 0) gfx->drawLine(prevX, prevY, x, gy, INK);
    gfx->fillCircle(x, gy, 3, slotP[i] >= 40 ? COL_AMB : INK);
    prevX = x;
    prevY = gy;
  }
  if (!anyRain) {
    gfx->setTextSize(2);
    gfx->setTextColor(COL_UP, COL_BG);
    centre("no rain", 400, 280);
  }
}

void drawWarn() {
  page = PG_WARN;
  gfx->fillScreen(COL_BG);
  drawBack("WARNINGS");
  if (!nWarn) {
    gfx->setTextColor(COL_UP, COL_BG);
    gfx->setTextSize(2);
    centre("ALL CLEAR", 400, 180);
  }
  int cards = nWarn > 3 ? 3 : nWarn;
  for (int i = 0; i < cards; i++) {
    int y = 60 + i * 120;
    gfx->fillRoundRect(16, y, 768, 110, 8, COL_DIM);
    gfx->setTextSize(2);
    gfx->setTextColor(strstr(warns[i].event, "Warning") ? COL_DN : SUN_C, COL_DIM);
    gfx->setCursor(28, y + 10);
    gfx->print(warns[i].event);
    gfx->setTextColor(COL_TXT, COL_DIM);
    gfx->setCursor(28, y + 38);
    char brief[42];
    strlcpy(brief, warns[i].headline, sizeof(brief));
    gfx->print(brief);
    char until[40];
    formatUntil(warns[i].ends, until, sizeof(until));
    gfx->setTextColor(COL_MID, COL_DIM);
    gfx->setCursor(28, y + 72);
    gfx->print(until);
  }
  if (auroraKp >= 0 && cards < 3) {
    const char *word = "quiet";
    if (auroraKp >= 7) word = "severe";
    else if (auroraKp >= 5) word = "active";
    else if (auroraKp >= 4) word = "unsettled";
    char line[40];
    snprintf(line, sizeof(line), "aurora  Kp %d  %s", auroraKp, word);
    gfx->setTextSize(2);
    gfx->setTextColor(auroraKp >= 5 ? COL_AMB : COL_MID, COL_BG);
    centre(line, 400, 430);
  }
}

void drawRadar() {
  page = PG_RADAR;
  gfx->fillScreen(COL_BG);
  chip(8, 8, 120, 36, "< BACK", false);
  chip(140, 8, 140, 36, "SAT", mapKind == MAP_SAT);
  chip(292, 8, 160, 36, "CLOUDS", mapKind == MAP_CLOUD);
  chip(464, 8, 160, 36, "STORM", mapKind == MAP_STORM);
  chip(652, 8, 140, 36, "HOME", true);
  gfx->fillRect(0, 52, PW, PH - 52, WELL);
  gfx->setTextColor(INK, WELL);
  gfx->setTextSize(2);
  centre(mapKind == MAP_STORM ? "loading storm picture" : (mapKind == MAP_CLOUD ? "loading cloud picture" : "loading satellite"),
         400, 220);
}

void paintMapNote() {
  gfx->fillRoundRect(16, 64, 360, 36, 6, WELL);
  gfx->drawRoundRect(16, 64, 360, 36, 6, INK);
  gfx->setTextColor(INK, WELL);
  gfx->setTextSize(2);
  gfx->setCursor(28, 74);
  gfx->print(placeSet ? placeName : "SET PLACE");
}

bool cloudBright(uint16_t p) {
  int r = (p >> 11) & 31;
  int g = (p >> 5) & 63;
  int b = p & 31;
  if (g < 36 || r < 16 || b < 16) return false;
  int rg = r * 2 - g;
  if (rg < 0) rg = -rg;
  int gb = g - b * 2;
  if (gb < 0) gb = -gb;
  return rg < 16 && gb < 18;
}

bool cloudOverPlot(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
  if (!gfx || !bitmap || y >= jpgClipY) return 1;
  for (uint16_t j = 0; j < h; j++) {
    int yy = y + j;
    if (yy < 0 || yy >= jpgClipY) continue;
    uint16_t i = 0;
    while (i < w) {
      while (i < w && (x + i < 0 || x + i >= PW || !cloudBright(bitmap[j * w + i]))) i++;
      uint16_t start = i;
      while (i < w && x + i >= 0 && x + i < PW && cloudBright(bitmap[j * w + i])) i++;
      if (i > start) gfx->draw16bitRGBBitmap(x + start, yy, bitmap + j * w + start, i - start, 1);
    }
  }
  return 1;
}

bool jpgPlot(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
  if (!gfx || !bitmap || y >= jpgClipY) return 1;
  int hh = h;
  if (y + hh > jpgClipY) hh = jpgClipY - y;
  if (hh <= 0 || x >= PW) return 1;
  gfx->draw16bitRGBBitmap(x, y, bitmap, w, hh);
  return 1;
}

bool radarColor(uint16_t p) {
  int r = (p >> 11) & 31;
  int g = (p >> 5) & 63;
  int b = p & 31;
  if (r + g + b < 16) return false;
  if (r > 28 && g > 56 && b > 28) return false;
  int rg = r * 2 - g;
  if (rg < 0) rg = -rg;
  int gb = g - b * 2;
  if (gb < 0) gb = -gb;
  if (rg < 8 && gb < 10 && r < 22) return false;
  return true;
}

bool radarPlot(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
  if (!gfx || !bitmap || y >= jpgClipY) return 1;
  for (uint16_t j = 0; j < h; j++) {
    int yy = y + j;
    if (yy < 0 || yy >= jpgClipY) continue;
    int run = -1;
    for (uint16_t i = 0; i < w; i++) {
      bool hot = (x + (int)i) >= 0 && (x + (int)i) < PW && radarColor(bitmap[j * w + i]);
      if (hot && run < 0) run = i;
      if (!hot && run >= 0) {
        gfx->draw16bitRGBBitmap(x + run, yy, bitmap + j * w + run, i - run, 1);
        run = -1;
      }
    }
    if (run >= 0) gfx->draw16bitRGBBitmap(x + run, yy, bitmap + j * w + run, w - run, 1);
  }
  return 1;
}

bool issJpgPlot(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
  if (!issJpg || !bitmap) return 0;
  for (uint16_t j = 0; j < h; j++) {
    int yy = y + j;
    if (yy < 0 || yy >= ISS_JPG_H) continue;
    for (uint16_t i = 0; i < w; i++) {
      int xx = x + i;
      if (xx < 0 || xx >= ISS_SRC_W) continue;
      issJpg[yy * ISS_SRC_W + xx] = bitmap[j * w + i];
    }
  }
  return 1;
}

void drawSky() {
  page = PG_SKY;
  gfx->fillScreen(COL_BG);
  drawBack("SKY");
  const char *sub[4];
  char moon[24], iss[24], suns[24], rock[24];
  int pct = 0;
  float phase = 0;
  const char *mn = moonName(&phase, &pct);
  if (mn) snprintf(moon, sizeof(moon), "%s  %d%%", mn, pct);
  else strlcpy(moon, "waiting", sizeof(moon));
  if (wxUp[0]) snprintf(suns, sizeof(suns), "up %s", wxUp);
  else strlcpy(suns, "sun path", sizeof(suns));
  if (nLaunch && launches[0].rocket[0]) strlcpy(rock, launches[0].rocket, sizeof(rock));
  else strlcpy(rock, "upcoming", sizeof(rock));
  if (issVisNow) snprintf(iss, sizeof(iss), "LOOK %s", lookWord(issAz));
  else if (issWhen[0]) strlcpy(iss, issWhen, sizeof(iss));
  else strlcpy(iss, "orbit", sizeof(iss));
  sub[0] = suns;
  sub[1] = rock;
  sub[2] = moon;
  sub[3] = iss;
  const char *title[] = {"CLOCK", "LAUNCH", "NIGHT", "ISS"};
  for (int i = 0; i < 4; i++) {
    int x = 20 + (i % 2) * 390;
    int y = 64 + (i / 2) * 200;
    gfx->fillRoundRect(x, y, 370, 184, 8, COL_DIM);
    ticks(x + 10, y + 10, 350, 164, INK);
    gfx->setTextColor(INK, COL_DIM);
    gfx->setTextSize(3);
    centre(title[i], x + 185, y + 58);
    gfx->setTextColor(COL_TXT, COL_DIM);
    gfx->setTextSize(2);
    if (i == 2 && mn) {
      drawMoonPhase(x + 64, y + 100, 32, phase);
      gfx->setTextColor(COL_TXT, COL_DIM);
      gfx->setTextSize(2);
      gfx->setCursor(x + 112, y + 78);
      gfx->print(mn);
      char lit[16];
      snprintf(lit, sizeof(lit), "%d%% lit", pct);
      gfx->setTextColor(INK, COL_DIM);
      gfx->setCursor(x + 112, y + 106);
      gfx->print(lit);
    } else
      centre(sub[i], x + 185, y + 108);
  }
}

void drawClockIcon(int cx, int cy, const char *w) {
  if (!w || !w[0]) return;
  if (!strcmp(w, "THUNDER") || !strcmp(w, "HAIL")) {
    gfx->fillCircle(cx - 10, cy - 4, 16, COL_MID);
    gfx->fillCircle(cx + 12, cy - 2, 14, COL_MID);
    gfx->fillTriangle(cx - 4, cy + 2, cx + 10, cy + 2, cx - 2, cy + 22, SUN_C);
    gfx->fillTriangle(cx + 2, cy + 12, cx + 14, cy + 12, cx + 4, cy + 32, SUN_C);
  } else if (!strcmp(w, "RAIN") || !strcmp(w, "SNOW")) {
    gfx->fillCircle(cx - 12, cy - 6, 14, COL_MID);
    gfx->fillCircle(cx + 10, cy - 4, 16, COL_MID);
    uint16_t drop = !strcmp(w, "SNOW") ? COL_TXT : INK;
    for (int i = -16; i <= 16; i += 8) gfx->fillCircle(cx + i, cy + 20, 3, drop);
  } else if (!strcmp(w, "CLOUDY")) {
    gfx->fillCircle(cx - 14, cy, 16, COL_MID);
    gfx->fillCircle(cx + 4, cy - 6, 18, COL_MID);
    gfx->fillCircle(cx + 16, cy + 2, 14, COL_MID);
  } else if (!strcmp(w, "PARTLY CLOUDY")) {
    gfx->fillCircle(cx - 10, cy - 8, 12, SUN_C);
    gfx->fillCircle(cx + 6, cy, 14, COL_MID);
    gfx->fillCircle(cx + 16, cy + 4, 11, COL_MID);
  } else if (!strcmp(w, "WIND")) {
    gfx->drawLine(cx - 22, cy - 8, cx + 22, cy - 8, INK);
    gfx->drawLine(cx - 18, cy, cx + 26, cy, INK);
    gfx->drawLine(cx - 22, cy + 8, cx + 18, cy + 8, INK);
  } else if (!strcmp(w, "NIGHT")) {
    int pct = 0;
    float phase = 0;
    if (moonName(&phase, &pct)) drawMoonPhase(cx, cy, 16, phase);
  } else {
    gfx->fillCircle(cx, cy, 16, SUN_C);
    for (int a = 0; a < 8; a++) {
      float r = a * 0.785398f;
      gfx->drawLine(cx + (int)(sinf(r) * 22), cy - (int)(cosf(r) * 22), cx + (int)(sinf(r) * 30),
                    cy - (int)(cosf(r) * 30), SUN_C);
    }
  }
}

void drawClockPage() {
  page = PG_CLOCK;
  gfx->fillScreen(COL_BG);
  drawBack("CLOCK");
  if (!liveClock(nullptr) || !placeTz[0]) {
    gfx->setTextColor(COL_MID, COL_BG);
    gfx->setTextSize(2);
    centre(placeTz[0] ? "waiting for live clock" : "place clock", 400, 200);
    return;
  }
  time_t now = time(nullptr);
  struct tm ti;
  localtime_r(&now, &ti);
  char line[48];
  int hr = ti.tm_hour > 12 ? ti.tm_hour - 12 : (ti.tm_hour ? ti.tm_hour : 12);
  snprintf(line, sizeof(line), "%d:%02d %s", hr, ti.tm_min, ti.tm_hour >= 12 ? "PM" : "AM");
  gfx->setTextColor(SUN_C, COL_BG);
  gfx->setTextSize(4);
  centre(line, 400, 80);
  gfx->fillRoundRect(40, 160, 340, 80, 6, COL_DIM);
  gfx->fillRoundRect(420, 160, 340, 80, 6, COL_DIM);
  gfx->setTextColor(COL_MID, COL_DIM);
  gfx->setTextSize(2);
  centre("SUN UP", 210, 172);
  centre("SUN DOWN", 590, 172);
  gfx->setTextColor(SUN_C, COL_DIM);
  gfx->setTextSize(3);
  centre(wxUp[0] ? wxUp : "--:--", 210, 204);
  centre(wxDown[0] ? wxDown : "--:--", 590, 204);
  bool night = ti.tm_hour >= 18 || ti.tm_hour < 6;
  const char *word = wxOk ? skyWord() : "SKY";
  if (night && wxOk && !strcmp(word, "SUN")) word = "NIGHT";
  drawClockIcon(90, 278, word);
  gfx->setTextColor(COL_HOT, COL_BG);
  gfx->setTextSize(2);
  centre(wxOk ? clockFaceWord() : "waiting", 400, 250);
  if (wxOk) snprintf(line, sizeof(line), "%d%s", showDeg(wxTemp), useC ? "C" : "F");
  else strlcpy(line, "--", sizeof(line));
  gfx->setTextColor(SUN_C, COL_BG);
  gfx->setTextSize(4);
  gfx->setCursor(620, 248);
  gfx->print(line);
  int pop = (nRain > 0) ? (int)rainPop[0] : 0;
  if (wxOk)
    snprintf(line, sizeof(line), "high %d  low %d   %s", showDeg(fcHigh), showDeg(fcLow), pop >= 40 ? "umbrella" : "dry");
  else
    strlcpy(line, "high --  low --", sizeof(line));
  gfx->setTextColor(COL_TXT, COL_BG);
  gfx->setTextSize(2);
  centre(line, 400, 318);
  if (wxOk) snprintf(line, sizeof(line), "AIR %d%s  feels %d  hum %d%%", showDeg(wxTemp), useC ? "C" : "F", showDeg(wxFeel), wxRh);
  else strlcpy(line, "AIR --", sizeof(line));
  centre(line, 400, 354);
  if (wxOk) {
    int kt = (int)(wxWind / 1.15078f + 0.5f);
    snprintf(line, sizeof(line), "wind %d deg  %d kt  %d mph", wxDir, kt, wxWind);
  } else
    strlcpy(line, "wind --", sizeof(line));
  gfx->setTextColor(COL_MID, COL_BG);
  centre(line, 400, 390);
}

void drawLaunchCard(int y, const Launch &L) {
  gfx->fillRoundRect(16, y, PW - 32, 80, 6, COL_DIM);
  gfx->setTextColor(SUN_C, COL_DIM);
  gfx->setTextSize(2);
  char line[56];
  snprintf(line, sizeof(line), "WHEN   %s", L.day[0] ? L.day : "----");
  gfx->setCursor(28, y + 8);
  gfx->print(line);
  gfx->setTextColor(COL_HOT, COL_DIM);
  snprintf(line, sizeof(line), "WHERE  %s", L.pad[0] ? L.pad : "----");
  gfx->setCursor(28, y + 34);
  gfx->print(line);
  gfx->setTextColor(COL_TXT, COL_DIM);
  snprintf(line, sizeof(line), "TIME   %s    %s", L.when[0] ? L.when : "--:--", L.agency);
  gfx->setCursor(28, y + 58);
  gfx->print(line);
}

void drawLaunchPage() {
  page = PG_LAUNCH;
  gfx->fillScreen(COL_BG);
  drawBack("LAUNCHES");
  if (!wifiOk || !launchOk) {
    gfx->setTextColor(COL_MID, COL_BG);
    gfx->setTextSize(2);
    centre(liveClock(nullptr) ? "waiting for live launches" : "waiting for live clock", 400, 220);
    return;
  }
  if (!nLaunch) {
    gfx->setTextColor(COL_MID, COL_BG);
    gfx->setTextSize(2);
    centre("no upcoming in the live list", 400, 220);
    return;
  }
  int y = 56;
  for (int i = 0; i < nLaunch && i < 4; i++) {
    drawLaunchCard(y, launches[i]);
    y += 86;
  }
}

void drawNight() {
  page = PG_NIGHT;
  gfx->fillScreen(COL_BG);
  drawBack("NIGHT SKY");
  if (!lastAstro || millis() - lastAstro > 1800000UL) computeAstro();
  int pct = 0;
  float phase = 0;
  const char *mn = moonName(&phase, &pct);
  char line[48];
  gfx->setTextSize(2);
  if (mn) {
    snprintf(line, sizeof(line), "moon %s   %d%%    sky %s", mn, pct, wxOk ? skyWord() : "--");
    gfx->setTextColor(INK, COL_BG);
    int16_t x1, y1;
    uint16_t tw, th;
    gfx->getTextBounds(line, 0, 0, &x1, &y1, &tw, &th);
    int tx = 400 - (int)tw / 2;
    if (tx > 48) drawMoonPhase(tx - 24, 62, 14, phase);
    centre(line, 400, 52);
  }
  int nPlanet = 0, nOther = 0;
  for (int i = 0; i < nSkyVis; i++) {
    if (!strcmp(skyVis[i].note, "planet")) nPlanet++;
    else nOther++;
  }
  if (!nSkyVis) {
    gfx->setTextColor(COL_MID, COL_BG);
    centre(liveClock(nullptr) ? "none up tonight" : "waiting for the clock", 400, 220);
    return;
  }
  int show[6];
  int nShow = 0;
  int seenP = 0;
  for (int i = 0; i < nSkyVis && nShow < 6; i++) {
    bool planet = !strcmp(skyVis[i].note, "planet");
    if (nightPage == 0) {
      if (planet) show[nShow++] = i;
    } else if (planet) {
      seenP++;
      if (seenP > 6) show[nShow++] = i;
    } else
      show[nShow++] = i;
  }
  int y = 84;
  for (int k = 0; k < nShow; k++) {
    int i = show[k];
    uint16_t row = (i % 2) ? COL_BG : COL_DIM;
    gfx->fillRect(12, y, 776, 58, row);
    gfx->setTextColor(SUN_C, row);
    gfx->setCursor(24, y + 8);
    gfx->print(skyVis[i].name);
    gfx->setTextColor(INK, row);
    gfx->setCursor(180, y + 8);
    gfx->print(skyVis[i].look);
    gfx->setTextColor(COL_TXT, row);
    snprintf(line, sizeof(line), "%s  %d deg", skyVis[i].note, skyVis[i].el);
    gfx->setCursor(250, y + 8);
    gfx->print(line);
    gfx->setTextColor(COL_MID, row);
    gfx->setCursor(24, y + 32);
    gfx->print(skyVis[i].hours);
    if (mn && !strcmp(skyVis[i].name, "Moon")) drawMoonPhase(748, y + 29, 16, phase);
    y += 62;
  }
  if (nOther || nPlanet > 6) chip(470, 8, 160, 36, nightPage ? "PAGE 1" : "PAGE 2", true);
}

int issMapX(float lon) { return (int)((lon + 180.0f) * (PW / 360.0f)); }
int issMapY(float lat) { return ISS_Y0 + (int)((72.0f - lat) * ((float)ISS_H / 134.0f)); }

void updateSunPos() {
  time_t t = 0;
  if (!liveClock(&t)) {
    sunOk = false;
    return;
  }
  double jd = unixToJd(t);
  double T = (jd - 2451545.0) / 36525.0;
  double L = 280.460 + 36000.770 * T;
  double M = 357.528 + 35999.050 * T;
  wrap360(L);
  wrap360(M);
  double Mr = M * 0.01745329252;
  double lam = L + 1.915 * sin(Mr) + 0.020 * sin(2 * Mr);
  double eps = 23.439 - 0.013 * T;
  double lr = lam * 0.01745329252, er = eps * 0.01745329252;
  double ra = atan2(cos(er) * sin(lr), cos(lr));
  double dec = asin(sin(er) * sin(lr));
  double gmst = 280.46061837 + 360.98564736629 * (jd - 2451545.0);
  wrap360(gmst);
  double raDeg = ra * 57.295779513;
  if (raDeg < 0) raDeg += 360.0;
  double gha = gmst - raDeg;
  wrap360(gha);
  sunLon = (float)(-gha);
  if (sunLon < -180.0f) sunLon += 360.0f;
  if (sunLon > 180.0f) sunLon -= 360.0f;
  sunLat = (float)(dec * 57.295779513);
  sunOk = true;
}

void drawIssSun() {
  if (!sunOk) return;
  int x = issMapX(sunLon);
  int y = issMapY(sunLat);
  if (x < -9 || x >= PW + 9 || y < ISS_Y0 - 9 || y >= ISS_Y0 + ISS_H + 9) return;
  gfx->fillCircle(x, y, 9, RGB565(255, 196, 48));
  gfx->fillCircle(x, y, 4, SUN_C);
}

void drawCoast(const int16_t *p, uint16_t edge) {
  int px = -999, py = -999;
  for (int i = 0; p[i] != -127; i += 2) {
    int x = issMapX((float)p[i]);
    int y = issMapY((float)p[i + 1]);
    if (px > -900) {
      gfx->drawLine(px, py, x, y, edge);
      gfx->drawLine(px, py + 1, x, y + 1, edge);
    }
    px = x;
    py = y;
  }
}

bool ensureIssShade() {
  if (issShade) return true;
  issShade = (uint16_t *)ps_malloc((size_t)PW * ISS_H * 2);
  return issShade != nullptr;
}

void blitIssSunNight() {
  if (!sunOk || !issMapOk || !issFb || !ensureIssShade()) return;
  const float deg = 0.01745329252f;
  const float sls = sinf(sunLat * deg);
  const float cls = cosf(sunLat * deg);
  static float slat[200];
  static float clat[200];
  static float cld[800];
  static bool latOk = false;
  if (!latOk) {
    for (int y = 0; y < ISS_H; y++) {
      float lat = (72.0f - (float)y * 134.0f / (float)ISS_H) * deg;
      slat[y] = sinf(lat);
      clat[y] = cosf(lat);
    }
    latOk = true;
  }
  for (int x = 0; x < PW; x++) {
    float lon = (float)x * 360.0f / (float)PW - 180.0f;
    cld[x] = cls * cosf((lon - sunLon) * deg);
  }
  for (int y = 0; y < ISS_H; y++) {
    float sy = slat[y], cy = clat[y];
    uint16_t *src = issFb + y * PW;
    uint16_t *dst = issShade + y * PW;
    for (int x = 0; x < PW; x++) {
      float c = sy * sls + cy * cld[x];
      uint16_t pix = src[x];
      int scale;
      if (c >= 0.20f)
        scale = 256;
      else if (c >= 0.0f)
        scale = 168 + (int)(c * 440.0f);
      else if (c >= -0.18f)
        scale = 140 + (int)((c + 0.18f) * 155.0f);
      else
        scale = 140;
      int r = ((pix >> 11) & 31) * scale >> 8;
      int g = ((pix >> 5) & 63) * scale >> 8;
      int b = (pix & 31) * scale >> 8;
      dst[x] = (uint16_t)((r << 11) | (g << 5) | b);
    }
    if ((y & 31) == 31) delay(0);
  }
  for (int y = 0; y < ISS_H; y += 20) {
    int rows = 20;
    if (y + rows > ISS_H) rows = ISS_H - y;
    gfx->draw16bitRGBBitmap(0, ISS_Y0 + y, issShade + y * PW, PW, rows);
    delay(0);
  }
}

void drawEarth() {
  if (issMapOk && issFb) {
    for (int y = 0; y < ISS_H; y += 20) {
      int rows = 20;
      if (y + rows > ISS_H) rows = ISS_H - y;
      gfx->draw16bitRGBBitmap(0, ISS_Y0 + y, issFb + y * PW, PW, rows);
      delay(0);
    }
    return;
  }
  gfx->fillRect(0, ISS_Y0, PW, ISS_H, RGB565(12, 38, 72));
  static const int16_t na[] = {-168, 66, -155, 61, -141, 70, -130, 69, -125, 72, -105, 68, -95, 72, -88, 68, -80, 73, -70, 62, -66, 58, -56, 54, -68, 46, -75, 35, -81, 25, -90, 29, -97, 26, -105, 22, -111, 24, -117, 33, -125, 40, -125, 49, -135, 55, -154, 59, -168, 62, -127, 0};
  static const int16_t sa[] = {-81, 12, -70, 12, -62, 10, -51, 8, -35, 0, -35, -10, -40, -23, -48, -35, -55, -45, -62, -50, -71, -55, -75, -40, -77, -22, -80, 2, -127, 0};
  static const int16_t af[] = {-17, 32, -9, 37, 10, 37, 32, 31, 43, 11, 51, 12, 48, 0, 40, -26, 32, -30, 18, -35, 12, -18, 9, 4, -5, 5, -17, 16, -127, 0};
  static const int16_t eu[] = {-10, 36, -9, 44, -6, 51, 2, 59, 12, 61, 24, 70, 30, 60, 40, 48, 29, 41, 18, 40, 10, 38, -127, 0};
  static const int16_t as[] = {28, 36, 44, 40, 60, 50, 75, 55, 90, 62, 110, 68, 135, 71, 170, 66, 160, 50, 145, 42, 130, 38, 120, 30, 100, 20, 88, 22, 78, 28, 68, 25, 50, 25, 40, 36, -127, 0};
  static const int16_t au[] = {114, -22, 129, -14, 146, -16, 153, -28, 146, -36, 136, -35, 116, -34, -127, 0};
  static const int16_t gr[] = {-62, 76, -45, 83, -20, 80, -22, 70, -44, 60, -53, 67, -127, 0};
  static const int16_t uk[] = {-8, 58, 2, 58, 1, 50, -6, 50, -127, 0};
  static const int16_t jp[] = {131, 45, 145, 44, 142, 35, 131, 33, -127, 0};
  static const int16_t md[] = {43, -12, 50, -16, 47, -25, 43, -25, -127, 0};
  static const int16_t idn[] = {95, 6, 119, 5, 131, -3, 115, -8, 105, -6, -127, 0};
  const int16_t *sets[] = {na, sa, af, eu, as, au, gr, uk, jp, md, idn};
  uint16_t edge = RGB565(210, 210, 180);
  for (int s = 0; s < 11; s++) drawCoast(sets[s], edge);
}

void drawIss() {
  page = PG_ISS;
  gfx->fillScreen(COL_BG);
  drawBack("ISS");
  time_t clockNow = 0;
  bool clockOk = liveClock(&clockNow);
  if (clockOk) {
    struct tm ti;
    localtime_r(&clockNow, &ti);
    int hr = ti.tm_hour % 12;
    if (!hr) hr = 12;
    char clk[16];
    snprintf(clk, sizeof(clk), "%d:%02d %s", hr, ti.tm_min, ti.tm_hour >= 12 ? "PM" : "AM");
    gfx->setTextColor(SUN_C, COL_BG);
    gfx->setTextSize(3);
    centre(clk, 400, 8);
  }
  if (!issMapOk) fetchIssMap();
  drawEarth();
  updateSunPos();
  blitIssSunNight();
  const uint16_t path = RGB565(0, 56, 168);
  int px = -999, py = -999;
  for (int i = 0; i < nIssTrack; i++) {
    int x = issMapX(issTrackLon[i]);
    int y = issMapY(issTrackLat[i]);
    if (x >= 0 && x < PW && y >= ISS_Y0 && y < ISS_Y0 + ISS_H) {
      gfx->fillCircle(x, y, 3, path);
      if (px > -900 && abs(x - px) < 200) {
        gfx->drawLine(px, py - 1, x, y - 1, path);
        gfx->drawLine(px, py, x, y, path);
        gfx->drawLine(px, py + 1, x, y + 1, path);
      }
    }
    px = x;
    py = y;
  }
  if (issOk || issTle.ok) {
    int x = issMapX(issLon);
    int y = issMapY(issLat);
    if (x >= 0 && x < PW && y >= ISS_Y0 && y < ISS_Y0 + ISS_H) {
      gfx->fillRect(x - 40, y - 6, 28, 10, SUN_C);
      gfx->fillRect(x + 12, y - 6, 28, 10, SUN_C);
      gfx->fillRect(x - 11, y - 10, 22, 20, COL_TXT);
      gfx->drawPixel(x, y - 12, COL_HOT);
    }
    if (placeSet) gfx->fillCircle(issMapX(placeLon), issMapY(placeLat), 4, RGB565(255, 70, 180));
  }
  drawIssSun();
  time_t now = 0;
  bool soon = false;
  clockOk = liveClock(&now);
  if (clockOk) soon = issNextPass && issNextPass > now && (issNextPass - now) <= 20 * 60;
  if (issVisNow || soon) {
    gfx->fillRoundRect(16, 72, 300, 36, 4, COL_HOT);
    gfx->setTextColor(COL_BG, COL_HOT);
    gfx->setTextSize(2);
    char look[32];
    if (issVisNow)
      snprintf(look, sizeof(look), "LOOK %s  %ddeg", issLook[0] ? issLook : lookWord(issAz), issEl);
    else {
      int sec = (int)(issNextPass - now);
      const char *from = nIssPass ? issPasses[0].from : issLook;
      snprintf(look, sizeof(look), "LOOK %s  %d:%02d", from[0] ? from : "?", sec / 60, sec % 60);
    }
    centre(look, 166, 80);
  }
  gfx->fillRect(0, ISS_Y0 + ISS_H, PW, PH - (ISS_Y0 + ISS_H), COL_BG);
  gfx->setTextColor(SUN_C, COL_BG);
  gfx->setTextSize(2);
  gfx->setCursor(24, 276);
  gfx->print("VISIBLE PASSES");
  if (!issTle.ok) {
    gfx->setTextColor(COL_MID, COL_BG);
    gfx->setCursor(24, 320);
    gfx->print("waiting for live ISS data");
  } else if (!clockOk) {
    gfx->setTextColor(COL_MID, COL_BG);
    gfx->setCursor(24, 320);
    gfx->print("waiting for live clock");
  } else if (!lastPassAt) {
    gfx->setTextColor(COL_MID, COL_BG);
    gfx->setCursor(24, 320);
    gfx->print("checking passes");
  } else if (!nIssPass) {
    gfx->setTextColor(COL_MID, COL_BG);
    gfx->setCursor(24, 320);
    gfx->print("none in the next 10 days");
  } else {
    int y = 312;
    for (int i = 0; i < nIssPass && y < 460; i++) {
      gfx->setTextColor(COL_TXT, COL_BG);
      gfx->setCursor(24, y);
      gfx->print(issPasses[i].line);
      y += 28;
    }
  }
}

void drawSet() {
  page = PG_SET;
  gfx->fillScreen(COL_BG);
  drawBack("SETUP");
  gfx->setTextColor(COL_MID, COL_BG);
  gfx->setTextSize(2);
  gfx->setCursor(40, 90);
  gfx->print("LOOK");
  chip(160, 78, 180, 44, "DARK", !dayMode);
  chip(360, 78, 180, 44, "LIGHT", dayMode);
  gfx->setCursor(40, 160);
  gfx->print("LAMP");
  chip(160, 148, 140, 44, "DIM", blLevel == 0);
  chip(320, 148, 140, 44, "MID", blLevel == 1);
  chip(480, 148, 180, 44, "BRIGHT", blLevel == 2);
  chip(160, 240, 500, 52, "FACTORY RESET", false);
  gfx->setTextColor(COL_MID, COL_BG);
  centre("clears this desk only", 400, 310);
  chip(160, 360, 500, 52, "WIFI", false);
  gfx->setTextColor(COL_MID, COL_BG);
  gfx->setCursor(40, 436);
  gfx->print("TEMP");
  chip(160, 424, 180, 44, "F", !useC);
  chip(360, 424, 180, 44, "C", useC);
}

void drawKey(int x, int y, int w, int h, const char *t) {
  gfx->fillRoundRect(x, y, w, h, 4, COL_DIM);
  gfx->setTextColor(COL_TXT, COL_DIM);
  gfx->setTextSize(2);
  centre(t, x + w / 2, y + h / 2 - 8);
}

void drawRow(const char *s, int y) {
  int n = (int)strlen(s);
  for (int i = 0; i < n; i++) {
    char t[2] = {s[i], 0};
    drawKey(i * KW + 2, y, KW - 6, 40, t);
  }
}

void drawKeys() {
  bool sh = keyShift && *keyShift;
  bool sy = keySym && *keySym;
  const char *r0 = "1234567890";
  const char *r1 = sy ? "!@#$%^&*()" : (sh ? "QWERTYUIOP" : "qwertyuiop");
  const char *r2 = sy ? "-_=+[]{};'" : (sh ? "ASDFGHJKL" : "asdfghjkl");
  const char *r3 = sy ? ",.<>/?\\|~`" : (sh ? "ZXCVBNM.@-" : "zxcvbnm.@-");
  drawRow(r0, 200);
  drawRow(r1, 248);
  drawRow(r2, 296);
  drawRow(r3, 344);
  drawKey(722, 296, 70, 40, "DEL");
  chip(8, 392, 120, 36, sh ? "ABC" : "abc", sh);
  chip(140, 392, 400, 36, "SPACE", false);
  chip(552, 392, 120, 36, sy ? "ABC" : "!#", sy);
}

void keyAppend(char ch) {
  if (!keyDest || keyCap < 2) return;
  int L = (int)strlen(keyDest);
  if (L < keyCap - 1) {
    keyDest[L] = ch;
    keyDest[L + 1] = 0;
  }
}

void keyDel() {
  if (!keyDest) return;
  int L = (int)strlen(keyDest);
  if (L) keyDest[L - 1] = 0;
}

bool hitRow(int x, int y, const char *s, int y0) {
  if (y < y0 || y >= y0 + 44) return false;
  int i = x / KW;
  int n = (int)strlen(s);
  if (i < 0 || i >= n) return false;
  keyAppend(s[i]);
  if (keyShift && *keyShift && s[i] >= 'A' && s[i] <= 'Z') *keyShift = false;
  return true;
}

bool hitKeys(int x, int y) {
  bool sh = keyShift && *keyShift;
  bool sy = keySym && *keySym;
  const char *r0 = "1234567890";
  const char *r1 = sy ? "!@#$%^&*()" : (sh ? "QWERTYUIOP" : "qwertyuiop");
  const char *r2 = sy ? "-_=+[]{};'" : (sh ? "ASDFGHJKL" : "asdfghjkl");
  const char *r3 = sy ? ",.<>/?\\|~`" : (sh ? "ZXCVBNM.@-" : "zxcvbnm.@-");
  if (hit(x, y, 722, 296, 792, 336)) {
    keyDel();
    return true;
  }
  if (hit(x, y, 8, 392, 128, 428)) {
    if (keyShift) *keyShift = !*keyShift;
    if (keySym) *keySym = false;
    return true;
  }
  if (hit(x, y, 140, 392, 540, 428)) {
    keyAppend(' ');
    return true;
  }
  if (hit(x, y, 552, 392, 672, 428)) {
    if (keySym) *keySym = !*keySym;
    if (keyShift) *keyShift = false;
    return true;
  }
  return hitRow(x, y, r0, 200) || hitRow(x, y, r1, 248) || hitRow(x, y, r2, 296) || hitRow(x, y, r3, 344);
}

void drawWifiField(int focus, const char *raw, const char *empty, int y) {
  bool on = wifiFocus == focus;
  gfx->fillRoundRect(120, y, 664, 40, 6, on ? COL_HOT : COL_DIM);
  gfx->setTextColor(on ? COL_BG : COL_TXT, on ? COL_HOT : COL_DIM);
  gfx->setTextSize(2);
  char shown[40];
  if (!raw[0]) strlcpy(shown, empty, sizeof(shown));
  else {
    int L = (int)strlen(raw);
    const char *p = raw;
    if (L > 36) p = raw + (L - 36);
    strlcpy(shown, p, sizeof(shown));
  }
  gfx->setCursor(132, y + 10);
  gfx->print(shown);
}

void bindWifiKeys() {
  keyDest = wifiFocus ? wifiPass : wifiSsid;
  keyCap = wifiFocus ? (int)sizeof(wifiPass) : (int)sizeof(wifiSsid);
  keyShift = &wifiShift;
  keySym = &wifiSym;
}

void drawWifi() {
  page = PG_WIFI;
  gfx->fillScreen(COL_BG);
  chip(8, 8, 140, 36, "< BACK", false);
  gfx->setTextColor(SUN_C, COL_BG);
  gfx->setTextSize(2);
  centre("JOIN WIFI", 400, 16);
  chip(660, 8, 132, 36, "SCAN", wifiScanBusy);
  chip(8, 52, 56, 36, "<", false);
  chip(736, 52, 56, 36, ">", false);
  for (int i = 0; i < 2; i++) {
    int idx = wifiScanTop + i;
    if (idx >= nWifiScan) break;
    char shown[18];
    strlcpy(shown, wifiScan[idx], sizeof(shown));
    chip(72 + i * 328, 52, 320, 36, shown, !strcmp(wifiScan[idx], wifiSsid));
  }
  if (wifiMsg[0] && nWifiScan == 0) {
    gfx->setTextColor(COL_HOT, COL_BG);
    centre(wifiMsg, 400, 60);
  }
  gfx->setTextColor(COL_MID, COL_BG);
  gfx->setCursor(16, 110);
  gfx->print("SSID");
  drawWifiField(0, wifiSsid, "tap net or type", 100);
  gfx->setCursor(16, 158);
  gfx->print("PASS");
  drawWifiField(1, wifiPass, "type password", 148);
  bindWifiKeys();
  drawKeys();
  chip(8, 436, 252, 36, "SAVE", true);
  chip(274, 436, 252, 36, "RESET WIFI", false);
  chip(540, 436, 252, 36, "FACTORY", false);
}

void drawPlace() {
  page = PG_PLACE;
  gfx->fillScreen(COL_BG);
  chip(8, 8, 140, 36, "< BACK", false);
  gfx->setTextColor(SUN_C, COL_BG);
  gfx->setTextSize(2);
  centre("PLACE", 400, 16);
  chip(660, 8, 132, 36, "SEARCH", false);
  gfx->fillRoundRect(16, 52, 768, 40, 6, COL_DIM);
  gfx->setTextColor(COL_TXT, COL_DIM);
  gfx->setTextSize(2);
  gfx->setCursor(28, 62);
  gfx->print(placeQuery[0] ? placeQuery : "city or ZIP");
  chip(8, 100, 56, 88, "<", false);
  chip(736, 100, 56, 88, ">", false);
  if (!nHits) {
    gfx->setTextColor(COL_MID, COL_BG);
    centre(placeMsg[0] ? placeMsg : "US places only", 400, 130);
  } else {
    for (int i = 0; i < 2; i++) {
      int idx = placeTop + i;
      if (idx >= nHits) break;
      chip(72, 100 + i * 48, 656, 44, hits[idx].label, false);
    }
  }
  keyDest = placeQuery;
  keyCap = (int)sizeof(placeQuery);
  keyShift = &placeShift;
  keySym = &placeSym;
  drawKeys();
}

int httpWaitMs = 12000;

void noteHttp(HTTPClient &http) {
  http.setTimeout(15000);
  http.setConnectTimeout(4000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setReuse(false);
  http.setUserAgent("SkyWeather/1.0 (desk display)");
}

bool httpGetBuf(const char *url, char *body, int cap, const char *accept) {
  body[0] = 0;
  if (WiFi.status() != WL_CONNECTED || cap < 8) return false;
  WiFiClientSecure cli;
  cli.setInsecure();
  HTTPClient http;
  noteHttp(http);
  bool ok = false;
  if (http.begin(cli, url)) {
    if (httpWaitMs > 15000) http.setTimeout(httpWaitMs);
    if (accept) http.addHeader("Accept", accept);
    int code = http.GET();
    if (code == 200) {
      int got = 0;
      uint32_t t0 = millis();
      while (got < cap - 1 && millis() - t0 < (uint32_t)httpWaitMs) {
        int n = http.getStream().available();
        if (n > 0) {
          if (got + n > cap - 1) n = cap - 1 - got;
          int r = http.getStream().readBytes(body + got, n);
          if (r <= 0) break;
          got += r;
          continue;
        }
        if (!http.connected()) break;
        delay(2);
      }
      body[got] = 0;
      ok = got > 8;
    }
    http.end();
  }
  cli.stop();
  return ok;
}

bool httpPostBuf(const char *url, const char *payload, char *body, int cap) {
  body[0] = 0;
  if (WiFi.status() != WL_CONNECTED || cap < 8 || !payload) return false;
  WiFiClientSecure cli;
  cli.setInsecure();
  HTTPClient http;
  noteHttp(http);
  bool ok = false;
  if (http.begin(cli, url)) {
    if (httpWaitMs > 15000) http.setTimeout(httpWaitMs);
    http.addHeader("Content-Type", "text/plain");
    int code = http.POST((uint8_t *)payload, strlen(payload));
    if (code == 200) {
      int got = 0;
      uint32_t t0 = millis();
      while (got < cap - 1 && millis() - t0 < (uint32_t)httpWaitMs) {
        int n = http.getStream().available();
        if (n > 0) {
          if (got + n > cap - 1) n = cap - 1 - got;
          int r = http.getStream().readBytes(body + got, n);
          if (r <= 0) break;
          got += r;
          continue;
        }
        if (!http.connected()) break;
        delay(2);
      }
      body[got] = 0;
      ok = got > 8;
    }
    http.end();
  }
  cli.stop();
  return ok;
}

bool jsonNum(const char *s, const char *key, float *out) {
  const char *p = strstr(s, key);
  if (!p) return false;
  p += strlen(key);
  while (*p == ' ') p++;
  if (*p == 'n') {
    *out = 0;
    return true;
  }
  *out = (float)atof(p);
  return true;
}

bool copyQuoted(const char *p, const char *end, char *out, int n, const char **after) {
  while (p < end && (*p == ' ' || *p == '\n' || *p == '\r')) p++;
  if (p + 4 <= end && !strncmp(p, "null", 4)) {
    out[0] = 0;
    if (after) *after = p + 4;
    return true;
  }
  if (p >= end || *p != '"') return false;
  p++;
  int i = 0;
  while (p < end && *p != '"') {
    if (i < n - 1) out[i++] = *p;
    p++;
  }
  out[i] = 0;
  if (p >= end || *p != '"') return false;
  if (after) *after = p + 1;
  return true;
}

int readList(const char *p, float *out, int n) {
  const char *b = strchr(p, '[');
  if (!b) return 0;
  b++;
  int c = 0;
  while (c < n && *b && *b != ']') {
    while (*b == ' ' || *b == '\n' || *b == '\r' || *b == ',') b++;
    if (*b == ']' || !*b) break;
    if (*b == '"') {
      b++;
      while (*b && *b != '"') b++;
      if (*b == '"') b++;
      continue;
    }
    out[c++] = (float)atof(b);
    while (*b && *b != ',' && *b != ']') b++;
  }
  return c;
}

void formatLocalHm(const char *iso, char *out, int n) {
  int y, mo, d, h, mi;
  if (!iso || sscanf(iso, "%d-%d-%dT%d:%d", &y, &mo, &d, &h, &mi) < 5) {
    out[0] = 0;
    return;
  }
  fmtHm(h, mi, out, n);
}

void formatUntil(const char *iso, char *out, int n) {
  int y, mo, d, h, mi, se, oh = 0, om = 0;
  char sign = '+';
  if (!iso || !iso[0]) {
    strlcpy(out, "", n);
    return;
  }
  int got = sscanf(iso, "%d-%d-%dT%d:%d:%d%c%d:%d", &y, &mo, &d, &h, &mi, &se, &sign, &oh, &om);
  if (got < 6) {
    out[0] = 0;
    return;
  }
  if (got < 9) {
    sign = '+';
    oh = om = 0;
  }
  int yy = y, mm = mo;
  yy -= mm <= 2;
  int era = (yy >= 0 ? yy : yy - 399) / 400;
  unsigned yoe = (unsigned)(yy - era * 400);
  unsigned doy = (153U * (unsigned)(mm + (mm > 2 ? -3 : 9)) + 2U) / 5U + (unsigned)d - 1U;
  unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
  long long days = (long long)era * 146097LL + (long long)doe - 719468LL;
  time_t utc = (time_t)(days * 86400LL + (long long)h * 3600 + mi * 60 + se);
  int off = (oh * 60 + om) * 60;
  if (sign == '+') utc -= off;
  else utc += off;
  struct tm loc;
  localtime_r(&utc, &loc);
  char hm[16];
  fmtHm(loc.tm_hour, loc.tm_min, hm, sizeof(hm));
  snprintf(out, n, "until %s %s", WDAY[loc.tm_wday], hm);
}

void adoptZone(const char *iana, int offsetSec) {
  char next[56];
  next[0] = 0;
  if (iana && iana[0]) {
    for (unsigned i = 0; i < sizeof(ZONES) / sizeof(ZONES[0]); i++) {
      if (!strcmp(ZONES[i].iana, iana)) {
        strlcpy(next, ZONES[i].posix, sizeof(next));
        break;
      }
    }
  }
  if (!next[0]) {
    int west = -offsetSec;
    int neg = west < 0;
    int a = neg ? -west : west;
    int h = a / 3600;
    int m = (a % 3600) / 60;
    if (m) snprintf(next, sizeof(next), "XXX%d:%02d", neg ? -h : h, m);
    else snprintf(next, sizeof(next), "XXX%d", neg ? -h : h);
  }
  if (strcmp(next, placeTz)) {
    strlcpy(placeTz, next, sizeof(placeTz));
    applyPlaceTz();
    savePrefs();
  }
}

void noteTemp(int t) {
  int ymd = localYmd();
  if (!ymd) return;
  if (ymd != extremeYmd) {
    extremeYmd = ymd;
    dayHot = dayCold = t;
    pressBase = pressHpa;
    pressLo = pressHi = pressHpa;
    pressTrend = 0;
    savePrefs();
    return;
  }
  bool ch = false;
  if (t > dayHot) {
    dayHot = t;
    ch = true;
  }
  if (dayCold == 0 || t < dayCold) {
    dayCold = t;
    ch = true;
  }
  if (pressHpa > 0 && pressBase > 0) {
    float d = pressHpa - pressBase;
    int tr = d > 0.6f ? 1 : (d < -0.6f ? -1 : 0);
    if (tr != pressTrend) {
      pressTrend = tr;
      ch = true;
    }
    if (pressLo <= 0 || pressHpa < pressLo) {
      pressLo = pressHpa;
      ch = true;
    }
    if (pressHpa > pressHi) {
      pressHi = pressHpa;
      ch = true;
    }
  }
  if (ch) savePrefs();
}

int sevRank(const char *s) {
  if (!strcmp(s, "Extreme")) return 4;
  if (!strcmp(s, "Severe")) return 3;
  if (!strcmp(s, "Moderate")) return 2;
  if (!strcmp(s, "Minor")) return 1;
  return 0;
}

void considerAlert(AlertParse *a) {
  if (!a->event[0]) return;
  if (a->nKeep < 3) {
    int k = a->nKeep++;
    strlcpy(a->keepEvent[k], a->event, sizeof(a->keepEvent[k]));
    strlcpy(a->keepEnds[k], a->ends, sizeof(a->keepEnds[k]));
    strlcpy(a->keepHead[k], a->headline, sizeof(a->keepHead[k]));
    strlcpy(a->keepSev[k], a->sev, sizeof(a->keepSev[k]));
  }
  int r = sevRank(a->sev) * 10;
  if (strstr(a->event, "Warning")) r += 3;
  else if (strstr(a->event, "Watch")) r += 2;
  else if (strstr(a->event, "Advisory")) r += 1;
  a->count++;
  if (a->count == 1 || r >= a->bestRank) {
    strlcpy(a->bestEvent, a->event, sizeof(a->bestEvent));
    strlcpy(a->bestSender, a->sender, sizeof(a->bestSender));
    strlcpy(a->bestEnds, a->ends, sizeof(a->bestEnds));
    strlcpy(a->bestHeadline, a->headline, sizeof(a->bestHeadline));
    a->bestRank = r;
  }
  a->event[0] = a->sender[0] = a->ends[0] = a->sev[0] = a->headline[0] = 0;
}

void takeAlerts(char *win, int *fill, AlertParse *a) {
  const char *keys[] = {"\"expires\":", "\"ends\":", "\"severity\":", "\"event\":", "\"senderName\":", "\"headline\":"};
  int f = *fill;
  int i = 0;
  while (i < f) {
    int which = -1, at = f;
    for (int k = 0; k < 6; k++) {
      const char *hitp = strstr(win + i, keys[k]);
      if (!hitp) continue;
      int pos = (int)(hitp - win);
      if (pos < at) {
        at = pos;
        which = k;
      }
    }
    if (which < 0) {
      i = f > 20 ? f - 20 : f;
      break;
    }
    char tmp[80];
    const char *after = nullptr;
    if (!copyQuoted(win + at + (int)strlen(keys[which]), win + f, tmp, sizeof(tmp), &after)) break;
    if (which == 0 || which == 1) {
      if (a->event[0]) considerAlert(a);
      if (which == 1) {
        if (tmp[0]) strlcpy(a->ends, tmp, sizeof(a->ends));
      } else if (!a->ends[0])
        strlcpy(a->ends, tmp, sizeof(a->ends));
    } else if (which == 2) {
      if (a->event[0]) considerAlert(a);
      strlcpy(a->sev, tmp, sizeof(a->sev));
    } else if (which == 3) {
      if (a->event[0]) considerAlert(a);
      strlcpy(a->event, tmp, sizeof(a->event));
    } else if (which == 4) {
      strlcpy(a->sender, tmp, sizeof(a->sender));
    } else {
      strlcpy(a->headline, tmp, sizeof(a->headline));
      considerAlert(a);
    }
    i = (int)(after - win);
  }
  if (i > 0 && i <= f) {
    memmove(win, win + i, (size_t)(f - i));
    *fill = f - i;
    win[*fill] = 0;
  }
}

bool pullAlerts(float lat, float lon) {
  char url[96];
  snprintf(url, sizeof(url), "https://api.weather.gov/alerts/active?point=%.4f,%.4f", lat, lon);
  WiFiClientSecure cli;
  cli.setInsecure();
  HTTPClient http;
  noteHttp(http);
  bool ok = false;
  AlertParse parsed;
  memset(&parsed, 0, sizeof(parsed));
  parsed.bestRank = -1;
  if (http.begin(cli, url)) {
    http.addHeader("Accept", "application/geo+json");
    int code = http.GET();
    if (code == 200) {
      WiFiClient *stream = http.getStreamPtr();
      stream->setTimeout(2000);
      char win[1024];
      int fill = 0;
      uint32_t t0 = millis();
      while (millis() - t0 < 12000) {
        int room = (int)sizeof(win) - 1 - fill;
        if (room < 80) {
          takeAlerts(win, &fill, &parsed);
          if (fill > 800) {
            memmove(win, win + 400, (size_t)(fill - 400));
            fill -= 400;
            win[fill] = 0;
          }
          room = (int)sizeof(win) - 1 - fill;
          if (room < 80) break;
        }
        int av = stream->available();
        if (av <= 0) {
          if (!http.connected()) break;
          delay(2);
          continue;
        }
        if (av > room) av = room;
        int r = stream->read((uint8_t *)win + fill, (size_t)av);
        if (r <= 0) break;
        fill += r;
        win[fill] = 0;
        takeAlerts(win, &fill, &parsed);
      }
      takeAlerts(win, &fill, &parsed);
      if (parsed.event[0]) considerAlert(&parsed);
      ok = true;
    }
    http.end();
  }
  cli.stop();
  if (!ok) return false;
  alertLive = true;
  alertOn = parsed.count > 0 && parsed.bestEvent[0];
  strlcpy(alertEvent, parsed.bestEvent, sizeof(alertEvent));
  strlcpy(alertEnds, parsed.bestEnds, sizeof(alertEnds));
  strlcpy(alertDetail, parsed.bestHeadline, sizeof(alertDetail));
  nWarn = parsed.nKeep;
  if (nWarn > 3) nWarn = 3;
  for (int i = 0; i < nWarn; i++) {
    strlcpy(warns[i].event, parsed.keepEvent[i], sizeof(warns[i].event));
    strlcpy(warns[i].ends, parsed.keepEnds[i], sizeof(warns[i].ends));
    strlcpy(warns[i].headline, parsed.keepHead[i], sizeof(warns[i].headline));
    strlcpy(warns[i].sev, parsed.keepSev[i], sizeof(warns[i].sev));
  }
  alertNote[0] = 0;
  return true;
}

void pullAurora() {
  char *body = (char *)malloc(4096);
  if (!body) return;
  if (!httpGetBuf("https://services.swpc.noaa.gov/products/noaa-planetary-k-index.json", body, 4096,
                  "application/json")) {
    free(body);
    return;
  }
  const char *p = body;
  const char *hit = nullptr;
  while ((p = strstr(p, "],[\""))) {
    hit = p;
    p += 4;
  }
  if (hit) {
    const char *s = strchr(hit, ',');
    if (s) {
      while (*s && (*s < '0' || *s > '9')) s++;
      float k = (float)atof(s);
      if (k >= 0 && k <= 9) auroraKp = (int)(k + 0.5f);
    }
  }
  free(body);
}

bool pullMeteo(float lat, float lon) {
  char url[640];
  snprintf(url, sizeof(url),
           "https://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
           "&current=temperature_2m,apparent_temperature,relative_humidity_2m,weather_code,"
           "wind_speed_10m,wind_direction_10m,is_day,shortwave_radiation,uv_index,pressure_msl"
           "&hourly=precipitation_probability,precipitation,temperature_2m,weather_code"
           "&daily=temperature_2m_max,temperature_2m_min,sunrise,sunset,precipitation_sum,uv_index_max,"
           "weather_code,precipitation_probability_max"
           "&temperature_unit=fahrenheit&wind_speed_unit=mph&precipitation_unit=inch"
           "&timezone=auto&forecast_days=7&forecast_hours=24",
           lat, lon);
  if (!httpGetBuf(url, packBuf, sizeof(packBuf), "application/json")) return false;
  char iana[48];
  iana[0] = 0;
  const char *tz = strstr(packBuf, "\"timezone\":\"");
  if (tz) copyQuoted(tz + 11, packBuf + strlen(packBuf), iana, sizeof(iana), nullptr);
  float off = 0;
  jsonNum(packBuf, "\"utc_offset_seconds\":", &off);
  adoptZone(iana, (int)off);
  const char *cur = strstr(packBuf, "\"current\":");
  if (!cur) return false;
  float code = 0, day = 1, wind = 0, wdir = 0, temp = 0, feel = 0, rh = 0, uv = 0, rad = 0, press = 0;
  jsonNum(cur, "\"weather_code\":", &code);
  jsonNum(cur, "\"is_day\":", &day);
  jsonNum(cur, "\"wind_speed_10m\":", &wind);
  jsonNum(cur, "\"wind_direction_10m\":", &wdir);
  jsonNum(cur, "\"temperature_2m\":", &temp);
  jsonNum(cur, "\"apparent_temperature\":", &feel);
  jsonNum(cur, "\"relative_humidity_2m\":", &rh);
  jsonNum(cur, "\"uv_index\":", &uv);
  jsonNum(cur, "\"shortwave_radiation\":", &rad);
  jsonNum(cur, "\"pressure_msl\":", &press);
  wxCode = (int)(code + 0.5f);
  wxDay = day >= 0.5f;
  wxWind = (int)(wind + 0.5f);
  wxDir = (int)(wdir + 0.5f);
  wxTemp = (int)(temp + (temp >= 0 ? 0.5f : -0.5f));
  wxFeel = (int)(feel + (feel >= 0 ? 0.5f : -0.5f));
  wxRh = (int)(rh + 0.5f);
  wxUv = (int)(uv + 0.5f);
  wxRad = rad;
  pressHpa = press;
  const char *hourly = strstr(packBuf, "\"hourly\":");
  const char *daily = strstr(packBuf, "\"daily\":");
  nRain = 0;
  if (hourly) {
    const char *pp = strstr(hourly, "\"precipitation_probability\":");
    const char *pr = strstr(hourly, "\"precipitation\":");
    const char *pt = strstr(hourly, "\"temperature_2m\":");
    const char *pc = strstr(hourly, "\"weather_code\":");
    float pops[24], amts[24], temps[24], codes[24];
    int np = (pp && (!daily || pp < daily)) ? readList(pp, pops, 24) : 0;
    int na = (pr && (!daily || pr < daily)) ? readList(pr, amts, 24) : 0;
    int nt = (pt && (!daily || pt < daily)) ? readList(pt, temps, 24) : 0;
    int nc = (pc && (!daily || pc < daily)) ? readList(pc, codes, 24) : 0;
    nRain = np < na ? np : na;
    if (nRain > 12) nRain = 12;
    for (int i = 0; i < nRain; i++) {
      rainPop[i] = pops[i];
      rainAmt[i] = amts[i];
    }
    int have = np < nt ? np : nt;
    nSlot = 0;
    for (int i = 0; i + 1 < have && nSlot < 12; i += 2) {
      slotT[nSlot] = (int)(temps[i] + (temps[i] >= 0 ? 0.5f : -0.5f));
      slotP[nSlot] = (int)(pops[i] + 0.5f);
      slotC[nSlot] = (i < nc) ? (int)(codes[i] + 0.5f) : 0;
      nSlot++;
    }
  }
  if (daily) {
    float hi[7], lo[7], today = 0, dpop[7], dcode[7];
    const char *khi = strstr(daily, "\"temperature_2m_max\":");
    const char *klo = strstr(daily, "\"temperature_2m_min\":");
    const char *kr = strstr(daily, "\"precipitation_sum\":");
    const char *kp = strstr(daily, "\"precipitation_probability_max\":");
    const char *kc = strstr(daily, "\"weather_code\":");
    int nh = khi ? readList(khi, hi, 7) : 0;
    int nl = klo ? readList(klo, lo, 7) : 0;
    int npd = kp ? readList(kp, dpop, 7) : 0;
    int ncd = kc ? readList(kc, dcode, 7) : 0;
    if (kr) readList(kr, &today, 1);
    nDays = nh < nl ? nh : nl;
    if (nDays > 7) nDays = 7;
    for (int i = 0; i < nDays; i++) {
      dayHi[i] = (int)(hi[i] + 0.5f);
      dayLo[i] = (int)(lo[i] + 0.5f);
      dayPop[i] = (i < npd) ? (int)(dpop[i] + 0.5f) : 0;
      dayCode[i] = (i < ncd) ? (int)(dcode[i] + 0.5f) : 0;
    }
    fcHigh = nDays ? dayHi[0] : 0;
    fcLow = nDays ? dayLo[0] : 0;
    rainToday = today;
    char iso[28];
    const char *up = strstr(daily, "\"sunrise\":");
    if (up) {
      const char *q = strchr(up, '[');
      if (q && copyQuoted(q + 1, packBuf + strlen(packBuf), iso, sizeof(iso), nullptr))
        formatLocalHm(iso, wxUp, sizeof(wxUp));
    }
    const char *dn = strstr(daily, "\"sunset\":");
    if (dn) {
      const char *q = strchr(dn, '[');
      if (q && copyQuoted(q + 1, packBuf + strlen(packBuf), iso, sizeof(iso), nullptr))
        formatLocalHm(iso, wxDown, sizeof(wxDown));
    }
  }
  char aq[400];
  snprintf(url, sizeof(url),
           "https://air-quality-api.open-meteo.com/v1/air-quality?latitude=%.4f&longitude=%.4f&current=us_aqi",
           lat, lon);
  if (httpGetBuf(url, aq, sizeof(aq), "application/json")) {
    float aqi = -1;
    const char *c2 = strstr(aq, "\"current\":");
    if (c2 && jsonNum(c2, "\"us_aqi\":", &aqi)) wxAqi = (int)(aqi + 0.5f);
  }
  noteTemp(wxTemp);
  wxOk = true;
  return true;
}

float kmBetween(float lat1, float lon1, float lat2, float lon2) {
  float dLat = (lat2 - lat1) * 0.0174533f;
  float dLon = (lon2 - lon1) * 0.0174533f;
  float a = sinf(dLat / 2) * sinf(dLat / 2) +
            cosf(lat1 * 0.0174533f) * cosf(lat2 * 0.0174533f) * sinf(dLon / 2) * sinf(dLon / 2);
  return 6371.0f * 2 * atan2f(sqrtf(a), sqrtf(1 - a));
}

void considerStation(const char *win, const char *latp) {
  float lat = (float)atof(latp + 6);
  const char *lngp = strstr(latp, "\"lng\":");
  if (!lngp || lngp > latp + 80) return;
  float lng = (float)atof(lngp + 6);
  const char *id = nullptr;
  for (const char *q = latp; q > win && latp - q < 220; q--) {
    if (!strncmp(q, "\"id\":", 5)) {
      id = q + 5;
      while (*id == ' ') id++;
      if (*id == '"') id++;
      break;
    }
  }
  if (!id || *id == '"') return;
  float km = kmBetween(placeLat, placeLon, lat, lng);
  if (km > 160.0f) return;
  if (tideId[0] && km >= tideBestKm) return;
  tideBestKm = km;
  int i = 0;
  while (id[i] && id[i] != '"' && i < (int)sizeof(tideId) - 1) {
    tideId[i] = id[i];
    i++;
  }
  tideId[i] = 0;
  tideName[0] = 0;
  const char *nm = strstr(id, "\"name\":");
  if (nm && nm < latp + 180) {
    const char *v = nm + 7;
    while (*v == ' ') v++;
    copyQuoted(v, v + 80, tideName, sizeof(tideName), nullptr);
  }
  tideKnown = tideId[0] != 0;
}

bool findTideStation() {
  if (WiFi.status() != WL_CONNECTED) return false;
  WiFiClientSecure cli;
  cli.setInsecure();
  HTTPClient http;
  noteHttp(http);
  bool ok = false;
  const char *url =
      "https://api.tidesandcurrents.noaa.gov/mdapi/prod/webapi/stations.json?type=tidepredictions";
  tideId[0] = 0;
  tideBestKm = 99999.0f;
  if (http.begin(cli, url)) {
    http.addHeader("Accept", "application/json");
    int code = http.GET();
    if (code == 200) {
      char win[900];
      int fill = 0;
      uint32_t t0 = millis();
      int got = 0;
      while (got < 2300000 && millis() - t0 < 45000) {
        int room = (int)sizeof(win) - 1 - fill;
        if (room < 100 && fill > 160) {
          memmove(win, win + fill - 160, 160);
          fill = 160;
          win[fill] = 0;
          room = (int)sizeof(win) - 1 - fill;
        }
        int av = http.getStream().available();
        if (av <= 0) {
          if (!http.connected()) break;
          delay(2);
          continue;
        }
        if (av > room) av = room;
        int r = http.getStream().readBytes(win + fill, av);
        if (r <= 0) break;
        fill += r;
        got += r;
        win[fill] = 0;
        char *p = win;
        while ((p = strstr(p, "\"lat\":"))) {
          considerStation(win, p);
          p += 6;
        }
        delay(1);
      }
      ok = tideId[0] != 0;
    }
    http.end();
  }
  cli.stop();
  if (ok) savePrefs();
  return ok;
}

long long packTime(int y, int mo, int d, int h, int mi) {
  return (long long)y * 100000000LL + mo * 1000000LL + d * 10000LL + h * 100LL + mi;
}

bool pullTides() {
  tideHigh[0] = tideLow[0] = 0;
  if (!tideId[0] && !findTideStation()) return false;
  time_t now = 0;
  if (!liveClock(&now)) now = time(nullptr);
  struct tm begin;
  localtime_r(&now, &begin);
  char url[280];
  snprintf(url, sizeof(url),
           "https://api.tidesandcurrents.noaa.gov/api/prod/datagetter?begin_date=%04d%02d%02d&range=48"
           "&station=%s&product=predictions&datum=MLLW&time_zone=lst_ldt&interval=hilo"
           "&units=english&format=json&application=SkyWeather",
           begin.tm_year + 1900, begin.tm_mon + 1, begin.tm_mday, tideId);
  char body[1800];
  if (!httpGetBuf(url, body, sizeof(body), "application/json")) return false;
  long long nowP = 0;
  {
    struct tm t;
    localtime_r(&now, &t);
    nowP = packTime(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min);
  }
  const char *p = body;
  while ((p = strstr(p, "\"t\":\""))) {
    int y, mo, d, h, mi;
    if (sscanf(p + 5, "%d-%d-%d %d:%d", &y, &mo, &d, &h, &mi) < 5) {
      p += 5;
      continue;
    }
    const char *ty = strstr(p, "\"type\":\"");
    if (!ty || ty > p + 80) break;
    char kind = ty[8];
    float v = 0;
    const char *vv = strstr(p, "\"v\":\"");
    if (vv && vv < ty) v = (float)atof(vv + 5);
    long long tp = packTime(y, mo, d, h, mi);
    if (nowP && tp + 30 < nowP) {
      p = ty + 8;
      continue;
    }
    char hm[16], line[28];
    fmtHm(h, mi, hm, sizeof(hm));
    snprintf(line, sizeof(line), "%s %s %.1f", kind == 'H' ? "H" : "L", hm, v);
    if (kind == 'H' && !tideHigh[0]) strlcpy(tideHigh, line, sizeof(tideHigh));
    if (kind == 'L' && !tideLow[0]) strlcpy(tideLow, line, sizeof(tideLow));
    p = ty + 8;
    if (tideHigh[0] && tideLow[0]) break;
  }
  return tideHigh[0] || tideLow[0];
}

void pullPack(float lat, float lon) {
  bool met = pullMeteo(lat, lon);
  if (lat != placeLat || lon != placeLon) return;
  if (!met) {
    if (!wxOk) strlcpy(alertNote, "no sky link", sizeof(alertNote));
  } else if (!pullAlerts(lat, lon)) {
    if (!alertLive) strlcpy(alertNote, "no alert link", sizeof(alertNote));
  }
  if (lat == placeLat && lon == placeLon) pullTides();
  if (lat == placeLat && lon == placeLon) pullAurora();
  lastPack = millis();
}

void shortPlace(const char *display, char *out, int n) {
  char buf[120];
  strlcpy(buf, display, sizeof(buf));
  char *parts[6];
  int np = 0;
  parts[np++] = buf;
  for (char *p = buf; *p && np < 6; p++) {
    if (*p == ',') {
      *p = 0;
      char *q = p + 1;
      while (*q == ' ') q++;
      parts[np++] = q;
      p = q - 1;
    }
  }
  for (int i = 0; i < np; i++) {
    int len = (int)strlen(parts[i]);
    while (len > 0 && parts[i][len - 1] == ' ') parts[i][--len] = 0;
  }
  if (np >= 4) snprintf(out, n, "%s, %s", parts[0], parts[2]);
  else if (np >= 2) snprintf(out, n, "%s, %s", parts[0], parts[np - 2]);
  else strlcpy(out, display, n);
}

void urlEncode(const char *in, char *out, int n) {
  int j = 0;
  for (int i = 0; in[i] && j < n - 4; i++) {
    char c = in[i];
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) out[j++] = c;
    else if (c == ' ') {
      out[j++] = '%';
      out[j++] = '2';
      out[j++] = '0';
    } else if (c == ',') {
      out[j++] = '%';
      out[j++] = '2';
      out[j++] = 'C';
    }
  }
  out[j] = 0;
}

void pullFind() {
  char enc[140];
  urlEncode(placeQuery, enc, sizeof(enc));
  char url[260];
  snprintf(url, sizeof(url),
           "https://nominatim.openstreetmap.org/search?q=%s&format=json&limit=4&countrycodes=us&addressdetails=0",
           enc);
  nHits = 0;
  char body[3200];
  if (!httpGetBuf(url, body, sizeof(body), "application/json")) {
    strlcpy(placeMsg, "search failed", sizeof(placeMsg));
    return;
  }
  const char *p = body;
  while (nHits < 4) {
    const char *latk = strstr(p, "\"lat\":\"");
    if (!latk) break;
    float la = (float)atof(latk + 7);
    const char *lonk = strstr(latk, "\"lon\":\"");
    if (!lonk) break;
    float lo = (float)atof(lonk + 7);
    const char *nk = strstr(lonk, "\"display_name\":\"");
    if (!nk) break;
    char raw[110];
    const char *after = nullptr;
    if (!copyQuoted(nk + 15, body + strlen(body), raw, sizeof(raw), &after)) break;
    shortPlace(raw, hits[nHits].label, sizeof(hits[nHits].label));
    hits[nHits].lat = la;
    hits[nHits].lon = lo;
    nHits++;
    p = after ? after : nk + 16;
  }
  placeTop = 0;
  if (!nHits) strlcpy(placeMsg, "nothing in the US", sizeof(placeMsg));
  else placeMsg[0] = 0;
}

bool httpGetBin(const char *url, uint8_t **out, int *outLen) {
  *out = nullptr;
  *outLen = 0;
  if (WiFi.status() != WL_CONNECTED) return false;
  WiFiClientSecure cli;
  cli.setInsecure();
  HTTPClient http;
  noteHttp(http);
  bool ok = false;
  if (http.begin(cli, url)) {
    http.addHeader("Accept", "image/jpeg,*/*");
    int code = http.GET();
    if (code == 200) {
      const int CAP = 220000;
      uint8_t *buf = (uint8_t *)ps_malloc(CAP);
      if (!buf) buf = (uint8_t *)malloc(CAP);
      if (buf) {
        int got = 0;
        uint32_t t0 = millis();
        while (got < CAP && millis() - t0 < 12000) {
          int n = http.getStream().available();
          if (n > 0) {
            if (got + n > CAP) n = CAP - got;
            int r = http.getStream().readBytes(buf + got, n);
            if (r <= 0) break;
            got += r;
            continue;
          }
          if (!http.connected()) break;
          delay(2);
        }
        if (got > 400) {
          *out = buf;
          *outLen = got;
          ok = true;
        } else
          free(buf);
      }
    }
    http.end();
  }
  cli.stop();
  return ok;
}

bool sameTown(const char *a, const char *b) {
  while (*a && *b) {
    char ca = *a, cb = *b;
    if (ca >= 'A' && ca <= 'Z') ca = (char)(ca + 32);
    if (cb >= 'A' && cb <= 'Z') cb = (char)(cb + 32);
    if (ca != cb) return false;
    a++;
    b++;
  }
  return *a == 0 && *b == 0;
}

void addMapTown(const char *name, float lat, float lon) {
  if (!name || !name[0] || nMapTowns >= 18) return;
  for (int i = 0; i < nMapTowns; i++)
    if (sameTown(mapTowns[i].name, name)) return;
  strlcpy(mapTowns[nMapTowns].name, name, sizeof(mapTowns[nMapTowns].name));
  mapTowns[nMapTowns].lat = lat;
  mapTowns[nMapTowns].lon = lon;
  nMapTowns++;
}

float jsonAt(const char *p) {
  p += 6;
  while (*p == ' ' || *p == '"') p++;
  return (float)atof(p);
}

void takeMapNames(const char *body, bool water, int limit, float south, float west, float north, float east) {
  const char *p = body;
  const char *end = body + strlen(body);
  while (nMapTowns < limit && p < end) {
    const char *latk = strstr(p, "\"lat\":");
    if (!latk) break;
    const char *lonk = strstr(latk, "\"lon\":");
    if (!lonk) break;
    float la = jsonAt(latk);
    float lo = jsonAt(lonk);
    const char *nextLat = strstr(latk + 6, "\"lat\":");
    const char *nk = strstr(lonk, "\"name\":\"");
    if (!nk || (nextLat && nk > nextLat)) {
      p = lonk + 6;
      continue;
    }
    if (water) {
      const char *cat = strstr(lonk, "\"category\":\"");
      bool okCat = false;
      if (cat && (!nextLat || cat < nextLat)) {
        const char *v = cat + 12;
        if (!strncmp(v, "waterway\"", 9) || !strncmp(v, "natural\"", 8)) okCat = true;
      }
      if (!okCat) {
        p = nextLat ? nextLat : end;
        continue;
      }
    }
    char raw[24];
    const char *after = nullptr;
    if (!copyQuoted(nk + 7, end, raw, sizeof(raw), &after)) {
      p = nk + 8;
      continue;
    }
    if (la > south && la < north && lo > west && lo < east) addMapTown(raw, la, lo);
    p = after ? after : nk + 8;
  }
}

void takeMajorCities(const char *body, float south, float west, float north, float east) {
  char nm[24][24];
  float clat[24], clon[24];
  long cpop[24];
  int n = 0;
  const char *p = body;
  while (n < 24 && p && *p) {
    const char *tab = strchr(p, '\t');
    if (!tab) break;
    const char *tab2 = strchr(tab + 1, '\t');
    if (!tab2) break;
    const char *tab3 = strchr(tab2 + 1, '\t');
    float la = (float)atof(p);
    float lo = (float)atof(tab + 1);
    char raw[24];
    int i = 0;
    const char *r = tab2 + 1;
    const char *rend = tab3 ? tab3 : r + 40;
    while (r < rend && *r && *r != '\n' && *r != '\r' && i < (int)sizeof(raw) - 1) raw[i++] = *r++;
    raw[i] = 0;
    long pop = tab3 ? atol(tab3 + 1) : 0;
    const char *line = tab3 ? tab3 + 1 : r;
    while (*line && *line != '\n' && *line != '\r') line++;
    while (*line == '\n' || *line == '\r') line++;
    p = line;
    if (!raw[0] || la <= south || la >= north || lo <= west || lo >= east) continue;
    int found = -1;
    for (int k = 0; k < n; k++)
      if (sameTown(nm[k], raw)) found = k;
    if (found >= 0) {
      if (pop > cpop[found]) {
        cpop[found] = pop;
        clat[found] = la;
        clon[found] = lo;
      }
      continue;
    }
    strlcpy(nm[n], raw, sizeof(nm[n]));
    clat[n] = la;
    clon[n] = lo;
    cpop[n] = pop;
    n++;
  }
  for (int a = 1; a < n; a++) {
    long pop = cpop[a];
    float la = clat[a], lo = clon[a];
    char tmp[24];
    strlcpy(tmp, nm[a], sizeof(tmp));
    int b = a;
    while (b > 0 && cpop[b - 1] < pop) {
      cpop[b] = cpop[b - 1];
      clat[b] = clat[b - 1];
      clon[b] = clon[b - 1];
      strlcpy(nm[b], nm[b - 1], sizeof(nm[b]));
      b--;
    }
    cpop[b] = pop;
    clat[b] = la;
    clon[b] = lo;
    strlcpy(nm[b], tmp, sizeof(nm[b]));
  }
  int added = 0;
  for (int i = 0; i < n && added < 6; i++) {
    int before = nMapTowns;
    addMapTown(nm[i], clat[i], clon[i]);
    if (nMapTowns > before) added++;
  }
}

void pullMapCities() {
  nMapTowns = 0;
  if (!placeSet) return;
  char home[24];
  const char *s = placeName;
  const char *comma = strchr(placeName, ',');
  if (placeName[0] >= '0' && placeName[0] <= '9' && comma) s = comma + 1;
  while (*s == ' ') s++;
  strlcpy(home, s, sizeof(home));
  char *cut = strchr(home, ',');
  if (cut) *cut = 0;
  char *county = strstr(home, " County");
  if (county) *county = 0;
  if (home[0]) addMapTown(home, placeLat, placeLon);
  float south = placeLat - 0.55f, north = placeLat + 0.55f;
  float west = placeLon - 0.85f, east = placeLon + 0.85f;
  char stackBody[1600];
  char *body = (char *)ps_malloc(12000);
  bool heapBody = body != nullptr;
  if (!body) body = (char *)malloc(12000);
  if (body) heapBody = true;
  else body = stackBody;
  int cap = heapBody ? 12000 : (int)sizeof(stackBody);
  char data[480];
  snprintf(data, sizeof(data),
           "[out:csv(::lat,::lon,name,population;false)][timeout:8];"
           "(node[place=city](%.3f,%.3f,%.3f,%.3f);"
           "way[place=city](%.3f,%.3f,%.3f,%.3f);"
           "relation[place=city](%.3f,%.3f,%.3f,%.3f););out center;",
           south, west, north, east, south, west, north, east, south, west, north, east);
  char url[240];
  httpWaitMs = 20000;
  bool gotCities = httpPostBuf("https://overpass.openstreetmap.fr/api/interpreter", data, body, cap);
  if (!gotCities)
    gotCities = httpPostBuf("https://overpass-api.de/api/interpreter", data, body, cap);
  httpWaitMs = 12000;
  if (gotCities) takeMajorCities(body, south, west, north, east);
  snprintf(url, sizeof(url),
           "https://nominatim.openstreetmap.org/search?format=jsonv2&bounded=1&limit=6"
           "&viewbox=%.3f,%.3f,%.3f,%.3f&q=river",
           west, north, east, south);
  if (httpGetBuf(url, body, cap, "application/json"))
    takeMapNames(body, true, 15, south, west, north, east);
  delay(1100);
  snprintf(url, sizeof(url),
           "https://nominatim.openstreetmap.org/search?format=jsonv2&bounded=1&limit=6"
           "&viewbox=%.3f,%.3f,%.3f,%.3f&layer=natural&q=water",
           west, north, east, south);
  if (httpGetBuf(url, body, cap, "application/json"))
    takeMapNames(body, true, 18, south, west, north, east);
  if (heapBody) free(body);
}

void pullMap() {
  if (!placeSet) return;
  float padLon = 0.85f, padLat = 0.55f;
  char url[420];
  snprintf(url, sizeof(url),
           "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/export?"
           "bbox=%.4f,%.4f,%.4f,%.4f&bboxSR=4326&imageSR=4326&size=800,420&format=jpg&f=image",
           placeLon - padLon, placeLat - padLat, placeLon + padLon, placeLat + padLat);
  uint8_t *buf = nullptr;
  int len = 0;
  if (mapRadar) {
    free(mapRadar);
    mapRadar = nullptr;
    mapRadarLen = 0;
  }
  mapFail = !httpGetBin(url, &buf, &len) || !buf;
  if (!mapFail) {
    mapHold = buf;
    mapHoldLen = len;
  }
  if (mapKind == MAP_CLOUD && !mapFail) {
    snprintf(url, sizeof(url),
             "https://gibs.earthdata.nasa.gov/wms/epsg4326/best/wms.cgi?SERVICE=WMS&REQUEST=GetMap"
             "&VERSION=1.1.1&LAYERS=GOES-East_ABI_GeoColor&FORMAT=image/jpeg&SRS=EPSG:4326"
             "&BBOX=%.4f,%.4f,%.4f,%.4f&WIDTH=800&HEIGHT=420",
             placeLon - padLon, placeLat - padLat, placeLon + padLon, placeLat + padLat);
    uint8_t *layer = nullptr;
    int layerLen = 0;
    if (httpGetBin(url, &layer, &layerLen) && layer) {
      mapRadar = layer;
      mapRadarLen = layerLen;
    }
  }
  if (mapKind == MAP_STORM && !mapFail) {
    snprintf(url, sizeof(url),
             "https://mesonet.agron.iastate.edu/cgi-bin/wms/nexrad/n0q.cgi?SERVICE=WMS&VERSION=1.1.1"
             "&REQUEST=GetMap&LAYERS=nexrad-n0q&FORMAT=image/jpeg&SRS=EPSG:4326"
             "&BBOX=%.4f,%.4f,%.4f,%.4f&WIDTH=800&HEIGHT=420",
             placeLon - padLon, placeLat - padLat, placeLon + padLon, placeLat + padLat);
    uint8_t *layer = nullptr;
    int layerLen = 0;
    if (httpGetBin(url, &layer, &layerLen) && layer) {
      mapRadar = layer;
      mapRadarLen = layerLen;
    }
  }
  pullMapCities();
  lastMap = millis();
}

void paintMapCities() {
  if (!placeSet || !nMapTowns) return;
  float west = placeLon - 0.85f, east = placeLon + 0.85f;
  float south = placeLat - 0.55f, north = placeLat + 0.55f;
  float dw = east - west, dh = north - south;
  if (dw < 0.01f || dh < 0.01f) return;
  gfx->setTextSize(2);
  int px[18], py[18], pw[18], placed = 0;
  for (int i = 0; i < nMapTowns; i++) {
    float lon = mapTowns[i].lon, lat = mapTowns[i].lat;
    if (lon < west || lon > east || lat < south || lat > north) continue;
    int tw = (int)strlen(mapTowns[i].name) * 12 + 10;
    int x = (int)((lon - west) / dw * 799.0f);
    int y = 56 + (int)((north - lat) / dh * 400.0f);
    if (x < 8) x = 8;
    if (x + tw + 16 > 792) x = 792 - tw - 16;
    if (y < 112) y = 112;
    bool room = false;
    for (int n = 0; n < 5; n++) {
      bool hit = false;
      for (int k = 0; k < placed; k++) {
        if (x < px[k] + pw[k] && x + tw + 20 > px[k] && y < py[k] + 26 && y + 26 > py[k]) hit = true;
      }
      if (!hit) {
        room = true;
        break;
      }
      y += 26;
      if (y > 440) break;
    }
    if (!room || y > 440) continue;
    px[placed] = x;
    py[placed] = y;
    pw[placed] = tw + 20;
    placed++;
    gfx->fillCircle(x, y, 4, SUN_C);
    gfx->fillRoundRect(x + 8, y - 12, tw, 24, 4, WELL);
    gfx->setTextColor(COL_TXT, WELL);
    gfx->setCursor(x + 12, y - 8);
    gfx->print(mapTowns[i].name);
  }
}

void showMap() {
  if (page != PG_RADAR) {
    if (mapHold) {
      free(mapHold);
      mapHold = nullptr;
      mapHoldLen = 0;
    }
    if (mapRadar) {
      free(mapRadar);
      mapRadar = nullptr;
      mapRadarLen = 0;
    }
    return;
  }
  if (mapFail || !mapHold) {
    gfx->setTextColor(COL_AMB, WELL);
    gfx->setTextSize(2);
    centre("picture did not load", 400, 240);
    return;
  }
  jpgOx = 0;
  jpgOy = 56;
  jpgClipY = 470;
  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(jpgPlot);
  TJpgDec.setSwapBytes(false);
  int rc = TJpgDec.drawJpg(jpgOx, jpgOy, mapHold, mapHoldLen);
  if (rc == 0 && mapKind == MAP_CLOUD && mapRadar && mapRadarLen > 400) {
    TJpgDec.setCallback(cloudOverPlot);
    TJpgDec.drawJpg(jpgOx, jpgOy, mapRadar, mapRadarLen);
    TJpgDec.setCallback(jpgPlot);
  }
  if (rc == 0 && mapKind == MAP_STORM && mapRadar && mapRadarLen > 400) {
    TJpgDec.setCallback(radarPlot);
    TJpgDec.drawJpg(jpgOx, jpgOy, mapRadar, mapRadarLen);
    TJpgDec.setCallback(jpgPlot);
  }
  free(mapHold);
  mapHold = nullptr;
  mapHoldLen = 0;
  if (mapRadar) {
    free(mapRadar);
    mapRadar = nullptr;
    mapRadarLen = 0;
  }
  if (rc == 0) {
    paintMapNote();
    paintMapCities();
  }
  else {
    gfx->setTextColor(COL_AMB, WELL);
    gfx->setTextSize(2);
    centre("picture did not open", 400, 240);
  }
}

void wrap360(double &a) {
  if (a != a || a > 1.0e7 || a < -1.0e7) {
    a = 0;
    return;
  }
  a = fmod(a, 360.0);
  if (a < 0) a += 360.0;
}
double unixToJd(time_t t) { return 2440587.5 + (double)t / 86400.0; }

void azelFromRaDec(double raRad, double dec, time_t t, int *az, int *el) {
  double jd = unixToJd(t);
  double gmst = 280.46061837 + 360.98564736629 * (jd - 2451545.0);
  wrap360(gmst);
  double lst = (gmst + placeLon) * 0.01745329252;
  double ha = lst - raRad;
  double lat = placeLat * 0.01745329252;
  double alt = asin(sin(lat) * sin(dec) + cos(lat) * cos(dec) * cos(ha));
  double azi = atan2(-cos(dec) * sin(ha), sin(dec) * cos(lat) - cos(dec) * sin(lat) * cos(ha));
  *el = (int)(alt * 57.2957795);
  int a = (int)(azi * 57.2957795);
  if (a < 0) a += 360;
  *az = a;
}

void sunAlt(time_t t, float *altOut) {
  double jd = unixToJd(t);
  double T = (jd - 2451545.0) / 36525.0;
  double L = 280.460 + 36000.770 * T;
  double M = 357.528 + 35999.050 * T;
  wrap360(L);
  wrap360(M);
  double Mr = M * 0.01745329252;
  double lam = L + 1.915 * sin(Mr) + 0.020 * sin(2 * Mr);
  double eps = 23.439 - 0.013 * T;
  double lr = lam * 0.01745329252, er = eps * 0.01745329252;
  double ra = atan2(cos(er) * sin(lr), cos(lr));
  double dec = asin(sin(er) * sin(lr));
  int az, el;
  azelFromRaDec(ra, dec, t, &az, &el);
  if (altOut) *altOut = (float)el;
}

void planetRaDec(double L, double er, double *ra, double *dec) {
  double lr = L * 0.01745329252;
  *ra = atan2(cos(er) * sin(lr), cos(lr));
  *dec = asin(sin(er) * sin(lr));
}

void helioLam(double L, double a, double Ls, double *lam) {
  double lr = L * 0.01745329252, sr = Ls * 0.01745329252;
  double x = a * cos(lr) + cos(sr);
  double y = a * sin(lr) + sin(sr);
  double t = atan2(y, x) * 57.2957795;
  if (t < 0) t += 360;
  *lam = t;
}

void bestNightAzEl(double ra, double dec, const time_t *s, int n, int *az, int *el) {
  *el = -99;
  *az = 0;
  for (int i = 0; i < n; i++) {
    int a, e;
    azelFromRaDec(ra, dec, s[i], &a, &e);
    if (e > *el) {
      *el = e;
      *az = a;
    }
  }
}

const char *lookWord(int az) {
  static const char *d[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return d[((az + 22) / 45) & 7];
}

void skyHours(double ra, double dec, time_t now, char *hrs) {
  char rs[12] = "", ss[12] = "";
  int prev = -99;
  int step = 0;
  for (int m = 0; m <= 24 * 60; m += 20) {
    if ((step++ & 15) == 0) delay(1);
    int az, el;
    azelFromRaDec(ra, dec, now + m * 60, &az, &el);
    struct tm ti;
    time_t t = now + (time_t)m * 60;
    localtime_r(&t, &ti);
    char hm[16];
    fmtHm(ti.tm_hour, ti.tm_min, hm, sizeof(hm));
    if (prev < 0 && el >= 0 && !rs[0]) strlcpy(rs, hm, sizeof(rs));
    if (prev >= 0 && el < 0 && !ss[0]) strlcpy(ss, hm, sizeof(ss));
    prev = el;
  }
  if (rs[0] && ss[0]) snprintf(hrs, 36, "up %s  down %s", rs, ss);
  else strlcpy(hrs, "tonight", 36);
}

int skyCandCmp(const void *a, const void *b) {
  return ((const SkyCand *)b)->score - ((const SkyCand *)a)->score;
}

void addSky(const char *name, int az, int el, const char *note, const char *hours) {
  if (nSkyVis >= 18) return;
  SkyObj &o = skyVis[nSkyVis++];
  strlcpy(o.name, name, sizeof(o.name));
  strlcpy(o.look, lookWord(az), sizeof(o.look));
  strlcpy(o.note, note, sizeof(o.note));
  strlcpy(o.hours, hours ? hours : "", sizeof(o.hours));
  o.az = az;
  o.el = el;
}

void pushSky(SkyCand *c, int *n, const char *name, const char *note, int az, int el, int score,
             const char *hrs) {
  if (*n >= 16 || el < 8) return;
  SkyCand &o = c[(*n)++];
  strlcpy(o.name, name, sizeof(o.name));
  strlcpy(o.note, note, sizeof(o.note));
  strlcpy(o.hours, hrs, sizeof(o.hours));
  o.az = az;
  o.el = el;
  o.score = score + el / 2;
}

float moonFrac() {
  time_t now;
  if (!liveClock(&now)) return -1;
  double days = ((double)now - 947182440.0) / 86400.0;
  double syn = 29.53058867;
  return (float)(days / syn - floor(days / syn));
}

const char *moonName(float *phaseOut, int *pctOut) {
  float p = moonFrac();
  if (p < 0) return nullptr;
  if (phaseOut) *phaseOut = p;
  int pct = (int)(50.0 * (1.0 - cos(2.0 * M_PI * p)) + 0.5);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  if (pctOut) *pctOut = pct;
  if (p < 0.03f || p > 0.97f) return "new";
  if (p < 0.22f) return "crescent";
  if (p < 0.28f) return "first quarter";
  if (p < 0.47f) return "gibbous";
  if (p < 0.53f) return "full";
  if (p < 0.72f) return "gibbous";
  if (p < 0.78f) return "last quarter";
  return "crescent";
}

void computeAstro() {
  nSkyVis = 0;
  time_t now;
  if (!liveClock(&now) || !placeSet) return;
  float sunNow = 0;
  sunAlt(now, &sunNow);
  time_t t0 = (sunNow > -6) ? now + 4 * 3600 : now;
  time_t samples[4];
  for (int i = 0; i < 4; i++) samples[i] = t0 + (time_t)i * 7200;
  double jd = unixToJd(t0);
  double T = (jd - 2451545.0) / 36525.0;
  double er = 23.439 * 0.01745329252;
  double Ls = 280.460 + 36000.770 * T;
  wrap360(Ls);
  char hrs[36];
  double Lp = 218.316 + 481267.881 * T;
  double Mp = 134.963 + 477198.868 * T;
  wrap360(Lp);
  wrap360(Mp);
  double mlam = Lp + 6.289 * sin(Mp * 0.01745329252);
  double ra, dec;
  planetRaDec(mlam, er, &ra, &dec);
  int az, el;
  bestNightAzEl(ra, dec, samples, 4, &az, &el);
  const char *phase = moonName(nullptr, nullptr);
  int moonAz = az, moonEl = el;
  char moonHrs[36] = "";
  bool moonUp = phase && strcmp(phase, "new") && el >= 5;
  if (moonUp) skyHours(ra, dec, now, moonHrs);
  struct P {
    const char *n;
    double L0, Ld, a;
  };
  const P ps[] = {{"Mercury", 252.25, 4.09233, 0.387}, {"Venus", 181.98, 1.60213, 0.723},
                  {"Mars", 355.43, 0.52402, 1.524},    {"Jupiter", 34.35, 0.08309, 5.203},
                  {"Saturn", 50.08, 0.03346, 9.537},   {"Uranus", 314.05, 0.01173, 19.22},
                  {"Neptune", 304.35, 0.00602, 30.07}};
  for (int i = 0; i < 7; i++) {
    double L = ps[i].L0 + ps[i].Ld * (jd - 2451545.0);
    wrap360(L);
    double lam;
    helioLam(L, ps[i].a, Ls, &lam);
    double elong = fabs(lam - Ls);
    if (elong > 180) elong = 360 - elong;
    if (elong < 8) continue;
    double raP, decP;
    planetRaDec(lam, er, &raP, &decP);
    bestNightAzEl(raP, decP, samples, 4, &az, &el);
    if (el < 5) continue;
    skyHours(raP, decP, now, hrs);
    addSky(ps[i].n, az, el, "planet", hrs);
  }
  if (moonUp) addSky("Moon", moonAz, moonEl, phase, moonHrs);
  struct Dso {
    const char *n;
    const char *note;
    double raH, decD;
    int score, minEl;
  };
  const Dso ds[] = {{"M45", "cluster", 3.792, 24.12, 76, 15}, {"M42", "nebula", 5.588, -5.39, 80, 15},
                    {"Albireo", "double", 19.512, 27.96, 78, 18}, {"M13", "cluster", 16.695, 36.46, 73, 20},
                    {"M31", "galaxy", 0.712, 41.27, 71, 18},       {"Sirius", "star", 6.752, -16.72, 88, 10},
                    {"Vega", "star", 18.615, 38.78, 70, 8},        {"Arcturus", "star", 14.261, 19.18, 70, 8},
                    {"Betelgeuse", "star", 5.919, 7.41, 68, 8},    {"Polaris", "star", 2.530, 89.26, 60, 8}};
  for (int i = 0; i < 10; i++) {
    double raD = ds[i].raH * 15.0 * 0.01745329252;
    double decD = ds[i].decD * 0.01745329252;
    bestNightAzEl(raD, decD, samples, 4, &az, &el);
    if (el < ds[i].minEl) continue;
    skyHours(raD, decD, now, hrs);
    addSky(ds[i].n, az, el, ds[i].note, hrs);
  }
  lastAstro = millis();
}

void llToAzEl(float lat, float lon, float altKm, int *az, int *el) {
  float dLat = (lat - placeLat) * 0.0174533f;
  float dLon = (lon - placeLon) * 0.0174533f;
  float rlat = placeLat * 0.0174533f;
  float a = sinf(dLat / 2) * sinf(dLat / 2) +
            cosf(rlat) * cosf(lat * 0.0174533f) * sinf(dLon / 2) * sinf(dLon / 2);
  float c = 2 * atan2f(sqrtf(a), sqrtf(1 - a));
  float elev = atan2f((6371.0f + altKm) * cosf(c) - 6371.0f, (6371.0f + altKm) * sinf(c));
  *el = (int)lroundf(elev * 57.2958f);
  float y = sinf(dLon) * cosf(lat * 0.0174533f);
  float x = cosf(rlat) * sinf(lat * 0.0174533f) - sinf(rlat) * cosf(lat * 0.0174533f) * cosf(dLon);
  int brg = (int)(atan2f(y, x) * 57.2958f);
  if (brg < 0) brg += 360;
  *az = brg;
}

bool issPosAt(time_t t, float *lat, float *lon, float *altKm) {
  if (!issTle.ok) return false;
  double dt = unixToJd(t) - issTle.epochJd;
  double n = issTle.n * 2.0 * M_PI;
  double M = issTle.m0 * 0.01745329252 + n * dt;
  double e = issTle.e;
  double E = M;
  for (int i = 0; i < 7; i++) E = M + e * sin(E);
  double nu = 2 * atan2(sqrt(1 + e) * sin(E / 2), sqrt(1 - e) * cos(E / 2));
  double ns = issTle.n * 2.0 * M_PI / 86400.0;
  double a = pow(398600.4418 / (ns * ns), 1.0 / 3.0);
  double r = a * (1 - e * cos(E));
  double inc = issTle.inc * 0.01745329252;
  double cosi = cos(inc);
  double raanDot = -1.5 * ns * 1.08263e-3 * pow(6378.137 / a, 2) * cosi / pow(1 - e * e, 2);
  double raan = issTle.raan * 0.01745329252 + raanDot * dt * 86400.0;
  double arg = issTle.argp * 0.01745329252 + nu;
  double x = r * cos(arg), y = r * sin(arg);
  double ecix = x * cos(raan) - y * cosi * sin(raan);
  double eciy = x * sin(raan) + y * cosi * cos(raan);
  double eciz = y * sin(inc);
  double gmst = 280.46061837 + 360.98564736629 * (unixToJd(t) - 2451545.0);
  wrap360(gmst);
  double g = gmst * 0.01745329252;
  double ex = ecix * cos(g) + eciy * sin(g);
  double ey = -ecix * sin(g) + eciy * cos(g);
  *lon = (float)(atan2(ey, ex) * 57.2957795);
  *lat = (float)(atan2(eciz, sqrt(ex * ex + ey * ey)) * 57.2957795);
  *altKm = (float)(sqrt(ex * ex + ey * ey + eciz * eciz) - 6371.0);
  return true;
}

void buildIssOrbitPath() {
  nIssTrack = 0;
  if (!issTle.ok) return;
  time_t now;
  if (!liveClock(&now)) return;
  double period = 86400.0 / issTle.n;
  for (int i = 0; i < 72; i++) {
    float la, lo, al;
    time_t t = now + (time_t)(period * i / 72.0);
    if (!issPosAt(t, &la, &lo, &al)) break;
    issTrackLat[nIssTrack] = la;
    issTrackLon[nIssTrack] = lo;
    nIssTrack++;
  }
}

bool issViewAt(time_t t, int *az, int *el) {
  float la, lo, al;
  if (!issPosAt(t, &la, &lo, &al)) return false;
  llToAzEl(la, lo, al, az, el);
  float sunA = 0;
  sunAlt(t, &sunA);
  return *el >= 10 && sunA < -4;
}

void fmtIssDur(int sec, char *out, int n) {
  if (sec < 0) sec = 0;
  int m = sec / 60;
  int s = sec % 60;
  if (m && s)
    snprintf(out, n, "%d min %d sec", m, s);
  else if (m)
    snprintf(out, n, "%d min", m);
  else
    snprintf(out, n, "%d sec", s);
}

void recordIssPass(time_t start, time_t end, const char *from) {
  if (nIssPass >= 5) return;
  int dur = (int)(end - start);
  if (dur < 30) return;
  struct tm ti;
  localtime_r(&start, &ti);
  char durS[18];
  fmtIssDur(dur, durS, sizeof(durS));
  IssPass &P = issPasses[nIssPass++];
  P.when = start;
  P.durSec = dur;
  strlcpy(P.from, from, sizeof(P.from));
  snprintf(P.line, sizeof(P.line), "%02d/%02d %02d:%02d  from %s  %s", ti.tm_mon + 1, ti.tm_mday,
           ti.tm_hour, ti.tm_min, P.from, durS);
}

void closeIssPass(time_t start, time_t lastVis, const char *from) {
  int az = 0, el = 0;
  time_t t0 = start;
  char rise[4];
  strlcpy(rise, from, sizeof(rise));
  while (t0 > start - 480) {
    if (!issViewAt(t0 - 10, &az, &el)) break;
    t0 -= 10;
    strlcpy(rise, lookWord(az), sizeof(rise));
  }
  time_t t1 = lastVis;
  int steps = 0;
  while (t1 < lastVis + 3600 && issViewAt(t1 + 10, &az, &el)) {
    t1 += 10;
    if ((++steps & 15) == 0) vTaskDelay(1);
  }
  recordIssPass(t0, t1, rise);
}

void buildIssPasses() {
  nIssPass = 0;
  issNextPass = 0;
  time_t now;
  if (!liveClock(&now) || !issTle.ok) return;
  strlcpy(issLook, lookWord(issAz), sizeof(issLook));
  bool inPass = false;
  time_t start = 0, lastVis = 0;
  char riseD[4] = "";
  for (int s = 0; s < 10 * 24 * 60 && nIssPass < 5; s += 1) {
    if ((s & 31) == 0) vTaskDelay(pdMS_TO_TICKS(1));
    time_t t = now + (time_t)s * 60;
    int az = 0, el = 0;
    bool vis = issViewAt(t, &az, &el);
    if (vis && !inPass) {
      inPass = true;
      start = t;
      lastVis = t;
      strlcpy(riseD, lookWord(az), sizeof(riseD));
    } else if (vis && inPass) {
      lastVis = t;
    } else if (!vis && inPass) {
      closeIssPass(start, lastVis, riseD);
      inPass = false;
      s += 8;
    }
  }
  if (inPass) closeIssPass(start, lastVis, riseD);
  if (nIssPass) issNextPass = issPasses[0].when;
  lastPassAt = millis();
}

fs::FS *issStore = nullptr;

bool issFlashReady() {
  if (issStore) return true;
  if (LittleFS.begin(false)) {
    issStore = &LittleFS;
    return true;
  }
  if (FFat.begin(false)) {
    issStore = &FFat;
    return true;
  }
  return false;
}

bool issJpgFromFlash(uint8_t **buf, int *got) {
  *buf = nullptr;
  *got = 0;
  if (!issFlashReady() || !issStore->exists("/issmap.jpg")) return false;
  File f = issStore->open("/issmap.jpg", "r");
  if (!f) return false;
  int n = (int)f.size();
  if (n < 800 || n > 200000) {
    f.close();
    return false;
  }
  uint8_t *p = (uint8_t *)ps_malloc((size_t)n);
  if (!p) p = (uint8_t *)malloc((size_t)n);
  if (!p) {
    f.close();
    return false;
  }
  int r = f.read(p, (size_t)n);
  f.close();
  if (r != n) {
    free(p);
    return false;
  }
  *buf = p;
  *got = n;
  return true;
}

void issJpgToFlash(uint8_t *buf, int got) {
  if (!buf || got < 800 || got > 200000 || !issFlashReady()) return;
  File f = issStore->open("/issmap.jpg", "w");
  if (!f) return;
  f.write(buf, (size_t)got);
  f.close();
}

void issJpgDropFlash() {
  if (issFlashReady() && issStore->exists("/issmap.jpg")) issStore->remove("/issmap.jpg");
}

bool fetchIssMap() {
  lastIssMapTry = millis();
  if (!issJpg) issJpg = (uint16_t *)ps_malloc((size_t)ISS_SRC_W * ISS_JPG_H * 2);
  if (!issFb) issFb = (uint16_t *)ps_malloc((size_t)PW * ISS_H * 2);
  if (!issJpg || !issFb) return false;
  const uint16_t SENT = 0x0001;
  for (int i = 0; i < ISS_SRC_W * ISS_JPG_H; i++) issJpg[i] = SENT;
  uint8_t *owned = nullptr;
  int got = 0;
  bool fromFile = issJpgFromFlash(&owned, &got);
  const uint8_t *jpg = owned;
  if (!fromFile || got < 800 || got > 200000 || !jpg || jpg[0] != 0xFF || jpg[1] != 0xD8) {
    if (owned) free(owned);
    if (fromFile) issJpgDropFlash();
    owned = nullptr;
    jpg = ISS_WORLD_JPG;
    got = (int)ISS_WORLD_JPG_LEN;
  }
  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(issJpgPlot);
  TJpgDec.setSwapBytes(false);
  bool ok = TJpgDec.drawJpg(0, 0, jpg, (uint32_t)got) == 0;
  if (!ok) {
    TJpgDec.setSwapBytes(true);
    ok = TJpgDec.drawJpg(0, 0, jpg, (uint32_t)got) == 0;
  }
  TJpgDec.setCallback(jpgPlot);
  TJpgDec.setSwapBytes(false);
  if (owned) free(owned);
  if (!ok) {
    if (fromFile) issJpgDropFlash();
    return false;
  }
  int filled = 0;
  for (int i = 0; i < ISS_SRC_W * ISS_SRC_H; i++) {
    int x = i % ISS_SRC_W;
    int y = i / ISS_SRC_W;
    if (issJpg[y * ISS_SRC_W + x] != SENT) filled++;
  }
  if (filled < (ISS_SRC_W * ISS_SRC_H) / 20) {
    if (fromFile) issJpgDropFlash();
    return false;
  }
  for (int y = 0; y < ISS_H; y++) {
    int sy = y * ISS_SRC_H / ISS_H;
    if (sy >= ISS_SRC_H) sy = ISS_SRC_H - 1;
    for (int x = 0; x < PW; x++) {
      int sx = x * ISS_SRC_W / PW;
      if (sx >= ISS_SRC_W) sx = ISS_SRC_W - 1;
      issFb[y * PW + x] = issJpg[sy * ISS_SRC_W + sx];
    }
    if ((y & 31) == 31) vTaskDelay(1);
  }
  issMapOk = true;
  return true;
}

void pullIss() {
  if (!placeSet) return;
  if (!issMapOk && millis() - lastIssMapTry > 600000UL) fetchIssMap();
  char body[700];
  if (!issTle.ok || !lastIssPass || millis() - lastIssPass > 21600000UL) {
    if (httpGetBuf("https://celestrak.org/NORAD/elements/gp.php?CATNR=25544&FORMAT=JSON", body,
                   sizeof(body), "application/json")) {
      char ep[32];
      const char *ek = strstr(body, "\"EPOCH\":");
      int y, mo, d, h, mi;
      float se = 0;
      if (ek && copyQuoted(ek + 8, body + strlen(body), ep, sizeof(ep), nullptr) &&
          sscanf(ep, "%d-%d-%dT%d:%d:%f", &y, &mo, &d, &h, &mi, &se) >= 5) {
        if (mo <= 2) {
          y--;
          mo += 12;
        }
        int A = y / 100;
        int B = 2 - A + A / 4;
        double jd = floor(365.25 * (y + 4716)) + floor(30.6001 * (mo + 1)) + d + B - 1524.5 +
                    (h + mi / 60.0 + se / 3600.0) / 24.0;
        float n = 0, e = 0, inc = 0, raan = 0, argp = 0, m0 = 0;
        jsonNum(body, "\"MEAN_MOTION\":", &n);
        jsonNum(body, "\"ECCENTRICITY\":", &e);
        jsonNum(body, "\"INCLINATION\":", &inc);
        jsonNum(body, "\"RA_OF_ASC_NODE\":", &raan);
        jsonNum(body, "\"ARG_OF_PERICENTER\":", &argp);
        jsonNum(body, "\"MEAN_ANOMALY\":", &m0);
        if (n > 1) {
          issTle.epochJd = jd;
          issTle.n = n;
          issTle.e = e;
          issTle.inc = inc;
          issTle.raan = raan;
          issTle.argp = argp;
          issTle.m0 = m0;
          issTle.ok = true;
        }
      }
    }
  }
  float la = 0, lo = 0, al = 0;
  bool live = false;
  if (httpGetBuf("https://api.wheretheiss.at/v1/satellites/25544", body, sizeof(body), "application/json")) {
    jsonNum(body, "\"latitude\":", &la);
    jsonNum(body, "\"longitude\":", &lo);
    jsonNum(body, "\"altitude\":", &al);
    live = true;
  } else if (issTle.ok) {
    time_t now;
    if (liveClock(&now)) live = issPosAt(now, &la, &lo, &al);
  }
  if (live) {
    issLat = la;
    issLon = lo;
    issAltKm = al;
    llToAzEl(la, lo, al, &issAz, &issEl);
    strlcpy(issLook, lookWord(issAz), sizeof(issLook));
    float sun = 0;
    time_t now;
    if (liveClock(&now)) sunAlt(now, &sun);
    issVisNow = issEl >= 10 && sun < -4;
    issOk = true;
  }
  issWhen[0] = 0;
  if (issTle.ok && placeSet && liveClock(nullptr)) {
    time_t now = time(nullptr);
    bool needGap = issVisNow;
    for (int i = 2; i < 18 * 30; i++) {
      if ((i & 15) == 0) vTaskDelay(1);
      time_t t = now + (time_t)i * 120;
      float pla, plo, pal;
      if (!issPosAt(t, &pla, &plo, &pal)) break;
      int az, el;
      llToAzEl(pla, plo, pal, &az, &el);
      float sun = 0;
      sunAlt(t, &sun);
      bool vis = el >= 10 && sun < -4.0f;
      if (!vis) needGap = false;
      if (vis && !needGap) {
        struct tm tm;
        localtime_r(&t, &tm);
        char hm[16];
        fmtHm(tm.tm_hour, tm.tm_min, hm, sizeof(hm));
        snprintf(issWhen, sizeof(issWhen), "%s %s  LOOK %s", WDAY[tm.tm_wday], hm, lookWord(az));
        if (!issVisNow) {
          issAz = az;
          issEl = el;
        }
        break;
      }
    }
    lastIssPass = millis();
  }
  if (issTle.ok) {
    buildIssOrbitPath();
    buildIssPasses();
  }
  lastIss = millis();
}

time_t utcYmd(int y, int mo, int d, int h, int mi, int se) {
  if (mo <= 2) {
    y--;
    mo += 12;
  }
  int A = y / 100;
  int B = 2 - A + A / 4;
  double jd = floor(365.25 * (y + 4716)) + floor(30.6001 * (mo + 1)) + d + B - 1524.5 +
              (h + mi / 60.0 + se / 3600.0) / 24.0;
  return (time_t)((jd - 2440587.5) * 86400.0);
}

bool wordIs(const char *a, const char *b) {
  if (!a || !b) return false;
  while (*a && *b) {
    char ca = *a, cb = *b;
    if (ca >= 'A' && ca <= 'Z') ca = (char)(ca + 32);
    if (cb >= 'A' && cb <= 'Z') cb = (char)(cb + 32);
    if (ca != cb) return false;
    a++;
    b++;
  }
  return !*a && !*b;
}

bool launchJunk(const char *s) {
  return !s || !s[0] || wordIs(s, "minute") || wordIs(s, "hour") || wordIs(s, "second") ||
         wordIs(s, "min") || wordIs(s, "hr") || wordIs(s, "sec") || wordIs(s, "id") ||
         wordIs(s, "url") || wordIs(s, "name");
}

bool hasIgnoreCase(const char *hay, const char *needle) {
  if (!hay || !needle || !needle[0]) return false;
  int n = (int)strlen(needle);
  for (const char *p = hay; *p; p++) {
    int i = 0;
    while (i < n && p[i]) {
      char a = p[i];
      char b = needle[i];
      if (a >= 'A' && a <= 'Z') a = (char)(a + 32);
      if (b >= 'A' && b <= 'Z') b = (char)(b + 32);
      if (a != b) break;
      i++;
    }
    if (i == n) return true;
  }
  return false;
}

void launchSite(const char *loc, const char *pad, char *out, int n) {
  char blob[140];
  snprintf(blob, sizeof(blob), "%s %s", loc ? loc : "", pad ? pad : "");
  struct {
    const char *key;
    const char *name;
  } sites[] = {{"vandenberg", "Vandenberg"},
               {"cape canaveral", "Cape Canaveral"},
               {"canaveral", "Cape Canaveral"},
               {"kennedy", "Kennedy"},
               {"starbase", "Starbase"},
               {"boca chica", "Starbase"},
               {"wallops", "Wallops"},
               {"jiuquan", "Jiuquan"},
               {"taiyuan", "Taiyuan"},
               {"xichang", "Xichang"},
               {"wenchang", "Wenchang"},
               {"baikonur", "Baikonur"},
               {"plesetsk", "Plesetsk"},
               {"vostochny", "Vostochny"},
               {"kourou", "Kourou"},
               {"guiana", "Kourou"},
               {"mahia", "Mahia"},
               {"kodiak", "Kodiak"},
               {"pacific spaceport", "Kodiak"},
               {"tanegashima", "Tanegashima"},
               {"uchinoura", "Uchinoura"},
               {"sriharikota", "Sriharikota"},
               {"satish dhawan", "Sriharikota"},
               {"semnan", "Semnan"},
               {"alcantara", "Alcantara"},
               {"palmachim", "Palmachim"},
               {nullptr, nullptr}};
  for (int i = 0; sites[i].key; i++) {
    if (hasIgnoreCase(blob, sites[i].key)) {
      strlcpy(out, sites[i].name, n);
      return;
    }
  }
  const char *s = (loc && loc[0]) ? loc : pad;
  if (!s || !s[0] || launchJunk(s)) {
    out[0] = 0;
    return;
  }
  int i = 0;
  while (s[i] && s[i] != ',' && i < n - 1) {
    out[i] = s[i];
    i++;
  }
  out[i] = 0;
}

void pullLaunch() {
  lastLaunch = millis();
  if (!liveClock(nullptr)) return;
  char *buf = (char *)ps_malloc(48000);
  if (!buf) return;
  nLaunch = 0;
  if (!httpGetBuf("https://ll.thespacedevs.com/2.2.0/launch/upcoming/?limit=12&mode=list", buf, 48000,
                  "application/json")) {
    free(buf);
    return;
  }
  time_t now = time(nullptr);
  char *end = buf + strlen(buf);
  char *p = buf;
  while (nLaunch < 4 && (p = strstr(p, "\"net\":"))) {
    if (p >= end) break;
    char *lim = strstr(p + 6, "\"net\":");
    if (!lim || lim > end) lim = end;
    char net[32] = "", name[64] = "", locn[48] = "", padn[48] = "", lsp[32] = "";
    copyQuoted(p + 6, lim, net, sizeof(net), nullptr);
    const char *back = (p - buf > 400) ? p - 400 : buf;
    const char *lastName = nullptr;
    for (const char *q = back; q < p;) {
      const char *nk = strstr(q, "\"name\":");
      if (!nk || nk >= p) break;
      lastName = nk;
      q = nk + 7;
    }
    if (lastName) copyQuoted(lastName + 7, p, name, sizeof(name), nullptr);
    if (launchJunk(name)) {
      name[0] = 0;
      for (const char *q = p; q < lim;) {
        const char *nk = strstr(q, "\"name\":");
        if (!nk || nk >= lim) break;
        char tmp[64];
        if (copyQuoted(nk + 7, lim, tmp, sizeof(tmp), nullptr) && !launchJunk(tmp)) {
          strlcpy(name, tmp, sizeof(name));
          break;
        }
        q = nk + 7;
      }
    }
    const char *lk = strstr(p, "\"location\":");
    if (lk && lk < lim) {
      for (const char *q = lk; q < lim;) {
        const char *ln = strstr(q, "\"name\":");
        if (!ln || ln >= lim) break;
        char tmp[48];
        if (copyQuoted(ln + 7, lim, tmp, sizeof(tmp), nullptr) && !launchJunk(tmp)) {
          strlcpy(locn, tmp, sizeof(locn));
          break;
        }
        q = ln + 7;
      }
    }
    const char *pk = strstr(p, "\"pad\":");
    if (pk && pk < lim) {
      for (const char *q = pk; q < lim;) {
        const char *pn = strstr(q, "\"name\":");
        if (!pn || pn >= lim) break;
        char tmp[48];
        if (copyQuoted(pn + 7, lim, tmp, sizeof(tmp), nullptr) && !launchJunk(tmp)) {
          strlcpy(padn, tmp, sizeof(padn));
          break;
        }
        q = pn + 7;
      }
    }
    const char *ak = strstr(p, "\"lsp_name\":");
    if (ak && ak < lim) copyQuoted(ak + 11, lim, lsp, sizeof(lsp), nullptr);
    int y, mo, d, h, mi;
    float se = 0;
    if (sscanf(net, "%d-%d-%dT%d:%d:%f", &y, &mo, &d, &h, &mi, &se) >= 5) {
      time_t t = utcYmd(y, mo, d, h, mi, (int)se);
      if (t >= now - 3600) {
        struct tm loc;
        struct tm today;
        localtime_r(&t, &loc);
        localtime_r(&now, &today);
        int dayDiff = (loc.tm_yday + loc.tm_year * 366) - (today.tm_yday + today.tm_year * 366);
        Launch &L = launches[nLaunch];
        if (dayDiff <= 0)
          strlcpy(L.day, "TODAY", sizeof(L.day));
        else {
          static const char *moName[] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN",
                                         "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
          snprintf(L.day, sizeof(L.day), "%d %s", loc.tm_mday, moName[loc.tm_mon]);
        }
        snprintf(L.when, sizeof(L.when), "%02d:%02d", loc.tm_hour, loc.tm_min);
        const char *bar = strstr(name, " | ");
        if (bar) {
          int n = (int)(bar - name);
          if (n > 26) n = 26;
          memcpy(L.rocket, name, n);
          L.rocket[n] = 0;
        } else
          strlcpy(L.rocket, name, sizeof(L.rocket));
        if (launchJunk(locn)) locn[0] = 0;
        if (launchJunk(padn)) padn[0] = 0;
        launchSite(locn, padn, L.pad, sizeof(L.pad));
        strlcpy(L.agency, lsp, sizeof(L.agency));
        nLaunch++;
      }
    }
    p += 6;
  }
  free(buf);
  launchOk = true;
}

void askNet(int job) {
  if (!job) return;
  if (netBusy || netJob) {
    netPend = job;
    return;
  }
  netJob = job;
}

void netTask(void *pv) {
  (void)pv;
  for (;;) {
    int job = netJob;
    if (job) {
      netBusy = true;
      netJob = NET_NONE;
      if (job == NET_PACK && placeSet) pullPack(placeLat, placeLon);
      else if (job == NET_MAP) pullMap();
      else if (job == NET_FIND) pullFind();
      else if (job == NET_LAUNCH) pullLaunch();
      else if (job == NET_ISS) pullIss();
      netDoneJob = job;
      netDone = true;
      netBusy = false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void takeNetResult() {
  if (!netDone) return;
  netDone = false;
  int j = netDoneJob;
  if (j == NET_PACK && page == PG_HOME) drawHome();
  else if (j == NET_PACK && page == PG_RAIN) drawRainPage();
  else if (j == NET_PACK && page == PG_CLOCK) drawClockPage();
  else if (j == NET_PACK && page == PG_DAYS) drawDays();
  else if (j == NET_PACK && page == PG_HOURS) drawHours();
  else if (j == NET_PACK && page == PG_WARN) drawWarn();
  else if (j == NET_PACK && page == PG_STATION) drawStation();
  else if (j == NET_MAP) showMap();
  else if (j == NET_FIND && page == PG_PLACE) drawPlace();
  else if (j == NET_LAUNCH && (page == PG_LAUNCH || page == PG_SKY)) {
    if (page == PG_LAUNCH) drawLaunchPage();
    else drawSky();
  } else if (j == NET_ISS && (page == PG_ISS || page == PG_SKY)) {
    if (page == PG_ISS) drawIss();
    else drawSky();
  }
  if (netPend && !netBusy && !netJob) {
    int pend = netPend;
    netPend = NET_NONE;
    askNet(pend);
  }
}

void choosePlace(int idx) {
  if (idx < 0 || idx >= nHits) return;
  placeLat = hits[idx].lat;
  placeLon = hits[idx].lon;
  strlcpy(placeName, hits[idx].label, sizeof(placeName));
  placeSet = true;
  placeTz[0] = 0;
  wxOk = false;
  wxAqi = -1;
  alertLive = alertOn = false;
  alertEvent[0] = alertEnds[0] = 0;
  tideId[0] = tideName[0] = tideHigh[0] = tideLow[0] = 0;
  tideKnown = false;
  extremeYmd = 0;
  dayHot = dayCold = 0;
  pressBase = 0;
  pressTrend = 0;
  pressHpa = 0;
  strlcpy(alertNote, "waiting", sizeof(alertNote));
  savePrefs();
  drawHome();
  askNet(NET_PACK);
}

void startWifiScan() {
  nWifiScan = 0;
  wifiScanTop = 0;
  wifiScanBusy = true;
  strlcpy(wifiMsg, "scanning", sizeof(wifiMsg));
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);
  WiFi.scanDelete();
  WiFi.scanNetworks(true, false);
}

void pollWifiScan() {
  if (!wifiScanBusy) return;
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) return;
  wifiScanBusy = false;
  if (n < 0) n = 0;
  nWifiScan = 0;
  for (int i = 0; i < n && nWifiScan < 16; i++) {
    String s = WiFi.SSID(i);
    if (!s.length()) continue;
    strlcpy(wifiScan[nWifiScan], s.c_str(), 33);
    nWifiScan++;
  }
  WiFi.scanDelete();
  wifiScanTop = 0;
  if (nWifiScan) wifiMsg[0] = 0;
  else strlcpy(wifiMsg, "none found  type name", sizeof(wifiMsg));
  if (page == PG_WIFI) drawWifi();
}

void goHome() { drawHome(); }

void goBack() {
  if (page == PG_NIGHT || page == PG_LAUNCH || page == PG_ISS || page == PG_CLOCK) drawSky();
  else drawHome();
}

void onTap(int x, int y) {
  if (page != PG_HOME && hit(x, y, 652, 8, 792, 44) && page != PG_WIFI && page != PG_PLACE) {
    goHome();
    return;
  }
  if (page != PG_HOME && page != PG_WIFI && page != PG_PLACE && hit(x, y, 8, 8, 148, 44)) {
    goBack();
    return;
  }
  if (page == PG_HOME) {
    int b = footAt(x, y);
    if (b == 0) drawPlace();
    else if (b == 1) {
      drawRadar();
      askNet(NET_MAP);
    } else if (b == 2) drawRainPage();
    else if (b == 3) drawSky();
    else if (b == 4) drawWifi();
    else if (b == 5) drawSet();
    else if (hit(x, y, 8, 46, 792, 78)) drawWarn();
    else if (hit(x, y, 300, 246, 792, 290)) drawStation();
    else if (hit(x, y, 8, 220, 292, 290)) drawDays();
    else if (hit(x, y, 536, 298, 792, 412)) drawHours();
    return;
  }
  if (page == PG_SET) {
    if (hit(x, y, 160, 78, 340, 122)) {
      dayMode = false;
      cols();
      savePrefs();
      drawSet();
    } else if (hit(x, y, 360, 78, 540, 122)) {
      dayMode = true;
      cols();
      savePrefs();
      drawSet();
    } else if (hit(x, y, 160, 148, 300, 192)) {
      blLevel = 0;
      applyBl();
      savePrefs();
      drawSet();
    } else if (hit(x, y, 320, 148, 460, 192)) {
      blLevel = 1;
      applyBl();
      savePrefs();
      drawSet();
    } else if (hit(x, y, 480, 148, 660, 192)) {
      blLevel = 2;
      applyBl();
      savePrefs();
      drawSet();
    } else if (hit(x, y, 160, 424, 340, 468)) {
      useC = false;
      savePrefs();
      drawSet();
    } else if (hit(x, y, 360, 424, 540, 468)) {
      useC = true;
      savePrefs();
      drawSet();
    } else if (hit(x, y, 160, 240, 660, 292)) {
      factoryReset();
      delay(200);
      ESP.restart();
    } else if (hit(x, y, 160, 360, 660, 412))
      drawWifi();
    return;
  }
  if (page == PG_SKY) {
    if (hit(x, y, 20, 64, 390, 248)) drawClockPage();
    else if (hit(x, y, 410, 64, 780, 248)) {
      drawLaunchPage();
      if (!launchOk) askNet(NET_LAUNCH);
    } else if (hit(x, y, 20, 264, 390, 448)) {
      nightPage = 0;
      drawNight();
    }
    else if (hit(x, y, 410, 264, 780, 448)) {
      drawIss();
      askNet(NET_ISS);
    }
    return;
  }
  if (page == PG_NIGHT) {
    if (hit(x, y, 470, 8, 630, 44)) {
      int nPlanet = 0, nOther = 0;
      for (int i = 0; i < nSkyVis; i++) {
        if (!strcmp(skyVis[i].note, "planet")) nPlanet++;
        else nOther++;
      }
      if (nOther || nPlanet > 6) {
        nightPage = nightPage ? 0 : 1;
        drawNight();
      }
    }
    return;
  }
  if (page == PG_RADAR) {
    if (hit(x, y, 8, 8, 128, 44)) {
      goHome();
      return;
    }
    int next = -1;
    if (hit(x, y, 140, 8, 280, 44)) next = MAP_SAT;
    else if (hit(x, y, 292, 8, 452, 44)) next = MAP_CLOUD;
    else if (hit(x, y, 464, 8, 624, 44)) next = MAP_STORM;
    if (next >= 0 && next != mapKind) {
      mapKind = next;
      drawRadar();
      askNet(NET_MAP);
    }
    return;
  }
  if (page == PG_PLACE) {
    if (hit(x, y, 8, 8, 148, 44)) {
      goHome();
      return;
    }
    if (hit(x, y, 660, 8, 792, 44)) {
      strlcpy(placeMsg, "searching", sizeof(placeMsg));
      drawPlace();
      askNet(NET_FIND);
      return;
    }
    if (hit(x, y, 8, 100, 64, 188)) {
      if (placeTop >= 2) placeTop -= 2;
      drawPlace();
      return;
    }
    if (hit(x, y, 736, 100, 792, 188)) {
      if (placeTop + 2 < nHits) placeTop += 2;
      drawPlace();
      return;
    }
    if (hit(x, y, 72, 100, 728, 144)) {
      choosePlace(placeTop);
      return;
    }
    if (hit(x, y, 72, 148, 728, 192)) {
      choosePlace(placeTop + 1);
      return;
    }
    keyDest = placeQuery;
    keyCap = (int)sizeof(placeQuery);
    keyShift = &placeShift;
    keySym = &placeSym;
    if (hitKeys(x, y)) drawPlace();
    return;
  }
  if (page != PG_WIFI) return;
  if (hit(x, y, 8, 8, 148, 44)) {
    goHome();
    return;
  }
  if (hit(x, y, 660, 8, 792, 44)) {
    startWifiScan();
    drawWifi();
    return;
  }
  if (hit(x, y, 8, 52, 64, 88)) {
    if (wifiScanTop > 0) wifiScanTop--;
    drawWifi();
    return;
  }
  if (hit(x, y, 736, 52, 792, 88)) {
    if (wifiScanTop + 2 < nWifiScan) wifiScanTop++;
    drawWifi();
    return;
  }
  for (int i = 0; i < 2; i++) {
    int idx = wifiScanTop + i;
    if (idx < nWifiScan && hit(x, y, 72 + i * 328, 52, 392 + i * 328, 88)) {
      strlcpy(wifiSsid, wifiScan[idx], sizeof(wifiSsid));
      wifiFocus = 1;
      drawWifi();
      return;
    }
  }
  if (hit(x, y, 120, 100, 784, 140)) {
    wifiFocus = 0;
    drawWifi();
    return;
  }
  if (hit(x, y, 120, 148, 784, 188)) {
    wifiFocus = 1;
    drawWifi();
    return;
  }
  if (hit(x, y, 8, 436, 260, 472)) {
    savePrefs();
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(wifiSsid, wifiPass);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) delay(50);
    wifiOk = WiFi.status() == WL_CONNECTED;
    if (wifiOk) {
      configTime(0, 0, "pool.ntp.org", "time.nist.gov");
      applyPlaceTz();
      drawHome();
      if (placeSet) askNet(NET_PACK);
    } else {
      strlcpy(wifiMsg, "no join", sizeof(wifiMsg));
      drawWifi();
    }
    return;
  }
  if (hit(x, y, 274, 436, 526, 472)) {
    wifiSsid[0] = wifiPass[0] = 0;
    WiFi.disconnect(true, true);
    savePrefs();
    drawWifi();
    return;
  }
  if (hit(x, y, 540, 436, 792, 472)) {
    factoryReset();
    delay(200);
    ESP.restart();
    return;
  }
  bindWifiKeys();
  if (hitKeys(x, y)) drawWifi();
}

int gt911Touches(int *xs, int *ys, int maxN) {
  Wire.beginTransmission(GT911_ADDR);
  Wire.write(0x81);
  Wire.write(0x4E);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom((uint8_t)GT911_ADDR, (uint8_t)1) != 1) return 0;
  uint8_t st = Wire.read();
  if ((st & 0x80) == 0) return 0;
  int n = st & 0x0F;
  if (n == 0) {
    Wire.beginTransmission(GT911_ADDR);
    Wire.write(0x81);
    Wire.write(0x4E);
    Wire.write(0);
    Wire.endTransmission();
    return 0;
  }
  if (n > maxN) n = maxN;
  Wire.beginTransmission(GT911_ADDR);
  Wire.write(0x81);
  Wire.write(0x50);
  if (Wire.endTransmission(false) != 0) return 0;
  uint8_t want = (uint8_t)(n * 8);
  if (Wire.requestFrom((uint8_t)GT911_ADDR, want) != want) {
    Wire.beginTransmission(GT911_ADDR);
    Wire.write(0x81);
    Wire.write(0x4E);
    Wire.write(0);
    Wire.endTransmission();
    return 0;
  }
  uint8_t p[16];
  for (int i = 0; i < want; i++) p[i] = Wire.read();
  Wire.beginTransmission(GT911_ADDR);
  Wire.write(0x81);
  Wire.write(0x4E);
  Wire.write(0);
  Wire.endTransmission();
  for (int i = 0; i < n; i++) {
    int rx = p[i * 8] | (p[i * 8 + 1] << 8);
    int ry = p[i * 8 + 2] | (p[i * 8 + 3] << 8);
    xs[i] = constrain(rx, 0, PW - 1);
    ys[i] = constrain(ry, 0, PH - 1);
  }
  return n;
}

void connectWifi() {
  loadPrefs();
  if (!wifiSsid[0]) {
    drawWifi();
    return;
  }
  gfx->fillScreen(COL_BG);
  gfx->setTextColor(INK, COL_BG);
  gfx->setTextSize(2);
  centre("linking", 400, 220);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(wifiSsid, wifiPass);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(50);
  wifiOk = WiFi.status() == WL_CONNECTED;
  drawHome();
  if (!wifiOk) return;
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  applyPlaceTz();
  if (placeSet) askNet(NET_PACK);
}

void setup() {
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
  Serial.begin(115200);
  pinMode(TOUCH_RST, OUTPUT);
  digitalWrite(TOUCH_RST, LOW);
  delay(10);
  digitalWrite(TOUCH_RST, HIGH);
  delay(50);
  rgbBus = new Arduino_ESP32RGBPanel(41, 40, 39, 42, 14, 21, 47, 48, 45, 9, 46, 3, 8, 16, 1, 15, 7,
                                    6, 5, 4, 0, 180, 30, 16, 0, 12, 13, 10, 1, 12000000, false, 0, 0,
                                    800 * 10);
  gfx = new Arduino_RGB_Display(PW, PH, rgbBus, 0, true);
  if (!gfx->begin()) {
    digitalWrite(GFX_BL, HIGH);
    Serial.println("gfx begin failed");
    return;
  }
  digitalWrite(GFX_BL, HIGH);
  gfx->setTextWrap(false);
  Wire.begin(TOUCH_SDA, TOUCH_SCL);
  cols();
  applyBl();
  xTaskCreatePinnedToCore(netTask, "skynet", 40960, nullptr, 1, nullptr, 0);
  connectWifi();
  lastTap = millis();
}

void loop() {
  pollWifiScan();
  int x, y;
  int nTouch = gt911Touches(&x, &y, 1);
  if (nTouch == 1 && millis() - lastTap > 160) {
    lastTap = millis();
    onTap(x, y);
  }
  takeNetResult();
  wifiOk = WiFi.status() == WL_CONNECTED;
  if (page == PG_HOME && wifiOk) {
    int ymd = localYmd();
    if (ymd && extremeYmd && ymd != extremeYmd && wxOk) {
      noteTemp(wxTemp);
      drawHome();
    }
    time_t now;
    if (liveClock(&now)) {
      struct tm t;
      localtime_r(&now, &t);
      int m = t.tm_hour * 60 + t.tm_min;
      if (m != shownMin) drawClockCorner();
    }
  }
  if (wifiOk && placeSet && !netBusy && !netJob) {
    if (!lastPack || millis() - lastPack > 300000UL) askNet(NET_PACK);
    else if (page == PG_RADAR && millis() - lastMap > 600000UL) askNet(NET_MAP);
    else if (page == PG_ISS && millis() - lastIss > 60000UL) askNet(NET_ISS);
    else if (page == PG_LAUNCH && !launchOk && millis() - lastLaunch > 20000UL) askNet(NET_LAUNCH);
  }
  delay(8);
}
