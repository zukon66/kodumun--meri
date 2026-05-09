#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <RF24.h>
#include <Keypad.h>
#include <esp_sleep.h>

#define SCREEN_W 320
#define SCREEN_H 240
#define POWDER_BLUE tft.color565(176, 224, 230)
#define UI_BG TFT_BLACK
#define UI_TEXT TFT_WHITE
#define UI_DIM TFT_DARKGREY
#define UI_OK TFT_GREEN
#define UI_WARN TFT_RED
#define UI_YELLOW TFT_YELLOW

#define PIN_JOY_L_X 35
#define PIN_JOY_L_Y 34
#define PIN_JOY_R_X 39
#define PIN_JOY_R_Y 36
#define PIN_TX_BATT 12

#define PIN_ENC_A 16
#define PIN_ENC_B 17

#define PIN_RF_CE 22
#define PIN_RF_CSN 15

#define RF_SCK 18
#define RF_MISO 19
#define RF_MOSI 23
#define RF_CHANNEL 108
#define RF_PA_LEVEL RF24_PA_LOW

#define LINK_TIMEOUT_MS 1000UL
#define SEND_PERIOD_MS 20UL
#define UI_PERIOD_MS 100UL
#define POPUP_MS 1500UL
#define POWER_SLEEP_HOLD_MS 3000UL
#define PATH_LEN 100

const byte ROWS = 4;
const byte COLS = 4;
byte rowPins[ROWS] = {13, 14, 27, 26};
byte colPins[COLS] = {25, 33, 32, 21};
char keyMap[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

const byte txAddress[6] = "TX001";
const byte rxAddress[6] = "RX001";

struct TX_Payload {
  uint16_t joyX1;
  uint16_t joyY1;
  uint16_t joyX2;
  uint16_t joyY2;
  uint8_t activeMode;
  uint16_t activeToggle;
};

struct RX_Payload {
  uint16_t aircraftMv;
  uint8_t satelliteCount;
  uint16_t speedKmh;
  int16_t altitudeM;
  uint16_t headingDeg;
  int16_t rollDeg10;
  int16_t pitchDeg10;
};

struct LocalData {
  uint16_t joyLX;
  uint16_t joyLY;
  uint16_t joyRX;
  uint16_t joyRY;
  uint16_t txBattMv;
};

struct RemoteData {
  uint16_t aircraftMv;
  uint8_t satelliteCount;
  uint16_t speedKmh;
  int16_t altitudeM;
  uint16_t headingDeg;
  int16_t rollDeg10;
  int16_t pitchDeg10;
};

struct Point2i {
  int16_t x;
  int16_t y;
};

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
Keypad keypad = Keypad(makeKeymap(keyMap), rowPins, colPins, ROWS, COLS);

volatile long encoderCount = 0;
volatile uint8_t lastEncState = 0;

LocalData localData;
RemoteData remoteData;
TX_Payload txPayload;
RX_Payload rxPayload;

bool spriteReady = false;
bool radioReady = false;
bool linkConnected = false;
bool safetyLock = true;
bool loggingEnabled = false;
bool key16Armed = false;
bool key16Slept = false;

uint8_t activeMode = 2;
uint8_t lightMode = 0;
uint8_t currentPage = 0;
uint8_t lastPage = 255;
uint16_t activeToggle = 0;
uint16_t pendingToggle = 0;

uint32_t lastSendMs = 0;
uint32_t lastUiMs = 0;
uint32_t lastAckTime = 0;
uint32_t lastStatsMs = 0;
uint32_t popupUntil = 0;
uint32_t bootMs = 0;
uint32_t key16StartMs = 0;
uint32_t packetsSent = 0;
uint32_t packetsLost = 0;

uint16_t maxSpeedKmh = 0;
int16_t maxAltitudeM = 0;
float radarX = 0.0f;
float radarY = 0.0f;
Point2i path[PATH_LEN];
uint8_t pathHead = 0;
uint8_t pathCount = 0;

char popupText[36] = "";

void IRAM_ATTR encoderIsr() {
  uint8_t a = (uint8_t)digitalRead(PIN_ENC_A);
  uint8_t b = (uint8_t)digitalRead(PIN_ENC_B);
  uint8_t state = (a << 1) | b;
  uint8_t transition = (lastEncState << 2) | state;
  if (transition == 0b0001 || transition == 0b0111 || transition == 0b1110 || transition == 0b1000) {
    encoderCount++;
  } else if (transition == 0b0010 || transition == 0b1011 || transition == 0b1101 || transition == 0b0100) {
    encoderCount--;
  }
  lastEncState = state;
}

uint16_t adcToPulse(uint16_t raw) {
  return (uint16_t)map(constrain(raw, 0, 4095), 0, 4095, 1000, 2200);
}

uint16_t readTxBatteryMv() {
  uint32_t raw = analogRead(PIN_TX_BATT);
  uint32_t voutMv = (raw * 3300UL) / 4095UL;
  return (uint16_t)(voutMv * 2UL);
}

const char *modeText() {
  if (activeMode == 1) return "TURTLE";
  if (activeMode == 2) return "NORMAL";
  if (activeMode == 3) return "SPORT";
  if (activeMode == 4) return "ECO";
  return "---";
}

void setPopup(const char *text) {
  strncpy(popupText, text, sizeof(popupText) - 1);
  popupText[sizeof(popupText) - 1] = '\0';
  popupUntil = millis() + POPUP_MS;
}

void resetRemoteData() {
  memset(&remoteData, 0, sizeof(remoteData));
}

void copyRemoteData() {
  remoteData.aircraftMv = rxPayload.aircraftMv;
  remoteData.satelliteCount = rxPayload.satelliteCount;
  remoteData.speedKmh = rxPayload.speedKmh;
  remoteData.altitudeM = rxPayload.altitudeM;
  remoteData.headingDeg = rxPayload.headingDeg;
  remoteData.rollDeg10 = rxPayload.rollDeg10;
  remoteData.pitchDeg10 = rxPayload.pitchDeg10;
}

void readLocalInputs() {
  localData.joyLX = analogRead(PIN_JOY_L_X);
  localData.joyLY = analogRead(PIN_JOY_L_Y);
  localData.joyRX = analogRead(PIN_JOY_R_X);
  localData.joyRY = analogRead(PIN_JOY_R_Y);
  localData.txBattMv = readTxBatteryMv();
}

void buildTxPayload() {
  txPayload.joyX1 = adcToPulse(localData.joyLX);
  txPayload.joyY1 = adcToPulse(localData.joyLY);
  txPayload.joyX2 = adcToPulse(localData.joyRX);
  txPayload.joyY2 = adcToPulse(localData.joyRY);
  txPayload.activeMode = activeMode;
  txPayload.activeToggle = pendingToggle;
  activeToggle = pendingToggle;
  pendingToggle = 0;
}

void sendRadio() {
  buildTxPayload();
  if (!radioReady) {
    linkConnected = false;
    resetRemoteData();
    return;
  }

  packetsSent++;
  bool ok = radio.write(&txPayload, sizeof(txPayload));
  if (!ok) {
    packetsLost++;
  }

  if (radio.isAckPayloadAvailable()) {
    radio.read(&rxPayload, sizeof(rxPayload));
    lastAckTime = millis();
    copyRemoteData();
  }

  linkConnected = (millis() - lastAckTime) <= LINK_TIMEOUT_MS;
  if (!linkConnected) {
    resetRemoteData();
  }
}

uint8_t keyIdFromChar(char key) {
  switch (key) {
    case '1': return 1;
    case '2': return 2;
    case '3': return 3;
    case 'A': return 4;
    case '4': return 5;
    case '5': return 6;
    case '6': return 7;
    case 'B': return 8;
    case '7': return 9;
    case '8': return 10;
    case '9': return 11;
    case 'C': return 12;
    case '*': return 13;
    case '0': return 14;
    case '#': return 15;
    case 'D': return 16;
    default: return 0;
  }
}

void nextPage() {
  currentPage = (currentPage + 1) % 4;
  lastPage = 255;
}

void enterPowerSleep() {
  if (spriteReady) {
    sprite.fillSprite(UI_BG);
    sprite.setTextColor(POWDER_BLUE, UI_BG);
    sprite.setTextSize(3);
    sprite.drawString("POWER SLEEP", 56, 104);
    sprite.pushSprite(0, 0);
  } else {
    tft.fillScreen(UI_BG);
    tft.setTextColor(POWDER_BLUE, UI_BG);
    tft.setTextSize(3);
    tft.drawString("POWER SLEEP", 56, 104);
  }
  tft.writecommand(TFT_DISPOFF);
  esp_sleep_enable_timer_wakeup(60000000ULL);
  esp_light_sleep_start();
  tft.writecommand(TFT_DISPON);
  lastPage = 255;
  setPopup("WAKE");
}

void handleKeyPress(uint8_t keyId) {
  if (keyId == 0) {
    return;
  }

  if (keyId != 14 && keyId != 16) {
    pendingToggle = keyId;
  }

  switch (keyId) {
    case 1:
      activeMode = 1;
      setPopup("TURTLE MOD (%50 LIMIT)");
      break;
    case 2:
      activeMode = 2;
      setPopup("NORMAL MOD (%75 LIMIT)");
      break;
    case 3:
      activeMode = 3;
      setPopup("SPORT MOD (NO LIMIT)");
      break;
    case 4:
      activeMode = 4;
      setPopup("ECO MOD");
      break;
    case 5:
      lightMode = (lightMode + 1) % 3;
      setPopup("LIGHTS TOGGLE");
      break;
    case 6:
      setPopup("RTH AKTIF");
      break;
    case 7:
      setPopup("SAVE HOME");
      break;
    case 8:
      setPopup("CALIBRATE IMU");
      break;
    case 9:
      setPopup("FIND PLANE");
      break;
    case 10:
      setPopup("ZERO ALT");
      break;
    case 11:
      setPopup("SAVE TRIM");
      break;
    case 12:
      setPopup("LEVEL FLIGHT");
      break;
    case 13:
      safetyLock = !safetyLock;
      setPopup(safetyLock ? "SAFETY LOCK ON" : "SAFETY LOCK OFF");
      break;
    case 14:
      nextPage();
      setPopup("NEXT PAGE");
      break;
    case 15:
      loggingEnabled = !loggingEnabled;
      setPopup("LOG DATA");
      break;
    case 16:
      key16Armed = true;
      key16Slept = false;
      key16StartMs = millis();
      setPopup("HOLD FOR SLEEP");
      break;
  }
}

void handleKeypad() {
  char key = keypad.getKey();
  if (key) {
    handleKeyPress(keyIdFromChar(key));
  }

  if (key16Armed) {
    KeyState state = keypad.getState();
    if (state == HOLD && !key16Slept && millis() - key16StartMs >= POWER_SLEEP_HOLD_MS) {
      key16Slept = true;
      setPopup("POWER SLEEP");
      enterPowerSleep();
    }
    if (state == RELEASED || state == IDLE) {
      key16Armed = false;
    }
  }
}

void updateStatsAndRadar(uint32_t now) {
  if (lastStatsMs == 0) {
    lastStatsMs = now;
    return;
  }

  float dt = (float)(now - lastStatsMs) / 1000.0f;
  lastStatsMs = now;

  if (!linkConnected) {
    return;
  }

  if (remoteData.speedKmh > maxSpeedKmh) {
    maxSpeedKmh = remoteData.speedKmh;
  }
  if (remoteData.altitudeM > maxAltitudeM) {
    maxAltitudeM = remoteData.altitudeM;
  }

  float speedMs = (float)remoteData.speedKmh / 3.6f;
  float headingRad = (float)remoteData.headingDeg * DEG_TO_RAD;
  radarX += sin(headingRad) * speedMs * dt;
  radarY += cos(headingRad) * speedMs * dt;

  path[pathHead].x = (int16_t)constrain((int)radarX, -1600, 1600);
  path[pathHead].y = (int16_t)constrain((int)radarY, -1600, 1600);
  pathHead = (pathHead + 1) % PATH_LEN;
  if (pathCount < PATH_LEN) {
    pathCount++;
  }
}

void clearDraw() {
  if (spriteReady) {
    sprite.fillSprite(UI_BG);
  } else {
    tft.fillScreen(UI_BG);
  }
}

void pushDraw() {
  if (spriteReady) {
    sprite.pushSprite(0, 0);
  }
}

void setText(uint16_t color, uint8_t size) {
  if (spriteReady) {
    sprite.setTextColor(color, UI_BG);
    sprite.setTextSize(size);
  } else {
    tft.setTextColor(color, UI_BG);
    tft.setTextSize(size);
  }
}

void drawStringAt(int16_t x, int16_t y, const char *text, uint16_t color, uint8_t size) {
  setText(color, size);
  if (spriteReady) {
    sprite.drawString(text, x, y);
  } else {
    tft.drawString(text, x, y);
  }
}

void drawLineAt(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
  if (spriteReady) sprite.drawLine(x0, y0, x1, y1, color);
  else tft.drawLine(x0, y0, x1, y1, color);
}

void drawCircleAt(int16_t x, int16_t y, int16_t r, uint16_t color) {
  if (spriteReady) sprite.drawCircle(x, y, r, color);
  else tft.drawCircle(x, y, r, color);
}

void fillCircleAt(int16_t x, int16_t y, int16_t r, uint16_t color) {
  if (spriteReady) sprite.fillCircle(x, y, r, color);
  else tft.fillCircle(x, y, r, color);
}

void fillRectAt(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  if (spriteReady) sprite.fillRect(x, y, w, h, color);
  else tft.fillRect(x, y, w, h, color);
}

void drawBox(int16_t x, int16_t y, int16_t w, int16_t h, const char *label) {
  if (spriteReady) {
    sprite.drawRoundRect(x, y, w, h, 5, POWDER_BLUE);
    sprite.setTextColor(POWDER_BLUE, UI_BG);
    sprite.setTextSize(1);
    sprite.drawString(label, x + 4, y + 3);
  } else {
    tft.drawRoundRect(x, y, w, h, 5, POWDER_BLUE);
    tft.setTextColor(POWDER_BLUE, UI_BG);
    tft.setTextSize(1);
    tft.drawString(label, x + 4, y + 3);
  }
}

void drawCentered(int16_t x, int16_t y, int16_t w, const char *text, uint16_t color, uint8_t size) {
  int16_t textW = (int16_t)strlen(text) * 6 * size;
  int16_t tx = x + ((w - textW) / 2);
  drawStringAt(tx, y, text, color, size);
}

void drawValueBox(int16_t x, int16_t y, int16_t w, int16_t h, const char *label, const char *value, uint16_t color, uint8_t size) {
  drawBox(x, y, w, h, label);
  drawCentered(x, y + h - 8 * size - 5, w, value, color, size);
}

void drawJoyBox(int16_t x, int16_t y, int16_t w, int16_t h, const char *label, uint16_t rawX, uint16_t rawY) {
  drawBox(x, y, w, h, label);
  int16_t cx = x + w / 2;
  int16_t cy = y + h / 2 + 4;
  int16_t r = ((w < h) ? w : h) / 2 - 10;
  int16_t px = map(rawX, 0, 4095, cx - r, cx + r);
  int16_t py = map(rawY, 0, 4095, cy + r, cy - r);
  drawCircleAt(cx, cy, r, UI_DIM);
  drawLineAt(cx - r, cy, cx + r, cy, UI_DIM);
  drawLineAt(cx, cy - r, cx, cy + r, UI_DIM);
  fillCircleAt(px, py, 4, POWDER_BLUE);
}

void drawThrottle(int16_t x, int16_t y, int16_t w, int16_t h) {
  drawBox(x, y, w, h, "THR");
  int16_t barH = h - 24;
  int16_t fillH = map(localData.joyLY, 0, 4095, 0, barH);
  fillRectAt(x + 13, y + 17, w - 26, barH, UI_DIM);
  fillRectAt(x + 13, y + 17 + barH - fillH, w - 26, fillH, POWDER_BLUE);
}

void drawAttitude(int16_t x, int16_t y, int16_t w, int16_t h) {
  drawBox(x, y, w, h, "ROLL PITCH");
  int16_t cx = x + w / 2;
  int16_t cy = y + h / 2 + 5;
  int16_t roll = linkConnected ? remoteData.rollDeg10 / 10 : 0;
  int16_t pitch = linkConnected ? remoteData.pitchDeg10 / 10 : 0;
  int16_t half = w / 2 - 12;
  int16_t pOff = constrain(pitch, -25, 25);
  float a = roll * DEG_TO_RAD;
  int16_t x0 = cx - (int16_t)(cos(a) * half);
  int16_t y0 = cy - pOff - (int16_t)(sin(a) * half);
  int16_t x1 = cx + (int16_t)(cos(a) * half);
  int16_t y1 = cy - pOff + (int16_t)(sin(a) * half);
  drawLineAt(x + 7, cy, x + w - 7, cy, UI_DIM);
  drawLineAt(x0, y0, x1, y1, POWDER_BLUE);
}

void drawCompass(int16_t x, int16_t y, int16_t w, int16_t h) {
  char buf[20];
  drawBox(x, y, w, h, "COMPASS");
  if (linkConnected) snprintf(buf, sizeof(buf), "%03u DEG", remoteData.headingDeg % 360);
  else snprintf(buf, sizeof(buf), "---");
  drawCentered(x, y + 25, w, buf, UI_TEXT, 2);
  drawLineAt(x + 10, y + h - 11, x + w - 10, y + h - 11, POWDER_BLUE);
}

void drawPopup() {
  if (millis() > popupUntil || popupText[0] == '\0') {
    return;
  }
  int16_t x = 24;
  int16_t y = 88;
  int16_t w = 272;
  int16_t h = 55;
  if (spriteReady) {
    sprite.fillRoundRect(x, y, w, h, 7, TFT_NAVY);
    sprite.drawRoundRect(x, y, w, h, 7, POWDER_BLUE);
  } else {
    tft.fillRoundRect(x, y, w, h, 7, TFT_NAVY);
    tft.drawRoundRect(x, y, w, h, 7, POWDER_BLUE);
  }
  drawCentered(x, y + 19, w, popupText, UI_TEXT, 2);
}

void drawPage0() {
  char buf[32];
  char satBuf[8];

  snprintf(satBuf, sizeof(satBuf), "%u", linkConnected ? remoteData.satelliteCount : 0);
  drawValueBox(0, 0, 64, 30, "SAT", satBuf, UI_TEXT, 1);
  snprintf(buf, sizeof(buf), "%umV", linkConnected ? remoteData.aircraftMv : 0);
  drawValueBox(64, 0, 64, 30, "AIR BAT", buf, UI_TEXT, 1);
  snprintf(buf, sizeof(buf), "%umV", localData.txBattMv);
  drawValueBox(128, 0, 64, 30, "TX BAT", buf, UI_TEXT, 1);
  drawValueBox(192, 0, 64, 30, "MODE", modeText(), POWDER_BLUE, 1);
  drawValueBox(256, 0, 64, 30, "SIGNAL", linkConnected ? "LINK OK" : "NO CONN", linkConnected ? UI_OK : UI_WARN, 1);

  drawThrottle(0, 30, 53, 90);
  drawValueBox(53, 30, 53, 90, "YAW", localData.joyLX < 1850 ? "LEFT" : (localData.joyLX > 2250 ? "RIGHT" : "MID"), UI_TEXT, 1);
  drawValueBox(106, 30, 53, 90, "LOCK", safetyLock ? "ON" : "OFF", safetyLock ? UI_WARN : UI_OK, 2);
  drawAttitude(159, 30, 107, 90);
  snprintf(buf, sizeof(buf), "%ld", encoderCount);
  drawValueBox(266, 30, 54, 90, "ENC", buf, POWDER_BLUE, 1);

  snprintf(buf, sizeof(buf), "%dm", linkConnected ? remoteData.altitudeM : 0);
  drawValueBox(0, 120, 96, 50, "ALT", buf, UI_TEXT, 2);
  snprintf(buf, sizeof(buf), "%u", linkConnected ? remoteData.speedKmh : 0);
  drawValueBox(96, 120, 128, 50, "KMH", buf, POWDER_BLUE, 3);
  snprintf(buf, sizeof(buf), "%u", activeToggle);
  drawValueBox(224, 120, 96, 50, "TOGGLE", buf, UI_TEXT, 2);

  drawJoyBox(0, 170, 70, 70, "L JOY", localData.joyLX, localData.joyLY);
  drawValueBox(70, 170, 80, 70, "GPS", linkConnected ? "OK" : "---", linkConnected ? UI_OK : UI_TEXT, 2);
  drawCompass(150, 170, 100, 70);
  drawJoyBox(250, 170, 70, 70, "R JOY", localData.joyRX, localData.joyRY);
}

void drawPage1() {
  drawBox(0, 0, SCREEN_W, SCREEN_H, "RADAR");
  int16_t cx = 160;
  int16_t cy = 120;
  drawCircleAt(cx, cy, 20, UI_DIM);
  drawCircleAt(cx, cy, 40, UI_DIM);
  drawCircleAt(cx, cy, 60, UI_DIM);
  drawCircleAt(cx, cy, 80, UI_DIM);
  drawLineAt(cx - 90, cy, cx + 90, cy, UI_DIM);
  drawLineAt(cx, cy - 90, cx, cy + 90, UI_DIM);
  fillCircleAt(cx, cy, 4, UI_OK);
  drawStringAt(8, 18, linkConnected ? "HOME CENTER" : "NO CONN", linkConnected ? UI_TEXT : UI_WARN, 1);

  for (uint8_t i = 0; i < pathCount; i++) {
    uint8_t idx = (pathHead + PATH_LEN - pathCount + i) % PATH_LEN;
    int16_t px = cx + path[idx].x / 5;
    int16_t py = cy - path[idx].y / 5;
    if (px > 2 && px < SCREEN_W - 2 && py > 16 && py < SCREEN_H - 2) {
      fillCircleAt(px, py, 1, POWDER_BLUE);
    }
  }

  if (linkConnected) {
    int16_t px = cx + (int16_t)radarX / 5;
    int16_t py = cy - (int16_t)radarY / 5;
    fillCircleAt(px, py, 5, UI_YELLOW);
  }
}

void drawPage2() {
  char buf[48];
  drawBox(0, 0, SCREEN_W, SCREEN_H, "DEBUG");
  snprintf(buf, sizeof(buf), "JOY L RAW X:%u Y:%u", localData.joyLX, localData.joyLY);
  drawStringAt(12, 28, buf, UI_TEXT, 2);
  snprintf(buf, sizeof(buf), "JOY R RAW X:%u Y:%u", localData.joyRX, localData.joyRY);
  drawStringAt(12, 54, buf, UI_TEXT, 2);
  snprintf(buf, sizeof(buf), "ENCODER:%ld", encoderCount);
  drawStringAt(12, 80, buf, UI_TEXT, 2);
  uint8_t loss = packetsSent == 0 ? 0 : (uint8_t)((packetsLost * 100UL) / packetsSent);
  snprintf(buf, sizeof(buf), "NRF LOSS:%u%%", loss);
  drawStringAt(12, 106, buf, linkConnected ? UI_TEXT : UI_WARN, 2);
  snprintf(buf, sizeof(buf), "AIR LIPO:%umV", linkConnected ? remoteData.aircraftMv : 0);
  drawStringAt(12, 132, buf, UI_TEXT, 2);
  snprintf(buf, sizeof(buf), "TX LIPO:%umV", localData.txBattMv);
  drawStringAt(12, 158, buf, UI_TEXT, 2);
  snprintf(buf, sizeof(buf), "PAGE:%u RF:%s", currentPage, radioReady ? "READY" : "ERR");
  drawStringAt(12, 184, buf, UI_TEXT, 2);
}

void drawPage3() {
  char buf[32];
  drawBox(0, 0, SCREEN_W, SCREEN_H, "STATS");
  snprintf(buf, sizeof(buf), "%u KMH", maxSpeedKmh);
  drawValueBox(15, 32, 290, 50, "MAX SPEED", buf, POWDER_BLUE, 3);
  snprintf(buf, sizeof(buf), "%dm", maxAltitudeM);
  drawValueBox(15, 94, 290, 50, "MAX ALT", buf, POWDER_BLUE, 3);
  uint32_t sec = (millis() - bootMs) / 1000UL;
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", (unsigned long)(sec / 3600), (unsigned long)((sec / 60) % 60), (unsigned long)(sec % 60));
  drawValueBox(15, 156, 290, 50, "TOTAL TIME", buf, POWDER_BLUE, 3);
}

void drawUi() {
  if (lastPage != currentPage) {
    clearDraw();
    lastPage = currentPage;
  } else {
    clearDraw();
  }

  if (currentPage == 0) drawPage0();
  else if (currentPage == 1) drawPage1();
  else if (currentPage == 2) drawPage2();
  else drawPage3();

  drawPopup();
  pushDraw();
}

void initDisplay() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(UI_BG);
  sprite.setColorDepth(8);
  spriteReady = sprite.createSprite(SCREEN_W, SCREEN_H) != nullptr;
  clearDraw();
  drawStringAt(12, 12, "TX BOOT", POWDER_BLUE, 2);
  drawStringAt(12, 36, spriteReady ? "SPRITE 8BIT OK" : "SPRITE FAIL", spriteReady ? UI_OK : UI_WARN, 1);
  pushDraw();
}

void initRadio() {
  SPI.begin(RF_SCK, RF_MISO, RF_MOSI, PIN_RF_CSN);
  radioReady = radio.begin();
  if (!radioReady) {
    Serial.println("NRF INIT FAIL");
    setPopup("NRF INIT FAIL");
    return;
  }
  radio.setChannel(RF_CHANNEL);
  radio.setPALevel(RF_PA_LEVEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(3, 5);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openWritingPipe(rxAddress);
  radio.openReadingPipe(1, txAddress);
  radio.stopListening();
  Serial.println("NRF READY");
  setPopup("NRF READY");
}

void initEncoder() {
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  lastEncState = ((uint8_t)digitalRead(PIN_ENC_A) << 1) | (uint8_t)digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderIsr, CHANGE);
}

void setup() {
  Serial.begin(115200);
  bootMs = millis();
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_JOY_L_X, ADC_11db);
  analogSetPinAttenuation(PIN_JOY_L_Y, ADC_11db);
  analogSetPinAttenuation(PIN_JOY_R_X, ADC_11db);
  analogSetPinAttenuation(PIN_JOY_R_Y, ADC_11db);
  analogSetPinAttenuation(PIN_TX_BATT, ADC_11db);
  keypad.setHoldTime(POWER_SLEEP_HOLD_MS);
  initDisplay();
  initEncoder();
  initRadio();
  lastStatsMs = millis();
}

void loop() {
  uint32_t now = millis();
  readLocalInputs();
  handleKeypad();

  if (now - lastSendMs >= SEND_PERIOD_MS) {
    lastSendMs = now;
    sendRadio();
  } else {
    linkConnected = radioReady && ((now - lastAckTime) <= LINK_TIMEOUT_MS);
    if (!linkConnected) {
      resetRemoteData();
    }
  }

  updateStatsAndRadar(now);

  if (now - lastUiMs >= UI_PERIOD_MS) {
    lastUiMs = now;
    drawUi();
  }
}
