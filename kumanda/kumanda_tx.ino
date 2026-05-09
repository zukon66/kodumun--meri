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
#define UI_WARN TFT_RED
#define UI_OK TFT_GREEN
#define UI_DIM TFT_DARKGREY

#define PIN_JOY_L_X 34
#define PIN_JOY_L_Y 35
#define PIN_JOY_R_X 32
#define PIN_JOY_R_Y 33
#define PIN_TX_BATT 36

#define PIN_ENC_A 25
#define PIN_ENC_B 26

#define PIN_RF_CE 4
#define PIN_RF_CSN 5

#define RF_CHANNEL 108
#define RF_PA_LEVEL RF24_PA_LOW
#define LINK_TIMEOUT_MS 1000UL
#define SEND_PERIOD_MS 20UL
#define UI_PERIOD_MS 50UL
#define POPUP_MS 1500UL
#define POWER_SLEEP_HOLD_MS 3000UL

const byte ROWS = 4;
const byte COLS = 4;
byte rowPins[ROWS] = {13, 12, 14, 27};
byte colPins[COLS] = {16, 17, 21, 22};
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

const byte txAddress[6] = "TX001";
const byte rxAddress[6] = "RX001";

enum FlightMode : uint8_t {
  MODE_TURTLE = 0,
  MODE_NORMAL = 1,
  MODE_SPORT = 2,
  MODE_ECO = 3
};

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
  int32_t gpsLatE7;
  int32_t gpsLonE7;
  int16_t offsetXm;
  int16_t offsetYm;
};

struct LocalInput {
  uint16_t lxRaw;
  uint16_t lyRaw;
  uint16_t rxRaw;
  uint16_t ryRaw;
  uint16_t txBattMv;
};

struct RemoteView {
  uint16_t aircraftMv;
  uint8_t satelliteCount;
  uint16_t speedKmh;
  int16_t altitudeM;
  uint16_t headingDeg;
  int16_t rollDeg10;
  int16_t pitchDeg10;
  int32_t gpsLatE7;
  int32_t gpsLonE7;
  int16_t offsetXm;
  int16_t offsetYm;
};

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

volatile long encoderCount = 0;
volatile uint8_t lastEncState = 0;

LocalInput localInput;
RemoteView remoteView;
TX_Payload txPayload;
RX_Payload rxAck;

bool spriteReady = false;
bool radioReady = false;
bool linkConnected = false;
bool safetyLock = true;
bool loggingEnabled = false;
bool rthEnabled = false;
bool findPlaneEnabled = false;
uint8_t lightMode = 0;
FlightMode activeMode = MODE_NORMAL;
uint16_t activeToggle = 0;
uint8_t currentPage = 0;
uint8_t lastPage = 255;

uint32_t lastSendMs = 0;
uint32_t lastUiMs = 0;
uint32_t lastAckTime = 0;
uint32_t popupUntil = 0;
uint32_t startMs = 0;
uint32_t packetSent = 0;
uint32_t packetLost = 0;
uint32_t key16StartMs = 0;
bool key16Held = false;
bool powerSleepArmed = false;

uint16_t maxSpeedKmh = 0;
int16_t maxAltitudeM = 0;
uint32_t totalDistanceM = 0;
int16_t lastPathX = 0;
int16_t lastPathY = 0;
bool hasLastPath = false;

struct Point2i {
  int16_t x;
  int16_t y;
};

Point2i pathBuffer[100];
uint8_t pathHead = 0;
uint8_t pathCount = 0;

char popupText[32] = "";

void IRAM_ATTR encoderIsr() {
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
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

uint16_t readBatteryMv(uint8_t pin) {
  uint32_t raw = analogRead(pin);
  return (uint16_t)((raw * 3300UL * 2UL) / 4095UL);
}

const char *modeName() {
  switch (activeMode) {
    case MODE_TURTLE: return "TURTLE";
    case MODE_NORMAL: return "NORMAL";
    case MODE_SPORT: return "SPORT";
    case MODE_ECO: return "ECO";
  }
  return "---";
}

void setPopup(const char *text) {
  strncpy(popupText, text, sizeof(popupText) - 1);
  popupText[sizeof(popupText) - 1] = '\0';
  popupUntil = millis() + POPUP_MS;
}

void resetRemoteView() {
  memset(&remoteView, 0, sizeof(remoteView));
}

void copyAckToRemote() {
  remoteView.aircraftMv = rxAck.aircraftMv;
  remoteView.satelliteCount = rxAck.satelliteCount;
  remoteView.speedKmh = rxAck.speedKmh;
  remoteView.altitudeM = rxAck.altitudeM;
  remoteView.headingDeg = rxAck.headingDeg;
  remoteView.rollDeg10 = rxAck.rollDeg10;
  remoteView.pitchDeg10 = rxAck.pitchDeg10;
  remoteView.gpsLatE7 = rxAck.gpsLatE7;
  remoteView.gpsLonE7 = rxAck.gpsLonE7;
  remoteView.offsetXm = rxAck.offsetXm;
  remoteView.offsetYm = rxAck.offsetYm;
}

void updateStatsAndPath() {
  if (!linkConnected) {
    hasLastPath = false;
    return;
  }

  if (remoteView.speedKmh > maxSpeedKmh) {
    maxSpeedKmh = remoteView.speedKmh;
  }
  if (remoteView.altitudeM > maxAltitudeM) {
    maxAltitudeM = remoteView.altitudeM;
  }

  if (hasLastPath) {
    int32_t dx = remoteView.offsetXm - lastPathX;
    int32_t dy = remoteView.offsetYm - lastPathY;
    uint32_t step = (uint32_t)sqrt((float)(dx * dx + dy * dy));
    if (step < 200) {
      totalDistanceM += step;
    }
  }
  lastPathX = remoteView.offsetXm;
  lastPathY = remoteView.offsetYm;
  hasLastPath = true;

  pathBuffer[pathHead].x = remoteView.offsetXm;
  pathBuffer[pathHead].y = remoteView.offsetYm;
  pathHead = (pathHead + 1) % 100;
  if (pathCount < 100) {
    pathCount++;
  }
}

void readInputs() {
  localInput.lxRaw = analogRead(PIN_JOY_L_X);
  localInput.lyRaw = analogRead(PIN_JOY_L_Y);
  localInput.rxRaw = analogRead(PIN_JOY_R_X);
  localInput.ryRaw = analogRead(PIN_JOY_R_Y);
  localInput.txBattMv = readBatteryMv(PIN_TX_BATT);

  txPayload.joyX1 = adcToPulse(localInput.lxRaw);
  txPayload.joyY1 = adcToPulse(localInput.lyRaw);
  txPayload.joyX2 = adcToPulse(localInput.rxRaw);
  txPayload.joyY2 = adcToPulse(localInput.ryRaw);
  txPayload.activeMode = activeMode;
  txPayload.activeToggle = activeToggle;
}

void sendRadioPacket() {
  if (!radioReady) {
    linkConnected = false;
    resetRemoteView();
    return;
  }

  packetSent++;
  bool ok = radio.write(&txPayload, sizeof(txPayload));
  if (!ok) {
    packetLost++;
  }

  if (radio.isAckPayloadAvailable()) {
    radio.read(&rxAck, sizeof(rxAck));
    lastAckTime = millis();
    copyAckToRemote();
  }

  linkConnected = (millis() - lastAckTime) <= LINK_TIMEOUT_MS;
  if (!linkConnected) {
    resetRemoteView();
  }
}

void nextPage() {
  currentPage = (currentPage + 1) % 4;
  lastPage = 255;
}

uint8_t keyToId(char key) {
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
  }
  return 0;
}

void handleKeyPress(uint8_t keyId) {
  switch (keyId) {
    case 1:
      activeMode = MODE_TURTLE;
      setPopup("TURTLE MODE 50 ESC");
      break;
    case 2:
      activeMode = MODE_NORMAL;
      setPopup("NORMAL MODE 75 ESC");
      break;
    case 3:
      activeMode = MODE_SPORT;
      setPopup("SPORT MODE NO LIMIT");
      break;
    case 4:
      activeMode = MODE_ECO;
      setPopup("ECO MODE");
      break;
    case 5:
      lightMode = (lightMode + 1) % 3;
      setPopup(lightMode == 0 ? "LIGHTS OFF" : (lightMode == 1 ? "LIGHTS ON" : "LIGHTS FLASH"));
      break;
    case 6:
      rthEnabled = !rthEnabled;
      setPopup("RTH ACTIVE");
      break;
    case 7:
      setPopup("HOME SAVED");
      break;
    case 8:
      setPopup("CALIBRATE IMU");
      break;
    case 9:
      findPlaneEnabled = !findPlaneEnabled;
      setPopup("FIND PLANE");
      break;
    case 10:
      setPopup("ALT ZERO SET");
      break;
    case 11:
      setPopup("TRIM SAVED");
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
      setPopup(loggingEnabled ? "LOG START" : "LOG STOP");
      break;
    case 16:
      key16StartMs = millis();
      key16Held = true;
      powerSleepArmed = false;
      setPopup("HOLD FOR SLEEP");
      break;
  }
  activeToggle++;
}

void enterPowerSleep() {
  clearCanvas();
  drawText(70, 104, "POWER SLEEP", POWDER_BLUE, 3);
  pushCanvas();
  tft.writecommand(TFT_DISPOFF);
  esp_sleep_enable_timer_wakeup(60000000ULL);
  esp_light_sleep_start();
  tft.writecommand(TFT_DISPON);
  lastPage = 255;
  setPopup("WAKE");
}

void handleKeypad() {
  char key = keypad.getKey();
  if (key) {
    uint8_t keyId = keyToId(key);
    if (keyId > 0) {
      handleKeyPress(keyId);
    }
  }

  if (key16Held) {
    KeyState state = keypad.getState();
    if (state == HOLD && !powerSleepArmed && (millis() - key16StartMs >= POWER_SLEEP_HOLD_MS)) {
      powerSleepArmed = true;
      setPopup("POWER SLEEP");
      enterPowerSleep();
    }
    if (state == RELEASED || state == IDLE) {
      key16Held = false;
    }
  }
}

TFT_eSprite *canvas() {
  return spriteReady ? &sprite : nullptr;
}

void clearCanvas() {
  if (spriteReady) {
    sprite.fillSprite(UI_BG);
  } else {
    tft.fillScreen(UI_BG);
  }
}

void pushCanvas() {
  if (spriteReady) {
    sprite.pushSprite(0, 0);
  }
}

void drawText(int16_t x, int16_t y, const char *text, uint16_t color, uint8_t size) {
  if (spriteReady) {
    sprite.setTextColor(color, UI_BG);
    sprite.setTextSize(size);
    sprite.drawString(text, x, y);
  } else {
    tft.setTextColor(color, UI_BG);
    tft.setTextSize(size);
    tft.drawString(text, x, y);
  }
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

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  if (spriteReady) {
    sprite.fillRect(x, y, w, h, color);
  } else {
    tft.fillRect(x, y, w, h, color);
  }
}

void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
  if (spriteReady) {
    sprite.drawLine(x0, y0, x1, y1, color);
  } else {
    tft.drawLine(x0, y0, x1, y1, color);
  }
}

void drawCircle(int16_t x, int16_t y, int16_t r, uint16_t color) {
  if (spriteReady) {
    sprite.drawCircle(x, y, r, color);
  } else {
    tft.drawCircle(x, y, r, color);
  }
}

void fillCircle(int16_t x, int16_t y, int16_t r, uint16_t color) {
  if (spriteReady) {
    sprite.fillCircle(x, y, r, color);
  } else {
    tft.fillCircle(x, y, r, color);
  }
}

void drawCenteredText(int16_t x, int16_t y, int16_t w, const char *text, uint16_t color, uint8_t size) {
  int16_t textW = strlen(text) * 6 * size;
  int16_t tx = x + (w - textW) / 2;
  drawText(tx, y, text, color, size);
}

void drawValueBox(int16_t x, int16_t y, int16_t w, int16_t h, const char *label, const char *value, uint16_t valueColor, uint8_t size) {
  drawBox(x, y, w, h, label);
  drawCenteredText(x, y + h - (8 * size) - 5, w, value, valueColor, size);
}

void drawJoystickBox(int16_t x, int16_t y, int16_t w, int16_t h, const char *label, uint16_t rawX, uint16_t rawY) {
  drawBox(x, y, w, h, label);
  int16_t cx = x + w / 2;
  int16_t cy = y + h / 2 + 4;
  int16_t radius = min(w, h) / 2 - 9;
  int16_t px = map(rawX, 0, 4095, cx - radius, cx + radius);
  int16_t py = map(rawY, 0, 4095, cy + radius, cy - radius);
  drawCircle(cx, cy, radius, UI_DIM);
  drawLine(cx - radius, cy, cx + radius, cy, UI_DIM);
  drawLine(cx, cy - radius, cx, cy + radius, UI_DIM);
  fillCircle(px, py, 3, POWDER_BLUE);
}

void drawThrottleBar(int16_t x, int16_t y, int16_t w, int16_t h) {
  drawBox(x, y, w, h, "THR");
  int16_t barH = h - 22;
  int16_t fillH = map(localInput.lyRaw, 0, 4095, 0, barH);
  fillRect(x + 12, y + 16, w - 24, barH, UI_DIM);
  fillRect(x + 12, y + 16 + barH - fillH, w - 24, fillH, POWDER_BLUE);
}

void drawDirectionIcon(int16_t x, int16_t y, int16_t w, int16_t h) {
  drawBox(x, y, w, h, "DIR");
  int16_t cx = x + w / 2;
  int16_t cy = y + h / 2 + 5;
  int16_t dx = map(localInput.rxRaw, 0, 4095, -18, 18);
  int16_t dy = map(localInput.ryRaw, 0, 4095, 18, -18);
  drawLine(cx, cy, cx + dx, cy + dy, POWDER_BLUE);
  fillCircle(cx + dx, cy + dy, 4, POWDER_BLUE);
}

void drawLevelBox(int16_t x, int16_t y, int16_t w, int16_t h) {
  drawBox(x, y, w, h, "ROLL PITCH");
  int16_t cx = x + w / 2;
  int16_t cy = y + h / 2 + 4;
  int16_t roll = linkConnected ? remoteView.rollDeg10 / 10 : 0;
  int16_t pitch = linkConnected ? remoteView.pitchDeg10 / 10 : 0;
  int16_t offset = constrain(pitch, -30, 30);
  int16_t half = w / 2 - 9;
  float angle = roll * DEG_TO_RAD;
  int16_t x0 = cx - (int16_t)(cos(angle) * half);
  int16_t y0 = cy - offset - (int16_t)(sin(angle) * half);
  int16_t x1 = cx + (int16_t)(cos(angle) * half);
  int16_t y1 = cy - offset + (int16_t)(sin(angle) * half);
  drawLine(x + 5, cy, x + w - 5, cy, UI_DIM);
  drawLine(x0, y0, x1, y1, POWDER_BLUE);
}

void drawCompassStrip(int16_t x, int16_t y, int16_t w, int16_t h) {
  drawBox(x, y, w, h, "COMPASS");
  char buf[24];
  if (linkConnected) {
    snprintf(buf, sizeof(buf), "%03u DEG", remoteView.headingDeg % 360);
  } else {
    snprintf(buf, sizeof(buf), "---");
  }
  drawCenteredText(x, y + 24, w, buf, UI_TEXT, 2);
  drawLine(x + 8, y + h - 10, x + w - 8, y + h - 10, POWDER_BLUE);
}

void drawSignalBox(int16_t x, int16_t y, int16_t w, int16_t h) {
  drawBox(x, y, w, h, "SIGNAL");
  drawCenteredText(x, y + 17, w, linkConnected ? "LINK OK" : "NO CONN", linkConnected ? UI_OK : UI_WARN, 1);
}

void drawPopup() {
  if (millis() > popupUntil || popupText[0] == '\0') {
    return;
  }
  int16_t x = 35;
  int16_t y = 88;
  int16_t w = 250;
  int16_t h = 54;
  if (spriteReady) {
    sprite.fillRoundRect(x, y, w, h, 8, TFT_NAVY);
    sprite.drawRoundRect(x, y, w, h, 8, POWDER_BLUE);
  } else {
    tft.fillRoundRect(x, y, w, h, 8, TFT_NAVY);
    tft.drawRoundRect(x, y, w, h, 8, POWDER_BLUE);
  }
  drawCenteredText(x, y + 18, w, popupText, UI_TEXT, 2);
}

void drawPage0() {
  char buf[32];
  int w5 = 64;
  drawValueBox(0, 0, w5, 30, "SAT", linkConnected ? itoa(remoteView.satelliteCount, buf, 10) : "0", UI_TEXT, 1);
  snprintf(buf, sizeof(buf), "%umV", linkConnected ? remoteView.aircraftMv : 0);
  drawValueBox(64, 0, w5, 30, "AIR BAT", buf, UI_TEXT, 1);
  snprintf(buf, sizeof(buf), "%umV", localInput.txBattMv);
  drawValueBox(128, 0, w5, 30, "TX BAT", buf, UI_TEXT, 1);
  drawValueBox(192, 0, w5, 30, "MODE", modeName(), POWDER_BLUE, 1);
  drawSignalBox(256, 0, w5, 30);

  drawThrottleBar(0, 30, 53, 90);
  drawDirectionIcon(53, 30, 53, 90);
  drawValueBox(106, 30, 53, 90, "LOCK", safetyLock ? "ON" : "OFF", safetyLock ? UI_WARN : UI_OK, 2);
  drawLevelBox(159, 30, 107, 90);
  snprintf(buf, sizeof(buf), "%ld", encoderCount);
  drawValueBox(266, 30, 54, 90, "CAM", buf, POWDER_BLUE, 1);

  snprintf(buf, sizeof(buf), "%dm", linkConnected ? remoteView.altitudeM : 0);
  drawValueBox(0, 120, 96, 50, "ALT", buf, UI_TEXT, 2);
  snprintf(buf, sizeof(buf), "%u", linkConnected ? remoteView.speedKmh : 0);
  drawValueBox(96, 120, 128, 50, "KMH", buf, POWDER_BLUE, 3);
  snprintf(buf, sizeof(buf), "%lum", (unsigned long)totalDistanceM);
  drawValueBox(224, 120, 96, 50, "DIST", buf, UI_TEXT, 1);

  drawJoystickBox(0, 170, 70, 70, "L JOY", localInput.lxRaw, localInput.lyRaw);
  if (linkConnected) {
    snprintf(buf, sizeof(buf), "%ld", (long)remoteView.gpsLatE7);
    drawValueBox(70, 170, 80, 70, "GPS LAT", buf, UI_TEXT, 1);
  } else {
    drawValueBox(70, 170, 80, 70, "GPS", "---", UI_TEXT, 2);
  }
  drawCompassStrip(150, 170, 100, 70);
  drawJoystickBox(250, 170, 70, 70, "R JOY", localInput.rxRaw, localInput.ryRaw);
}

void drawPage1() {
  drawBox(0, 0, SCREEN_W, SCREEN_H, "RADAR");
  int16_t cx = SCREEN_W / 2;
  int16_t cy = SCREEN_H / 2 + 8;
  drawCircle(cx, cy, 20, UI_DIM);
  drawCircle(cx, cy, 40, UI_DIM);
  drawCircle(cx, cy, 60, UI_DIM);
  drawLine(cx - 80, cy, cx + 80, cy, UI_DIM);
  drawLine(cx, cy - 80, cx, cy + 80, UI_DIM);
  fillCircle(cx, cy, 4, UI_OK);
  drawText(8, 18, linkConnected ? "HOME CENTER" : "NO CONN", linkConnected ? UI_TEXT : UI_WARN, 1);

  for (uint8_t i = 0; i < pathCount; i++) {
    uint8_t idx = (pathHead + 100 - pathCount + i) % 100;
    int16_t px = cx + pathBuffer[idx].x / 5;
    int16_t py = cy - pathBuffer[idx].y / 5;
    if (px > 2 && px < SCREEN_W - 2 && py > 20 && py < SCREEN_H - 2) {
      fillCircle(px, py, 1, POWDER_BLUE);
    }
  }

  if (linkConnected) {
    int16_t px = cx + remoteView.offsetXm / 5;
    int16_t py = cy - remoteView.offsetYm / 5;
    fillCircle(px, py, 5, TFT_YELLOW);
  }
}

void drawPage2() {
  char buf[48];
  drawBox(0, 0, SCREEN_W, SCREEN_H, "DEBUG");
  snprintf(buf, sizeof(buf), "JOY L RAW X:%u Y:%u", localInput.lxRaw, localInput.lyRaw);
  drawText(12, 28, buf, UI_TEXT, 2);
  snprintf(buf, sizeof(buf), "JOY R RAW X:%u Y:%u", localInput.rxRaw, localInput.ryRaw);
  drawText(12, 54, buf, UI_TEXT, 2);
  snprintf(buf, sizeof(buf), "ENCODER:%ld", encoderCount);
  drawText(12, 80, buf, UI_TEXT, 2);
  uint8_t lossPct = packetSent == 0 ? 0 : (uint8_t)((packetLost * 100UL) / packetSent);
  snprintf(buf, sizeof(buf), "NRF LOSS:%u%%", lossPct);
  drawText(12, 106, buf, linkConnected ? UI_TEXT : UI_WARN, 2);
  snprintf(buf, sizeof(buf), "AIR LIPO:%umV", linkConnected ? remoteView.aircraftMv : 0);
  drawText(12, 132, buf, UI_TEXT, 2);
  snprintf(buf, sizeof(buf), "TX LIPO:%umV", localInput.txBattMv);
  drawText(12, 158, buf, UI_TEXT, 2);
  snprintf(buf, sizeof(buf), "PAGE:%u RF:%s", currentPage, radioReady ? "READY" : "ERR");
  drawText(12, 184, buf, UI_TEXT, 2);
}

void drawPage3() {
  char buf[32];
  drawBox(0, 0, SCREEN_W, SCREEN_H, "STATS");
  snprintf(buf, sizeof(buf), "%u KMH", maxSpeedKmh);
  drawValueBox(15, 30, 290, 50, "MAX SPEED", buf, POWDER_BLUE, 3);
  snprintf(buf, sizeof(buf), "%dm", maxAltitudeM);
  drawValueBox(15, 92, 290, 50, "MAX ALT", buf, POWDER_BLUE, 3);
  uint32_t sec = (millis() - startMs) / 1000UL;
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", (unsigned long)(sec / 3600), (unsigned long)((sec / 60) % 60), (unsigned long)(sec % 60));
  drawValueBox(15, 154, 290, 50, "TOTAL TIME", buf, POWDER_BLUE, 3);
}

void drawUi() {
  if (lastPage != currentPage) {
    clearCanvas();
    lastPage = currentPage;
  } else {
    clearCanvas();
  }

  switch (currentPage) {
    case 0: drawPage0(); break;
    case 1: drawPage1(); break;
    case 2: drawPage2(); break;
    case 3: drawPage3(); break;
  }

  drawPopup();
  pushCanvas();
}

void initDisplay() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(UI_BG);
  sprite.setColorDepth(16);
  spriteReady = sprite.createSprite(SCREEN_W, SCREEN_H) != nullptr;
  clearCanvas();
  drawText(12, 12, "TX BOOT", POWDER_BLUE, 2);
  drawText(12, 36, spriteReady ? "SPRITE OK" : "SPRITE FAIL", spriteReady ? UI_OK : UI_WARN, 1);
  pushCanvas();
}

void initRadio() {
  radioReady = radio.begin();
  if (!radioReady) {
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
  setPopup("NRF READY");
}

void initEncoder() {
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  lastEncState = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderIsr, CHANGE);
}

void setup() {
  startMs = millis();
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  initDisplay();
  initEncoder();
  initRadio();
}

void loop() {
  uint32_t now = millis();
  readInputs();
  handleKeypad();

  if (now - lastSendMs >= SEND_PERIOD_MS) {
    lastSendMs = now;
    sendRadioPacket();
    updateStatsAndPath();
  } else {
    linkConnected = radioReady && ((now - lastAckTime) <= LINK_TIMEOUT_MS);
    if (!linkConnected) {
      resetRemoteView();
    }
  }

  if (now - lastUiMs >= UI_PERIOD_MS) {
    lastUiMs = now;
    drawUi();
  }
}
