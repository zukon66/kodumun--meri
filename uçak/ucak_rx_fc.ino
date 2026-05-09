#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <TinyGPSPlus.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

#define PIN_GPS_RX 16
#define PIN_GPS_TX 17
#define GPS_BAUD 9600

#define PIN_RF_CE 4
#define PIN_RF_CSN 5
#define PIN_RF_SCK 18
#define PIN_RF_MOSI 23
#define PIN_RF_MISO 19

#define PIN_ESC 26
#define PIN_SERVO_AILERON 13
#define PIN_SERVO_ELEVATOR 14
#define PIN_SERVO_RUDDER 27
#define PIN_BUZZER 32
#define PIN_LIGHTS 25
#define PIN_BATTERY_ADC 34

#define PWM_MIN_US 1000
#define PWM_CENTER_US 1500
#define PWM_MAX_US 2200
#define MODE_TURTLE_LIMIT_US 1500
#define MODE_NORMAL_LIMIT_US 1800
#define MODE_SPORT_LIMIT_US 2200

#define RF_CHANNEL 108
#define RF_PA_LEVEL RF24_PA_LOW
#define LINK_TIMEOUT_MS 1000UL
#define TELEMETRY_PERIOD_MS 20UL
#define SENSOR_PERIOD_MS 20UL
#define BATTERY_PERIOD_MS 100UL
#define GPS_READ_BUDGET 96
#define STROBE_PERIOD_MS 160UL
#define FIND_BEEP_PERIOD_MS 500UL
#define FIND_BEEP_ON_MS 100UL
#define SOS_STEP_MS 180UL
#define HOME_SET_MIN_SATS 4

const byte txAddress[6] = "TX001";
const byte rxAddress[6] = "RX001";

struct RX_Command {
  uint16_t joyX1;
  uint16_t joyY1;
  uint16_t joyX2;
  uint16_t joyY2;
  uint8_t activeMode;
  uint16_t activeToggle;
};

struct TX_Telemetry {
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

enum LightState : uint8_t {
  LIGHT_OFF = 0,
  LIGHT_ON = 1,
  LIGHT_STROBE = 2
};

RF24 radio(PIN_RF_CE, PIN_RF_CSN);
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
Servo esc;
Servo servoAileron;
Servo servoElevator;
Servo servoRudder;

RX_Command commandIn = {PWM_CENTER_US, PWM_MIN_US, PWM_CENTER_US, PWM_CENTER_US, 1, 0};
TX_Telemetry telemetryOut = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

bool radioReady = false;
bool imuReady = false;
bool failsafeActive = true;
bool safetyLock = true;
bool findPlaneActive = false;
bool homeValid = false;
bool strobePhase = false;
bool buzzerPhase = false;
bool sosPhase = false;

LightState lightState = LIGHT_OFF;

uint16_t lastToggle = 0;
uint32_t lastCommandMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastSensorMs = 0;
uint32_t lastBatteryMs = 0;
uint32_t lastStrobeMs = 0;
uint32_t lastFindBeepMs = 0;
uint32_t lastSosMs = 0;

double homeLat = 0.0;
double homeLng = 0.0;

float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float gyroRollBias = 0.0f;
float gyroPitchBias = 0.0f;

uint16_t clampPulse(uint16_t value) {
  return constrain(value, PWM_MIN_US, PWM_MAX_US);
}

uint16_t throttleLimitForMode(uint8_t mode) {
  if (mode == 0 || mode == 1) {
    return MODE_TURTLE_LIMIT_US;
  }
  if (mode == 2) {
    return MODE_NORMAL_LIMIT_US;
  }
  if (mode == 3) {
    return MODE_SPORT_LIMIT_US;
  }
  return MODE_NORMAL_LIMIT_US;
}

uint16_t limitedThrottleUs() {
  uint16_t throttle = clampPulse(commandIn.joyY1);
  uint16_t limitUs = throttleLimitForMode(commandIn.activeMode);
  if (safetyLock) {
    return PWM_MIN_US;
  }
  return min(throttle, limitUs);
}

uint16_t lipoMillivolts() {
  uint32_t raw = analogRead(PIN_BATTERY_ADC);
  uint32_t voutMv = (raw * 3300UL) / 4095UL;
  return (uint16_t)(voutMv * 2UL);
}

void writeActuatorsFailsafe() {
  esc.writeMicroseconds(PWM_MIN_US);
  servoAileron.writeMicroseconds(PWM_CENTER_US);
  servoElevator.writeMicroseconds(PWM_CENTER_US);
  servoRudder.writeMicroseconds(PWM_CENTER_US);
}

void writeActuatorsCommanded() {
  esc.writeMicroseconds(limitedThrottleUs());
  servoAileron.writeMicroseconds(clampPulse(commandIn.joyX2));
  servoElevator.writeMicroseconds(clampPulse(commandIn.joyY2));
  servoRudder.writeMicroseconds(clampPulse(commandIn.joyX1));
}

void setHomeFromGps() {
  if (gps.location.isValid() && gps.satellites.value() >= HOME_SET_MIN_SATS) {
    homeLat = gps.location.lat();
    homeLng = gps.location.lng();
    homeValid = true;
  }
}

void processToggleCommand(uint16_t toggleValue) {
  if (toggleValue == lastToggle) {
    return;
  }
  lastToggle = toggleValue;

  uint16_t commandId = toggleValue;
  if (commandId == 0) {
    return;
  }

  switch (commandId) {
    case 1:
    case 5:
      lightState = (LightState)((lightState + 1) % 3);
      break;
    case 2:
    case 9:
      findPlaneActive = !findPlaneActive;
      break;
    case 3:
    case 13:
      safetyLock = !safetyLock;
      break;
    case 4:
    case 7:
      setHomeFromGps();
      break;
    default:
      if ((toggleValue % 5) == 0) {
        lightState = (LightState)((lightState + 1) % 3);
      }
      if ((toggleValue % 9) == 0) {
        findPlaneActive = !findPlaneActive;
      }
      if ((toggleValue % 13) == 0) {
        safetyLock = !safetyLock;
      }
      break;
  }
}

void readRadio() {
  if (!radioReady) {
    return;
  }

  if (radio.available()) {
    RX_Command freshCommand;
    radio.read(&freshCommand, sizeof(freshCommand));
    commandIn = freshCommand;
    lastCommandMs = millis();
    failsafeActive = false;
    processToggleCommand(commandIn.activeToggle);
    radio.writeAckPayload(1, &telemetryOut, sizeof(telemetryOut));
  }
}

void feedGps() {
  for (uint16_t i = 0; i < GPS_READ_BUDGET && Serial2.available() > 0; i++) {
    gps.encode((char)Serial2.read());
  }
}

void updateBattery() {
  telemetryOut.aircraftMv = lipoMillivolts();
}

void updateGpsTelemetry() {
  if (gps.satellites.isValid()) {
    telemetryOut.satelliteCount = (uint8_t)min((uint32_t)gps.satellites.value(), 255UL);
  } else {
    telemetryOut.satelliteCount = 0;
  }

  if (gps.speed.isValid()) {
    telemetryOut.speedKmh = (uint16_t)constrain((int)gps.speed.kmph(), 0, 65535);
  } else {
    telemetryOut.speedKmh = 0;
  }

  if (gps.altitude.isValid()) {
    telemetryOut.altitudeM = (int16_t)constrain((int)gps.altitude.meters(), -32768, 32767);
  } else {
    telemetryOut.altitudeM = 0;
  }

  if (gps.course.isValid()) {
    telemetryOut.headingDeg = (uint16_t)((int)gps.course.deg() % 360);
  } else {
    telemetryOut.headingDeg = 0;
  }

  if (gps.location.isValid()) {
    telemetryOut.gpsLatE7 = (int32_t)(gps.location.lat() * 10000000.0);
    telemetryOut.gpsLonE7 = (int32_t)(gps.location.lng() * 10000000.0);
    if (homeValid) {
      telemetryOut.offsetXm = (int16_t)constrain((int)TinyGPSPlus::distanceBetween(homeLat, homeLng, homeLat, gps.location.lng()), -32768, 32767);
      telemetryOut.offsetYm = (int16_t)constrain((int)TinyGPSPlus::distanceBetween(homeLat, homeLng, gps.location.lat(), homeLng), -32768, 32767);
      if (gps.location.lng() < homeLng) {
        telemetryOut.offsetXm = -telemetryOut.offsetXm;
      }
      if (gps.location.lat() < homeLat) {
        telemetryOut.offsetYm = -telemetryOut.offsetYm;
      }
    }
  } else {
    telemetryOut.gpsLatE7 = 0;
    telemetryOut.gpsLonE7 = 0;
    telemetryOut.offsetXm = 0;
    telemetryOut.offsetYm = 0;
  }
}

void updateImu() {
  if (!imuReady) {
    telemetryOut.rollDeg10 = 0;
    telemetryOut.pitchDeg10 = 0;
    return;
  }

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mpu.getEvent(&accel, &gyro, &temp);

  float accelRoll = atan2(accel.acceleration.y, accel.acceleration.z) * RAD_TO_DEG;
  float accelPitch = atan2(-accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * RAD_TO_DEG;

  static uint32_t lastImuUpdateMs = 0;
  uint32_t now = millis();
  float dt = lastImuUpdateMs == 0 ? 0.02f : (float)(now - lastImuUpdateMs) / 1000.0f;
  lastImuUpdateMs = now;
  dt = constrain(dt, 0.001f, 0.1f);

  rollDeg = 0.96f * (rollDeg + (gyro.gyro.x - gyroRollBias) * RAD_TO_DEG * dt) + 0.04f * accelRoll;
  pitchDeg = 0.96f * (pitchDeg + (gyro.gyro.y - gyroPitchBias) * RAD_TO_DEG * dt) + 0.04f * accelPitch;

  telemetryOut.rollDeg10 = (int16_t)constrain((int)(rollDeg * 10.0f), -32768, 32767);
  telemetryOut.pitchDeg10 = (int16_t)constrain((int)(pitchDeg * 10.0f), -32768, 32767);
}

void updateTelemetry() {
  updateGpsTelemetry();
  if (radioReady) {
    radio.writeAckPayload(1, &telemetryOut, sizeof(telemetryOut));
  }
}

void updateLights(uint32_t now) {
  if (failsafeActive) {
    digitalWrite(PIN_LIGHTS, HIGH);
    return;
  }

  if (lightState == LIGHT_OFF) {
    digitalWrite(PIN_LIGHTS, LOW);
  } else if (lightState == LIGHT_ON) {
    digitalWrite(PIN_LIGHTS, HIGH);
  } else {
    if (now - lastStrobeMs >= STROBE_PERIOD_MS) {
      lastStrobeMs = now;
      strobePhase = !strobePhase;
      digitalWrite(PIN_LIGHTS, strobePhase ? HIGH : LOW);
    }
  }
}

void updateBuzzer(uint32_t now) {
  if (failsafeActive) {
    if (now - lastSosMs >= SOS_STEP_MS) {
      lastSosMs = now;
      sosPhase = !sosPhase;
      digitalWrite(PIN_BUZZER, sosPhase ? HIGH : LOW);
    }
    return;
  }

  if (!findPlaneActive) {
    digitalWrite(PIN_BUZZER, LOW);
    return;
  }

  uint32_t span = now - lastFindBeepMs;
  if (span >= FIND_BEEP_PERIOD_MS) {
    lastFindBeepMs = now;
    span = 0;
  }
  digitalWrite(PIN_BUZZER, span < FIND_BEEP_ON_MS ? HIGH : LOW);
}

void updateFailsafe(uint32_t now) {
  if (now - lastCommandMs > LINK_TIMEOUT_MS) {
    failsafeActive = true;
  }

  if (failsafeActive) {
    writeActuatorsFailsafe();
  } else {
    writeActuatorsCommanded();
  }
}

void attachServoPwm() {
  esc.setPeriodHertz(50);
  servoAileron.setPeriodHertz(50);
  servoElevator.setPeriodHertz(50);
  servoRudder.setPeriodHertz(50);

  esc.attach(PIN_ESC, PWM_MIN_US, PWM_MAX_US);
  servoAileron.attach(PIN_SERVO_AILERON, PWM_MIN_US, PWM_MAX_US);
  servoElevator.attach(PIN_SERVO_ELEVATOR, PWM_MIN_US, PWM_MAX_US);
  servoRudder.attach(PIN_SERVO_RUDDER, PWM_MIN_US, PWM_MAX_US);
  writeActuatorsFailsafe();
}

void initRadio() {
  SPI.begin(PIN_RF_SCK, PIN_RF_MISO, PIN_RF_MOSI, PIN_RF_CSN);
  radioReady = radio.begin();
  if (!radioReady) {
    Serial.println("NRF INIT FAIL");
    return;
  }

  radio.setChannel(RF_CHANNEL);
  radio.setPALevel(RF_PA_LEVEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(3, 5);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openWritingPipe(txAddress);
  radio.openReadingPipe(1, rxAddress);
  radio.writeAckPayload(1, &telemetryOut, sizeof(telemetryOut));
  radio.startListening();
  Serial.println("NRF READY");
}

void initImu() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  imuReady = mpu.begin(0x68, &Wire);
  if (!imuReady) {
    Serial.println("IMU INIT FAIL");
    return;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("IMU READY");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BATTERY_ADC, ADC_11db);

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LIGHTS, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_LIGHTS, LOW);

  attachServoPwm();
  initImu();
  initRadio();

  lastCommandMs = millis();
  lastTelemetryMs = millis();
  lastSensorMs = millis();
  lastBatteryMs = millis();
  failsafeActive = true;
  Serial.println("RX FC BOOT");
}

void loop() {
  uint32_t now = millis();

  feedGps();
  readRadio();

  if (now - lastSensorMs >= SENSOR_PERIOD_MS) {
    lastSensorMs = now;
    updateImu();
  }

  if (now - lastBatteryMs >= BATTERY_PERIOD_MS) {
    lastBatteryMs = now;
    updateBattery();
  }

  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    updateTelemetry();
  }

  updateFailsafe(now);
  updateLights(now);
  updateBuzzer(now);
}
