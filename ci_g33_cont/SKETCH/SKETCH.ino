/*
  LUXSPA PTC MOSFET CONTINUOUS TEST G33 V1
  Generated: 2026-08-14 14:24 GMT+7

  - DS18B20: GPIO33 only.
  - START/STOP: IO23 only.
  - PTC command: GPIO2.
  - START -> PTC continuously ON.
  - STOP -> PTC OFF.
  - TEMP >= 90.0 C -> PTC OFF + latched over-temp.
  - Any sensor read fault -> PTC OFF + latched sensor fault.
  - Faults never auto-resume; IO23 manual re-arm required.

  GPIO33 must already be isolated from the old FAN2/Q17 branch.
  DS18B20 DQ must use RAW GPIO33 with 4.7k pull-up to 3.3V.
  DO NOT connect DS18B20 to the CN4 FAN2 12V output.

  Buzzer GPIO15 is ACTIVE LOW:
  HIGH = OFF, LOW = ON.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

namespace Pins {
constexpr uint8_t PTC = 2;
constexpr uint8_t TEMP_DQ = 33;
constexpr uint8_t START_STOP = 23;
constexpr uint8_t BUZZER = 15;
constexpr uint8_t STATUS_LED = 13;
constexpr uint8_t OLED_SDA = 26;
constexpr uint8_t OLED_SCL = 27;

constexpr uint8_t FAN1 = 25;
constexpr uint8_t ATOMIZER_A = 14;
constexpr uint8_t ATOMIZER_B = 12;
constexpr uint8_t LED_STRIP = 18;
constexpr uint8_t SOLENOID = 21;
constexpr uint8_t PUMP = 32;
}

namespace Timing {
constexpr uint32_t BUTTON_DEBOUNCE_MS = 60;
constexpr uint32_t SENSOR_READ_MS = 1000;
constexpr uint32_t DISPLAY_REFRESH_MS = 100;
}

namespace Limits {
constexpr uint8_t GOOD_READS_READY = 3;
constexpr uint8_t GOOD_READS_RECOVER = 3;
constexpr float CUT_OFF_TEMP_C = 90.0f;
constexpr float REARM_TEMP_C = 80.0f;
constexpr float MIN_VALID_TEMP_C = -40.0f;
constexpr float MAX_VALID_TEMP_C = 125.0f;
}

static_assert(Pins::TEMP_DQ == 33, "DS18B20 must use GPIO33");
static_assert(Pins::START_STOP == 23, "START/STOP must use IO23");
static_assert(Pins::TEMP_DQ != Pins::PTC, "Sensor/PTC pin conflict");
static_assert(Pins::TEMP_DQ != Pins::START_STOP, "Sensor/button pin conflict");

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
OneWire oneWire(Pins::TEMP_DQ);
DallasTemperature tempBus(&oneWire);
DeviceAddress sensorAddress{};
bool sensorAddressFound = false;

class DebouncedButton {
 public:
  explicit DebouncedButton(uint8_t pin) : pin_(pin) {}
  void begin() {
    pinMode(pin_, INPUT_PULLUP);
    raw_ = digitalRead(pin_);
    stable_ = raw_;
    changedAt_ = millis();
  }
  bool pressed() {
    const bool raw = digitalRead(pin_);
    const uint32_t now = millis();
    if (raw != raw_) {
      raw_ = raw;
      changedAt_ = now;
    }
    if ((now - changedAt_) >= Timing::BUTTON_DEBOUNCE_MS && raw != stable_) {
      stable_ = raw;
      return stable_ == LOW;
    }
    return false;
  }
 private:
  uint8_t pin_;
  bool raw_ = HIGH;
  bool stable_ = HIGH;
  uint32_t changedAt_ = 0;
};

DebouncedButton startButton(Pins::START_STOP);

enum class State : uint8_t {
  QUALIFY,
  READY,
  RUNNING,
  SENSOR_FAULT,
  OVER_TEMP
};

enum class TempError : uint8_t {
  NONE,
  NOT_FOUND,
  DISCONNECTED,
  POWERUP_85,
  INVALID_NAN,
  OUT_OF_RANGE
};

struct TempSample {
  bool valid = false;
  float valueC = NAN;
  TempError error = TempError::NONE;
};

struct ToneStep {
  bool on;
  uint16_t durationMs;
};

class BuzzerPattern {
 public:
  void begin() {
    pinMode(Pins::BUZZER, OUTPUT);
    off();
  }
  void update() {
    if (!active_) return;
    const uint32_t now = millis();
    if ((int32_t)(now - nextChangeMs_) < 0) return;
    ++index_;
    if (index_ >= count_) {
      stop();
      return;
    }
    apply(steps_[index_].on);
    nextChangeMs_ = now + steps_[index_].durationMs;
  }
  void stop() {
    active_ = false;
    count_ = 0;
    index_ = 0;
    off();
  }
  void playSensorFault() {
    static const ToneStep p[] = {
      {true,120},{false,100},{true,120},{false,100},{true,120},{false,1}
    };
    play(p, sizeof(p)/sizeof(p[0]));
  }
  void playOverTemp() {
    static const ToneStep p[] = {
      {true,350},{false,150},{true,350},{false,150},{true,350},{false,1}
    };
    play(p, sizeof(p)/sizeof(p[0]));
  }
  void playStopped() {
    static const ToneStep p[] = {
      {true,100},{false,100},{true,100},{false,1}
    };
    play(p, sizeof(p)/sizeof(p[0]));
  }
 private:
  void on() { digitalWrite(Pins::BUZZER, LOW); }
  void off() { digitalWrite(Pins::BUZZER, HIGH); }
  void apply(bool onState) { onState ? on() : off(); }
  void play(const ToneStep* pattern, uint8_t count) {
    stop();
    if (count == 0 || count > MAX_STEPS) return;
    for (uint8_t i=0; i<count; ++i) steps_[i] = pattern[i];
    count_ = count;
    index_ = 0;
    active_ = true;
    apply(steps_[0].on);
    nextChangeMs_ = millis() + steps_[0].durationMs;
  }
  static constexpr uint8_t MAX_STEPS = 8;
  ToneStep steps_[MAX_STEPS]{};
  uint8_t count_ = 0;
  uint8_t index_ = 0;
  bool active_ = false;
  uint32_t nextChangeMs_ = 0;
};

BuzzerPattern buzzer;
State state = State::QUALIFY;
bool oledOK = false;
bool ptcOn = false;
bool overTempLatched = false;

float currentTempC = NAN;
float lastGoodTempC = NAN;
float maxTempC = NAN;

TempError activeFault = TempError::NONE;
uint8_t goodStreak = 0;
uint8_t recoverStreak = 0;
uint32_t lossCount = 0;
uint32_t testStartedMs = 0;
uint32_t nextSensorReadMs = 0;
uint32_t lastDisplayMs = 0;

const char* errorName(TempError error) {
  switch (error) {
    case TempError::NOT_FOUND: return "NOT FOUND";
    case TempError::DISCONNECTED: return "DISCONNECTED";
    case TempError::POWERUP_85: return "POWERUP 85C";
    case TempError::INVALID_NAN: return "INVALID/NAN";
    case TempError::OUT_OF_RANGE: return "OUT RANGE";
    default: return "NONE";
  }
}

void setPtc(bool on) {
  digitalWrite(Pins::PTC, on ? HIGH : LOW);
  ptcOn = on;
}
void forceSafeOff() { setPtc(false); }

void resetRunStats() {
  maxTempC = currentTempC;
  lossCount = 0;
  activeFault = TempError::NONE;
  recoverStreak = 0;
  testStartedMs = millis();
}

void startContinuousTest() {
  if (state != State::READY) return;
  if (isnan(currentTempC)) return;
  if (currentTempC >= Limits::CUT_OFF_TEMP_C) return;
  if (overTempLatched) return;
  buzzer.stop();
  resetRunStats();
  setPtc(true);
  state = State::RUNNING;
  Serial.println("TEST,START");
}

void stopContinuousTest() {
  forceSafeOff();
  buzzer.playStopped();
  activeFault = TempError::NONE;
  recoverStreak = 0;
  goodStreak = Limits::GOOD_READS_READY;
  state = State::READY;
  Serial.println("TEST,STOP");
}

bool discoverSensor() {
  if (tempBus.getDeviceCount() < 1) {
    sensorAddressFound = false;
    return false;
  }
  if (!tempBus.getAddress(sensorAddress, 0)) {
    sensorAddressFound = false;
    return false;
  }
  tempBus.setResolution(sensorAddress, 10);
  sensorAddressFound = true;
  return true;
}

TempSample readTemperature() {
  TempSample sample;
  if (!sensorAddressFound && !discoverSensor()) {
    sample.error = TempError::NOT_FOUND;
    return sample;
  }

  const bool requestOK = tempBus.requestTemperaturesByAddress(sensorAddress);
  if (!requestOK) {
    sensorAddressFound = false;
    sample.error = TempError::DISCONNECTED;
    return sample;
  }

  const float value = tempBus.getTempC(sensorAddress);
  if (value == DEVICE_DISCONNECTED_C) {
    sensorAddressFound = false;
    sample.error = TempError::DISCONNECTED;
    return sample;
  }
  if (isnan(value)) {
    sample.error = TempError::INVALID_NAN;
    return sample;
  }
  if (fabsf(value - 85.0f) < 0.01f) {
    sample.error = TempError::POWERUP_85;
    return sample;
  }
  if (value < Limits::MIN_VALID_TEMP_C || value > Limits::MAX_VALID_TEMP_C) {
    sample.error = TempError::OUT_OF_RANGE;
    return sample;
  }

  sample.valid = true;
  sample.valueC = value;
  return sample;
}

void updateMaxTemp(float valueC) {
  if (isnan(maxTempC) || valueC > maxTempC) maxTempC = valueC;
}

void handleValidTemp(float valueC) {
  currentTempC = valueC;
  lastGoodTempC = valueC;
  updateMaxTemp(valueC);

  if (valueC >= Limits::CUT_OFF_TEMP_C) {
    forceSafeOff();
    overTempLatched = true;
    recoverStreak = 0;
    if (state != State::OVER_TEMP) {
      buzzer.playOverTemp();
      Serial.println("FAULT,OVER_TEMP");
    }
    state = State::OVER_TEMP;
    return;
  }

  switch (state) {
    case State::QUALIFY:
      if (goodStreak < Limits::GOOD_READS_READY) ++goodStreak;
      if (goodStreak >= Limits::GOOD_READS_READY) state = State::READY;
      break;
    case State::READY:
      goodStreak = Limits::GOOD_READS_READY;
      break;
    case State::RUNNING:
      break;
    case State::SENSOR_FAULT:
      if (recoverStreak < Limits::GOOD_READS_RECOVER) ++recoverStreak;
      break;
    case State::OVER_TEMP:
      if (valueC <= Limits::REARM_TEMP_C) {
        if (recoverStreak < Limits::GOOD_READS_RECOVER) ++recoverStreak;
      } else {
        recoverStreak = 0;
      }
      break;
  }
}

void handleTempFault(TempError error) {
  ++lossCount;
  activeFault = error;
  goodStreak = 0;
  recoverStreak = 0;
  forceSafeOff();

  // Never weaken an over-temperature latch because of a later sensor fault.
  // PTC must stay OFF until temperature is proven <= 80 C for 3 valid reads
  // and IO23 is pressed to re-arm.
  if (overTempLatched || state == State::OVER_TEMP) {
    if (state != State::OVER_TEMP) buzzer.playOverTemp();
    state = State::OVER_TEMP;
  } else {
    if (state != State::SENSOR_FAULT) buzzer.playSensorFault();
    state = State::SENSOR_FAULT;
  }

  Serial.print("FAULT,SENSOR,");
  Serial.println(errorName(error));
}

void updateSensor() {
  const uint32_t now = millis();
  if ((int32_t)(now - nextSensorReadMs) < 0) return;
  nextSensorReadMs = now + Timing::SENSOR_READ_MS;

  const TempSample sample = readTemperature();

  Serial.print("SAMPLE,");
  Serial.print(now);
  Serial.print(',');
  if (sample.valid) {
    Serial.print(sample.valueC, 2);
    Serial.print(",OK,");
  } else {
    Serial.print("NA,");
    Serial.print(errorName(sample.error));
    Serial.print(',');
  }
  Serial.print(ptcOn ? "PTC_ON," : "PTC_OFF,");
  Serial.println(lossCount);

  if (sample.valid) handleValidTemp(sample.valueC);
  else handleTempFault(sample.error);
}

void handleButton() {
  if (!startButton.pressed()) return;

  if (state == State::RUNNING) {
    stopContinuousTest();
    return;
  }

  if (state == State::READY) {
    startContinuousTest();
    return;
  }

  if (state == State::SENSOR_FAULT) {
    if (recoverStreak >= Limits::GOOD_READS_RECOVER &&
        !isnan(currentTempC) &&
        currentTempC < Limits::CUT_OFF_TEMP_C) {
      buzzer.stop();
      resetRunStats();
      setPtc(true);
      state = State::RUNNING;
      Serial.println("TEST,REARM_AFTER_SENSOR");
    }
    return;
  }

  if (state == State::OVER_TEMP) {
    if (recoverStreak >= Limits::GOOD_READS_RECOVER &&
        !isnan(currentTempC) &&
        currentTempC <= Limits::REARM_TEMP_C) {
      buzzer.stop();
      overTempLatched = false;
      activeFault = TempError::NONE;
      resetRunStats();
      setPtc(true);
      state = State::RUNNING;
      Serial.println("TEST,REARM_AFTER_OVERTEMP");
    }
  }
}

void updateStatusLed() {
  if (state == State::RUNNING) {
    digitalWrite(Pins::STATUS_LED, HIGH);
  } else if (state == State::SENSOR_FAULT || state == State::OVER_TEMP) {
    digitalWrite(Pins::STATUS_LED, ((millis()/300)&1) ? HIGH : LOW);
  } else {
    digitalWrite(Pins::STATUS_LED, LOW);
  }
}

void drawLine(uint8_t row, const String& text) {
  oled.setCursor(0, row*8);
  oled.print(text);
}

String tempText(float value) {
  return isnan(value) ? String("NA") : String(value,1);
}

String formatElapsed(uint32_t elapsedMs) {
  const uint32_t totalSeconds = elapsedMs/1000;
  const uint32_t minutes = totalSeconds/60;
  const uint32_t seconds = totalSeconds%60;
  char buffer[12];
  snprintf(buffer, sizeof(buffer), "%02lu:%02lu",
           static_cast<unsigned long>(minutes),
           static_cast<unsigned long>(seconds));
  return String(buffer);
}

void drawQualify() {
  drawLine(0, "LUX PTC G33 V1");
  drawLine(1, "TEMP GPIO33");
  drawLine(2, "TEMP: " + tempText(currentTempC) + " C");
  drawLine(3, "SENSOR: " + String(goodStreak) + "/3");
  drawLine(5, "PTC: OFF");
  drawLine(6, "CUT OFF: 90 C");
  drawLine(7, "WAIT SENSOR");
}

void drawReady() {
  drawLine(0, "LUX PTC G33 V1");
  drawLine(1, "TEMP GPIO33");
  drawLine(2, "TEMP: " + tempText(currentTempC) + " C");
  drawLine(3, "SENSOR: READY");
  drawLine(4, "PTC: OFF");
  drawLine(5, "CUT OFF: 90 C");
  drawLine(7, "IO23=START");
}

void drawRunning() {
  drawLine(0, "LUX PTC G33 V1");
  drawLine(1, "TEMP: " + tempText(currentTempC) + " C");
  drawLine(2, "MAX: " + tempText(maxTempC) + " C");
  drawLine(3, "SENSOR: OK");
  drawLine(4, "PTC: ON");
  drawLine(5, "ON: " + formatElapsed(millis()-testStartedMs));
  drawLine(6, "CUT OFF: 90 C");
  drawLine(7, "IO23=STOP");
}

void drawSensorFault() {
  drawLine(0, "TEMP SENSOR FAULT");
  drawLine(1, "LAST: " + tempText(lastGoodTempC) + " C");
  drawLine(2, "FAULT: " + String(errorName(activeFault)));
  drawLine(3, "LOSS: " + String(lossCount));
  drawLine(4, "PTC: SAFE OFF");
  drawLine(5, "RECOVER: " + String(recoverStreak) + "/3");
  drawLine(7, recoverStreak >= Limits::GOOD_READS_RECOVER ? "IO23=REARM" : "WAIT SENSOR");
}

void drawOverTemp() {
  drawLine(0, "OVER TEMP 90 C");
  drawLine(1, "TEMP: " + tempText(currentTempC) + " C");
  drawLine(2, "MAX: " + tempText(maxTempC) + " C");
  drawLine(3, "PTC: SAFE OFF");
  drawLine(4, "REARM <= 80 C");
  drawLine(5, "READY: " + String(recoverStreak) + "/3");
  drawLine(7, recoverStreak >= Limits::GOOD_READS_RECOVER ? "IO23=REARM" : "COOLING...");
}

void drawScreen() {
  if (!oledOK) return;
  const uint32_t now = millis();
  if ((now-lastDisplayMs) < Timing::DISPLAY_REFRESH_MS) return;
  lastDisplayMs = now;

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  switch (state) {
    case State::QUALIFY: drawQualify(); break;
    case State::READY: drawReady(); break;
    case State::RUNNING: drawRunning(); break;
    case State::SENSOR_FAULT: drawSensorFault(); break;
    case State::OVER_TEMP: drawOverTemp(); break;
  }
  oled.display();
}

void setup() {
  pinMode(Pins::PTC, OUTPUT);
  digitalWrite(Pins::PTC, LOW);
  ptcOn = false;

  pinMode(Pins::BUZZER, OUTPUT);
  digitalWrite(Pins::BUZZER, HIGH);

  const uint8_t safeOffPins[] = {
    Pins::FAN1,
    Pins::ATOMIZER_A,
    Pins::ATOMIZER_B,
    Pins::LED_STRIP,
    Pins::SOLENOID,
    Pins::PUMP
  };

  for (uint8_t pin : safeOffPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  pinMode(Pins::STATUS_LED, OUTPUT);
  digitalWrite(Pins::STATUS_LED, LOW);

  startButton.begin();

  digitalWrite(Pins::BUZZER, HIGH);
  buzzer.begin();

  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println("LUXSPA PTC MOSFET CONTINUOUS TEST G33 V1");
  Serial.println("TEMP_DQ,GPIO33");
  Serial.println("START_STOP,IO23");
  Serial.println("PTC_CMD,GPIO2");
  Serial.println("CUT_OFF_C,90.0");
  Serial.println("REARM_C,80.0");
  Serial.println("SENSOR_READ_MS,1000");
  Serial.println("BUZZER,GPIO15,ACTIVE_LOW");

  Wire.begin(Pins::OLED_SDA, Pins::OLED_SCL);
  oledOK = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  tempBus.begin();
  tempBus.setWaitForConversion(true);

  nextSensorReadMs = millis();
  drawScreen();
}

void loop() {
  buzzer.update();
  handleButton();
  updateStatusLed();
  updateSensor();
  drawScreen();
}
