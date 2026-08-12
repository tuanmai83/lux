/*
  LUXSPA DUAL SENSOR + PTC BENCH TEST V2 BUZZER
  Generated: 2026-08-12 09:21 GMT+7

  PURPOSE
  -------
  One firmware, one flash:
  - IO23 button selects/starts DS18B20 test on GPIO0.
  - IO17 SET button selects/starts DS18B20 test on GPIO33.
  - Selected sensor must qualify with 3 consecutive valid reads.
  - PTC command GPIO2 then cycles ON 10 s / OFF 2 s continuously.
  - OLED always shows selected sensor, temperature, PTC ON/OFF time,
    cycle count, and sensor-loss information.
  - Buzzer on GPIO15 is ACTIVE-LOW through the existing PNP driver.
  - Buzzer is silent by default and only chirps for completion/recovery
    or emits warning patterns for sensor/over-temperature faults.

  IMPORTANT HARDWARE
  ------------------
  GPIO0 mode:
    DS18B20 DQ -> GPIO0/BOOT.
    Use existing 4.7k pull-up to 3.3 V on the intended sensor circuit.

  GPIO33 mode:
    GPIO33 is originally routed to the FAN2 Q17 gate branch.
    BEFORE using GPIO33 as DS18B20 DQ, electrically isolate the
    GPIO33 -> FAN2/Q17 gate branch (verify R39/PCB revision first).
    DO NOT connect DS18B20 to CN4 FAN2 12 V connector.
    Add 4.7k pull-up from GPIO33/DQ to 3.3 V.

  GPIO2:
    Heater/PTC command only. Active HIGH. No PWM.

  SAFETY
  ------
  Even for bench testing:
  - GPIO2 is forced LOW before all other initialization.
  - Any selected-sensor read failure turns GPIO2 OFF immediately.
  - >= 60.0 C turns GPIO2 OFF.
  - GPIO2 only resumes after 3 valid recovery reads and safe temperature.
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

constexpr uint8_t TEMP_G0 = 0;
constexpr uint8_t TEMP_G33 = 33;

constexpr uint8_t START_G0 = 23;
constexpr uint8_t START_G33 = 17;   // Physical SET button in normal product firmware.

constexpr uint8_t OLED_SDA = 26;
constexpr uint8_t OLED_SCL = 27;
constexpr uint8_t STATUS_LED = 13;

// Outputs not used by this bench firmware.
constexpr uint8_t FAN1 = 25;
constexpr uint8_t ATOMIZER_A = 14;
constexpr uint8_t ATOMIZER_B = 12;
constexpr uint8_t LED_STRIP = 18;
constexpr uint8_t SOLENOID = 21;
constexpr uint8_t PUMP = 32;
constexpr uint8_t BUZZER = 15;
}

namespace Timing {
constexpr uint32_t BUTTON_DEBOUNCE_MS = 60;
constexpr uint32_t SENSOR_READ_MS = 1000;
constexpr uint32_t PTC_ON_MS = 10000;
constexpr uint32_t PTC_OFF_MS = 2000;
constexpr uint32_t DISPLAY_REFRESH_MS = 100;
}

namespace BuzzerPattern {
constexpr uint16_t SHORT_ON_MS = 90;
constexpr uint16_t SHORT_OFF_MS = 110;
constexpr uint16_t WARNING_ON_MS = 130;
constexpr uint16_t WARNING_OFF_MS = 140;
constexpr uint16_t OVERTEMP_ON_MS = 260;
constexpr uint16_t OVERTEMP_OFF_MS = 170;

constexpr uint8_t PRIORITY_INFO = 1;
constexpr uint8_t PRIORITY_FINISH = 2;
constexpr uint8_t PRIORITY_WARNING = 3;
}

namespace Limits {
constexpr uint8_t GOOD_READS_TO_START = 3;
constexpr uint8_t GOOD_READS_TO_RECOVER = 3;
constexpr float MAX_TEMP_C = 60.0f;
constexpr float AUTO_RESUME_TEMP_C = 45.0f;
constexpr float MIN_VALID_TEMP_C = -40.0f;
constexpr float MAX_VALID_TEMP_C = 125.0f;
}

Adafruit_SSD1306 oled(128, 64, &Wire, -1);

OneWire oneWireG0(Pins::TEMP_G0);
DallasTemperature tempG0(&oneWireG0);
DeviceAddress addrG0{};
bool addrG0Found = false;

OneWire oneWireG33(Pins::TEMP_G33);
DallasTemperature tempG33(&oneWireG33);
DeviceAddress addrG33{};
bool addrG33Found = false;
bool g33BusInitialized = false;

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

class BuzzerController {
 public:
  void begin() {
    // Hardware uses a PNP high-side driver:
    // IO15 HIGH = transistor OFF = buzzer OFF.
    // IO15 LOW  = transistor ON  = buzzer ON.
    digitalWrite(Pins::BUZZER, HIGH);
    pinMode(Pins::BUZZER, OUTPUT);
    digitalWrite(Pins::BUZZER, HIGH);
    active_ = false;
    outputOn_ = false;
    priority_ = 0;
  }

  void update() {
    if (!active_) return;

    const uint32_t now = millis();
    if ((int32_t)(now - deadlineMs_) < 0) return;

    if (outputOn_) {
      setOutput(false);

      if (pulsesRemaining_ == 0) {
        active_ = false;
        priority_ = 0;
        return;
      }

      deadlineMs_ = now + offMs_;
    } else {
      setOutput(true);
      --pulsesRemaining_;
      deadlineMs_ = now + onMs_;
    }
  }

  void silence() {
    active_ = false;
    pulsesRemaining_ = 0;
    priority_ = 0;
    setOutput(false);
  }

  void playCycleDone() {
    play(1, BuzzerPattern::SHORT_ON_MS, BuzzerPattern::SHORT_OFF_MS,
         BuzzerPattern::PRIORITY_INFO);
  }

  void playRecovery() {
    play(2, BuzzerPattern::SHORT_ON_MS, BuzzerPattern::SHORT_OFF_MS,
         BuzzerPattern::PRIORITY_FINISH);
  }

  void playStopped() {
    play(2, BuzzerPattern::SHORT_ON_MS, BuzzerPattern::SHORT_OFF_MS,
         BuzzerPattern::PRIORITY_FINISH);
  }

  void playSensorWarning() {
    play(3, BuzzerPattern::WARNING_ON_MS, BuzzerPattern::WARNING_OFF_MS,
         BuzzerPattern::PRIORITY_WARNING);
  }

  void playOverTempWarning() {
    play(3, BuzzerPattern::OVERTEMP_ON_MS, BuzzerPattern::OVERTEMP_OFF_MS,
         BuzzerPattern::PRIORITY_WARNING);
  }

 private:
  void play(uint8_t pulses, uint16_t onMs, uint16_t offMs, uint8_t priority) {
    if (pulses == 0) return;
    if (active_ && priority < priority_) return;

    active_ = true;
    priority_ = priority;
    onMs_ = onMs;
    offMs_ = offMs;
    pulsesRemaining_ = pulses - 1;

    setOutput(true);
    deadlineMs_ = millis() + onMs_;
  }

  void setOutput(bool on) {
    outputOn_ = on;
    digitalWrite(Pins::BUZZER, on ? LOW : HIGH);
  }

  bool active_ = false;
  bool outputOn_ = false;
  uint8_t pulsesRemaining_ = 0;
  uint8_t priority_ = 0;
  uint16_t onMs_ = 0;
  uint16_t offMs_ = 0;
  uint32_t deadlineMs_ = 0;
};

DebouncedButton buttonG0(Pins::START_G0);
DebouncedButton buttonG33(Pins::START_G33);
BuzzerController buzzer;

enum class Mode : uint8_t {
  NONE,
  G0,
  G33
};

enum class State : uint8_t {
  HOME,
  QUALIFY,
  PTC_ON,
  PTC_OFF,
  SENSOR_WARNING,
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

Mode mode = Mode::NONE;
State state = State::HOME;

bool oledOK = false;
bool ptcCommand = false;

float currentTempC = NAN;
float lastGoodTempC = NAN;

uint8_t goodStreak = 0;
uint8_t recoveryStreak = 0;
uint32_t lossCount = 0;
uint32_t cycleCount = 0;

TempError lastError = TempError::NONE;

uint32_t stateStartedMs = 0;
uint32_t nextSensorReadMs = 0;
uint32_t lastDisplayMs = 0;

const char* modeName(Mode m) {
  switch (m) {
    case Mode::G0: return "G0";
    case Mode::G33: return "G33";
    default: return "NONE";
  }
}

const char* errorName(TempError e) {
  switch (e) {
    case TempError::NOT_FOUND: return "NOT FOUND";
    case TempError::DISCONNECTED: return "DISCONNECTED";
    case TempError::POWERUP_85: return "POWERUP 85C";
    case TempError::INVALID_NAN: return "INVALID/NAN";
    case TempError::OUT_OF_RANGE: return "OUT OF RANGE";
    default: return "NONE";
  }
}

void setPtc(bool on) {
  digitalWrite(Pins::PTC, on ? HIGH : LOW);
  ptcCommand = on;
}

void setState(State next) {
  state = next;
  stateStartedMs = millis();
}

void setMode(Mode selected) {
  setPtc(false);
  buzzer.silence();

  mode = selected;
  state = State::QUALIFY;
  stateStartedMs = millis();

  currentTempC = NAN;
  lastGoodTempC = NAN;
  goodStreak = 0;
  recoveryStreak = 0;
  lossCount = 0;
  cycleCount = 0;
  lastError = TempError::NONE;
  nextSensorReadMs = millis();

  if (mode == Mode::G0) {
    addrG0Found = false;
    tempG0.begin();
    tempG0.setWaitForConversion(true);
  } else if (mode == Mode::G33) {
    // GPIO33 must be hardware-isolated from the old FAN2/Q17 branch.
    addrG33Found = false;
    if (!g33BusInitialized) {
      tempG33.begin();
      tempG33.setWaitForConversion(true);
      g33BusInitialized = true;
    }
  }

  Serial.print("MODE_SELECTED,");
  Serial.println(modeName(mode));
}

void returnHome() {
  const bool hadActiveMode = (mode != Mode::NONE);
  setPtc(false);
  mode = Mode::NONE;
  currentTempC = NAN;
  lastGoodTempC = NAN;
  goodStreak = 0;
  recoveryStreak = 0;
  lastError = TempError::NONE;
  setState(State::HOME);

  if (hadActiveMode) {
    buzzer.playStopped();
  }
}

bool discoverSelectedSensor() {
  if (mode == Mode::G0) {
    if (tempG0.getDeviceCount() < 1) {
      addrG0Found = false;
      return false;
    }
    if (!tempG0.getAddress(addrG0, 0)) {
      addrG0Found = false;
      return false;
    }
    tempG0.setResolution(addrG0, 10);
    addrG0Found = true;
    return true;
  }

  if (mode == Mode::G33) {
    if (tempG33.getDeviceCount() < 1) {
      addrG33Found = false;
      return false;
    }
    if (!tempG33.getAddress(addrG33, 0)) {
      addrG33Found = false;
      return false;
    }
    tempG33.setResolution(addrG33, 10);
    addrG33Found = true;
    return true;
  }

  return false;
}

TempSample readSelectedTemperature() {
  TempSample sample;

  if (mode == Mode::NONE) {
    sample.error = TempError::NOT_FOUND;
    return sample;
  }

  bool found = (mode == Mode::G0) ? addrG0Found : addrG33Found;
  if (!found && !discoverSelectedSensor()) {
    sample.error = TempError::NOT_FOUND;
    return sample;
  }

  bool requestOK = false;
  float value = DEVICE_DISCONNECTED_C;

  if (mode == Mode::G0) {
    requestOK = tempG0.requestTemperaturesByAddress(addrG0);
    if (requestOK) value = tempG0.getTempC(addrG0);
  } else {
    requestOK = tempG33.requestTemperaturesByAddress(addrG33);
    if (requestOK) value = tempG33.getTempC(addrG33);
  }

  if (!requestOK || value == DEVICE_DISCONNECTED_C) {
    if (mode == Mode::G0) addrG0Found = false;
    else addrG33Found = false;

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
  sample.error = TempError::NONE;
  return sample;
}

void logSample(const TempSample& sample) {
  Serial.print(millis());
  Serial.print(',');
  Serial.print(modeName(mode));
  Serial.print(',');

  if (sample.valid) Serial.print(sample.valueC, 2);
  else Serial.print("NA");

  Serial.print(',');
  Serial.print(sample.valid ? "OK" : errorName(sample.error));
  Serial.print(',');
  Serial.print(ptcCommand ? "PTC_ON" : "PTC_OFF");
  Serial.print(',');
  Serial.print(cycleCount);
  Serial.print(',');
  Serial.println(lossCount);
}

void handleValidTemperature(float valueC) {
  currentTempC = valueC;
  lastGoodTempC = valueC;
  lastError = TempError::NONE;

  if (valueC >= Limits::MAX_TEMP_C) {
    const bool enteringOverTemp = (state != State::OVER_TEMP);
    setPtc(false);
    recoveryStreak = 0;
    setState(State::OVER_TEMP);

    if (enteringOverTemp) {
      buzzer.playOverTempWarning();
    }
    return;
  }

  if (state == State::QUALIFY) {
    if (goodStreak < Limits::GOOD_READS_TO_START) ++goodStreak;

    if (goodStreak >= Limits::GOOD_READS_TO_START) {
      ++cycleCount;
      setPtc(true);
      setState(State::PTC_ON);
    }
    return;
  }

  if (state == State::SENSOR_WARNING) {
    if (valueC < Limits::AUTO_RESUME_TEMP_C) {
      if (recoveryStreak < Limits::GOOD_READS_TO_RECOVER) ++recoveryStreak;
    } else {
      recoveryStreak = 0;
    }

    if (recoveryStreak >= Limits::GOOD_READS_TO_RECOVER) {
      buzzer.playRecovery();
      ++cycleCount;
      setPtc(true);
      setState(State::PTC_ON);
    }
    return;
  }

  if (state == State::OVER_TEMP) {
    if (valueC < Limits::AUTO_RESUME_TEMP_C) {
      if (recoveryStreak < Limits::GOOD_READS_TO_RECOVER) ++recoveryStreak;
    } else {
      recoveryStreak = 0;
    }

    if (recoveryStreak >= Limits::GOOD_READS_TO_RECOVER) {
      buzzer.playRecovery();
      ++cycleCount;
      setPtc(true);
      setState(State::PTC_ON);
    }
  }
}

void handleTemperatureError(TempError error) {
  const bool enteringWarning = (state != State::SENSOR_WARNING);

  lastError = error;
  ++lossCount;
  goodStreak = 0;
  recoveryStreak = 0;
  setPtc(false);
  setState(State::SENSOR_WARNING);

  if (enteringWarning) {
    buzzer.playSensorWarning();
  }
}

void updateSensor() {
  if (mode == Mode::NONE) return;
  if ((int32_t)(millis() - nextSensorReadMs) < 0) return;

  nextSensorReadMs = millis() + Timing::SENSOR_READ_MS;

  const TempSample sample = readSelectedTemperature();
  logSample(sample);

  if (sample.valid) handleValidTemperature(sample.valueC);
  else handleTemperatureError(sample.error);
}

void updatePtcCycle() {
  if (state == State::PTC_ON) {
    if ((millis() - stateStartedMs) >= Timing::PTC_ON_MS) {
      setPtc(false);
      setState(State::PTC_OFF);
    }
  } else if (state == State::PTC_OFF) {
    if ((millis() - stateStartedMs) >= Timing::PTC_OFF_MS) {
      buzzer.playCycleDone();
      ++cycleCount;
      setPtc(true);
      setState(State::PTC_ON);
    }
  }
}

void drawLine(uint8_t row, const String& text) {
  oled.setCursor(0, row * 8);
  oled.print(text);
}

String tempText(float value) {
  return isnan(value) ? String("NA") : String(value, 1);
}

void drawHome() {
  drawLine(0, "LUX DUAL PTC V2");
  drawLine(1, "G0 TEMP: GPIO0");
  drawLine(2, "START G0: IO23");
  drawLine(3, "G33 TEMP: GPIO33");
  drawLine(4, "START G33: IO17");
  drawLine(6, "PTC: OFF");
  drawLine(7, "SELECT TEST MODE");
}

void drawQualify() {
  drawLine(0, "LUX DUAL PTC V2");
  drawLine(1, "MODE: " + String(modeName(mode)));
  drawLine(2, "TEMP: " + tempText(currentTempC) + " C");
  drawLine(3, "SENSOR GOOD: " + String(goodStreak) + "/3");
  drawLine(5, "PTC: OFF");
  drawLine(6, "AUTO ON AT 3/3");
  drawLine(7, mode == Mode::G0 ? "IO23=STOP" : "IO17=STOP");
}

void drawPtcOn() {
  const uint32_t elapsed = millis() - stateStartedMs;
  uint32_t sec = elapsed / 1000 + 1;
  if (sec > 10) sec = 10;

  drawLine(0, "LUX DUAL PTC V2");
  drawLine(1, "MODE: " + String(modeName(mode)));
  drawLine(2, "TEMP: " + tempText(currentTempC) + " C");
  drawLine(3, "SENSOR: OK");
  drawLine(4, "PTC: ON");
  drawLine(5, "ON TIME: " + String(sec) + "/10s");
  drawLine(6, "CYCLE: " + String(cycleCount));
  drawLine(7, mode == Mode::G0 ? "IO23=STOP" : "IO17=STOP");
}

void drawPtcOff() {
  const uint32_t elapsed = millis() - stateStartedMs;
  uint32_t sec = elapsed / 1000 + 1;
  if (sec > 2) sec = 2;

  drawLine(0, "LUX DUAL PTC V2");
  drawLine(1, "MODE: " + String(modeName(mode)));
  drawLine(2, "TEMP: " + tempText(currentTempC) + " C");
  drawLine(3, "SENSOR: OK");
  drawLine(4, "PTC: OFF");
  drawLine(5, "OFF TIME: " + String(sec) + "/2s");
  drawLine(6, "CYCLE: " + String(cycleCount));
  drawLine(7, mode == Mode::G0 ? "IO23=STOP" : "IO17=STOP");
}

void drawWarning() {
  drawLine(0, "TEMP SENSOR WARNING");
  drawLine(1, "MODE: " + String(modeName(mode)));
  drawLine(2, "LAST: " + tempText(lastGoodTempC) + " C");
  drawLine(3, "ERR: " + String(errorName(lastError)));
  drawLine(4, "LOSS: " + String(lossCount));
  drawLine(5, "PTC: SAFE OFF");
  drawLine(6, "RECOVER: " + String(recoveryStreak) + "/3");
  drawLine(7, mode == Mode::G0 ? "IO23=STOP" : "IO17=STOP");
}

void drawOverTemp() {
  drawLine(0, "OVER TEMP WARNING");
  drawLine(1, "MODE: " + String(modeName(mode)));
  drawLine(2, "TEMP: " + tempText(currentTempC) + " C");
  drawLine(3, "LIMIT: 60.0 C");
  drawLine(5, "PTC: SAFE OFF");
  drawLine(6, "WAIT BELOW 45 C");
  drawLine(7, "RECOVER: " + String(recoveryStreak) + "/3");
}

void drawScreen() {
  if (!oledOK) return;
  if ((millis() - lastDisplayMs) < Timing::DISPLAY_REFRESH_MS) return;
  lastDisplayMs = millis();

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  switch (state) {
    case State::HOME: drawHome(); break;
    case State::QUALIFY: drawQualify(); break;
    case State::PTC_ON: drawPtcOn(); break;
    case State::PTC_OFF: drawPtcOff(); break;
    case State::SENSOR_WARNING: drawWarning(); break;
    case State::OVER_TEMP: drawOverTemp(); break;
  }

  oled.display();
}

void handleButtons() {
  const bool g0Pressed = buttonG0.pressed();
  const bool g33Pressed = buttonG33.pressed();

  if (g0Pressed) {
    if (mode == Mode::G0) {
      returnHome();
    } else {
      setMode(Mode::G0);
    }
  }

  if (g33Pressed) {
    if (mode == Mode::G33) {
      returnHome();
    } else {
      setMode(Mode::G33);
    }
  }
}

void updateStatusLed() {
  if (state == State::SENSOR_WARNING || state == State::OVER_TEMP) {
    digitalWrite(Pins::STATUS_LED, ((millis() / 300) & 1) ? HIGH : LOW);
  } else if (mode != Mode::NONE) {
    digitalWrite(Pins::STATUS_LED, HIGH);
  } else {
    digitalWrite(Pins::STATUS_LED, LOW);
  }
}

void setup() {
  // Heater/PTC must be safe before all other initialization.
  pinMode(Pins::PTC, OUTPUT);
  digitalWrite(Pins::PTC, LOW);
  ptcCommand = false;

  // Buzzer is active-LOW through Q19 PNP. Keep it OFF at boot.
  buzzer.begin();

  // Outputs not used in this bench test.
  const uint8_t safeOffPins[] = {
      Pins::FAN1, Pins::ATOMIZER_A, Pins::ATOMIZER_B,
      Pins::LED_STRIP, Pins::SOLENOID, Pins::PUMP};

  for (uint8_t pin : safeOffPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  // IMPORTANT: GPIO33 is NOT configured as an output anywhere in this firmware.

  pinMode(Pins::STATUS_LED, OUTPUT);
  digitalWrite(Pins::STATUS_LED, LOW);

  buttonG0.begin();
  buttonG33.begin();

  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println("LUXSPA DUAL SENSOR + PTC BENCH TEST V2 BUZZER");
  Serial.println("GPIO0_START_BUTTON,IO23");
  Serial.println("GPIO33_START_BUTTON,IO17");
  Serial.println("BUZZER,GPIO15,ACTIVE_LOW,SILENT_DEFAULT");
  Serial.println("CSV:ms,mode,temp,status,ptc,cycle,loss");

  Wire.begin(Pins::OLED_SDA, Pins::OLED_SCL);
  oledOK = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Initialize GPIO0 sensor bus now; GPIO33 bus is lazy-initialized only
  // when IO17 selects G33 mode.
  tempG0.begin();
  tempG0.setWaitForConversion(true);

  stateStartedMs = millis();
  nextSensorReadMs = millis();
  drawScreen();
}

void loop() {
  handleButtons();
  buzzer.update();
  updateStatusLed();

  updateSensor();
  updatePtcCycle();
  drawScreen();
}
