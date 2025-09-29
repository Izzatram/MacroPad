// main.cpp - Polished Macropad firmware
// - Matrix: ROW 25, COLs: 26,27,32
// - Encoder: CLK=14, DT=16, SW=13
// - Battery: ADC pin 33 (adjust if needed)
// - BLE HID keyboard with battery level

#include <Arduino.h>
#include <BleKeyboard.h>
#include <RotaryEncoder.h>
#include "nvs_flash.h"

// ----- BLE -----
BleKeyboard bleKeyboard("Macropad", "YourCompany", 100);

// ----- Pins (match your wiring) -----
#define ROW_PIN         25
#define COL0_PIN        32
#define COL1_PIN        27
#define COL2_PIN        26

#define ENC_CLK         14
#define ENC_DT          16
#define ENC_SW          13

#define LED_STATUS      4

#define BAT_CHARGE_PIN  34
#define BAT_FULL_PIN    35
#define BAT_ADC_PIN     33

// ----- Globals -----
const uint8_t columnPins[3] = {COL0_PIN, COL1_PIN, COL2_PIN};
bool keyState[3] = {false, false, false};
unsigned long lastDebounceTime[3] = {0, 0, 0};
const unsigned long debounceDelay = 50;

bool profile = 0; // 0 or 1
unsigned long switch1_press_time = 0;
bool switch1_long_press_triggered = false;

RotaryEncoder encoder(ENC_DT, ENC_CLK, RotaryEncoder::LatchMode::TWO03);
int lastEncoderPos = 0;

// ----- Utility -----
void resetBLE() {
  Serial.println("Resetting BLE pairing info (erasing NVS)...");
  nvs_flash_erase();
  nvs_flash_init();
  ESP.restart();
}

void indicateProfile() {
  // 1 blink -> profile 0, 2 blinks -> profile 1
  int blinks = profile ? 2 : 1;
  for (int i = 0; i < blinks; ++i) {
    digitalWrite(LED_STATUS, HIGH);
    delay(150);
    digitalWrite(LED_STATUS, LOW);
    delay(150);
  }
}

// ----- Battery -----
int readBatteryPercent() {
  // Read ADC and convert to battery percent.
  // Adjust the voltage mapping if your divider or battery chemistry differs.
  const float vmin = 3.0f; // 0%
  const float vmax = 4.2f; // 100%
  int raw = analogRead(BAT_ADC_PIN);              // 0..4095
  float vadc = (raw / 4095.0f) * 3.3f;            // V at ADC input
  float batteryV = vadc * 2.0f;                   // if using 2:1 divider
  int pct = (int)round((batteryV - vmin) / (vmax - vmin) * 100.0f);
  pct = constrain(pct, 0, 100);
  return pct;
}

void reportBatteryToHost() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < 1000) return; // every 1s
  last = now;
  int pct = readBatteryPercent();
  bleKeyboard.setBatteryLevel(pct);
  // Also print to Serial for debugging
  Serial.print("Battery: ");
  Serial.print(pct);
  Serial.print("%  (V=");
  float raw = analogRead(BAT_ADC_PIN);
  float batteryV = (raw / 4095.0f) * 3.3f * 2.0f;
  Serial.print(batteryV, 2);
  Serial.println("V)");
}

// ----- Matrix scanning -----
void scanKeys() {
  for (int col = 0; col < 3; ++col) {
    // set all columns HIGH then pull active LOW
    for (int j = 0; j < 3; ++j) digitalWrite(columnPins[j], HIGH);
    digitalWrite(columnPins[col], LOW);

    delay(1); // allow IO to settle (1 ms is safe)

    bool pressed = (digitalRead(ROW_PIN) == LOW); // active low
    unsigned long now = millis();

    if (pressed != keyState[col] && (now - lastDebounceTime[col]) > debounceDelay) {
      keyState[col] = pressed;
      lastDebounceTime[col] = now;

      if (pressed) {
        Serial.printf("Key %d PRESSED\n", col);
        // on press actions
        switch (col) {
          case 0:
            // start timing for long press
            switch1_press_time = now;
            switch1_long_press_triggered = false;
            break;
          case 1:
            if (profile == 0) {
              bleKeyboard.write(KEY_LEFT_ARROW);
              Serial.println("Action: LEFT ARROW");
            } else {
              bleKeyboard.write(KEY_MEDIA_MUTE);
              Serial.println("Action: MUTE");
            }
            break;
          case 2:
            bleKeyboard.write(KEY_RIGHT_ARROW);
            Serial.println("Action: RIGHT ARROW");
            break;
        }
      } else {
        Serial.printf("Key %d RELEASED\n", col);
        // on release actions
        if (col == 0) {
          unsigned long duration = now - switch1_press_time;
          if (duration > 1000) {
            if (!switch1_long_press_triggered) {
              switch1_long_press_triggered = true;
              Serial.println("Long press detected -> resetting BLE");
              resetBLE();
            }
          } else {
            profile = !profile;
            Serial.print("Profile toggled -> ");
            Serial.println(profile ? "1" : "0");
            indicateProfile();
          }
        }
      }
    }

    // return column to idle HIGH
    digitalWrite(columnPins[col], HIGH);
  }
}

// ----- Encoder handling -----
void handleEncoder() {
  encoder.tick();
  int pos = encoder.getPosition();

  if (pos != lastEncoderPos) {
    if (pos > lastEncoderPos) {
      // CW
      if (profile == 0) {
        bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
        Serial.println("Encoder CW -> VOL UP");
      } else {
        bleKeyboard.write(KEY_MEDIA_NEXT_TRACK);
        Serial.println("Encoder CW -> NEXT TRACK");
      }
    } else {
      // CCW
      if (profile == 0) {
        bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
        Serial.println("Encoder CCW -> VOL DOWN");
      } else {
        bleKeyboard.write(KEY_MEDIA_PREVIOUS_TRACK);
        Serial.println("Encoder CCW -> PREV TRACK");
      }
    }
    lastEncoderPos = pos;
  }

  static bool lastSW = HIGH;
  bool curSW = digitalRead(ENC_SW);
  if (lastSW == HIGH && curSW == LOW) {
    delay(12); // small debounce
    if (digitalRead(ENC_SW) == LOW) {
      bleKeyboard.write(KEY_MEDIA_PLAY_PAUSE);
      Serial.println("Encoder button -> PLAY/PAUSE");
    }
  }
  lastSW = curSW;
}

// ----- Battery LED -----
void handleBatteryLED() {
  int charge = digitalRead(BAT_CHARGE_PIN); // 0 = charging (depends on your hardware)
  int full = digitalRead(BAT_FULL_PIN);     // 0 = full (depends on your hardware)

  if (full == 0) {
    digitalWrite(LED_STATUS, HIGH); // solid on = full
  } else if (charge == 0) {
    // blink while charging
    digitalWrite(LED_STATUS, (millis() % 1000) < 500 ? HIGH : LOW);
  } else {
    digitalWrite(LED_STATUS, LOW);
  }
}

// ----- Setup & Loop -----
void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println("Macropad starting...");

  // matrix
  pinMode(ROW_PIN, INPUT_PULLUP);
  for (int i = 0; i < 3; ++i) {
    pinMode(columnPins[i], OUTPUT);
    digitalWrite(columnPins[i], HIGH); // idle HIGH
  }

  // encoder
  pinMode(ENC_SW, INPUT_PULLUP);
  encoder.setPosition(0);

  // LED + battery pins
  pinMode(LED_STATUS, OUTPUT);
  pinMode(BAT_CHARGE_PIN, INPUT_PULLUP);
  pinMode(BAT_FULL_PIN, INPUT_PULLUP);
  pinMode(BAT_ADC_PIN, INPUT); // ADC

  // ADC resolution (optional)
  analogReadResolution(12); // 0..4095

  // BLE
  bleKeyboard.begin();
  Serial.println("BLE initialized; waiting for host");
}

void loop() {
  // Always scan and handle encoder so they are responsive
  scanKeys();
  handleEncoder();

  // update LED and battery
  handleBatteryLED();
  reportBatteryToHost();

  // debug BLE status occasionally
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 2000) {
    lastDebug = millis();
    if (bleKeyboard.isConnected()) Serial.println("BLE connected");
    else Serial.println("BLE advertising...");
  }

  delay(5); // small giveback
}
