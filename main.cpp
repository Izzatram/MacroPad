#include <Arduino.h>
#include <BleKeyboard.h>
#include <RotaryEncoder.h>
#include "nvs_flash.h"

// ========== BLE Keyboard ==========
BleKeyboard bleKeyboard("Macropad", "YourCompany", 100);

// ========== Pin Definitions ==========
#define ROW_PIN         25
#define COL0_PIN        26
#define COL1_PIN        27
#define COL2_PIN        32

#define ENC_CLK         14
#define ENC_DT          12
#define ENC_SW          13

#define LED_STATUS      4

#define BAT_CHARGE_PIN  34
#define BAT_FULL_PIN    35

// ========== Globals ==========
bool profile = 0;  // 0 = Profile 1, 1 = Profile 2
unsigned long switch1_press_time = 0;
bool switch1_long_press_triggered = false;

RotaryEncoder encoder(ENC_DT, ENC_CLK, RotaryEncoder::LatchMode::TWO03);
int lastEncoderPos = 0;

// Key matrix state
const uint8_t columnPins[3] = {COL0_PIN, COL1_PIN, COL2_PIN};
bool keyState[3] = {false, false, false};
const unsigned long debounceDelay = 50;
unsigned long lastDebounceTime[3] = {0, 0, 0};

// ========== Utility ==========
void resetBLE() {
  Serial.println("Resetting BLE pairing info...");
  nvs_flash_erase();   // Erase NVS
  nvs_flash_init();    // Re-init NVS
  ESP.restart();       // Restart ESP32
}

// ========== Matrix Scanning ==========
void scanKeys() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(columnPins[i], LOW);
    delayMicroseconds(5);

    bool pressed = !digitalRead(ROW_PIN); // active low
    unsigned long now = millis();

    if (pressed != keyState[i] && (now - lastDebounceTime[i]) > debounceDelay) {
      keyState[i] = pressed;
      lastDebounceTime[i] = now;

      if (pressed) {
        switch (i) {
          case 0:  // Switch 1
            switch1_press_time = now;
            switch1_long_press_triggered = false;
            break;

          case 1:  // Switch 2
            if (profile == 0) {
              bleKeyboard.write(KEY_LEFT_ARROW);
            } else {
              bleKeyboard.write(KEY_MEDIA_MUTE);
            }
            break;

          case 2:  // Switch 3
            bleKeyboard.write(KEY_RIGHT_ARROW); // same in both profiles
            break;
        }
      } else {
        // On release
        if (i == 0) {
          unsigned long pressDuration = now - switch1_press_time;
          if (pressDuration > 1000) {
            if (!switch1_long_press_triggered) {
              switch1_long_press_triggered = true;
              resetBLE();
            }
          } else {
            // Short press: switch profile
            profile = !profile;
            Serial.print("Switched to profile: ");
            Serial.println(profile);
          }
        }
      }
    }

    digitalWrite(columnPins[i], HIGH);
  }
}

// ========== Encoder Handling ==========
void handleEncoder() {
  encoder.tick();
  int newPos = encoder.getPosition();

  if (newPos != lastEncoderPos) {
    if (newPos > lastEncoderPos) {
      // Clockwise
      if (profile == 0) {
        bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
      } else {
        bleKeyboard.write(KEY_MEDIA_NEXT_TRACK);
      }
    } else {
      // Counter-clockwise
      if (profile == 0) {
        bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
      } else {
        bleKeyboard.write(KEY_MEDIA_PREVIOUS_TRACK);
      }
    }
    lastEncoderPos = newPos;
  }

  // Handle encoder button (play/pause)
  static bool lastSW = HIGH;
  bool currentSW = digitalRead(ENC_SW);

  if (lastSW == HIGH && currentSW == LOW) {
    delay(10);  // debounce
    if (digitalRead(ENC_SW) == LOW) {
      bleKeyboard.write(KEY_MEDIA_PLAY_PAUSE);
    }
  }

  lastSW = currentSW;
}

// ========== Battery LED ==========
void handleBatteryLED() {
  int charge = digitalRead(BAT_CHARGE_PIN);  // 0 = charging
  int full = digitalRead(BAT_FULL_PIN);      // 0 = full

  if (full == 0) {
    digitalWrite(LED_STATUS, HIGH); // ON (full)
  } else if (charge == 0) {
    digitalWrite(LED_STATUS, millis() % 1000 < 500); // Blink (charging)
  } else {
    digitalWrite(LED_STATUS, LOW); // Off (idle)
  }
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);

  // Matrix setup
  pinMode(ROW_PIN, INPUT_PULLUP);
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(columnPins[i], OUTPUT);
    digitalWrite(columnPins[i], HIGH);
  }

  // Encoder
  pinMode(ENC_SW, INPUT_PULLUP);
  encoder.setPosition(0);

  // Battery LED and status
  pinMode(LED_STATUS, OUTPUT);
  pinMode(BAT_CHARGE_PIN, INPUT_PULLUP);
  pinMode(BAT_FULL_PIN, INPUT_PULLUP);

  // Start BLE
  bleKeyboard.begin();
}

// ========== Main Loop ==========
void loop() {
  scanKeys();
  handleEncoder();
  handleBatteryLED();
  delay(5);  // small delay for stability
}
