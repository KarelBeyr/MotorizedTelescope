
#include <Arduino.h>
#include <BleKeyboard.h>

BleKeyboard bleKeyboard("Panorama Shut2", "DIY", 100);

void setup() {
  Serial.begin(115200);
  bleKeyboard.begin();

  Serial.println("ESP32 shutter controller ready");
  Serial.println("Commands: c = capture, ? = status");
}

void loop() {
  while (Serial.available()) {
    char command = Serial.read();

    switch (command) {
      case 'c':
        if (!bleKeyboard.isConnected()) {
          Serial.println("ERR: Bluetooth disconnected");
          break;
        }

        bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
        Serial.println("OK: Shutter key sent");
        break;

      case '?':
        Serial.println(
          bleKeyboard.isConnected()
            ? "CONNECTED"
            : "DISCONNECTED"
        );
        break;
    }
  }

  delay(1);
}
