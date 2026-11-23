/**
 * S3 ↔ MSPM0G3507 UART1 通信测试
 * S3 GPIO16(RX) ← MSPM0 PA8(TX)
 * S3 GPIO17(TX) → MSPM0 PA9(RX)
 */
#include <Arduino.h>

#define S3_RX 16
#define S3_TX 17

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, S3_RX, S3_TX);
    delay(500);
    Serial.println("\n=== S3 UART Test ===");
    Serial.printf("RX=GPIO%d TX=GPIO%d\n", S3_RX, S3_TX);
}

void loop() {
    while(Serial2.available()) {
        Serial.write(Serial2.read());
    }
    while(Serial.available()) {
        Serial2.write(Serial.read());
    }
    static unsigned long last = 0;
    if(millis() - last > 2000) {
        last = millis();
        Serial2.print("hello from S3\r\n");
        Serial.println("[sent: hello]");
    }
}
