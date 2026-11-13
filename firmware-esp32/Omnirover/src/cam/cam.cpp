#include "cam.h"
#include "pins.h"

static HardwareSerial piSerial(2);
static String lineBuf;
static bool discardingLine = false; // oversized line: skip to the next terminator
static uint32_t lastByteMs = 0;
static const size_t PI_LINE_MAX = 512; // Pi sends ~200-300 byte JSON lines
static const uint32_t PARTIAL_FLUSH_MS = 200; // echo unterminated data so a missing '\n' isn't mistaken for silence

void cam_init()
{
    piSerial.setRxBufferSize(1024); // must precede begin(); 10 Hz x ~300 B JSON would overrun the 256 B default
    piSerial.begin(PI_UART_BAUD, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);
    lineBuf.reserve(PI_LINE_MAX);
}

void cam_poll()
{
    while (piSerial.available() > 0) {
        char c = (char)piSerial.read();
        lastByteMs = millis();
        if (c == '\n' || c == '\r') {
            if (discardingLine) {
                Serial.println("[overflow] line > 512 bytes dropped");
                discardingLine = false;
            } else if (lineBuf.length() > 0) {
                Serial.println(lineBuf);
            }
            lineBuf = "";
        } else if (discardingLine) {
            continue;
        } else if (lineBuf.length() < PI_LINE_MAX) {
            lineBuf += c;
        } else {
            lineBuf = "";
            discardingLine = true;
        }
    }
    if (lineBuf.length() > 0 && millis() - lastByteMs > PARTIAL_FLUSH_MS) {
        Serial.print("[partial] ");
        Serial.println(lineBuf);
        lineBuf = "";
    }
}