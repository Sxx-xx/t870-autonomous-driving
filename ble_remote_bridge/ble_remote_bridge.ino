// BROON BLE remote receiver -> I2C bridge for an Arduino UNO-compatible board.
//
// Wiring:
//   BLE TX  -> D2 (SoftwareSerial RX, 3.3 V UART is accepted by the UNO)
//   BLE RX  -> not connected
//   BLE GND -> auxiliary UNO GND
//   BLE VDD -> regulated 3.3 V
//   A4/SDA  -> main UNO A4/SDA
//   A5/SCL  -> main UNO A5/SCL
//   GND     -> main UNO GND

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

const byte BLE_RX_PIN = 2;
const byte BLE_TX_UNUSED_PIN = 3;
const unsigned long BLE_BAUD = 9600;
const byte I2C_ADDRESS = 0x12;

const byte STX = 0x02;
const byte ETX = 0x03;
const byte PAYLOAD_LENGTH = 8;
const byte RC_STOP_MASK = 0x10;
const unsigned long RC_TIMEOUT_MS = 300;

const int THROTTLE_REVERSE = 877;
const int THROTTLE_CENTER = 939;
const int THROTTLE_FORWARD = 1088;
const int THROTTLE_DEADBAND = 5;

const int STEERING_LEFT = 566;
const int STEERING_CENTER = 897;
const int STEERING_RIGHT = 1248;
const int STEERING_DEADBAND = 8;

SoftwareSerial bleSerial(BLE_RX_PIN, BLE_TX_UNUSED_PIN);

// The main UNO requests exactly sizeof(RemoteReport) bytes. All multi-byte
// raw values are little-endian on both AVR boards. Normalized commands use
// explicit signed 8-bit storage (-100..100).
struct __attribute__((packed)) RemoteReport
{
  byte magic;
  byte version;
  byte sequence;
  int8_t throttleCommand;
  int8_t steeringCommand;
  byte flags;
  byte connected;
  uint16_t throttleRaw;
  uint16_t steeringRaw;
  byte checksum;
};

const byte REPORT_MAGIC = 0xB7;
const byte REPORT_VERSION = 1;

volatile RemoteReport report = {
  REPORT_MAGIC, REPORT_VERSION, 0, 0, 0, 0, 0, 0, 0, 0
};

char payload[PAYLOAD_LENGTH + 1];
byte payloadIndex = 0;
bool receiving = false;
unsigned long lastValidPacketMs = 0;
unsigned long lastPrintMs = 0;
unsigned long receivedByteCount = 0;
unsigned long validPacketCount = 0;
byte lastReceivedByte = 0;

int hexValue(char character)
{
  if (character >= '0' && character <= '9') return character - '0';
  if (character >= 'A' && character <= 'F') return character - 'A' + 10;
  if (character >= 'a' && character <= 'f') return character - 'a' + 10;
  return -1;
}

bool decodeHex(const char *text, byte length, uint16_t &value)
{
  value = 0;
  for (byte index = 0; index < length; ++index)
  {
    int digit = hexValue(text[index]);
    if (digit < 0) return false;
    value = (uint16_t)((value << 4) | (uint16_t)digit);
  }
  return true;
}

int normalizeAxis(int raw, int low, int center, int high, int deadband)
{
  if (raw >= center - deadband && raw <= center + deadband) return 0;

  if (raw > center)
  {
    long value = map(raw, center + deadband, high, 0, 100);
    return constrain((int)value, 0, 100);
  }

  long value = map(raw, center - deadband, low, 0, -100);
  return constrain((int)value, -100, 0);
}

byte checksumOf(const RemoteReport &value)
{
  const byte *bytes = reinterpret_cast<const byte *>(&value);
  byte checksum = 0;
  for (size_t index = 0; index < sizeof(RemoteReport) - 1; ++index)
  {
    checksum ^= bytes[index];
  }
  return checksum;
}

void publishReport(uint16_t throttleRaw, uint16_t steeringRaw, byte flags,
                   int throttleCommand, int steeringCommand, bool connected)
{
  RemoteReport next;
  next.magic = REPORT_MAGIC;
  next.version = REPORT_VERSION;
  next.sequence = report.sequence + 1;
  next.throttleCommand = (int8_t)throttleCommand;
  next.steeringCommand = (int8_t)steeringCommand;
  next.flags = flags;
  next.connected = connected ? 1 : 0;
  next.throttleRaw = throttleRaw;
  next.steeringRaw = steeringRaw;
  next.checksum = checksumOf(next);

  noInterrupts();
  report = next;
  interrupts();
}

void processPacket()
{
  uint16_t throttleRaw;
  uint16_t steeringRaw;
  uint16_t flagsValue;
  if (!decodeHex(payload, 3, throttleRaw)
      || !decodeHex(payload + 3, 3, steeringRaw)
      || !decodeHex(payload + 6, 2, flagsValue))
  {
    return;
  }

  int throttle = normalizeAxis(
    throttleRaw, THROTTLE_REVERSE, THROTTLE_CENTER,
    THROTTLE_FORWARD, THROTTLE_DEADBAND);
  int steering = normalizeAxis(
    steeringRaw, STEERING_LEFT, STEERING_CENTER,
    STEERING_RIGHT, STEERING_DEADBAND);

  lastValidPacketMs = millis();
  ++validPacketCount;
  publishReport(
    throttleRaw, steeringRaw, (byte)flagsValue,
    throttle, steering, true);
}

void readBleRemote()
{
  while (bleSerial.available() > 0)
  {
    byte data = (byte)bleSerial.read();
    ++receivedByteCount;
    lastReceivedByte = data;

    if (data == STX)
    {
      receiving = true;
      payloadIndex = 0;
    }
    else if (data == ETX && receiving)
    {
      receiving = false;
      if (payloadIndex == PAYLOAD_LENGTH)
      {
        payload[PAYLOAD_LENGTH] = '\0';
        processPacket();
      }
      payloadIndex = 0;
    }
    else if (receiving)
    {
      if (payloadIndex < PAYLOAD_LENGTH && hexValue((char)data) >= 0)
      {
        payload[payloadIndex++] = (char)data;
      }
      else
      {
        receiving = false;
        payloadIndex = 0;
      }
    }
    // 0x00 after ETX and all bytes outside a frame are ignored.
  }
}

void updateFailsafe()
{
  bool connected;
  noInterrupts();
  connected = report.connected != 0;
  interrupts();

  if (connected && millis() - lastValidPacketMs > RC_TIMEOUT_MS)
  {
    // Clear flags as well. A stale STOP bit must never request AUTO mode.
    publishReport(0, 0, 0, 0, 0, false);
  }
}

void onI2cRequest()
{
  // Wire invokes this callback with interrupts already disabled, so report
  // cannot be changed by loop() while these bytes are copied.
  Wire.write(
    reinterpret_cast<const uint8_t *>(
      const_cast<const RemoteReport *>(&report)),
    sizeof(RemoteReport));
}

void printStatus()
{
  RemoteReport snapshot;
  noInterrupts();
  byte *destination = reinterpret_cast<byte *>(&snapshot);
  volatile const byte *source =
    reinterpret_cast<volatile const byte *>(&report);
  for (size_t index = 0; index < sizeof(RemoteReport); ++index)
  {
    destination[index] = source[index];
  }
  interrupts();

  Serial.print(F("BLE:"));
  Serial.print(snapshot.connected ? F("OK") : F("LOST"));
  Serial.print(F(" TH:"));
  Serial.print(snapshot.throttleRaw);
  Serial.print(F("/"));
  Serial.print(snapshot.throttleCommand);
  Serial.print(F(" ST:"));
  Serial.print(snapshot.steeringRaw);
  Serial.print(F("/"));
  Serial.print(snapshot.steeringCommand);
  Serial.print(F(" FLAGS:0x"));
  if (snapshot.flags < 0x10) Serial.print('0');
  Serial.print(snapshot.flags, HEX);
  if (snapshot.flags & RC_STOP_MASK) Serial.print(F(" AUTO_REQUEST"));
  Serial.print(F(" SEQ:"));
  Serial.print(snapshot.sequence);
  Serial.print(F(" BYTES:"));
  Serial.print(receivedByteCount);
  Serial.print(F(" PACKETS:"));
  Serial.print(validPacketCount);
  Serial.print(F(" LAST:0x"));
  if (lastReceivedByte < 0x10) Serial.print('0');
  Serial.println(lastReceivedByte, HEX);
}

void setup()
{
  Serial.begin(115200);
  bleSerial.begin(BLE_BAUD);

  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(onI2cRequest);

  Serial.println(F("BROON BLE remote I2C bridge"));
  Serial.println(F("BLE D2 RX 9600 baud; I2C slave address 0x12"));
  Serial.println(F("Remote timeout 300 ms -> commands zero, connected=false"));
}

void loop()
{
  readBleRemote();
  updateFailsafe();

  unsigned long now = millis();
  if (now - lastPrintMs >= 200)
  {
    lastPrintMs = now;
    printStatus();
  }
}
