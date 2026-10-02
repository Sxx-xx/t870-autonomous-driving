// Standard RC PWM receiver -> I2C bridge for the T870 auxiliary Arduino Uno.
//
// Receiver wiring (signal and receiver GND are both required):
//   Steering signal    -> D4
//   Throttle signal    -> D5
//   AUTO/MANUAL signal -> D6
//   Receiver GND       -> auxiliary Arduino GND
//
// Inter-Arduino wiring (does not use either board's RX/TX pins):
//   Auxiliary A4/SDA -> main Arduino A4/SDA
//   Auxiliary A5/SCL -> main Arduino A5/SCL
//   Auxiliary GND    -> main Arduino GND
//
// The receiver outputs ordinary servo PWM pulses, normally 1000..2000 us at
// about 50 Hz. D4/D5/D6 are captured together with a pin-change interrupt so
// no channel is blocked while another channel is being measured.

#include <Arduino.h>
#include <Wire.h>

const byte STEERING_INPUT_PIN = 4;  // PD4 / PCINT20
const byte THROTTLE_INPUT_PIN = 5;  // PD5 / PCINT21
const byte MODE_INPUT_PIN     = 6;  // PD6 / PCINT22

const byte I2C_ADDRESS = 0x12;

const uint16_t PWM_VALID_MIN_US = 800;
const uint16_t PWM_VALID_MAX_US = 2200;
// 일반 50 Hz 리시버는 약 20 ms마다 새 펄스를 보낸다. 120 ms 동안
// 어느 한 채널이라도 이 시간 동안 들어오지 않으면 조종기/리시버 연결
// 단절로 보고 connected=false와 0 명령을 메인 보드로 전달한다.
// 짧은 무선/PWM 지연으로 주행 중 오검출하지 않도록 1초를 사용한다.
const uint32_t RC_TIMEOUT_US = 1000000UL;

// Initial standard-RC calibration. Read the serial output after upload and
// replace these values if the new receiver/transmitter endpoints differ.
const int STEERING_LEFT_US   = 1000;
const int STEERING_CENTER_US = 1500;
const int STEERING_RIGHT_US  = 2000;
const int STEERING_DEADBAND_US = 30;

const int THROTTLE_REVERSE_US = 1000;
const int THROTTLE_CENTER_US  = 1500;
const int THROTTLE_FORWARD_US = 2000;
const int THROTTLE_DEADBAND_US = 30;

const uint16_t MODE_MANUAL_MAX_US = 1300;
const uint16_t MODE_AUTO_MIN_US   = 1700;

const byte REMOTE_FLAG_AUTO       = 0x01;
const byte REMOTE_FLAG_MODE_VALID = 0x02;

// This is deliberately the same byte layout used by the previous auxiliary
// BLE/I2C bridge. The old flags field now carries AUTO and mode-valid bits.
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

volatile uint32_t riseUs[3] = {0, 0, 0};
volatile uint32_t lastPulseUs[3] = {0, 0, 0};
volatile uint16_t pulseWidthUs[3] = {0, 0, 0};
volatile byte previousPortD = 0;

volatile RemoteReport report = {
  REPORT_MAGIC, REPORT_VERSION, 0, 0, 0, 0, 0, 0, 0, 0
};

uint16_t modeRawUs = 0;
unsigned long lastPrintMs = 0;
bool lastSteeringFresh = false;
bool lastThrottleFresh = false;
bool lastModeFresh = false;
bool lastModeValid = false;

byte checksumOf(const RemoteReport &value)
{
  const byte *bytes = reinterpret_cast<const byte *>(&value);
  byte checksum = 0;
  for (size_t i = 0; i < sizeof(RemoteReport) - 1; ++i)
  {
    checksum ^= bytes[i];
  }
  return checksum;
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

void publishReport(uint16_t throttleRaw, uint16_t steeringRaw,
                   int throttleCommand, int steeringCommand,
                   byte flags, bool connected)
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

// D4, D5 and D6 all belong to the ATmega328P's PORTD pin-change group.
ISR(PCINT2_vect)
{
  const uint32_t nowUs = micros();
  const byte current = PIND;
  const byte changed = current ^ previousPortD;
  previousPortD = current;

  const byte masks[3] = {_BV(PD4), _BV(PD5), _BV(PD6)};
  for (byte i = 0; i < 3; ++i)
  {
    if (!(changed & masks[i])) continue;

    if (current & masks[i])
    {
      riseUs[i] = nowUs;
    }
    else
    {
      const uint32_t width = nowUs - riseUs[i];
      if (width >= PWM_VALID_MIN_US && width <= PWM_VALID_MAX_US)
      {
        pulseWidthUs[i] = (uint16_t)width;
        lastPulseUs[i] = nowUs;
      }
    }
  }
}

void updateReport()
{
  uint16_t steeringUs;
  uint16_t throttleUs;
  uint16_t modeUs;
  uint32_t steeringStamp;
  uint32_t throttleStamp;
  uint32_t modeStamp;

  noInterrupts();
  steeringUs = pulseWidthUs[0];
  throttleUs = pulseWidthUs[1];
  modeUs = pulseWidthUs[2];
  steeringStamp = lastPulseUs[0];
  throttleStamp = lastPulseUs[1];
  modeStamp = lastPulseUs[2];
  interrupts();

  const uint32_t nowUs = micros();
  const bool steeringFresh = steeringStamp != 0
    && (uint32_t)(nowUs - steeringStamp) <= RC_TIMEOUT_US;
  const bool throttleFresh = throttleStamp != 0
    && (uint32_t)(nowUs - throttleStamp) <= RC_TIMEOUT_US;
  const bool modeFresh = modeStamp != 0
    && (uint32_t)(nowUs - modeStamp) <= RC_TIMEOUT_US;

  bool modeValid = false;
  bool autoMode = false;
  if (modeFresh && modeUs <= MODE_MANUAL_MAX_US)
  {
    modeValid = true;
    autoMode = false;
  }
  else if (modeFresh && modeUs >= MODE_AUTO_MIN_US)
  {
    modeValid = true;
    autoMode = true;
  }

  const bool connected = steeringFresh && throttleFresh && modeFresh && modeValid;
  lastSteeringFresh = steeringFresh;
  lastThrottleFresh = throttleFresh;
  lastModeFresh = modeFresh;
  lastModeValid = modeValid;
  byte flags = 0;
  int throttleCommand = 0;
  int steeringCommand = 0;

  if (modeValid) flags |= REMOTE_FLAG_MODE_VALID;
  if (autoMode) flags |= REMOTE_FLAG_AUTO;

  // In AUTO the manual axes are deliberately forced to zero. The main board
  // may use the AUTO flag to select its autonomous command source.
  if (connected && !autoMode)
  {
    throttleCommand = normalizeAxis(
      throttleUs, THROTTLE_REVERSE_US, THROTTLE_CENTER_US,
      THROTTLE_FORWARD_US, THROTTLE_DEADBAND_US);
    steeringCommand = normalizeAxis(
      steeringUs, STEERING_LEFT_US, STEERING_CENTER_US,
      STEERING_RIGHT_US, STEERING_DEADBAND_US);
    // 현재 조종기/서보 기구 기준으로 수신기 조향 방향이 차량 조향과
    // 반대로 들어오므로 명령 부호만 뒤집는다. 스로틀에는 영향이 없다.
    steeringCommand = -steeringCommand;
  }

  modeRawUs = modeUs;
  publishReport(throttleUs, steeringUs, throttleCommand, steeringCommand,
                flags, connected);
}

void onI2cRequest()
{
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
  for (size_t i = 0; i < sizeof(RemoteReport); ++i)
  {
    destination[i] = source[i];
  }
  interrupts();

  Serial.print(F("RC:"));
  Serial.print(snapshot.connected ? F("OK") : F("LOST"));
  Serial.print(F(" ST:"));
  Serial.print(snapshot.steeringRaw);
  Serial.print('/');
  Serial.print(snapshot.steeringCommand);
  Serial.print(F(" TH:"));
  Serial.print(snapshot.throttleRaw);
  Serial.print('/');
  Serial.print(snapshot.throttleCommand);
  Serial.print(F(" MODE:"));
  Serial.print(modeRawUs);
  if (!(snapshot.flags & REMOTE_FLAG_MODE_VALID)) Serial.print(F("/INVALID"));
  else if (snapshot.flags & REMOTE_FLAG_AUTO) Serial.print(F("/AUTO"));
  else Serial.print(F("/MANUAL"));
  Serial.print(F(" FRESH:S="));
  Serial.print(lastSteeringFresh ? 1 : 0);
  Serial.print(F(" T="));
  Serial.print(lastThrottleFresh ? 1 : 0);
  Serial.print(F(" M="));
  Serial.print(lastModeFresh ? 1 : 0);
  Serial.print(F(" VALID="));
  Serial.print(lastModeValid ? 1 : 0);
  Serial.print(F(" SEQ:"));
  Serial.println(snapshot.sequence);
}

void setup()
{
  pinMode(STEERING_INPUT_PIN, INPUT);
  pinMode(THROTTLE_INPUT_PIN, INPUT);
  pinMode(MODE_INPUT_PIN, INPUT);

  previousPortD = PIND;
  PCICR |= _BV(PCIE2);
  PCMSK2 |= _BV(PCINT20) | _BV(PCINT21) | _BV(PCINT22);

  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(onI2cRequest);

  Serial.begin(115200);
  Serial.println(F("T870 standard PWM receiver bridge"));
  Serial.println(F("D4=steering D5=throttle D6=mode; I2C slave 0x12"));
  Serial.println(F("PWM timeout 120 ms -> E-STOP, axes zero, connected=false"));
}

void loop()
{
  updateReport();

  const unsigned long nowMs = millis();
  if (nowMs - lastPrintMs >= 200)
  {
    lastPrintMs = nowMs;
    printStatus();
  }
}
