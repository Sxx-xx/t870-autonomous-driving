// ============================================================
// 엔코더 배선 진단
//
// D2 ~ D6 을 전부 INPUT_PULLUP 으로 놓고 상태와 변화 횟수를 센다.
// 바퀴(또는 모터)를 돌렸을 때 어느 핀에 펄스가 들어오는지 본다.
//
// 읽는 법
//   D2/D3 만 변한다  -> 배선 정상. 본 스케치 문제를 다시 본다.
//   D5/D6 만 변한다  -> 엔코더가 아직 D5/D6 에 꽂혀 있다. D2/D3 로 옮긴다.
//   전부 안 변한다   -> 급전(빨강 5V / 검정 GND) 또는 엔코더 자체 문제.
//                      아래 A/B 레벨이 계속 1 이면 신호선이 안 물렸거나
//                      센서가 출력하지 않는 것이다.
//
// 주의: 이 스케치는 모터를 전혀 구동하지 않는다.
//       업로드 전에 본 스케치가 남긴 PWM 이 없도록 아두이노를
//       리셋(또는 재업로드) 하면 된다.
// ============================================================

const byte PINS[] = { 2, 3, 4, 5, 6 };
const byte PIN_COUNT = 5;

byte          lastState[PIN_COUNT];
unsigned long changeCount[PIN_COUNT];

const unsigned long PRINT_INTERVAL = 300;
unsigned long previousPrintTime = 0;


void setup()
{
  Serial.begin(115200);

  for (byte i = 0; i < PIN_COUNT; i++)
  {
    pinMode(PINS[i], INPUT_PULLUP);

    lastState[i]   = digitalRead(PINS[i]);
    changeCount[i] = 0;
  }

  Serial.println();
  Serial.println("==========================================");
  Serial.println("Encoder wiring check  (D2 ~ D6)");
  Serial.println("==========================================");
  Serial.println("format:  D<pin>:<level>(<change count>)");
  Serial.println("turn the wheel and watch which pin moves.");
  Serial.println("send any character to reset counters.");
  Serial.println();

  previousPrintTime = millis();
}


void loop()
{
  // 폴링. 인터럽트를 쓰지 않으므로 어느 핀이든 동등하게 본다.
  for (byte i = 0; i < PIN_COUNT; i++)
  {
    byte s = digitalRead(PINS[i]);

    if (s != lastState[i])
    {
      lastState[i] = s;
      changeCount[i]++;
    }
  }


  if (Serial.available() > 0)
  {
    while (Serial.available() > 0)
    {
      Serial.read();
    }

    for (byte i = 0; i < PIN_COUNT; i++)
    {
      changeCount[i] = 0;
    }

    Serial.println("-- counters reset --");
  }


  unsigned long now = millis();

  if (now - previousPrintTime >= PRINT_INTERVAL)
  {
    previousPrintTime = now;

    for (byte i = 0; i < PIN_COUNT; i++)
    {
      Serial.print("D");
      Serial.print(PINS[i]);
      Serial.print(":");
      Serial.print(lastState[i]);
      Serial.print("(");
      Serial.print(changeCount[i]);
      Serial.print(")  ");
    }

    Serial.println();
  }
}
