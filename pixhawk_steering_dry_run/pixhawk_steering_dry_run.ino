// T870 Pixhawk-to-Arduino steering dry-run diagnostic.
// This sketch NEVER drives the steering motor. It only reads:
//   D8: Pixhawk PWM command
//   A0: steering potentiometer
// and prints the direction the full controller would request.

#include <Arduino.h>

const byte PIXHAWK_PWM_PIN = 8;
const byte STEER_POT_PIN   = A0;

const int PWM_AT_LEFT   = 1100;
const int PWM_AT_CENTER = 1500;
const int PWM_AT_RIGHT  = 1900;

const int POT_AT_LEFT   = 55;   // software limit
const int POT_AT_CENTER = 485;
const int POT_AT_RIGHT  = 969;  // software limit
const int DEADBAND      = 15;

int pwmToTargetPot(unsigned long pwmUs)
{
  long limited = constrain((long)pwmUs, PWM_AT_LEFT, PWM_AT_RIGHT);

  if (limited <= PWM_AT_CENTER)
  {
    return (int)map(limited,
                    PWM_AT_LEFT, PWM_AT_CENTER,
                    POT_AT_LEFT, POT_AT_CENTER);
  }

  return (int)map(limited,
                  PWM_AT_CENTER, PWM_AT_RIGHT,
                  POT_AT_CENTER, POT_AT_RIGHT);
}

void setup()
{
  Serial.begin(115200);
  pinMode(PIXHAWK_PWM_PIN, INPUT);
  pinMode(STEER_POT_PIN, INPUT);

  // D7/D9 are deliberately left as inputs: no DIR/PWM motor output.
  Serial.println();
  Serial.println("T870 STEERING DRY RUN - MOTOR OUTPUT DISABLED");
  Serial.println("PWM | POT | TARGET | DECISION");
}

void loop()
{
  unsigned long pwmUs = pulseIn(PIXHAWK_PWM_PIN, HIGH, 30000UL);
  int potValue = analogRead(STEER_POT_PIN);

  if (pwmUs < 800UL || pwmUs > 2200UL)
  {
    Serial.print("NO SIGNAL | POT:");
    Serial.print(potValue);
    Serial.println(" | DECISION:STOP");
    delay(100);
    return;
  }

  int targetPot = pwmToTargetPot(pwmUs);
  int error = targetPot - potValue;

  Serial.print("PWM:");
  Serial.print(pwmUs);
  Serial.print(" | POT:");
  Serial.print(potValue);
  Serial.print(" | TARGET:");
  Serial.print(targetPot);
  Serial.print(" | DECISION:");

  if (abs(error) <= DEADBAND)
  {
    Serial.println("STOP");
  }
  else if (error > 0)
  {
    Serial.println("RIGHT / MA / DIR LOW");
  }
  else
  {
    Serial.println("LEFT / MB / DIR HIGH");
  }

  delay(100);
}
