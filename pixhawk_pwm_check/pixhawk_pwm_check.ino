// Pixhawk FMU PWM OUT 1 -> Arduino Uno D8 pulse-width checker.
// Wiring: PWM signal -> D8, PWM GND -> Arduino GND, PWM + left open.

#include <Arduino.h>

const byte PIXHAWK_PWM_PIN = 8;

void setup()
{
  Serial.begin(115200);
  pinMode(PIXHAWK_PWM_PIN, INPUT);

  Serial.println();
  Serial.println("Pixhawk PWM check on D8");
  Serial.println("Expected: about 1000..2000 us");
}

void loop()
{
  // A normal servo frame repeats about every 20 ms. Return 0 when no
  // complete HIGH pulse is received within 30 ms.
  unsigned long pwmUs = pulseIn(PIXHAWK_PWM_PIN, HIGH, 30000UL);

  if (pwmUs >= 800UL && pwmUs <= 2200UL)
  {
    Serial.print("PWM = ");
    Serial.print(pwmUs);
    Serial.println(" us");
  }
  else if (pwmUs == 0UL)
  {
    Serial.println("NO SIGNAL");
  }
  else
  {
    Serial.print("INVALID PWM = ");
    Serial.print(pwmUs);
    Serial.println(" us");
  }

  delay(100);
}
