#include <cobalt/cobalt.hpp>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <stdio.h>

using cobalt::math::linear_algebra::Vector;
using cobalt::math::linear_algebra::Vector;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

float angle2micros(float rad);

constexpr float q1_0 = 0.0f;
constexpr float q2_0 = M_PI_2;

constexpr float q1Min = -M_PI_2;
constexpr float q1Max = M_PI_2;
constexpr float q2Min = -M_PI_4;
constexpr float q2Max = M_PI_4;

constexpr float L1 = 0.12f;
constexpr float L2 = 0.13f;

Vector<2> x{L1, L2};
Vector<2> q{};
Vector<2> x_d{0.0f, 0.0f};


void setup() {
  Serial.begin(9600);

  pwm.begin();
 
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  pwm.writeMicroseconds(0, angle2micros(0));
  pwm.writeMicroseconds(1, angle2micros(M_PI_4));

  delay(20);
}

Vector<2> getQValues(const Vector<2> &x_d) {
    Vector<2> q;
    q[1] = std::acos((x_d[0]*x_d[0] + x_d[1]*x_d[1] - L1*L1 - L2*L2)/(2*L1*L2));
    q[0] = std::atan2(x_d[1], x_d[0]) - std::atan2(L2*std::sin(q[1]), L1 + L2*std::cos(q[1]));

    // Conversion per my model and offsets
    q[0] *= -1;
    q[1] -= M_PI_2;

    if(q[0] < q1Min) { q[0] = q1Min; Serial.printf("[ ERROR | q1 saturation-min ]"); }
    if(q[0] > q1Max) { q[0] = q1Max; Serial.printf("[ ERROR | q1 saturation-max ]"); }
    if(q[1] < q2Min) { q[1] = q2Min; Serial.printf("[ ERROR | q2 saturation-min ]"); }
    if(q[1] > q2Max) { q[1] = q2Max; Serial.printf("[ ERROR | q2 saturation-max ]"); }

    return q;
}

float angle2micros(float rad) {
  rad += M_PI_2;

  rad *= (USMAX - USMIN)/(M_PI);
  rad += USMIN;

  return rad;
}

void loop() {

  for(float i = 0.15f; i <= 0.20; i += 0.001) {
    x_d[1] = i;
    q = getQValues(x_d);
    printf("[ %3.2f %3.2f ]\n", q[0], q[1]);
    pwm.writeMicroseconds(0, angle2micros(q[0]));
    pwm.writeMicroseconds(1, angle2micros(q[1]));
  }

  for(float i = 0.20f; i >= 0.15; i -= 0.001) {
    x_d[1] = i;
    q = getQValues(x_d);
    printf("[ %3.2f %3.2f ]\n", q[0], q[1]);
    pwm.writeMicroseconds(0, angle2micros(q[0]));
    pwm.writeMicroseconds(1, angle2micros(q[1]));
  }

}