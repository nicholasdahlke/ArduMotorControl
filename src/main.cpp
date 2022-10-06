#include <Arduino.h>
#include <math.h>

class Stepper
{
  public:
    Stepper(uint8_t step, uint8_t dir, int steps_rev);
    ~Stepper();

    enum direction
    {
      forward = HIGH,
      reverse = LOW
    };

    void goto_angle_pos_abs(float pos);
    void goto_angle_pos_rel(float pos);
    float get_pos();
    void set_rpm(int rpm_val);
    void set_gear_ratio(float ratio_val);
    void print_info();


  private:
    void intialize();
    void calculate_step_time();
    void calculate_angular_position();
    void calculate_step_angle_factor();
    int do_steps(int steps_to_do);
    int current_pos_steps;
    float current_pos_angle;
    uint8_t step_pin;
    uint8_t dir_pin;
    int steps_per_rev;
    int step_time;
    int rpm;
    float ratio;
    float step_angle_factor;


};

Stepper::Stepper(uint8_t step, uint8_t dir, int steps_rev)
{
  step_pin = step;
  dir_pin = dir;
  steps_per_rev = steps_rev;

  set_rpm(60);
  set_gear_ratio(1);
  calculate_step_angle_factor();
  intialize();
}

float Stepper::get_pos()
{
  return current_pos_angle;
}

void Stepper::intialize()
{
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);

  digitalWrite(dir_pin, forward);
  digitalWrite(step_pin, HIGH);

  current_pos_steps = 0;
  current_pos_angle = 0;
}

void Stepper::set_rpm(int rpm_val)
{
  rpm = rpm_val;
  calculate_step_time();
}

void Stepper::set_gear_ratio(float ratio_val)
{
  if(ratio_val > 0.0f)
    ratio = ratio_val;
  calculate_step_time();
  calculate_step_angle_factor();
}

void Stepper::calculate_step_time()
{
  float step_time_seconds = 60 / (2 * rpm * ratio * steps_per_rev);
  step_time = 1e6 * step_time_seconds;
}

void Stepper::calculate_angular_position()
{
  current_pos_angle = current_pos_steps * (360/steps_per_rev);
}

void Stepper::calculate_step_angle_factor()
{
  float temp = static_cast<float>(steps_per_rev) * ratio;
  step_angle_factor = 360.0f / temp;
}

int Stepper::do_steps(int steps_to_do)
{
  int steps = abs(steps_to_do);
  direction dir = steps_to_do > 0 ? forward : reverse;
  digitalWrite(dir_pin, dir);
  for (int i = 0; i < steps; i++)
  {
    digitalWrite(step_pin, LOW);
    delayMicroseconds(step_time);
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(step_time);
  }
  return steps_to_do;
}

void Stepper::goto_angle_pos_rel(float pos)
{
  int steps_to_go = static_cast<int>(pos / step_angle_factor);
  current_pos_steps += do_steps(steps_to_go);
  calculate_angular_position();
}

void Stepper::goto_angle_pos_abs(float pos)
{
  int steps_to_go = static_cast<int>(pos / step_angle_factor) - current_pos_steps;
  current_pos_steps += do_steps(steps_to_go);
  calculate_angular_position();
}

void Stepper::print_info()
{
  Serial.print("Step pin: ");
  Serial.println(step_pin);
  Serial.print("Dir  pin: ");
  Serial.println(dir_pin);
  Serial.print("Step angle factor: ");
  Serial.println(step_angle_factor);
  Serial.print("Step time: ");
  Serial.println(step_time);
  Serial.print("Steps per rev: ");
  Serial.println(steps_per_rev);
  Serial.print("Ratio: ");
  Serial.println(ratio);
  Serial.print("RPM: ");
  Serial.println(rpm);
  Serial.print("Curr Pos Angle: ");
  Serial.println(current_pos_angle);
  Serial.print("Curr Pos Steps: ");
  Serial.println(current_pos_steps);
}

#define STEP 11
#define DIR 10
#define STEPS_PER_REV 400
#define RATIO 5

constexpr int curve_samples = 360 * 4;
int incoming_byte = 0;
bool motor_running = false;

float angles[curve_samples];
float rpms[curve_samples];
float x[curve_samples];

void calculate_angle_curve_sin(float max, float period)
{
    int iterator = 0;
    for (float i = 0; i<=5; i += 5.0f / curve_samples) {
        float b = (M_PI * 2) / period;
        angles[iterator] = sin(b*i) * max;
        x[iterator] = i;
        iterator++;
    }
}

void calculate_angular_velocity_curve()
{
  for (int i = 1; i < curve_samples; ++i) {
      float diff_angle = angles[i] - angles[i-1];
      float diff_time = x[i] - x[i-1];
      float angular_vel =  diff_angle / diff_time;
      if(diff_angle == 0.0f)
          rpms[i-1] = 0;
      else
        rpms[i-1] = angular_vel / 6;
  }
}

Stepper* motor;
void setup()
{
  Serial.begin(9600);
  motor = new Stepper(STEP, DIR, STEPS_PER_REV);
  motor->set_gear_ratio(RATIO);
  motor->set_rpm(200);
  motor->print_info();
}

void loop() 
{
  if(Serial.available() > 0)
  {
    incoming_byte = Serial.read();
    Serial.println(incoming_byte);
    if(incoming_byte == 99)
    {
      motor_running = true;
      Serial.println("Calculating curve");
      calculate_angle_curve_sin(60, 2);
      calculate_angular_velocity_curve();
      for (int i = 0; i < curve_samples; i++)
      {
        Serial.println(angles[i]);
      }
      
    }
    if(incoming_byte == 100)
      motor_running = false;
  }

  if(motor_running)
  {
    motor->goto_angle_pos_rel(60);
  }
}