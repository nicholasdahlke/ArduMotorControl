#include <Arduino.h>

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
    void set_rpm(float rpm_val);
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
    float rpm;
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

void Stepper::set_rpm(float rpm_val)
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
  Serial.println();
}

#define STEP 11
#define DIR 10
#define RATIO 5

int incoming_byte = 0;
bool motor_running = false;
float period = 1;
float max_val = 64;
int steps_rev = 400;
int manual_movement_size = 10;
uint8_t ser_buffer;
float time = 0;
Stepper* motor;
void setup()
{
  Serial.begin(9600);
  motor = new Stepper(STEP, DIR, steps_rev);
  motor->set_gear_ratio(RATIO);
  motor->set_rpm(200);
  
  time = millis();
}

void handleSerial()
{
  Serial.println(ser_buffer);
  if(ser_buffer == 99)  // Turn on motor
    motor_running = true;
  if (ser_buffer == 100) // Turn off motor
    motor_running = false;
  if(ser_buffer == 98) // Set max angle
    {
      float temp = Serial.parseFloat();
      if(temp > 0 && temp < 360)
        max_val = temp;
    }
  if(ser_buffer == 97) // Set period
    period = Serial.parseFloat();
  
  if(ser_buffer == 101) //Go forward
  {
    motor->set_rpm(60);
    motor->goto_angle_pos_rel(manual_movement_size);
  }
  if(ser_buffer == 102) //Go backwards
  {
    motor->set_rpm(60);
    motor->goto_angle_pos_rel(-1 * manual_movement_size);
  }
  if(ser_buffer == 103) //Set manual step size
    manual_movement_size = Serial.parseFloat();


}

void readSerial()
{
  if(Serial.available() > 0)
  {
    ser_buffer = Serial.read();
    if(ser_buffer != 10) //Linefeed
      handleSerial();
  }
}

unsigned long iterator = 0;

void loop() 
{ 
  readSerial();
  if(motor_running)
  {
    iterator = millis();
    time = static_cast<float>(iterator) / 1000;

    float pos = max_val * sin(((2 * PI) / period) * (time));
    float rpm = abs(max_val * cos(((2 * PI) / period) * (time))) + 0.5;
    motor->set_rpm(rpm);
    motor->goto_angle_pos_abs(pos);
    Serial.println(pos);
  }
}

//Pointer to a timer object, which the stepper class checks regularly