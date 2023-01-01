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
    void goto_angle_pos_abs_direct(float pos);
    void goto_angle_pos_rel_direct(float pos);
    float get_pos();
    void set_pos(int offset = 0);
    void set_rpm(float rpm_val);
    void set_gear_ratio(float ratio_val);
    void print_info();
    void run();
    void clear_steps();
    int get_remaining();

    bool continuous_stepping = false;

  private:
    void intialize();
    void calculate_step_time();
    void calculate_angular_position();
    void calculate_step_angle_factor();
    int do_steps(int steps_to_do);
    int do_steps_direct(int steps_to_do);
    void do_step();
    void set_dir();
    void set_dir_from_steps(int steps_to_do);
    int current_pos_steps;
    float current_pos_angle;
    uint8_t step_pin;
    uint8_t dir_pin;
    int steps_per_rev;
    unsigned long step_time;
    unsigned long step_time_last;
    float rpm;
    float ratio;
    float step_angle_factor;
    int remaining_steps = 0;
    direction dir;

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

void Stepper::set_pos(int offset)

{
  current_pos_steps = offset;
  calculate_angular_position();
}

void Stepper::set_dir()
{
  bool dir_changed = false;
  if((rpm > 0 ? forward : reverse) != dir)
    dir_changed = true;
  dir = rpm > 0 ? forward : reverse;
  if(dir_changed)
    digitalWrite(dir_pin, dir);
}

void Stepper::set_dir_from_steps(int steps_to_do)
{
  bool dir_changed = false;
  if((steps_to_do > 0 ? forward : reverse) != dir)
    dir_changed = true;
  dir = steps_to_do > 0 ? forward : reverse;
  if(dir_changed)
    digitalWrite(dir_pin, dir);
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

int Stepper::get_remaining()
{
  return remaining_steps;
}

void Stepper::set_rpm(float rpm_val)
{
  rpm = rpm_val;
  calculate_step_time();
  if(continuous_stepping)
    set_dir();
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
  //float step_time_seconds = 60 / (2 * rpm * ratio * steps_per_rev);
  float step_time_seconds = ((21600.0f / (abs(rpm))) - 2.88e-3f * ratio * static_cast<float>(steps_per_rev)) / (360.0f * static_cast<float>(steps_per_rev) * ratio);
  step_time = static_cast<unsigned long>(1e6f * step_time_seconds);
}

void Stepper::calculate_angular_position()
{
  current_pos_angle = static_cast<float>(current_pos_steps) * step_angle_factor;
}

void Stepper::calculate_step_angle_factor()
{
  float temp = static_cast<float>(steps_per_rev) * ratio;
  step_angle_factor = 360.0f / temp;
}

int Stepper::do_steps(int steps_to_do)
{
  int steps = abs(steps_to_do);
  if(continuous_stepping)
    set_dir();
  else
    set_dir_from_steps(steps_to_do);
  remaining_steps += steps;
  return steps_to_do;
}

int Stepper::do_steps_direct(int steps_to_do)
{
  int ret = do_steps(steps_to_do);
  while (get_remaining() != 0)
  {
    run();
  }
  return ret;
}

void Stepper::clear_steps()
{
  remaining_steps = 0;
}

void Stepper::do_step()
{
  digitalWrite(step_pin, LOW);
  delayMicroseconds(8);
  digitalWrite(step_pin, HIGH);
}

void Stepper::goto_angle_pos_rel(float pos)
{
  int steps_to_go = static_cast<int>(pos / step_angle_factor);
  do_steps(steps_to_go);
}

void Stepper::goto_angle_pos_abs(float pos)
{
  int steps_to_go = static_cast<int>(pos / step_angle_factor) - current_pos_steps;
  do_steps(steps_to_go);
}

void Stepper::goto_angle_pos_rel_direct(float pos)
{
  int steps_to_go = static_cast<int>(pos / step_angle_factor);
  do_steps_direct(steps_to_go);
}

void Stepper::goto_angle_pos_abs_direct(float pos)
{
  int steps_to_go = static_cast<int>(pos / step_angle_factor) - current_pos_steps;
  do_steps_direct(steps_to_go);
}

void Stepper::run()
{
  //if((micros() - step_time_last) >= step_time)  
  if((remaining_steps > 0 || continuous_stepping) && (micros() - step_time_last) >= step_time)
  {
  do_step();
  remaining_steps--;
  step_time_last = micros();
  if(dir == forward)
    current_pos_steps++;
  else
    current_pos_steps--;
  calculate_angular_position();
  }
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

#define STEP 11 // PB5 - 11
#define DIR 10 // PB4 - 10
#define RATIO 5

enum motorMotionType
{
  LINEAR,
  SINUSOIDAL
};

int incoming_byte = 0;
bool motor_running = false;
float period = 2.0;
float max_val = 25.0;
int steps_rev = 440;
int manual_movement_size = 10;
uint8_t ser_buffer;

Stepper* motor;

motorMotionType motion = SINUSOIDAL;

void setup()
{
  Serial.begin(1000000);
  motor = new Stepper(STEP, DIR, steps_rev);
  motor->set_gear_ratio(RATIO);
  motor->set_rpm(200);
}

void handleSerial()
{
  if(ser_buffer == 99)  // Turn on motor
    motor_running = true;

  if (ser_buffer == 100) // Turn off motor
    motor_running = false;

  if(ser_buffer == 98) // Set max angle
  {
    float temp = Serial.parseFloat();
    max_val = temp;
  }

  if(ser_buffer == 97) // Set period
    period = Serial.parseFloat();
  
  if(ser_buffer == 101) //Go forward
  {
    motor->set_rpm(60);
    motor->goto_angle_pos_rel_direct(manual_movement_size);
  }

  if(ser_buffer == 102) //Go backwards
  {
    motor->set_rpm(60);
    motor->goto_angle_pos_rel_direct(-manual_movement_size);
  }

  if(ser_buffer == 103) //Set manual step size
    manual_movement_size = Serial.parseFloat();
    
  if (ser_buffer == 121)  //Set sinusoidal motion
  {
    motion = SINUSOIDAL;
    motor->clear_steps();
  }

  if (ser_buffer == 122)  //Set Linear Ramp Motion
  {
    motion = LINEAR;
    motor->clear_steps();
    motor->set_rpm(60);
    motor->set_pos(0);
    Serial.println(motor->get_pos());
    motor->goto_angle_pos_abs_direct(-max_val);
    Serial.println(motor->get_pos());
  }
  
  if (ser_buffer == 120) // Goto Position
  {
    motor->clear_steps();
    motor->set_rpm(60);
    motor->goto_angle_pos_abs_direct(Serial.parseFloat());
    motor_running = false;
  }
  

  motor->clear_steps();
  delay(500); 

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


void loop() 
{ 
  readSerial();
  if(motor_running)
  {
    if(motion == SINUSOIDAL)
    //if(motor->get_remaining() == 0 && motion == SINUSOIDAL)
    {
      motor->continuous_stepping = true;
      float iterator = micros() / 1e6;
      //float pos = max_val * sin(((2 * PI) / period) * iterator);
      float rpm = ((2 * PI * max_val) / period) * cos(((2 * PI) / period) * iterator);     
      motor->set_rpm(rpm);
      //motor->goto_angle_pos_abs(pos);
    }

    if (motor->get_remaining() == 0 && motion == LINEAR)
    {
      motor->continuous_stepping = false;
      motor->set_rpm((2 * max_val)/(6 * period));
      motor->goto_angle_pos_abs(max_val);
     
    }
    if (abs(motor->get_pos()) > abs(max_val) && motion == LINEAR)
    {
      motor_running = false;
    }

    Serial.println(motor->get_pos());
    motor->run();
  }
}

