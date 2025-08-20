#include <Servo.h>

Servo base;
int servo_base_pin = 8;
int base_start_position = 90;

Servo shoulder;
int servo_shoulder_pin = 9;
int shoulder_start_position = 90;

Servo elbow;
int servo_elbow_pin = 10;
int elbow_start_position = 90;

Servo gripper;
int servo_gripper_pin = 11;
int gripper_start_position = 90;

uint8_t idx = 0;
uint8_t value_idx = 0;
char value_arr[4] = "000";


void init_servos(){
  base.attach(servo_base_pin);
  shoulder.attach(servo_shoulder_pin);
  elbow.attach(servo_elbow_pin);
  gripper.attach(servo_gripper_pin);

  base.write(base_start_position);
  shoulder.write(shoulder_start_position);
  elbow.write(elbow_start_position);
  gripper.write(gripper_start_position);

}

void reach_goal(Servo& motor, int goal){
  if(goal >= motor.read())
  {
    for(int pos = motor.read(); pos <= goal; pos++){
      motor.write(pos);
    }
  }
  else
  {
    for(int pos = motor.read(); pos >= goal; pos--)
    {
      motor.write(pos);
    }
  }
}

void setup() {
  init_servos();
  Serial.begin(115200);
}

void loop() {
  if(Serial.available())
  {
    char chr = Serial.read();

    switch (chr)
    {
      case 'b':
        idx = 0;
        value_idx = 0;
        break;
        
      case 's':
        idx = 1;
        value_idx = 0;
        break;

      case 'e':
        idx = 2;
        value_idx = 0;
        break;

      case 'g':
        idx = 3;
        value_idx = 0;
        break;

      case ',':
      {
        value_arr[value_idx] = '\0';
        int val = atoi(value_arr);

        if(idx == 0)
        {
          reach_goal(base, val);
        }
        else if(idx == 1)
        {
          reach_goal(shoulder, val);
        }
        else if(idx == 2)
        {
          reach_goal(elbow, val);
        }
        else if(idx == 3)
        {
          reach_goal(gripper, val);
        }
        value_idx = 0; 
        value_arr[0] = '0';
        value_arr[1] = '0';
        value_arr[2] = '0';
        value_arr[3] = '\0';


        break;
      }

      default:
        if (chr >= '0' && chr <= '9' && value_idx < 3) {
          value_arr[value_idx++] = chr;
        }
        break;
    }
  }
}
