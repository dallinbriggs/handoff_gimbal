#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>

ros::NodeHandle  nh;

Servo gimbal_yaw;
Servo gimbal_pitch;

float yaw_calibration = 0; //Angle offfset in degrees
float pitch_calibration = 130; //Angle offset in degrees

void command_cb( const geometry_msgs::Vector3Stamped& cmd_msg){
  float y;
  float z;
  const float Pi = 3.14159;

  // cmd_msg is expected to pass in pitch and yaw angles on the second and third values 
  // of the vector. These angles should be from 0-(-90) degrees on pitch and 0-180 on yaw.
  
  y = cmd_msg.vector.y*180/Pi; //cmd_msg is coming in as radians, but servos need degrees.
  if (y < -90)
  {
    y = -90;
  }
  else if (y > 0)
  {
    y = 0;
  }

  z = cmd_msg.vector.z*180/Pi;
  if (z < 0)
  {
    z = 0;
  }
  else if (z > 180)
  {
    z = 180;
  }

  gimbal_pitch.write(y+pitch_calibration); //set servo angle, should be from 0-180  
  gimbal_yaw.write(z+yaw_calibration); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13)); //toggle led
}

ros::Subscriber<geometry_msgs::Vector3Stamped> command_sub("/gimbal/control", command_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(command_sub);
  
  gimbal_yaw.attach(10); //attach it to pin 10
  gimbal_yaw.write(yaw_calibration); // should be from 0 - 90?;
  gimbal_pitch.attach(11); //attach pitch servo to pin 11
  gimbal_pitch.write(pitch_calibration);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
