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

void command_cb( const geometry_msgs::Vector3Stamped& cmd_msg){
  float y;
  float z;
  if (cmd_msg.vector.y < 0)
  {
    y = 0;
  }
  else if (cmd_msg.vector.x > 180)
  {
    y = 180;
  }
  else
  {
    y = cmd_msg.vector.y;
  }
  if (cmd_msg.vector.z < 0)
  {
    z = 0;
  }
  else if (cmd_msg.vector.z > 180)
  {
    z = 180;
  }
  else
  {
    z = cmd_msg.vector.z;
  }
  gimbal_pitch.write(y); //set servo angle, should be from 0-180  
  gimbal_yaw.write(z); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13)); //toggle led
}

ros::Subscriber<geometry_msgs::Vector3Stamped> command_sub("/gimbal/control", command_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(command_sub);
  
  gimbal_yaw.attach(10); //attach it to pin 10
  gimbal_yaw.write(0); // should be from 0 - 90?;
  gimbal_pitch.attach(11); //attach pitch servo to pin 11
  gimbal_pitch.write(130);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
