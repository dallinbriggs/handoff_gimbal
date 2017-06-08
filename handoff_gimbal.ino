/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Vector3Stamped.h>

ros::NodeHandle  nh;

Servo gimbal_yaw;
Servo gimbal_pitch;

void yaw_cb( const std_msgs::UInt16& cmd_msg){
  gimbal_yaw.write(cmd_msg.data); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

void pitch_cb( const std_msgs::UInt16& cmd_msg){
  gimbal_pitch.write(cmd_msg.data); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<std_msgs::UInt16> yaw_sub("/gimbal_yaw", yaw_cb);
ros::Subscriber<std_msgs::UInt16> pitch_sub("/gimbal_pitch", pitch_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(yaw_sub);
  nh.subscribe(pitch_sub);
  
  gimbal_yaw.attach(10); //attach it to pin 10
  gimbal_yaw.write(0); // should be from 0 - 90?;
  gimbal_pitch.attach(11); //attach pitch servo to pin 11
  gimbal_pitch.write(130);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
