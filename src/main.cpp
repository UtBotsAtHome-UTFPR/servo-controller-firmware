#include <Arduino.h>

// uROS related
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/string.h>

// std
#include <stdio.h>

#include <ESP32Servo.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This code expects Arduino framework with serial transport
#endif

#define RCCHECK(fn){rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.println("error: ");Serial.println(temp_rc);Serial.println(RCL_RET_OK);error_loop(temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define LED_PIN 0 // GPIO pin for the LED

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
#define SHOULDER_EXTENSION_PIN 2 
#define SHOULDER_ROTATION_PIN 4 
#define ELBOW_EXTENSION_PIN 18 

// number of ROS objetcts that are handled by the executor
#define EXECUTOR_SIZE 3 


rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t debug_publisher;
std_msgs__msg__String debug;
char debug_buffer[256]; // buffer to hold debug messages

// Joint subscribers 
rcl_subscription_t shoulder_extension_subscriber;
rcl_subscription_t shoulder_rotation_subscriber;
rcl_subscription_t elbow_extension_subscriber;

// Joint messages
std_msgs__msg__Int16 shoulder_extension_angle_msg;
std_msgs__msg__Int16 shoulder_rotation_angle_msg;
std_msgs__msg__Int16 elbow_extension_angle_msg;

// Joint servos objetcts
Servo shoulder_extension_servo; 
Servo shoulder_rotation_servo;  
Servo elbow_extension_servo; 

// error handling function called by the RCCHECK macro
void error_loop(rcl_ret_t returnCode)
{
    Serial.print("Error in ROS with return code: ");
    Serial.println(returnCode);
    while(1)
    {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(500);
    }
}

//Debug publish function
void publishDebug()
{
    debug.data.data = debug_buffer;
    debug.data.size = strlen(debug_buffer);
    debug.data.capacity = strlen(debug_buffer) + 1;
    RCSOFTCHECK(rcl_publish(&debug_publisher, &debug, NULL));
}

// Joint callbacks
void shoulder_extension_callback(const void * msgin)
{
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;

  sprintf(debug_buffer, "shoulder extension value: %d", msg->data);
  publishDebug();

  // Update servo positions based on received data
  // Ensure the angle is within the valid range for the servo
  // Assuming the servo can rotate from 0 to 180 degrees
  if (msg->data >= 0 && msg->data <= 180) {
    shoulder_extension_servo.write(msg->data); // Write the angle to the servo
  }
}

void shoulder_rotation_callback(const void * msgin)
{
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;

  sprintf(debug_buffer, "shoulder rotation value: %d", msg->data);
  publishDebug();

  // Update servo positions based on received data
  // Ensure the angle is within the valid range for the servo
  // Assuming the servo can rotate from 0 to 180 degrees
  if (msg->data >= 0 && msg->data <= 180) {
    shoulder_rotation_servo.write(msg->data); // Write the angle to the servo
  }
}

void elbow_extension_callback(const void * msgin)
{
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;

  sprintf(debug_buffer, "elbow extension value: %d", msg->data);
  publishDebug();

  // Update servo positions based on received data
  // Ensure the angle is within the valid range for the servo
  // Assuming the servo can rotate from 0 to 180 degrees
  if (msg->data >= 0 && msg->data <= 180) {
    elbow_extension_servo.write(msg->data); // Write the angle to the servo
  }
}

// GPIO configuration function
void configureGPIO() {
	// timer allocation
	ESP32PWM::allocateTimer(0);
  
	shoulder_extension_servo.setPeriodHertz(50);    // standard 50 hz servo
	shoulder_extension_servo.attach(SHOULDER_EXTENSION_PIN, 500, 2400); // attaches the servo on pin 18 to the servo object

	shoulder_rotation_servo.setPeriodHertz(50);    // standard 50 hz servo
	shoulder_rotation_servo.attach(SHOULDER_ROTATION_PIN, 500, 2400); // attaches the servo on pin 18 to the servo object

  elbow_extension_servo.setPeriodHertz(50);    // standard 50 hz servo  
  elbow_extension_servo.attach(ELBOW_EXTENSION_PIN, 500, 2400); // attaches the servo on pin 18 to the servo object
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
  Serial.println("Serial transport initialized");

  allocator = rcl_get_default_allocator();
  
  Serial.println("Initialized ROS allocator");

  // create support structure
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  

  Serial.println("Initialized ROS support init options");

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));
  Serial.println("uROS node created");

  
  // create subscriber for cmd_vel topic
  RCCHECK(rclc_subscription_init_default(
      &shoulder_extension_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "shoulder_extension_angle"));
  Serial.println("/shoulder extension subscriber created");

  // create subscriber for cmd_vel topic
  RCCHECK(rclc_subscription_init_default(
      &shoulder_rotation_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "shoulder_rotation_angle"));
  Serial.println("/shoulder rotation subscriber created");

  // create subscriber for cmd_vel topic
  RCCHECK(rclc_subscription_init_default(
      &elbow_extension_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "elbow_extension_angle"));
  Serial.println("/elbow extension subscriber created");

  RCCHECK(rclc_publisher_init_default(
    &debug_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "debug"));

  RCCHECK(rclc_executor_init(&executor, &support.context, EXECUTOR_SIZE, &allocator));

  RCCHECK(rclc_executor_add_subscription(
    &executor, 
    &shoulder_extension_subscriber, 
    &shoulder_extension_angle_msg, 
    shoulder_extension_callback, 
    ON_NEW_DATA
  ));

  RCCHECK(rclc_executor_add_subscription(
    &executor, 
    &shoulder_rotation_subscriber, 
    &shoulder_rotation_angle_msg,
    shoulder_rotation_callback, ON_NEW_DATA
  ));

  RCCHECK(rclc_executor_add_subscription(
    &executor, 
    &elbow_extension_subscriber, 
    &elbow_extension_angle_msg, 
    elbow_extension_callback, 
    ON_NEW_DATA
  ));

  sprintf(debug_buffer, "uROS executor initialized with %d handles", executor.max_handles);
  publishDebug();

  configureGPIO();
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}