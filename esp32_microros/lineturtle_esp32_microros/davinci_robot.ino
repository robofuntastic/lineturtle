// watch video about davinci robot based on esp32 , stepper motor and a4988
// https://youtu.be/HszcFh2yxTw

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <AccelStepper.h>
#include <std_msgs/msg/int32.h>


rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;

rcl_publisher_t left_encoder_publisher;
std_msgs__msg__Int32 left_encoder_msg;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//all units are mm and seconds
// it takes 3200 steps for a full revolution because of microstepping at 1/16 so 200*16 = 3200
#define wheel_radius 33.5
#define wheel_seperation 150

// 0.5 m/s = 500 mm/s will be max linear speed of robot = 
//pi*2*wheel_radiis * (wheel_speed=2.375447 rotaions/s) = 3200*2.375447 = 7602 steps/s
#define max_speed 3200

// Pin Definitions
#define stepPin_right 13
#define dirPin_right 14
#define stepPin_left 17
#define dirPin_left 18

#define motorInterfaceType 1
// Create a new instance of the AccelStepper class:
AccelStepper stepper_right = AccelStepper(motorInterfaceType, stepPin_right, dirPin_right);
AccelStepper stepper_left = AccelStepper(motorInterfaceType, stepPin_left, dirPin_left);

// Motor control variables
int motorSpeedLeft = 0;
int motorSpeedRight = 0;

int LeftEncoderCount = 0;
void LeftEncoderCallback();


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Function prototypes
void setMotorSpeed(int speedLeft, int speedRight);


void error_loop(){
  while(1){
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {


    RCSOFTCHECK(rcl_publish(&left_encoder_publisher, &left_encoder_msg, NULL));


  left_encoder_msg.data = motorSpeedLeft*3.2;
//  stepper_right.setSpeed(3200);
//  stepper_right.runSpeed();
//  stepper_left.setSpeed(3200);
//  stepper_left.runSpeed();
  }
}

// Function to set motor speed and direction
void setMotorSpeed(int speedLeft, int speedRight) {
//  stepper_right.setSpeed(speedRight*3200/1000);
//  stepper_right.runSpeed();
//
//  stepper_left.setSpeed(speedLeft*3200/1000);
//  stepper_left.runSpeed();
}


// Twist message callback
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // Calculate motor speeds based on twist message
  float linear = msg->linear.x;
  float angular = msg->angular.z;

  // Calculate individual motor speeds
  motorSpeedLeft = (int)((linear  - angular*wheel_seperation/2)*1000/wheel_radius);
  motorSpeedRight = (int)((linear  + angular*wheel_seperation/2)*1000/wheel_radius);
  //setMotorSpeed(motorSpeedLeft, motorSpeedRight);

}
void StepperTask(void *ptr){
  while(1){
    stepper_right.setSpeed(3200);
    stepper_right.runSpeed();
    stepper_left.setSpeed(3200);
    stepper_left.runSpeed();
    vTaskDelay(1);
  }
}

void setup() {
  //set_microros_transports();
  
  set_microros_wifi_transports("robofuntastic", "youtube1", "192.168.72.143", 8888);
  stepper_right.setMaxSpeed(max_speed);
  stepper_left.setMaxSpeed(max_speed);

  delay(500);
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "davinccibot_esp32", "", &support));

  // Create twist subscriber
  RCCHECK(rclc_subscription_init_default(
      &twist_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  RCCHECK(rclc_publisher_init_default(
    &left_encoder_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "left_motor_ticks"));

  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));

  left_encoder_msg.data = 0;
  xTaskCreate(StepperTask,"StepperTask",1024,NULL,1,NULL);

}

void loop() {
  //delay(100);
//      stepper_right.setSpeed(3200);
//  stepper_right.runSpeed();
//    stepper_left.setSpeed(3200);
//  stepper_left.runSpeed();
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
