#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <std_msgs/Int64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <ros/time.h> 


// Right Motor
#define right_RL_EN         5   // PWM
//RPWM = 1 và LPWM = 0 : Mô tơ quay thuận.
//RPWM = 0 và LPWM = 1 : Mô tơ quay nghịch
//RPWM = 1 và LPWM = 1 hoặc RPWM = 0 và LPWM = 0 : Dừng.
#define right_R_PWM         31    // int 3 
#define right_L_PWM         30    // int 4
#define right_encod_A       0     // Pin 2
#define right_encod_B       8     // Pin 8

// Left Motor
#define left_RL_EN          6
#define left_R_PWM          32    // int 1
#define left_L_PWM          33    // int 2
#define left_encod_A        1     // Pin 3
#define left_encod_B        9     // Pin 9

// Dimention of robot

float distance_wheels = 0.245;
float diameter_wheel = 0.070;  



signed long right_pose_encod = 0 , right_pose_encod_old = 0 ;
signed long left_pose_encod = 0 , left_pose_encod_old = 0;
signed long data_pose_encod[2] = {0,0};
unsigned long quakhu;
float linear_x, angular_z;
float vel_left_wheel , vel_right_wheel;

float error_left_now = 0 , error_left_last = 0, error_left_last_last = 0,error_left_all = 0, P_l = 0 , I_l = 0 , D_l = 0;
float error_right_now = 0 , error_right_last = 0, error_right_last_last = 0,error_right_all = 0, P_r = 0 , I_r = 0 , D_r = 0;


//float Kp_l =1, Ki_l =0, Kd_l=0;
//float Kp_r =17 , Ki_r = 0 , Kd_r=0;

float duty_l = 0 , duty_r = 0;
float duty_left =0 , duty_right = 0;
float rpm_right = 0, rpm_left = 0 ;
ros::NodeHandle  nh;

float tsamp = 0.02;

std_msgs::Int32MultiArray array_pose_encod;     // Publisher array pose encoder
ros::Publisher pub_array_pose_encod("pose_encod",&array_pose_encod);

std_msgs::Float32MultiArray array_vels_robot;     // Publisher array pose encoder
ros::Publisher pub_array_vels_robot("vels_robot",&array_vels_robot);


std_msgs::Float32MultiArray array_vels_wheels;     // Publisher array pose encoder
ros::Publisher pub_array_vels_wheels("vels_wheels",&array_vels_wheels);

std_msgs::Float32MultiArray array_vels_enc;     // Publisher array pose encoder
ros::Publisher pub_array_vels_enc("vels_enc",&array_vels_enc);

std_msgs::Int32MultiArray array_delta_enc;     // Publisher array pose encoder
ros::Publisher pub_array_delta_enc("delta_enc",&array_delta_enc);


void cmd_vel_Cb( const geometry_msgs::Twist& data_Twist){
  linear_x = data_Twist.linear.x;
  angular_z = data_Twist.angular.z;

//  vel_right_wheel = ((2 * linear_x) + (angular_z * 235 / 1000)) / 2 ;
//  vel_left_wheel = ((2 * linear_x) - (angular_z * 235 / 1000)) / 2 ;

}

void duty_Cb( const std_msgs::Int32MultiArray&  data){
  duty_right = data.data[0];
  duty_left = data.data[1];

}

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel",cmd_vel_Cb);

ros::Subscriber<std_msgs::Int32MultiArray> sub_duty_motor("/duty_motor",duty_Cb);

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  
  nh.advertise(pub_array_pose_encod);
  nh.advertise(pub_array_vels_robot);
  nh.advertise(pub_array_vels_wheels);
  nh.advertise(pub_array_vels_enc);
  nh.advertise(pub_array_delta_enc);
  
  nh.subscribe(sub_cmd_vel);
  nh.subscribe(sub_duty_motor);
  
  
  pinMode(right_encod_B, INPUT);
  pinMode(left_encod_B, INPUT);

  pinMode(right_RL_EN, OUTPUT);
  pinMode(right_R_PWM, OUTPUT);
  pinMode(right_L_PWM, OUTPUT);

  pinMode(left_RL_EN, OUTPUT);
  pinMode(left_R_PWM, OUTPUT);
  pinMode(left_L_PWM, OUTPUT);
  
  attachInterrupt(right_encod_A, read_right_encoder, RISING);
  attachInterrupt(left_encod_A, read_left_encoder, RISING);

  duty_l = 0;
  duty_r = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  vel_right_wheel = ((2 * linear_x) + (angular_z * distance_wheels / 1000)) / 2 ;
  vel_left_wheel = ((2 * linear_x) - (angular_z * distance_wheels / 1000)) / 2 ;

  // Xung / s mong muoon
  float vel_right_enc = (vel_right_wheel/ (3.14 * diameter_wheel)) * 2970 ;
  float vel_left_enc = (vel_left_wheel/(3.14 * diameter_wheel)) * 2970 ;

  float right_pose_encod_new = (right_pose_encod - right_pose_encod_old) ;
  float left_pose_encod_new = (left_pose_encod - left_pose_encod_old) ;


  
  double t=millis()- quakhu;
  
  double D_l= 0.145*3.14*(left_pose_encod_new/2970);
  double D_r= 0.145*3.14*(right_pose_encod_new/2970);

  rpm_right = right_pose_encod_new / t;  // xung /s thuc te
  rpm_left = left_pose_encod_new / t;


    

  set_pwm_right_motor(vel_right_wheel,vel_right_enc,rpm_right,duty_right);
  set_pwm_left_motor(vel_left_wheel,vel_left_enc,rpm_left,duty_left);
  

  
  
    float data_array_01[2] = {0.0,0.0};
  data_array_01[0] = P_r;
  data_array_01[1] = P_l;

    array_vels_robot.data = data_array_01;   
    array_vels_robot.data_length = 2 ;
    
    pub_array_vels_robot.publish(&array_vels_robot);

  
  float data_array[2] = {0,0};
  data_array[0] = duty_r;
  data_array[1] = duty_l;

  array_vels_wheels.data = data_array;
  array_vels_wheels.data_length = 2;
  
  pub_array_vels_wheels.publish(&array_vels_wheels);


  signed long data_array_02[2] = {0,0};
  data_array_02[0] = right_pose_encod_new;
  data_array_02[1] = left_pose_encod_new;

    array_delta_enc.data = data_array_02;   
    array_delta_enc.data_length = 2 ;
    
    pub_array_delta_enc.publish(&array_delta_enc);

  
  right_pose_encod_old = right_pose_encod;
  left_pose_encod_old = left_pose_encod;
  quakhu=millis();

  pub_vels_encs(vel_right_enc,vel_left_enc);
  pub_data_pose_encod();
  //pub_vels_robot();
  //pub_vel_wheels();


  
  nh.spinOnce();
}

void read_right_encoder(){
  if(digitalRead(right_encod_B)==HIGH) right_pose_encod --;
  if(digitalRead(right_encod_B)==LOW) right_pose_encod ++;
}

void read_left_encoder(){
  if(digitalRead(left_encod_B)==HIGH) left_pose_encod ++;
  if(digitalRead(left_encod_B)==LOW) left_pose_encod --;
}

void pub_data_pose_encod(){

    data_pose_encod[0] = right_pose_encod;
    data_pose_encod[1] = left_pose_encod;
    
    array_pose_encod.data = data_pose_encod;
    array_pose_encod.data_length = 2;
    pub_array_pose_encod.publish(&array_pose_encod);
}

void pub_vels_robot(){
    float data_array[2] = {0.0,0.0};
    data_array[0] = linear_x;
    data_array[1] = angular_z;

    array_vels_robot.data = data_array;
    array_vels_robot.data_length = 2 ;
    
    pub_array_vels_robot.publish(&array_vels_robot);
}

void pub_vels_encs(float vel_enc_01, float vel_enc_02){
    float data_array[2] = {0.0,0.0};
    data_array[0] = vel_enc_01;
    data_array[1] = vel_enc_02;

    array_vels_enc.data = data_array;
    array_vels_enc.data_length = 2 ;
    
    pub_array_vels_enc.publish(&array_vels_enc);
}

void pub_vel_wheels(){
  
  float v_r=((2 * linear_x) + (angular_z * distance_wheels / 1000)) / 2;
  float v_l=((2 * linear_x) - (angular_z * distance_wheels / 1000)) / 2;
//
//  float data_array[2] = {0.0,0.0};
//  data_array[0] = v_r;
//  data_array[1] = v_l;
//
//  array_vels_wheels.data = data_array;
//  array_vels_wheels.data_length = 2;
//  
//  pub_array_vels_wheels.publish(&array_vels_wheels);

}

void set_pwm_right_motor(float pwm , float rpm_target ,float rpm_real, float duty){
  int abs_pwm = 0;
  float Kp_r =0.01 , Ki_r = 0 , Kd_r=0;


  // ***** Check duty *****
  if (duty_r > 250) {
        duty_r = 250;
         }
  if (duty_r < -250) {
        duty_r = 250;}
int pwr_r = (int) fabs(duty_r);
        
  if (pwm > 0 ){
    digitalWrite(right_R_PWM, HIGH);
    digitalWrite(right_L_PWM , LOW);
    abs_pwm = duty;
  }
  else if ( pwm<0){
    digitalWrite(right_R_PWM, LOW);
    digitalWrite(right_L_PWM , HIGH);
    abs_pwm = abs(duty);
  }
  else{
    abs_pwm = 0;
  }
  // PWM Right motor
  analogWrite(right_RL_EN,abs_pwm);
}

void set_pwm_left_motor(float pwm, float rpm_target ,float rpm_real,float duty){
  
  int abs_pwm = 0;
  float Kp_l =0.01, Ki_l =0, Kd_l=0;

  
  if (duty_l > 250) { duty_l = 250;}
  if (duty_l < -250) {duty_l = -250;}

  int pwr_l = (int) fabs(duty_l);
  
  if (pwm > 0 ){
    digitalWrite(left_R_PWM, HIGH);
    digitalWrite(left_L_PWM , LOW);
    abs_pwm = duty;
  }
  
  else if (pwm < 0){
    digitalWrite(left_R_PWM, LOW);
    digitalWrite(left_L_PWM , HIGH);
    abs_pwm = abs(duty);
  }

  else{
    abs_pwm = 0;
  }
  // PWM Left motor
  
  analogWrite(left_RL_EN,abs_pwm);
}
