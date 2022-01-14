#ifndef MOTOR_NODE_H
#define MOTOR_NODE_H
#include <pigpiod_if2.h>

#define motor1_DIR 19
#define motor1_PWM 26
#define motor1_ENA 23
#define motor1_ENB 24

#define motor2_DIR 6
#define motor2_PWM 13
#define motor2_ENA 27
#define motor2_ENB 17

#define PI 3.141592

//Text_Input
void Text_Input(void);
int PWM_range;
int PWM_frequency;
int PWM_limit;
double Control_cycle;
int Acceleration_ratio;
double Wheel_radius;
double Robot_radius;
int Encoder_resolution;
double Wheel_round;
double Robot_round;

//Motor_Setup
int Motor_Setup(void);
int pinum;
int current_PWM1;
int current_PWM2;
bool current_Direction1;
bool current_Direction2;
int acceleration;

//Interrupt_Setting
void Interrupt_Setiing(void);
volatile int EncoderCounter1;
volatile int EncoderCounter2;
volatile int EncoderCounter1A;
volatile int EncoderCounter1B;
volatile int EncoderCounter2A;
volatile int EncoderCounter2B;
volatile int EncoderSpeedCounter1;
volatile int EncoderSpeedCounter2;
void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
int Motor1_Encoder_Sum();
int Motor2_Encoder_Sum();
void Init_Encoder(void);
////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void Initialize(void);
int flag = 0;
int get_dis = 200;
int get_pos = 0;
int pre_pos = 0;
int complete;
int come;
int arrive;
int start_flag = 0;
int finish_flag = 0; 
int standard = 150;

int gui_mode;
int finish;
//Motor_Controller
void Motor_Controller(int motor_num, bool direction, int pwm);
//void Accel_Controller(int motor_num, bool direction, int desired_pwm);
void move(int get_pos, int get_dis, int pwm);

//Example
//bool switch_direction;
//int Theta_Distance_Flag;
//void Switch_Turn_Example(int PWM1, int PWM2);
//void Theta_Turn(double Theta, int PWM);
//void Distance_Go(double Distance, int PWM);
//void Theta_Distance(double Theta, int Turn_PWM, double Distance, int Go_PWM);
void Theta_Distance(double Theta, double Distance, int PWM);


//Utiliy
int Limit_Function(int pwm);
double RPM_Value1;
double RPM_Value2;
void RPM_Calculator();
void Motor_View();

void Theta_Position(double target_x, double target_y, double target_theta);
void Pid_Right(int target_rpm);
void Pid_Left(int target_rpm);
//void Find(double target_x, double target_y, double target_theta);
void Start();
void Tracking(int pwm1, int pwm2);
//void Finish();

//donggyu's code
double Target_ang(double x, double y);
double Error_th(double th);
void Target_move(double ref_x, double ref_y, double ref_th);
void Find(double target_x, double target_y, double target_th);
void Finish();
//----------------
int pwm1_val;
int pub_pwm1;


//-----------------
int motor_mode = 0;
int find_mode = 1;
double p_x;
double p_y;
double p_th=90;
double start_th;

double Kp1;
double Ki1;
double Kd1;

double Kp2;
double Ki2;
double Kd2;

double m1_error_pre;
int m1_pwm;

double m2_error_pre;
int m2_pwm;
int stack;





#endif // MOTOR_NODE_H
