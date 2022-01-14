#include <ros/ros.h>
#include <motor/motor_node.h>
#include <encoder_msg/encoder.h>
#include <odom_msg/position.h>
#include <fstream>
//#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <iostream>
#include <cmath>
//#include "std_msgs/MultiArrayDimension.h"
//#include "std_msgs/Int32MultiArray.h"
//#include "std_msgs/Int32.h"
//#include "vision_msgs/tracking.h" //vision파트랑 한 번 더 확인


void Text_Input(void)
{
  int i = 0;
  std::size_t found;
  std::ifstream inFile;
  inFile.open("/home/ubuntu/catkin_ws/src/motor/motor_input.txt");
  for(std::string line; std::getline(inFile,line);)
  {
      found=line.find("=");

      switch(i)
      {
      case 0: PWM_range = atof(line.substr(found+2).c_str()); break;
      case 1: PWM_frequency = atof(line.substr(found+2).c_str()); break;
      case 2: PWM_limit = atof(line.substr(found+2).c_str()); break;
      case 3: Control_cycle = atof(line.substr(found+2).c_str()); break;
      case 4: Acceleration_ratio = atof(line.substr(found+2).c_str()); break;
      case 5: Wheel_radius = atof(line.substr(found+2).c_str()); break;
      case 6: Robot_radius = atof(line.substr(found+2).c_str()); break;
      case 7: Encoder_resolution = atof(line.substr(found+2).c_str()); break;
          //case :  = atof(line.substr(found+2).c_str()); break;
      }
      i +=1;
  }
  inFile.close();
}

int Motor_Setup(void)
{
  pinum=pigpio_start(NULL, NULL);
  
  if(pinum<0)
  {
    ROS_INFO("Setup failed");
    ROS_INFO("pinum is %d", pinum);
    return 1;
  }

  set_mode(pinum, motor1_DIR, PI_OUTPUT);
  set_mode(pinum, motor2_DIR, PI_OUTPUT);
  set_mode(pinum, motor1_PWM, PI_OUTPUT);
  set_mode(pinum, motor2_PWM, PI_OUTPUT);
  set_mode(pinum, motor1_ENA, PI_INPUT);
  set_mode(pinum, motor1_ENB, PI_INPUT);
  set_mode(pinum, motor2_ENA, PI_INPUT);
  set_mode(pinum, motor2_ENB, PI_INPUT);

  gpio_write(pinum, motor1_DIR, PI_LOW);
  gpio_write(pinum, motor2_DIR, PI_LOW);

  set_PWM_range(pinum, motor1_PWM, PWM_range);
  set_PWM_range(pinum, motor2_PWM, PWM_range);
  set_PWM_frequency(pinum, motor1_PWM, PWM_frequency);
  set_PWM_frequency(pinum, motor2_PWM, PWM_frequency);
  set_PWM_dutycycle(pinum, motor1_PWM, 0);
  set_PWM_dutycycle(pinum, motor1_PWM, 0);

  set_pull_up_down(pinum, motor1_ENA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor1_ENB, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_ENA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_ENB, PI_PUD_DOWN);

  current_PWM1 = 0;
  current_PWM2 = 0;

  current_Direction1 = true;
  current_Direction2 = true;

  acceleration = PWM_limit/(Acceleration_ratio);

  ROS_INFO("Setup Fin");
  return 0;
}

//--------------------------------
void Callback1(const std_msgs::Int64 &msg1)
{
	//ROS_INFO("tracking : %ld", msg1.data);
	get_pos = msg1.data;
	ROS_INFO("get: %ld", get_pos);
	//Motor_Controller(1, true, 50);
  //Motor_Controller(2, true, 50);
}
void Callback2(const std_msgs::Int64 &msg2)
{
	//ROS_INFO("distance : %ld", msg2.data);
	get_dis = msg2.data;
}

void odom_position(const odom_msg::position &msg){
	p_x = msg.x;
	p_y = msg.y;
	p_th = msg.th;
	
	ROS_INFO("(%f, %f), %f", p_x, p_y, p_th);
	
}
////////받아오는 방법

//--------------------------------

void Interrupt_Setting(void)
{
    callback(pinum, motor1_ENA, EITHER_EDGE, Interrupt1A);
    callback(pinum, motor1_ENB, EITHER_EDGE, Interrupt1B);
    callback(pinum, motor2_ENA, EITHER_EDGE, Interrupt2A);
    callback(pinum, motor2_ENB, EITHER_EDGE, Interrupt2B);
}

void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor1_DIR) == true)EncoderCounter1A ++;
  else EncoderCounter1A --;
  EncoderSpeedCounter1 ++;
}
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor1_DIR) == true)EncoderCounter1B ++;
  else EncoderCounter1B --;
  EncoderSpeedCounter1 ++;
}

void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor2_DIR) == true)EncoderCounter2A --;
  else EncoderCounter2A ++;
  EncoderSpeedCounter2 ++;
}
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor2_DIR) == true)EncoderCounter2B --;
  else EncoderCounter2B ++;
  EncoderSpeedCounter2 ++;
}

int Motor1_Encoder_Sum()
{
  EncoderCounter1 = EncoderCounter1A + EncoderCounter1B;
  return EncoderCounter1;
}
int Motor2_Encoder_Sum()
{
  EncoderCounter2 = EncoderCounter2A + EncoderCounter2B;
  return EncoderCounter2;
}




void Init_Encoder(void)
{
  EncoderCounter1 = 0;
  EncoderCounter2 = 0;
  EncoderCounter1A = 0;
  EncoderCounter1B = 0;
  EncoderCounter2A = 0;
  EncoderCounter2B = 0;
}

void Initialize(void)
{
  Text_Input();
  Motor_Setup();
  Init_Encoder();
  Interrupt_Setting();

  Wheel_round = 2*PI*Wheel_radius;
  Robot_round = 2*PI*Robot_radius;

	//flag = 0;//자리가 여기인지는 잘 모르겠음
  //switch_direction = true;
  //Theta_Distance_Flag = 0;

  ROS_INFO("PWM_range %d", PWM_range); //확인용
  ROS_INFO("PWM_frequency %d", PWM_frequency);
  ROS_INFO("PWM_limit %d", PWM_limit);
  ROS_INFO("Control_cycle %f", Control_cycle);
  ROS_INFO("Acceleration_ratio %d", Acceleration_ratio);
  ROS_INFO("Initialize Complete");

  printf("\033[2J");  
}

void Motor_Controller(int motor_num, bool direction, int pwm)
{
  int local_PWM = Limit_Function(pwm);

  if(motor_num == 1)
  {
    if(direction == true)
    {
      gpio_write(pinum, motor1_DIR, PI_LOW);
      set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
      current_PWM1 = local_PWM;
      current_Direction1 = true;
    }
    else if(direction == false)
    {
      gpio_write(pinum, motor1_DIR, PI_HIGH);
      set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
      current_PWM1 = local_PWM;
      current_Direction1 = false;
    }
  }
  
  else if(motor_num == 2)
  {
   if(direction == true)
   {
     gpio_write(pinum, motor2_DIR, PI_HIGH);
     set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
     current_PWM2 = local_PWM;
     current_Direction2 = true;
   }
   else if(direction == false)
   {
     gpio_write(pinum, motor2_DIR, PI_LOW);
     set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
     current_PWM2 = local_PWM;
     current_Direction2 = false;
   }
  }
}

void Theta_Position(double target_x, double target_y, double target_theta)
{
	double error_theta = target_theta - p_th;
	double error_x = target_x - p_x;
	double error_y = target_y - p_y;
	double bias = 5.0;	
	
	if((0.0 < error_theta && error_theta < 180.0) || (-360.0 < error_theta && error_theta < -180.0))
	{	
		while(abs(error_theta) > bias)
		{
			Motor_Controller(1, true, 50);
			Motor_Controller(2, false, 50);
		}
		Motor_Controller(1, true, 0);
		Motor_Controller(2, false, 0);
	}
	else		
	{		
		while(abs(error_theta) > bias)
		{
			Motor_Controller(1, false, 50);
			Motor_Controller(2, true, 50);
		}
		Motor_Controller(1, false, 0);
		Motor_Controller(2, true, 0);
	}
	
	while(abs(error_x) > bias || abs(error_y) > bias)
	{
		Motor_Controller(1, true, 50);
		Motor_Controller(2, true, 50);
		if(get_dis < 30)
		{
			Motor_Controller(1, true, 0);
			Motor_Controller(2, true, 0);
		}
		if(30 <= get_dis && get_dis< 60)
		{
			Motor_Controller(1, true, get_dis - 30);
			Motor_Controller(2, true, get_dis - 30);
		}
	}

	if(abs(error_x) < bias && abs(error_y) < bias)
	{
		Motor_Controller(1, true, 0);
		Motor_Controller(2, true, 0);
	}		
}
	
	
	/*
	double local_encoder_the = (Encoder_resolution*4/360)*(Robot_round/Wheel_round)*Theta;
	double local_encoder_dis = (Encoder_resolution*4*Distance)/Wheel_round; //위에 두 줄 지우고 x,y 값 받아와서 각도 변환
	int local_PWM = Limit_Function(PWM);
	double error_theta = Theta - local_encoder_the;
	double error_distance = Distance - local_encoder_dis;
	ROS_INFO("local distance: %f", local_encoder_the);
	double bias = 5.0;
	Motor1_Encoder_Sum();
	Motor2_Encoder_Sum();

	if(abs(error_theta) > bias)
	{
		if(error_theta < 0)
		{
			ROS_INFO("check_1");
			Motor_Controller(1, true, 50);
			Motor_Controller(2, false, 50);
		}
		else
		{
			ROS_INFO("chekc_2");
			Motor_Controller(1, false, 50);
			Motor_Controller(2, true, 50);
		}
	}
	else if(abs(error_distance) > bias)
	{
		if(error_distance < 0)
		{
			ROS_INFO("check_3");
			Motor_Controller(1, true, 50);
			Motor_Controller(2, true, 50);
		}
		else
		{
			ROS_INFO("check_4");
			Motor_Controller(1, false, 50);
			Motor_Controller(2, false, 50);
		}
	}
	else
	{
		ROS_INFO("check");
		Motor_Controller(1, true, 0);
		Motor_Controller(2, true, 0);
	}*/

void Pid_Right(int target_rpm)
{	

	Kp1 = 0.7;
	Ki1 = 0.5;
	Kd1 = 0.9;       
	double local_rpm = RPM_Value1;
	
	if(current_Direction1 == false) local_rpm = local_rpm * (-1);
	
	double m1_error = target_rpm - local_rpm;
	double m1_derr = m1_error - m1_error_pre;	
	double m1_sum = m1_sum + m1_error;

	double m1_error_pre = m1_error;
	
	double p1 = Kp1 * m1_error;
	double i1 = Ki1 * m1_sum;
	double d1 = Kd1 * m1_derr;

	double ctrl_m1 = (int)(p1 + i1 + d1);

	if(ctrl_m1 > 0)
	{	
		m1_pwm = (int)ctrl_m1;
    Motor_Controller(1, true, m1_pwm);
	}
	else
	{
		m1_pwm = (int)ctrl_m1 * (-1);
    Motor_Controller(1, false, m1_pwm);
	}
}

void Pid_Left(int target_rpm)
{               
  Kp2 = 0.8;
  Ki2 = 0.9;
  Kd2 = 0.1;
	
	double local_rpm = RPM_Value2;
	
	if(current_Direction2 == false) local_rpm = local_rpm * (-1);
	
  double m2_error = target_rpm - local_rpm;
  double m2_derr = m2_error - m2_error_pre;
  double m2_sum = m2_sum + m2_error;
  
  double m2_error_pre = m2_error;

  double p2 = Kp2 * m2_error;
  double i2 = Ki2 * m2_sum;
  double d2 = Kd2 * m2_derr;

  double ctrl_m2 = (int)(p2 + i2 + d2);

  if(ctrl_m2 > 0)
  {       
    m2_pwm = (int)ctrl_m2;
    Motor_Controller(2, true, m2_pwm);
  }
  else
  {
    m2_pwm = (int)ctrl_m2 * (-1);
    Motor_Controller(2, false, m2_pwm);
  }
}

void Find()
{
	
}

void Start()
{
	int pos_a = get_pos - 5;
	int pos_b = get_pos + 5; //어느 정도 범위인지 확인해보기
	int local_PWM1;
  int local_PWM2;
  int local;
	
	while(get_dis > 70) // 확인해보고 각도 안맞으면 수정 
	{
		local = get_dis - 60;
		if(pre_pos > get_pos)
		{
			pre_pos = get_pos;
			local_PWM1 = local + (acceleration / 2);
			local_PWM2 = local - (acceleration / 2);
			//Pid_Right(local_PWM1);
			//Pid_Left(local_PWM2);
			ROS_INFO("%d %d", local_PWM1, local_PWM2);
			Motor_Controller(1, true, local_PWM1);
			Motor_Controller(2, true, local_PWM2);
		}
		else
		{
			pre_pos = get_pos;
			local_PWM1 = local - (acceleration / 2);
			local_PWM2 = local + (acceleration / 2);
			//Pid_Right(local_PWM1);
			//Pid_Left(local_PWM2);
			ROS_INFO("%d %d", local_PWM1, local_PWM2);
			Motor_Controller(1, true, local_PWM1);
			Motor_Controller(2, true, local_PWM2);
		}
	}

	if(get_dis <= 70 && (pos_a <= get_pos && get_pos <= pos_b))
	{
		Motor_Controller(1, true, 0);
		Motor_Controller(2, true, 0);
	}
}

void Move(int get_pos, int get_dis, int pwm) //좌표값에 따른 방향 조절, 거리값에 따른 속도조절
{
	int local_PWM1;
  int local_PWM2;
  int local_basic_PWM1 = Limit_Function(pwm);
  int local_basic_PWM2 = Limit_Function(pwm);
  
/*
  while()//rfid인식할 때까지
  {
		if(get_dis > 80)
		{//fast
			local_PWM1 = local_basic_PWM1 + acceleration;
			local_PWM2 = local_basic_PWM2 + acceleration;
			if(pre_pos > get_pos)
			{
				pre_pos = get_pos;
				local_PWM1 = local_basic_PWM1 + (acceleration / 2);
				local_PWM2 = local_basic_PWM2 - (acceleration / 2);
				//Pid_Right(local_PWM1);
				//Pid_Left(local_PWM2);
				ROS_INFO("%d %d", local_PWM1, local_PWM2);
				Motor_Controller(1, true, local_PWM1);
				Motor_Controller(2, true, local_PWM2);
			}
			else
			{
				pre_pos = get_pos;
				local_PWM1 = local_basic_PWM1 - (acceleration / 2);
				local_PWM2 = local_basic_PWM2 + (acceleration / 2);
				//Pid_Right(local_PWM1);
				//Pid_Left(local_PWM2);
				ROS_INFO("%d %d", local_PWM1, local_PWM2);
				Motor_Controller(1, true, local_PWM1);
				Motor_Controller(2, true, local_PWM2);
			}
		}
		else
		{//slow
			local_PWM1 = local_basic_PWM1 - acceleration;
			local_PWM2 = local_basic_PWM2 - acceleration;
			if(pre_pos > get_pos)
			{
				pre_pos = get_pos;
				local_PWM1 = local_basic_PWM1 + (acceleration / 2);
				local_PWM2 = local_basic_PWM2 - (acceleration / 2);
				//Pid_Right(local_PWM1);
				//Pid_Left(local_PWM2);
				ROS_INFO("%d %d", local_PWM1, local_PWM2);
				Motor_Controller(1, false, local_PWM1);
				Motor_Controller(2, false, local_PWM2);
			}
			else
			{
				pre_pos = get_pos;
				local_PWM1 = local_basic_PWM1 - (acceleration / 2);
				local_PWM2 = local_basic_PWM2 + (acceleration / 2);
				//Pid_Right(local_PWM1);
				//Pid_Left(local_PWM2);
				ROS_INFO("%d %d", local_PWM1, local_PWM2);
				Motor_Controller(1, false, local_PWM1);
				Motor_Controller(2, false, local_PWM2);
			}
		}
	}*/
}

void Finish(double current_x, double current_y, double current_theta)
{ 
	double x = 0.0;
	double y = 0.0;
	//p_x, p_y, p_th
	
	double param = (x - p_x) / (y - p_y);	
	double result = atan (param) * 180 / PI;

	Theta_Position(x, y, result); //초기 위치 좌표 선정해야함
	
	//함수초기화 토픽 전송
}

int Limit_Function(int pwm)
{
  int output;
  if(pwm > PWM_limit)
  {
    output = PWM_limit;
    ROS_WARN("PWM too fast!!!");
  }
  else if(pwm < 0)
  {
    output = 0;
    ROS_WARN("trash value!!!");
  }
  else output = pwm;
  return output; 
}

void RPM_Calculator()
{
  RPM_Value1 = (EncoderSpeedCounter1*(60*Control_cycle))/(Encoder_resolution*4);
  EncoderSpeedCounter1 = 0;
  RPM_Value2 = (EncoderSpeedCounter2*(60*Control_cycle))/(Encoder_resolution*4);
  EncoderSpeedCounter2 = 0;
}

void Motor_View() //확인용
{
	RPM_Calculator();
	printf("\033[2J");
	printf("\033[1;1H");
	printf("Encoder1A : %5d  ||  Encoder2A : %5d\n", EncoderCounter1A, EncoderCounter2A);
	printf("Encoder1B : %5d  ||  Encoder2B : %5d\n", EncoderCounter1B, EncoderCounter2B);
	printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", RPM_Value1, RPM_Value2);
	printf("PWM1 : %10.0d    ||  PWM2 : %10.0d\n", current_PWM1, current_PWM2);
	printf("DIR1 :%10.0d     ||  DIR2 :%10.0d\n", current_Direction1, current_Direction2);
	printf("Acc  :%10.0d\n", acceleration);
	printf("\n");
}

void Boundary(float x, float y, float th){
	int x_boundary = 200;
	int y_boundary = 200;
	
	if((x > x_boundary/2 && y > y_boundary/2) || (x > x_boundary/2 && y < -y_boundary/2) || (x < -x_boundary/2 && y > y_boundary/2) || (x < -x_boundary/2 && y  -y_boundary/2)){
         if(th == 45 || th == 135 || th == 225 ||th == 275){
             //turn_180();
         }
     }
        else if(x > x_boundary / 2){
         if((th>0 && th<90) || (th<360 && th>270)){ 
             //stop();
            if(th < 180){
                 //turn_L(p_th, 0, x_boundary - p_x);
             }
             else{
                 //turn_R(p_th, 360, x_boundary - p_x);
             }
         }
     }
     else if(x < -1 * x_boundary/2){
         if (th == 90){
             //stop();
             //turn_180();
         }
         else if((th>90 && th<180) || (th>180 && th <270)){
             //stop();
             if(th < 180){
                 //turn_R(p_th, 180, x_boundary - p_x);
             }
             else{
                 //turn_L(p_th, 180, x_boundary - p_x);
             }
         }
     }

     else if(y > y_boundary/2){
         if((th>0 && th<90) || (th<90 && th>180)){ 
             //stop();
             if(p_th < 90){
                 //turn_R(p_th, 90, y_boundary - p_y);
             }
             else{
                 //turn_L(p_th, 90, y_boundary - p_y);
             }
         }
     }
    
     else if(y < -1 * y_boundary/2){
         if (th == 270){
             //stop();
             //turn_180();
     
         }
         else if((th>0 && th<270) || (th>270 && th <360)){
             //stop();
             if(th < 270){
                 //turn_R(p_th, 270, y_boundary - p_y);
                 //turn_L(p_th, 270, y_boundary - p_y);
             }
         }
     }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_node");
  ros::NodeHandle nh;
  Initialize();
	
	ros::Publisher encoder_pub = nh.advertise<encoder_msg::encoder>("/encoder", 10);
	//ros::Publisher init_pub = nh.advertise<std_msgs::Bool>("init", 10);
	encoder_msg::encoder counter;
  ros::Subscriber sub1 = nh.subscribe("tracking", 10, Callback1);
  ROS_INFO("--------------");
  ros::Subscriber sub2 = nh.subscribe("distance", 10, Callback2);
  ros::Subscriber sub3 = nh.subscribe("pose", 10, odom_position);
/*
  ros::Publisher pub1 = nh.advertise<std_msgs::Float64>("pub1", 10);
  ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("pub2", 10);
  ros::Publisher target = nh.advertise<std_msgs::Int64>("target", 10);
  std_msgs::Float64 pub1_val;
  std_msgs::Float64 pub2_val;
  std_msgs::Int64 target_val;*/


  ros::Rate loop_rate(Control_cycle);

  while(ros::ok())
  {	
  	ROS_INFO("--------");
  	Motor_View();
  	/*
  	Find(); // find안에 값 받으면 start로 넘어가도록 만들기
  	
  	//if 디스플레이에서 도움o 선택
  		Move();
  	//else
  		Finish(); // -> 이후 시나리오 다시 처음부터 시작하도록 만들기
  		
  	
  	//if 리액션 끝나면
  		Finish();
  	*/
  	
  	Motor_Controller(1, false, 200);
	Motor_Controller(2, true, 150);
  	
  	
  	/*
  	Pid_Right(50);
  	Pid_Left(50);
  	target_val.data = 50;
  	target.publish(target_val);

	//Pid_Left(70);

	  Motor_View();
  	pub1_val.data = RPM_Value1;
  	pub1.publish(pub1_val);
  	pub2_val.data = RPM_Value2;
  	pub2.publish(pub2_val);
  	
	
	Motor_Controller(1, false, 70);
	Motor_Controller(2, true, 70);
	Motor_View();
//		Move(get_pos, get_dis, 70);

  			//Boundary(p_x, p_y, p_th);
*/		
		//odometry
		ROS_INFO("Encoder counter %d, %d", Motor2_Encoder_Sum(), Motor1_Encoder_Sum());
		counter.E_L = Motor2_Encoder_Sum();
		counter.E_R = Motor1_Encoder_Sum();    
		encoder_pub.publish(counter);
		ROS_INFO("counter %f, %f", counter.E_L, counter.E_R);
		
		
		
	  ros::spinOnce();
	  loop_rate.sleep();
  }
  Motor_Controller(1, true, 0);
  Motor_Controller(2, true, 0);
  return 0;
}





        //Find(); //=> 1번 째 성공 후 해보는 걸로
        //if 사람 인식-----------------
        //ros::Subscriber sub_number = nh.subscribe("/", 10, NumberCallback); /////////////////////
        /*value receive
        ROS_INFO("--------------");
	ros::Subscriber sub1 = nh.subscribe("tracking", 10, Callback1);
	ROS_INFO("--------------");
	ros::Subscriber sub2 = nh.subscribe("distance", 10, Callback2);
        ROS_INFO("--------------");*/
        //if 도움 선택-----------------
                //flag = 1 //위치가 여기인지는 모르겠음
                //get_dis = nh.subscribe(); //나노에서 받아온 값 변수 저장
                //get_pos = nh.subscribe();
                //Move(get_pos, get_dis, 70);
        //----------------------------



        //Theta_Distance(0, 50, 50);
        //Motor_Controller(1, false, 50);
        //Motor_Controller(2, false, 50);
        //Find();
        //Motor_Controller(1, true, 0);
        //Motor_Controller(2, true, 0);
        //Move(get_pos, get_dis, 50); //=> 2번 째 성공 후 해 보는 걸로
	//Motor_View();
