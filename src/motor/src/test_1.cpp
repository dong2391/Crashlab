#include <ros/ros.h>
#include <motor/motor_node.h>
#include <encoder_msg/encoder.h>
#include <odom_msg/position.h>
#include <fstream>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <iostream>
#include <cmath>

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
  if(flag == 0){
  	pinum=pigpio_start(NULL, NULL);

  	if(pinum<0)
  	{
    	ROS_INFO("Setup failed");
    	ROS_INFO("pinum is %d", pinum);
    	return 1;
  	}
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
  if(gpio_read(pinum, motor2_DIR) == true)EncoderCounter2A ++;
  else EncoderCounter2A --;
  EncoderSpeedCounter2 ++;
}
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor2_DIR) == true)EncoderCounter2B ++;
  else EncoderCounter2B --;
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
  
  motor_mode = 0;
  find_mode = 1;
  start_flag = 0;
  finish_flag = 0; 
  finish = 0;

  get_dis = 200;
  get_pos = 0;
  pre_pos = 0;

  Wheel_round = 2*PI*Wheel_radius;
  Robot_round = 2*PI*Robot_radius;
/*
  ROS_INFO("PWM_range %d", PWM_range); 
  ROS_INFO("PWM_frequency %d", PWM_frequency);
  ROS_INFO("PWM_limit %d", PWM_limit);
  ROS_INFO("Control_cycle %f", Control_cycle);
  ROS_INFO("Acceleration_ratio %d", Acceleration_ratio);
  ROS_INFO("Initialize Complete");

  printf("\033[2J"); */ 
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

void Motor_View() 
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
	printf("dis  :%10.0d\n", get_dis);
	printf("\n");
}

//--------------------------------
void Callback1(const std_msgs::Int64 &msg1)
{
	get_pos = msg1.data;
	ROS_INFO("pos: %d", get_pos);
}
void Callback2(const std_msgs::Int64 &msg2)
{
	get_dis = msg2.data;
	ROS_INFO("dis: %d", get_dis);
}

void odom_position(const odom_msg::position &msg){
	p_x = msg.x;
	p_y = msg.y;
	p_th = msg.th;
	start_th = msg.th;
	//ROS_INFO("(%f, %f), %f", p_x, p_y, p_th);
}

void gui_callback(const std_msgs::Int32 &msg3){
	gui_mode = msg3.data;
	ROS_INFO("gui_mode = %d", gui_mode);
}

void finish_callback(const std_msgs::Int32 &msg4){
	finish = msg4.data;
	ROS_INFO("finish = %d", finish);
} 
//--------------------------------

void Start()
{
	int local_PWM1;
  int local_PWM2;
  int local1;
  int local2;

	ROS_INFO("start: %d %d", get_dis, get_pos);

	if(get_dis > standard) 
	{
		local1 = get_dis - 70;
		local2 = get_dis - 60;
		if(get_pos > 320)
		{
			local_PWM1 = local1 - (abs(320 - get_pos) / 18);
			local_PWM2 = local2 + (abs(320 - get_pos) / 18);
			//ROS_INFO("%d %d", local_PWM1, local_PWM2);
			Motor_Controller(1, true, local_PWM1);
			Motor_Controller(2, true, local_PWM2);
		}
		else
		{
			local_PWM1 = local1 + (abs(320 - get_pos) / 18);
			local_PWM2 = local2 - (abs(320 - get_pos) / 18);
			//ROS_INFO("%d %d", local_PWM1, local_PWM2);
			Motor_Controller(1, true, local_PWM1);
			Motor_Controller(2, true, local_PWM2);
		}
	}
	else
	{
		Motor_Controller(1, true, 0);
		Motor_Controller(2, true, 0);

		start_flag = 1;
	        ROS_INFO("start_flag: %d", start_flag);
	}
}

double Target_ang(double x, double y){
    double ang, t_ang;
    ang = atan(y/x) * (180/PI);
    t_ang = ang - p_th;
    if(p_y < y){
        if(p_x > x) t_ang = 180 - t_ang;
    }
    else{
        if(p_x > x) t_ang = 360 - t_ang;
        else t_ang = 180 + t_ang;
    }
    return t_ang;
}

double Error_th(double th){
    double e_th;
    e_th = th - p_th;
    //ROS_INFO("e_th =%f", e_th);

    if(abs(e_th) > 180){
        if(e_th > 0) e_th = -360 + e_th;
        else e_th = 360 + e_th;
    }
    return e_th;
}

void Target_move(double ref_x, double ref_y, double ref_th ){
    double ref_a, error_a, error_x, error_y, error_th;
    double bias_x = 15, bias_y = 15, bias = 10;

    ref_a = atan2(ref_y - p_y, ref_x - p_x) * (180/PI);
    if(ref_a < 0) ref_a = 360 + ref_a;

    error_a = Error_th(ref_a);
    error_x = ref_x - p_x;
    error_y = ref_y - p_y;
    error_th = Error_th(ref_th);
   // ROS_INFO("ref_x : %f, ref_y : %f, ref_a : %f, ref_th : %f", ref_x, ref_y, ref_a, ref_th);
   // ROS_INFO("error_x : %f , error_y : %f, error_a : %f, error_th : %f", error_x, error_y, error_a, error_th);
    ROS_INFO("p_x : %f , p_y : %f, p_th : %f \n\n", p_x, p_y, p_th);

    if(abs(error_x) > bias_x || abs(error_y) > bias_y){
        if(abs(error_a) > bias){
            if(error_a > 0){ //turn left
                Motor_Controller(1, true, 70); //right motor
                Motor_Controller(2, false, 80); // left motor   
            }
            else{ //turn right
                Motor_Controller(1, false, 70); //right motor
                Motor_Controller(2, true, 80); // left motor  
            } 
            ROS_INFO("target_move case : 1");
        }

        else{
            Motor_Controller(1, true, 70);
	    Motor_Controller(2, true, 80);
            ROS_INFO("target_move case : 2");
        }
    }
    else{
	ROS_INFO("target_move case : 3");

        if(abs(error_th) > bias){
            if(error_th > 0){ //turn left
                Motor_Controller(1, true, 70); //right motor
                Motor_Controller(2, false, 80); // left motor     
            }
            else{ //turn left
                Motor_Controller(1, false, 70); //right motor
                Motor_Controller(2, true, 80); // left motor 
            }
        }
        else {
            if(motor_mode == 1) find_mode += 1;
	    else{
		finish_flag = 1;

		ROS_INFO("finish_flag: %d", finish_flag);
	    }
        }
    }
}

void Find(double target_x, double target_y, double target_th){
    if(get_dis <= standard){
        Motor_Controller(1, true, get_dis - 70);
        Motor_Controller(2, true, get_dis - 60);
        ROS_INFO("WHO ARE YOU");
    }
    else{
        Target_move(target_x, target_y, target_th);
       // ROS_INFO("go target");
        }
}

void Finish(){
    if(get_dis <= 100){
        Motor_Controller(1, true, get_dis - 70);
        Motor_Controller(2, true, get_dis - 60);
    }
    else Target_move(0.0, 0.0, 270.0);
}

void Tracking(int pwm1, int pwm2) //좌표값에 따른 방향 조절, 거리값에 따른 속도조절
{
	int local_PWM1;
  int local_PWM2;
  int local_basic_PWM1 = Limit_Function(pwm1);
  int local_basic_PWM2 = Limit_Function(pwm2);
 	ROS_INFO("Tracking : %d %d", get_dis, get_pos);
 
	if(get_dis > standard)
	{//fast
		local_PWM1 = local_basic_PWM1 + (get_dis / 10);
		local_PWM2 = local_basic_PWM2 + (get_dis / 10);
		if(get_pos < 320)
		{
			local_PWM1 = local_PWM1 + (abs(320 - get_pos) / 18);
			local_PWM2 = local_PWM2 - (abs(320 - get_pos) / 18);
			ROS_INFO("%d %d", local_PWM1, local_PWM2);
			Motor_Controller(1, true, local_PWM1);
			Motor_Controller(2, true, local_PWM2);
		}
		else
		{
			local_PWM1 = local_PWM1 - (abs(320 - get_pos) / 18);
			local_PWM2 = local_PWM2 + (abs(320 - get_pos) / 18);
			ROS_INFO("%d %d", local_PWM1, local_PWM2);
			Motor_Controller(1, true, local_PWM1);
			Motor_Controller(2, true, local_PWM2);
		}
	}
	else
	{//fast
		local_PWM1 = 45;
                local_PWM2 = 55;

		if(get_pos < 320)
		{
			local_PWM1 = local_PWM1 + (abs(320 - get_pos) / 18);
			local_PWM2 = local_PWM2 - (abs(320 - get_pos) / 18);
			ROS_INFO("%d %d", local_PWM1, local_PWM2);
			Motor_Controller(1, false, local_PWM1);
			Motor_Controller(2, false, local_PWM2);
		}
		else
		{
			local_PWM1 = local_PWM1 - (abs(320 - get_pos) / 18);
			local_PWM2 = local_PWM2 + (abs(320 - get_pos) / 18);
			ROS_INFO("%d %d", local_PWM1, local_PWM2);
			Motor_Controller(1, false, local_PWM1);
			Motor_Controller(2, false, local_PWM2);
		}
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_node11");
  ros::NodeHandle nh;
  Initialize();
  flag = 1;

  ros::Publisher encoder_pub = nh.advertise<encoder_msg::encoder>("/encoder", 1);
  ros::Publisher start_complete_pub = nh.advertise<std_msgs::Int32>("/finish", 1);
  ros::Publisher come_pub = nh.advertise<std_msgs::Int32>("/come", 1);
  ros::Publisher arrive_pub = nh.advertise<std_msgs::Int32>("/go_back", 1);
  //ros::Publisher init_pub = nh.advertise<std_msgs::Bool>("init", 10);
  encoder_msg::encoder counter;
  std_msgs::Int32 complete;
  std_msgs::Int32 come;
  std_msgs::Int32 arrive;

  
  ros::Subscriber sub1 = nh.subscribe("tracking", 1, Callback1);
  ros::Subscriber sub2 = nh.subscribe("/distance", 10, Callback2);
  ros::Subscriber sub3 = nh.subscribe("pose", 1, odom_position);
  ros::Subscriber sub4 = nh.subscribe("gui_mode", 1, gui_callback);
  ros::Subscriber sub5 = nh.subscribe("finish", 1, finish_callback);


  ros::Rate loop_rate(Control_cycle);

	while(ros::ok())
	{
	counter.E_L = Motor2_Encoder_Sum();
	counter.E_R = Motor1_Encoder_Sum();
	encoder_pub.publish(counter);

	switch(motor_mode)
	{
		case 0://find, start할 때 버튼이 눌리는 경우 대처  
			ROS_INFO("prepare!");
			Motor_Controller(1, true, 0);
        		Motor_Controller(2, true, 0);
			if(gui_mode == 1) motor_mode = 1;
			break;

		case 1:

			if(gui_mode == 21) motor_mode = 10;

			ROS_INFO("find!: %d", find_mode);

			if(find_mode == 1) Find(150.0, 0.0, 0.0);
			else if(find_mode == 2) Find(-150.0, 0.0, 180.0);
			else if(find_mode == 3) Find(0.0, 0.0, 0.0);
			else if(find_mode == 4) Find(0.0, 150.0, 90.0);
			else if(find_mode == 5) Find(0.0, 0.0, 270.0);

                        if(find_mode >= 6) find_mode = 1;

			if(get_dis <  250 && get_pos != 0) motor_mode = 2;

			break;

		case 2:
			if(gui_mode == 21) motor_mode = 10;

			if(gui_mode == 99) motor_mode = 4;

			ROS_INFO("start!");
			Start();

			if(start_flag == 1){
				motor_mode = 3;
				complete.data = 0;
				start_complete_pub.publish(complete);
				start_flag = 2;
			}

			if(gui_mode == 5) motor_mode = 3;

			break;

		case 3:
			ROS_INFO("tracking!");
			if(finish == 1) {
				motor_mode = 4;
				Init_Encoder();
			}

                        if(gui_mode == 99) motor_mode = 4;


			if(get_dis >= 230)
			{
				come.data = 1;
                                come_pub.publish(come);
			}

			if(standard < get_dis && get_dis < 250 && get_pos != 0)
                        {
                        	Tracking(80, 90);
                        }
			else
			{
				Motor_Controller(1, true, 0);
                                Motor_Controller(2, true, 0);
			}

			break;

		case 10://find, start할 때 버튼이 눌리는 경우 대처  
                        ROS_INFO("case10!");
                        Motor_Controller(1, true, 0);
                        Motor_Controller(2, true, 0);
                        motor_mode = 3;
                        break;

		default:
			ROS_INFO("finish!");
			if(gui_mode == 10 || gui_mode == 99 || gui_mode == 1) Finish();
			else{
			        Motor_Controller(1, true, 0);
                                Motor_Controller(2, true, 0);
			}

			if(finish_flag == 1)
			{
				Motor_Controller(1, true, 0);
		                Motor_Controller(2, true, 0);

				ROS_INFO("init_position!");
				Initialize();
				arrive.data = 3;
				arrive_pub.publish(arrive);
			}
			break;
		}
	  	ros::spinOnce();
	  	loop_rate.sleep();
  	}
  	Motor_Controller(1, true, 0);
  	Motor_Controller(2, true, 0);
  	return 0;
}
