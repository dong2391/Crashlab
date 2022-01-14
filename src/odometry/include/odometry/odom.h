//Length Unit is "m"

#ifndef odom_H
#define odom_H

#define PI 3.141592
#define wheel_radius  5.75/100
#define robot_radius  20.52/100 //1m = 100cm
#define thetaPerCount  2*PI / (228 * 4)

//encoder
float encoder_L;
float encoder_R;

//init
int rfid_mode = 0;
int start_mode;

//odom
void Odometry(int L_Encodercount, int R_Encodercount);

// double current_time;
// double last_time;
double dt;

//#define x_boundary 200
//#define y_boundary 200

int d_E_L;
int d_E_R;

double w_L;
double w_R;

double wheel_v_L;
double wheel_v_R;

double robot_v;
double robot_w;

double v_x;
double v_y;

double p_x;
double p_y;
double p_th = 3 * PI/2;

int L_Encodercount_prev;
int R_Encodercount_prev;

#endif
