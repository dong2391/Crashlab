#include <ros/ros.h>
#include <odometry/odom.h>
#include <odom_msg/position.h>
#include <odom_msg/boundary.h>
#include <encoder_msg/encoder.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <cmath>

using namespace std;





void E_callback(const encoder_msg::encoder &msg){
    encoder_L = msg.E_L;
    encoder_R = msg.E_R;
}

void Initialize(const std_msgs::Int32 &msg1){ 
	if(rfid_mode == 0){
	    if (msg1.data == 1){ // rfid pub when arrive finish point
		encoder_L = 0;
		encoder_R = 0;
		L_Encodercount_prev=0;
		R_Encodercount_prev=0;
		p_x = 0.0;
		p_y = 5.0;
		p_th = PI/2;
		rfid_mode = 1;
		start_mode = 0;

    		cout << "arrive finish point!! \n\n\n\n\n\n\n" << endl;
	    }
	  }
}

//void RFID_cb(const std_msgs::Int32 &msg2){
//	if(msg2.data == 1){
//		p_x = 0.0;
//		p_y = 20.0;
//		p_th = 90.0;
//		cout << "rfid is scaned!!" << endl;
//	}
//}

void start_init_cb(const std_msgs::Int32 &msg3){
	if(start_mode ==  1){
		if(msg3.data == 3){
			p_x = 0.0;
			p_y = 0.0;
			p_th = PI * 3/2;
			encoder_L = 0;
	        	encoder_R = 0;
	        	L_Encodercount_prev=0;
        		R_Encodercount_prev=0;
			rfid_mode = 0;
			start_mode = 1;
			cout << " comback init point" <<endl;
			}
		
	
	
	}
}

void Odometry (int L_Encodercount, int R_Encodercount){
    d_E_L = L_Encodercount - L_Encodercount_prev;
    d_E_R = -R_Encodercount + R_Encodercount_prev; 

    w_L = d_E_L * thetaPerCount / dt; // L_encoder_counter * thetapercount / dt (rad/s)
    w_R = d_E_R * thetaPerCount / dt; // R_encoder_counter * thetapercount / dt (rad/s)

    wheel_v_L = wheel_radius * w_L; // wheel_r * w_L (m/s)
    wheel_v_R = wheel_radius * w_R; // wheel_r * w_R (m/s)

    robot_v = (wheel_v_R + wheel_v_L) / 2; // robot vel : average wheel vel (m/s)
    robot_w = (wheel_v_R - wheel_v_L) / (2*robot_radius); //robot rot : sub_wheel_vel/robot_diameter

    p_th = p_th + (robot_w * dt); // theta position = rot_vel * dt
    if (p_th >= 2*PI) p_th = p_th - 2*PI;
    else if (p_th < 0) p_th = p_th + 2*PI;

    v_x = robot_v * cos(p_th + robot_w * dt / 2);
    v_y = robot_v * sin(p_th + robot_w * dt / 2);

    p_x = p_x + (v_x * dt);
    p_y = p_y + (v_y * dt);

    L_Encodercount_prev = L_Encodercount;
    R_Encodercount_prev = R_Encodercount;
}

/*
void boundary(double p_x, double p_y, double p_th){ 
    //motor -> odom 끝, boundary 계산
    int x_boundary = 200;
    int y_boundary = 200;

    if((p_x > x_boundary/2 && p_y > y_boundary/2) || (p_x > x_boundary/2 && p_y < -y_boundary/2) || (p_x < -x_boundary/2 && p_y > y_boundary/2) || (p_x < -x_boundary/2 && p_y < -y_boundary/2)){
        if(p_th == 45 || p_th == 135 || p_th == 225 ||p_th == 275){
            turn_180();
        }
    }
    
    else if(p_x > x_boundary / 2){
        if((p_th>0 && p_th<90) || (p_th<360 && p_th>270)){ 
            //stop();
           if(p_th < 180){
                turn_L(p_th, 0, x_boundary - p_x);
            }
            else{
                turn_R(p_th, 360, x_boundary - p_x);
            }
        }
    }
    else if(p_x < -1 * x_boundary/2){
        if (p_th == 90){
            //stop();
            turn_180();
        }
        else if((p_th>90 && p_th<180) || (p_th>180 && p_th <270)){
            //stop();
            if(p_th < 180){
                turn_R(p_th, 180, x_boundary - p_x);
            }
            else{
                turn_L(p_th, 180, x_boundary - p_x);
            }
        }
    }

    else if(p_y > y_boundary/2){
        if((p_th>0 && p_th<90) || (p_th<90 && p_th>180)){ 
            //stop();
            if(p_th < 90){
                turn_R(p_th, 90, y_boundary - p_y);
            }
            else{
                turn_L(p_th, 90, y_boundary - p_y);
            }
        }
    }
    
    else if(p_y < -1 * y_boundary/2){
        if (p_th == 270){
            //stop();
            //turn_180();
            
            t_180_pub.publish();
        }
        else if((p_th>0 && p_th<270) || (p_th>270 && p_th <360)){
            //stop();
            if(p_th < 270){
                turn_R(p_th, 270, y_boundary - p_y);
                turn_L(p_th, 270, y_boundary - p_y);
            }
        }
    }
}
*/




int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    
    ros::Publisher pose_pub = nh.advertise<odom_msg::position>("pose", 1);
    

    ros::Subscriber sub1 = nh.subscribe("/encoder", 1, E_callback); 
    ros::Subscriber Init = nh.subscribe("/finish", 1, Initialize); // arrive finish point 
    //ros::Subscriber rfid = nh.subscribe("/rfid", 1, RFID_cb);
    ros::Subscriber start_init = nh.subscribe("/go_back", 1, start_init_cb); // go init point from finish point
	
    ros::Rate loop_rate(10);
    
    odom_msg::position odom;
    
    cout<<"odom ready"<<endl;

    while(ros::ok()){
        Odometry(encoder_L, encoder_R);
        odom.x = p_x * 100;
        odom.y = p_y * 100;
        odom.th = p_th *( 180 / PI);
        pose_pub.publish(odom);
        cout<<"L_encoder : "<<encoder_L<<"  ||  R_encoder : "<<encoder_R<<endl;
        cout<<"Robot rocation : "<<p_x<<", "<<p_y<<", theta : "<< odom.th <<endl;
        ros::spinOnce();
    }
    return 0;
}
