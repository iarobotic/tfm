/* 
@description:
This node executes the controller in order to solve the formation control problem, using the leader-follower scheme, with two simulated drones in Gazebo. The leader drone tracks its desired trajectory (using the velocity controller), and then the follower drone follows the offset leader's trajectory.
First, the "/enable_motors" service of each drone is called. Take off and land functions are defined.

@author: C. Mauricio Arteaga-Escamilla

-- To land the drones and finish this node, please press 'Esc' key.
INSTRUCTIONS:
In different shells:
$ roslaunch my_hector_uavs two_hector_uavs.launch
	# Launch two simulated quadrotors (my simplified drones) in Gazebo using an empty world. Note: Simulation starts paused.

$ rosrun my_hector_uavs two_hector_quadrotors_lf
	# Run this node
*/
#include "ros/ros.h"
#include "hector_uav_msgs/EnableMotors.h" //rosservice info /enable_motors
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
//Includes needed by keyboard events (kbhit function implemented in Linux)
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

//Create a custom structure to define the robot posture (position and orientation, using Euler angles)
struct robot_pose{ double x, y, z, roll, pitch, yaw; };  

//Define global objects
ros::Publisher  vel_pub1, vel_pub2;
ros::Subscriber pose_sub1, pose_sub2;
geometry_msgs::Twist vel1, vel2;
ros::ServiceClient enable_motors_client1, enable_motors_client2;

int key; //Variable to save the key value
robot_pose r_pose1, r_pose2; //Define global variables (position and orientation, using my custom structure)
float takeoff_alt = 0.5; //Desired initial altitude

double t, t0, Vxy_max = 2, Vz_max = 0.5, Wz_max = 8; //Timer, initial time, maximum translational velocities [m/s] and the maximum rotational velocities [rad/s]

//Leader drone variables
double T = 100, k = 0.1; //Trajectory period, controller gains kx = ky = k
double ex, ey, ez, e_yaw, Xd, Yd, Zd, Yawd, Xdp, Ydp, Zdp, Yawdp; //Tracking errors, desired positions, desired orientation and time derivative, respectively
double Vxl, Vyl, Vzl, Wzl; //Control signals

//Follower drone variables
double exf, eyf, ezf, e_yawf, s_d = 1.5, alp_d = M_PI, s_zd = 0; //Tracking errors and desired formation states
double Vxf, Vyf, Vzf, Wzf;  //Control signals


int kbhit(void){ //kbhit function of Windows implemented for Linux
  struct termios oldt, newt;
  int ch, oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

void enable_motors(){ //Function to call "/enable_motors" service, needed to move the drone
  ROS_WARN_ONCE("Calling the enable_motors services ...\n");
  hector_uav_msgs::EnableMotors enable_request1, enable_request2;

  enable_request1.request.enable = true;
  enable_motors_client1.call(enable_request1);
  if(enable_request1.response.success){ cout << "uav1: enable motors successful\n"; }
  else{ cout << "uav1: enable motors failed" << endl; }

  enable_request2.request.enable = true;
  enable_motors_client2.call(enable_request2);
  if(enable_request2.response.success){ cout << "uav2: enable motors successful\n"; }
  else{ cout << "uav2: enable motors failed" << endl; }
}

void poseCallback1(const nav_msgs::Odometry::ConstPtr msg){ //Callback function to obtain the posture of the drone 1
  r_pose1.x = msg->pose.pose.position.x;
  r_pose1.y = msg->pose.pose.position.y;
  r_pose1.z = msg->pose.pose.position.z;
  //Operations to convert from quaternion to Euler angles
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(r_pose1.roll, r_pose1.pitch, r_pose1.yaw);  //Euler angles are given in radians
}

void poseCallback2(const nav_msgs::Odometry::ConstPtr msg){ //Callback function to obtain the posture of the drone 2
  r_pose2.x = msg->pose.pose.position.x;
  r_pose2.y = msg->pose.pose.position.y;
  r_pose2.z = msg->pose.pose.position.z;
  //Operations to convert from quaternion to Euler angles
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(r_pose2.roll, r_pose2.pitch, r_pose2.yaw);  //Euler angles are given in radians
}

void move1(float x, float y, float z, float turn){ //Function to assign control signals and publish them, drone 1
  vel1.linear.x = x; vel1.linear.y = y; vel1.linear.z = z; vel1.angular.z = turn;
  vel_pub1.publish(vel1);
}
void move2(float x, float y, float z, float turn){ ///Function to assign control signals and publish them, drone 2
  vel2.linear.x = x; vel2.linear.y = y; vel2.linear.z = z; vel2.angular.z = turn;
  vel_pub2.publish(vel2);
}

void takeoff(){ //Takeoff function of all drones
  cout << " Desired takeoff altitude = " << takeoff_alt << "\n Taking off ...\n";

  //Quadrotor 1
  move1(0,0,0.3,0);
  
  while(r_pose1.z < takeoff_alt-0.1){ //Compare if the desired altitude is reached
    vel_pub1.publish(vel1);
    ros::spinOnce(); ros::Duration(0.1).sleep();
    if(kbhit())	key = getchar(); //Check any key press
    if(key == 27) break;
  }
  move1(0,0,0,0); //Stop

  //Quadrotor 2
  move2(0,0,0.3,0);

  while(r_pose2.z < takeoff_alt-0.1){ //Compare if the desired altitude is reached
    vel_pub2.publish(vel2);
    vel_pub1.publish(vel1); //Important: velocities of the drone 1 must keep being published to avoid a not desired landing
    ros::spinOnce(); ros::Duration(0.1).sleep();
    if(kbhit())	key = getchar(); //Check any key press
    if(key == 27) break;
  }
  move2(0,0,0,0); //Stop
}

void land(){ //Land function of all drones
  cout << "\n Landing ...\n";

  move1(0,0,-0.3,0); move2(0,0,-0.3,0);

  while(r_pose1.z > 0.3 || r_pose2.z > 0.3){
    vel_pub1.publish(vel1);  vel_pub2.publish(vel2);
    ros::spinOnce(); ros::Duration(0.1).sleep();
  }
  move1(0,0,0,0);  move2(0,0,0,0); //Turn off all drones
}

void leader_vel_control(){ //Function to generate the desired trajectory and to compute the signals control of the leader robot
  //Define the desired trajectory: Helix
  double X0 = 0, Y0 = 0, Z0 = 0.5, radius = 3, w = 2*M_PI/T, Vzd = 0.02;

  //Desired position in the space 3D
  Xd = X0+radius*sin(w*t);
  Yd = Y0+radius*cos(w*t);
  Zd = Z0+Vzd*t; //linear increasing
  Yawd = sin(w*t); //sinusoidal behavior

  //Corresponding time derivatives
  Xdp = radius*w*cos(w*t);
  Ydp = -radius*w*sin(w*t);
  Zdp = Vzd;
  Yawdp = w*cos(w*t);


  ex = r_pose1.x-Xd; ey = r_pose1.y-Yd; ez = r_pose1.z-Zd; e_yaw = r_pose1.yaw - Yawd; //Compute tracking errors

  //Cinematic controller. Auxiliar controls, in global coordinates (local variables)
  double Ux = Xdp-k*ex; double Uy = Ydp-k*ey;
  
  //Translational velocities with respect to the robot frame
  Vxl = Ux*cos(r_pose1.yaw)+Uy*sin(r_pose1.yaw);
  Vyl = -Ux*sin(r_pose1.yaw)+Uy*cos(r_pose1.yaw);

  //Cinematic controller. Note: Vz and Wz are compute directly since they are given with respect to the robot frame
  Vzl = Zdp-k*ez;
  Wzl = Yawdp-k*e_yaw;

  //Velocities saturation
  if(abs(Vxl)>Vxy_max){Vxl = Vxy_max*abs(Vxl)/Vxl; printf("Sat Vxl\t");}
  if(abs(Vyl)>Vxy_max){Vyl = Vxy_max*abs(Vyl)/Vyl; printf("Sat Vyl\t");}
  if(abs(Vzl)>Vz_max){Vzl = Vz_max*abs(Vzl)/Vzl; printf("Sat Vzl\t");}
  if(abs(Wzl)>Wz_max){Wzl = Wz_max*abs(Wzl)/Wzl; printf("Sat Wzl\t");}

  move1(Vxl,Vyl,Vzl,Wzl); //Publish the 4 control signals
}

void follower_vel_control(){ //Function to compute the signals control of the follower robot
  //Note: desired follower position and yaw orientation are given by the leader's ones
  
  double Xfd, Yfd, Xfdp, Yfdp, xlp, ylp, Ux, Uy; //Create some local variables

  //Follower desired position with respect to the gloabl frame
  Xfd = r_pose1.x+s_d*cos(r_pose1.yaw+alp_d);
  Yfd = r_pose1.y+s_d*sin(r_pose1.yaw+alp_d);

  //Translational velocities of the leader drone in the global frame
  xlp = Vxl*cos(r_pose1.yaw)-Vyl*sin(r_pose1.yaw);
  ylp = Vxl*sin(r_pose1.yaw)+Vyl*cos(r_pose1.yaw);

  //Corresponding desired time derivatives
  Xfdp = xlp-Wzl*s_d*sin(r_pose1.yaw+alp_d);
  Yfdp = ylp+Wzl*s_d*cos(r_pose1.yaw+alp_d);

  //Compute tracking errors (with respect to the global frame)
  exf = r_pose2.x-Xfd; 			eyf = r_pose2.y-Yfd; 
  ezf = r_pose2.z-(r_pose1.z+s_zd); 	e_yawf = r_pose2.yaw - r_pose1.yaw; //Both the altitude and the orientation are equal to leader drone's ones

  //Cinematic controller. Auxiliar controls, in global coordinates
  Ux = Xfdp-k*exf; 	Uy = Yfdp-k*eyf;
  
  //Translational velocities with respect to the robot frame
  Vxf = Ux*cos(r_pose2.yaw)+Uy*sin(r_pose2.yaw);
  Vyf = -Ux*sin(r_pose2.yaw)+Uy*cos(r_pose2.yaw);

  //Vertical and rotational velocities
  Vzf = Vzl-k*ezf;
  Wzf = Wzl-k*e_yawf;

  //Velocities saturation
  if(abs(Vxf)>Vxy_max){Vxf = Vxy_max*abs(Vxf)/Vxf; printf("Sat Vxf\t");}
  if(abs(Vyf)>Vxy_max){Vyf = Vxy_max*abs(Vyf)/Vyf; printf("Sat Vyf\t");}
  if(abs(Vzf)>Vz_max){Vzf = Vz_max*abs(Vzf)/Vzf; printf("Sat Vzf\t");}
  if(abs(Wzf)>Wz_max){Wzf = Wz_max*abs(Wzf)/Wzf; printf("Sat Wzf\t");}

  move2(Vxf,Vyf,Vzf,Wzf); //Publish the 4 control signals
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "two_hector_quadrotors_lf");  //Initialize the node
  ros::NodeHandle n; //Create the node handle
  int freq = 50; //Node frequency (Hz)
  ros::Rate loop_rate(freq);
  int counter = 0;
  bool finish = false;

  enable_motors_client1 = n.serviceClient<hector_uav_msgs::EnableMotors>("uav1/enable_motors"); //To call the service
  enable_motors_client2 = n.serviceClient<hector_uav_msgs::EnableMotors>("uav2/enable_motors");
  vel_pub1 = n.advertise<geometry_msgs::Twist>("uav1/cmd_vel", 1); //To publish in the topic
  vel_pub2 = n.advertise<geometry_msgs::Twist>("uav2/cmd_vel", 1);
  pose_sub1 = n.subscribe("uav1/ground_truth/state", 1, poseCallback1); //To subscribe to the topic
  pose_sub2 = n.subscribe("uav2/ground_truth/state", 1, poseCallback2);

  enable_motors(); //Execute the functions "enable_motors" and "takeoff" (for all robots)
  takeoff();
 
  printf(" Press 'ESC' to land the drones and finish the node\n");
  ROS_WARN_ONCE(" To start the movement, the simulation must be running\n\n"); //Warning message

  t0 = ros::Time::now().toSec(); //Get the initial time

  do{
    t = ros::Time::now().toSec()-t0; //printf("t: %.1f\n",t); //Compute the controller time    
    leader_vel_control(); //Compute the control signals of the leader robot
    follower_vel_control(); //Compute the control signals of the follower robot

    if(counter == 100){ //Frequency divisor
      printf("\nex: %.3f\tey: %.3f\tez: %.2f\te_yaw: %.2f\n", ex,ey,ez,e_yaw); //Print in terminal tracking errors [m] and orientation error [rad]
      printf("exf: %.3f\teyf: %.3f\tezf: %.2f\te_yawf: %.2f\n", exf,eyf,ezf,e_yawf); //Print follower formation errors and orientation error
      
      counter = 0; //Reset the counter
    }else counter++;

    ros::spinOnce();    //Required for receiving callback functions
    loop_rate.sleep();  //command to wait the rest of the time to complete the loop rate
    if(kbhit())	key = getchar();

    if(key == 27) finish = true;
  }while(finish == false); //Note: ros::ok() is not used because land function must be executed yet
  
  land(); //Execute the land function
  printf("\n Node finished\n");
  return 0;
}

