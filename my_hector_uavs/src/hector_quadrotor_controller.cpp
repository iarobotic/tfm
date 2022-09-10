/* 
@description:
This node executes the controller to solve the trajectory tracking problem using one simulated drone in Gazebo.
In the "velocity_controller" function both the desired trajectory and the signals control are computed.
For this example, the desired trajectory is a lemniscate.
First, the "/enable_motors" service is called for moving the simulated quadrotor. Also, take off and land functions are defined.
Both the traslational velocities (Vx, Vy, Vz) and the rotational velocity (Wz) are published in the "/cmd_vel" topic.
Both the position (x,y,z) and the orientation (quaternion) are received from the "/ground_truth/state" topic.

@author: C. Mauricio Arteaga-Escamilla

-- To land the drone and finish this node, please press 'Esc' key.
INSTRUCTIONS:
In different shells:
$ roslaunch my_hector_uavs my_hector_uav.launch
	# Launch one simulated quadrotor (my simplified drone) in Gazebo using an empty world. Note: Simulation starts paused.

$ rosrun my_hector_uavs hector_quadrotor_controller
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

//Define global objects
ros::Publisher  velocity_publisher;
ros::Subscriber pose_subscriber;
geometry_msgs::Twist vel;
ros::ServiceClient enable_motors_client;

double x, y, z, roll, pitch, yaw; //Define global variables (position and orientation, using Euler anlges)
float takeoff_alt = 1.2; //Desired initial altitude
int key;

double t, t0, Vxy_max = 1.5, Vz_max = 0.5, Wz_max = 4; ///timer, initial time, maximum translational velocities [m/s] and the maximum rotational velocities [rad/s]

double T = 100, k = 0.1; //Trajectory period, controller gains kx = ky = k
double ex, ey, ez, e_yaw, Xd, Yd, Zd, Yawd, Xdp, Ydp, Zdp, Yawdp; //Tracking errors, desired positions, desired orientation and time derivative, respectively

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
  ROS_WARN_ONCE("Calling the enable_motors service...\n");
  hector_uav_msgs::EnableMotors enable_request;
  enable_request.request.enable = true;
  enable_motors_client.call(enable_request);

  if(enable_request.response.success){ ROS_WARN_ONCE("Enable motors successful\n"); }
  else{ cout << "Enable motors failed" << endl; }
}

void poseCallback(const nav_msgs::Odometry::ConstPtr msg){ //Callback function to obtain the posture of the drone
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;
  z = msg->pose.pose.position.z;
  //Operations to convert from quaternion to Euler angles
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);  //Euler angles are given in radians
}

void movement(float x, float y, float z, float turn){ //Function to assign control signals (Vx, Vy, Vz, Wz)
  vel.linear.x = x;       vel.linear.y = y;
  vel.linear.z = z;       vel.angular.z = turn;
}

void takeoff(){ //Takeoff function
  movement(0,0,0.2,0);
  velocity_publisher.publish(vel);

  cout << " Desired takeoff altitude = " << takeoff_alt << "\n Taking off ...\n";
  while(z < takeoff_alt-0.1){ //Compare if the desired altitude is reached
    velocity_publisher.publish(vel);
    ros::spinOnce(); ros::Duration(0.1).sleep();
    if(kbhit())	key = getchar();
    if(key == 27) break;
  }
  movement(0,0,0,0);
  velocity_publisher.publish(vel);
}

void land(){ //Land function
  movement(0,0,-0.2,0);
  velocity_publisher.publish(vel);

  cout << "\n Landing ...\n";
  while(z > 0.3){
    velocity_publisher.publish(vel);
    ros::spinOnce(); ros::Duration(0.1).sleep();
  }
  movement(0,0,0,0);
  velocity_publisher.publish(vel);
}

void velocity_controller(){ //Function to generate the desired trajectory and to compute the signals control
  double Vx, Vy, Vz, Wz; //Define control signals of the drone

  //Desired trajectory: Lemniscate
  double a = 3, b = 1.5, X0 = 0, Y0 = -0.5, Z0 = 1.5, w = 2*M_PI/T, c = 0.5, d = M_PI/2;
  //Desired position in the space 3D
  Xd = X0+a*sin(w*t);
  Yd = Y0+b*sin(2*w*t);
  Zd = Z0+c*sin(w*t); //Note: 1 <= Zd <= 2
  Yawd = d*sin(w*t);

  //Corresponding time derivatives
  Xdp = a*w*cos(w*t);
  Ydp = 2*b*w*cos(2*w*t);
  Zdp = c*w*cos(w*t);
  Yawdp = d*w*cos(w*t);


  ex = x-Xd; ey = y-Yd; ez = z-Zd; e_yaw = yaw - Yawd; //Compute tracking errors

  //Cinematic controller. Auxiliar controls, in global coordinates
  double Ux = Xdp-k*ex; double Uy = Ydp-k*ey;
  
  //Translational velocities with respect to the robot frame
  Vx = Ux*cos(yaw)+Uy*sin(yaw);
  Vy = -Ux*sin(yaw)+Uy*cos(yaw);

  //Cinematic controller. Note: Vz and Wz are compute directly since they are given with respect to the robot frame
  Vz = Zdp-k*ez;
  Wz = Yawdp-k*e_yaw;

  //Velocities saturation
  if(abs(Vx)>Vxy_max){Vx = Vxy_max*abs(Vx)/Vx; printf("Sat Vx\t");}
  if(abs(Vy)>Vxy_max){Vy = Vxy_max*abs(Vy)/Vy; printf("Sat Vy\t");}
  if(abs(Vz)>Vz_max){Vz = Vz_max*abs(Vz)/Vz; printf("Sat Vz\t");}
  if(abs(Wz)>Wz_max){Wz = Wz_max*abs(Wz)/Wz; printf("Sat Wz\t");}

  movement(Vx,Vy,Vz,Wz);
  velocity_publisher.publish(vel); //Publish the 4 control signals
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_quadrotor_controller");  //Initialize the node
  ros::NodeHandle n; //Create the node handle
  int freq = 50; //Node frequency (Hz)
  ros::Rate loop_rate(freq);
  int counter = 0;
  bool finish = false;

  enable_motors_client = n.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors"); //To call the service
  velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //To publish in the topic
  pose_subscriber = n.subscribe("/ground_truth/state", 1, poseCallback); //To subscribe to the topic

  enable_motors(); //Execute the functions "enable_motors" and "takeoff"
  takeoff();
 
  printf("Press 'ESC' to land the drone and finish the node");
  ROS_WARN_ONCE("To start the movement, the simulation must be running\n\n"); //Warning message

  t0 = ros::Time::now().toSec(); //Get the initial time

  do{
    t = ros::Time::now().toSec()-t0; //printf("t: %.1f\n",t); //Compute the controller time    
    velocity_controller(); //Compute the control signals

    if(counter == 50){ //Frequency divisor
      printf("ex: %.3f ey: %.3f ez: %.2f e_yaw: %.3f\n", ex,ey,ez,e_yaw); //Print in terminal tracking errors [m] and orientation error [rad]
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

