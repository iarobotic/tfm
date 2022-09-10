/* 
@brief: An example of both how to publish velocities to a simulated drone and how to subscribe to the drone odometry is shown.
 
@description:
This node publishes velocities (in open-loop) to the simulated drone in Gazebo, and it subscribes to the drone odometry in order to obtain the position and the orientation of the quadrotor.
First, the "/enable_motors" service is called for moving the simulated quadrotor. Also, take off and land functions are defined.
Both the traslational velocities (Vx, Vy, Vz) and the rotational velocity (Wz) are published in the "/cmd_vel" topic.
Both the position (x,y,z) and the orientation (quaternion) are received from the "/ground_truth/state" topic.

@author: C. Mauricio Arteaga-Escamilla

-- To land the drone and finish this node, please press 'Esc' key.
INSTRUCTIONS:
In different shells:
$ roslaunch my_hector_uavs my_hector_uav.launch
	# Launch one simulated quadrotor (my simplified drone) in Gazebo using an empty world. Note: Simulation starts paused.

$ rosrun my_hector_uavs hector_quadrotor_simple_movement
	# Run this node
*/
#include "ros/ros.h"
#include "hector_uav_msgs/EnableMotors.h" //rosservice info /enable_motors
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include <iostream>
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
    if(kbhit())	key = getchar(); //Check any key press
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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_quadrotor_simple_movement");  //Initialize the node
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

  //Circular movement on the horizontal plane
  movement(0.4,0,0,0.5);
 
  printf("Press 'ESC' to land the drone and finish the node\n");
  ROS_WARN_ONCE("To start the movement, the simulation must be running\n\n"); //Warning message

  do{
    velocity_publisher.publish(vel); //Publish the velocities
    if(counter == 50){ //Frequency divisor
      printf("x: %.3f y: %.3f z: %.3f roll: %.3f pitch: %.3f yaw: %.3f\n", x,y,z, roll,pitch,yaw);
      //Print in terminal position [m] and orientation [rad]
      counter = 0; //Reset the counter
    }else counter++;

    ros::spinOnce();    //Required for receiving callback functions
    loop_rate.sleep();  //Command to wait the rest of the time to complete the loop rate
    if(kbhit())	key = getchar();

    if(key == 27) finish = true; //ESC key = 27 in Ascii code
  }while(finish == false); //Note: ros::ok() is not used because land function must be executed yet
  
  land(); //Execute the land function
  printf("\n Node finished\n");
  return 0;
}

