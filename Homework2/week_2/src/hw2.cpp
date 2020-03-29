// include ros library
#include "ros/ros.h"

// include msg library
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

// include math 
#include <math.h>

double Kp = 0.2;
double Ka = 0.5;
double Kb = 0.1;


using namespace std;

turtlesim::Pose pose;
geometry_msgs::Twist vel_msg;
geometry_msgs::Point goal_point;
//Define a data structure to 3D
struct XYZ{
  float x;
  float y;
  float z;
};
////Declare a variable.Its name is pos_err with XYZ data type
//struct XYZ pos_err;

// declare call back function(call back the pose of robot)
void pos_cb(const turtlesim::Pose::ConstPtr& msg)
{
  pose = *msg;
} 

void coordinateTransform(XYZ& err, double& rho, double& alpha, double& belta)
{
  double delta_x = err.x;
  double delta_y = err.y;
  double theta = pose.theta;

  rho = sqrt(delta_x*delta_x + delta_y*delta_y);
  alpha = -theta + atan2(delta_y, delta_x);
  belta = -theta - alpha;
 }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tutorial_1");
  ros::NodeHandle n;

  // declare publisher & subscriber
  ros::Subscriber pos_sub = n.subscribe<turtlesim::Pose>("turtle1/pose", 10, pos_cb);
  ros::Publisher turtlesim_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
  //input your desired position
  ROS_INFO("Please input (x,y). x>0,y>0");
  cout<<"desired_X:";
  cin>>goal_point.x;
  cout<<"desired_Y:";
  cin>>goal_point.y;
  // setting frequency as 10 Hz
  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok()){

//    printf("\ncount : %d\n",count);
    printf("goal x : %f \t y : %f\n",goal_point.x,goal_point.y);
    printf("pose x : %f \t y : %f\n",pose.x,pose.y);

    // Calculate position error(feedback term)
    XYZ pos_err;
    pos_err.x = goal_point.x - pose.x;
    pos_err.y = goal_point.y - pose.y;
    
    /*Your error-driven controller design*/
    // transformation
    double rho = 0.0;
    double alpha = 0.0;
    double belta = 0.0;
    coordinateTransform(pos_err, rho, alpha, belta);

//    printf("err.x: %f \n", rho);
    if(rho < 0.1)
    {
      printf("error is small enough! break!");
      break;
    }

    vel_msg.linear.x = Kp * rho;
    vel_msg.angular.z = Ka * alpha - Kb * belta;
    turtlesim_pub.publish(vel_msg);

//    count ++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}



