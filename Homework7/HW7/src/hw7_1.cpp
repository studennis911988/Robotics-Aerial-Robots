#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>

// Callback of timer
// Broadcaster transform from A to B and B to C
void broadcastTF(const ros::TimerEvent& timer_event)
{
    static tf::TransformBroadcaster br;

    // B2A
    tf::Transform tf_B_A;

    // transform from A to B
    Eigen::Quaterniond q_B_A = Eigen::Quaterniond(std::sqrt(2)/2,0,0,std::sqrt(2)/2);
    Eigen::Vector3d t_B_A = Eigen::Vector3d(1,0,0);


    tf::Quaternion tf_q_B_A = tf::Quaternion(q_B_A.x(), q_B_A.y(), q_B_A.z(), q_B_A.w());
    tf::Vector3 tf_t_B_A = tf::Vector3(t_B_A.x(), t_B_A.y(), t_B_A.z());
    tf_B_A.setOrigin(tf_t_B_A);
    tf_B_A.setRotation(tf_q_B_A);



    br.sendTransform(tf::StampedTransform(tf_B_A, // transform
                                          ros::Time::now(), // timestamp with this transform
                                          "B", // paranet frame ID
                                          "A")); // child frame ID

    // C2B
    tf::Transform tf_C_B;

    Eigen::Quaterniond q_C_B = Eigen::Quaterniond(std::sqrt(3)/2,0,0,0.5);
    Eigen::Vector3d t_C_B = Eigen::Vector3d(1,0,0);


    tf::Quaternion tf_q_C_B = tf::Quaternion(q_C_B.x(), q_C_B.y(), q_C_B.z(), q_C_B.w());
    tf::Vector3 tf_t_C_B = tf::Vector3(t_C_B.x(), t_C_B.y(), t_C_B.z());
    tf_C_B.setOrigin(tf_t_C_B);
    tf_C_B.setRotation(tf_q_C_B);



    br.sendTransform(tf::StampedTransform(tf_C_B, // transform
                                          ros::Time::now(), // timestamp with this transform
                                          "C", // paranet frame ID
                                          "B")); // child frame ID

    // A2C
    tf::Transform tf_C_A = tf_C_B * tf_B_A;

    br.sendTransform(tf::StampedTransform(tf_C_A, // transform
                                          ros::Time::now(), // timestamp with this transform
                                          "C", // paranet frame ID
                                          "A")); // child frame ID

    // A2A1
    tf::Transform tf_A1_A;
    tf::Quaternion tf_q_A1_A = tf::Quaternion(0,0,0,1);
    tf::Vector3 tf_t_A1_A = tf::Vector3(0,0,0);
    tf_A1_A.setOrigin(tf_t_A1_A);
    tf_A1_A.setRotation(tf_q_A1_A);
    br.sendTransform(tf::StampedTransform(tf_A1_A, // transform
                                          ros::Time::now(), // timestamp with this transform
                                          "A", // paranet frame ID
                                          "A1")); // child frame ID


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hw7_1");
  ros::NodeHandle nh;
  // Create timer with 2.0 Hz
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), broadcastTF);
  while (ros::ok()){ros::spinOnce();}
  return 0;
}
