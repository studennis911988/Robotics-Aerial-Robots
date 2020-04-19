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
    tf::Transform tf_A_B;

    // transform from A to B
    Eigen::Quaterniond q_A_B = Eigen::Quaterniond(1,0,0,0);
    Eigen::Vector3d t_A_B = Eigen::Vector3d(1,0,0);


    tf::Quaternion tf_q_A_B = tf::Quaternion(q_A_B.x(), q_A_B.y(), q_A_B.z(), q_A_B.w());
    tf::Vector3 tf_t_A_B = tf::Vector3(t_A_B.x(), t_A_B.y(), t_A_B.z());
    tf_A_B.setOrigin(tf_t_A_B);
    tf_A_B.setRotation(tf_q_A_B);



    br.sendTransform(tf::StampedTransform(tf_A_B, // transform
                                          ros::Time::now(), // timestamp with this transform
                                          "A", // paranet frame ID
                                          "B")); // child frame ID

    // A2C
    tf::Transform tf_A_C;

    Eigen::Quaterniond q_A_C = Eigen::Quaterniond(1,0,0,0);
    Eigen::Vector3d t_A_C = Eigen::Vector3d(0,1,0);


    tf::Quaternion tf_q_A_C = tf::Quaternion(q_A_C.x(), q_A_C.y(), q_A_C.z(), q_A_C.w());
    tf::Vector3 tf_t_A_C = tf::Vector3(t_A_C.x(), t_A_C.y(), t_A_C.z());
    tf_A_C.setOrigin(tf_t_A_C);
    tf_A_C.setRotation(tf_q_A_C);



    br.sendTransform(tf::StampedTransform(tf_A_C, // transform
                                          ros::Time::now(), // timestamp with this transform
                                          "A", // paranet frame ID
                                          "C")); // child frame ID

    // C2B
    tf::Transform tf_C_B = tf_A_C.inverse() * tf_A_B;

    br.sendTransform(tf::StampedTransform(tf_C_B, // transform
                                          ros::Time::now(), // timestamp with this transform
                                          "C", // paranet frame ID
                                          "B")); // child frame ID


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hw7_2");
  ros::NodeHandle nh;
  // Create timer with 2.0 Hz
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), broadcastTF);
  while (ros::ok()){ros::spinOnce();}
  return 0;
}
