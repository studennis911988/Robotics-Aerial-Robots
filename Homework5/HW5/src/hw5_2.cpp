#include "ros/ros.h"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <sys/stat.h>

Eigen::Vector3d rad2deg(Eigen::Vector3d& radians)
{
    // Implement your code here
    radians *= 180 / M_PI;
    return radians;
}

Eigen::Vector3d Quaternion2Euler(Eigen::Quaterniond& Q)
{
    Eigen::Vector3d Euler(0, 0, 0);
    Euler.x() = atan2(2 * (Q.w() * Q.x() + Q.y() * Q.z()), (1 - 2 * (Q.x() * Q.x() + Q.y() * Q.y())));
    // Implement your code here
    Euler.y() = asin(2 * (Q.w() * Q.y() + Q.z() * Q.x()));
    Euler.z() = atan2(2 * (Q.w() * Q.z() + Q.x() * Q.y()), (1 - 2 * (Q.y() * Q.y() + Q.z() * Q.z())));

    return Euler;
}

//double intuitiveDegree(double degree)
//{
//  return (degree >= 0 || degree < 180
//}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "eigen_transform");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    double t = 0.0;
    std::ofstream results;
    std::string path = "/home/dennis/ROS/aerial_robots_ws/src/Homework5/results.txt";
    results.open(path);


    // read from xls file
    std::string filename = argv[1];
    std::ifstream ifs(filename.c_str());
    std::string line;


    std::cout << "reading ... plz wait" << "\n";

    while(std::getline(ifs, line)){
      double ax, ay, az;

      if(sscanf(line.c_str(), "%lf,%lf,%lf", &ax, &ay, &az) == 3)
      {
          // get quternion
          Eigen::Vector3d acc_measurement = Eigen::Vector3d(ax, ay, az);
          Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.8);
          Eigen::Quaterniond q_E_S = Eigen::Quaterniond::FromTwoVectors(acc_measurement, gravity);

          q_E_S.normalized();
          std::cout << "quaternion (w,x,y,z) = " << "(" << q_E_S.w() << "," << q_E_S.x() << "," << q_E_S.y() << "," << q_E_S.z() << ")"<< "\n";

          // transform to euler
          Eigen::Vector3d euler_rad = Quaternion2Euler(q_E_S);

          // rad to degree
          Eigen::Vector3d euler_deg = rad2deg(euler_rad);

          std::cout << "euler, roll: = " << euler_deg.x()<<", pitch: "<<euler_deg.y()<<", yaw: "<<euler_deg.z()<<std::endl;



          results<<t<<" "
          <<euler_deg.x()<<" "<<euler_deg.y()<<" "<<euler_deg.z()<<std::endl;

          t += 0.05;


      }
        loop_rate.sleep();
        ros::spinOnce();
    }

    std::cout << "finish reading ... exit" << "\n";

    return 0;
}
