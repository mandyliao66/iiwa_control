//subscribes to aruco detect, converts quat to rotational matrix, calculates the pose difference between the grabber and the marker

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
using namespace std;


void poseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    
      Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
      Eigen::Matrix<double, 3, 3> r = q.normalized().toRotationMatrix();
      double px = msg->position.x;
      double py = msg->position.y;
      double pz = msg->position.z;

      Eigen::Matrix<double, 4,4> Tac;
      Tac << r(0, 0), r(0, 1), r(0, 2), px,
            r(1, 0), r(1, 1), r(1, 2), py,
            r(2, 0), r(2, 1), r(2, 2), pz,
            0, 0, 0, 1;

     /* cout << "rotation" << endl << "x: " << r(0, 0) << ", " << r(0, 1) << ", " << r(0, 2) << endl
            << "y: " << r(1, 0) << ", " << r(1, 1) << ", " << r(1, 2) << endl
            << "z: " << r(2, 0) << ", " << r(2, 1) << ", " << r(2, 2) << endl;
      cout << "translation" << endl << "x " << px << endl << "y " << py << endl << "z " << pz << endl;*/
      cout << "----------------------------"<< endl;
      Eigen::Matrix<double,4,4> Tcg;
      Tcg << 1, 0, 0, 0,
            0, 1, 0, -0.078,
            0, 0, 1, 0.021,
            0, 0, 0, 1;

      Eigen::Matrix<double,4,4> Tag = Tac + Tcg;
      cout << "rotation" << endl << "x: " << Tag(0, 0) << ", " << Tag(0, 1) << ", " << Tag(0, 2) << endl
            << "y: " << Tag(1, 0) << ", " << Tag(1, 1) << ", " << Tag(1, 2) << endl
            << "z: " << Tag(2, 0) << ", " << Tag(2, 1) << ", " << Tag(2, 2) << endl;
      cout << "translation" << endl << "x " << Tag(0,3) << endl << "y " << Tag(1,3) << endl << "z " << Tag(2,3) << endl;
      cout << "-----------------------------------------------------------------"<< endl;
}


int main(int argc, char** argv)
{
      ros::init(argc, argv, "pose_matrix");
      ros::NodeHandle n;

      ros::Subscriber pose = n.subscribe("aruco_detection", 100, poseCallback);
      ros::spin(); 
      
      return 0;
}