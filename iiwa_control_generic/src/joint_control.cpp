#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>
#include <fstream>
#include <ros/ros.h>
#include <ros/service.h>
#include <chrono>
#include <string>

#include "libraries/utils.cpp"
#include "libraries/distance.cpp"
#include "libraries/robot.cpp"
#include "iiwa_msgs/JointVelocity.h"
#include "iiwa_msgs/JointPosition.h"

using std::fstream;

// Main

//Global variables
vector<VectorXd> global_histq;
ros::Publisher* global_jointPub;
bool global_SimulationMode = false;
Manipulator global_iiwa = Manipulator::createKukaIIWA();


void callbackConfig(const iiwa_msgs::JointPosition::ConstPtr& msg)
{
  iiwa_msgs::JointPosition qROS = *msg;

  // Capture the joint position
  VectorXd q(7);
  q[0] = qROS.position.a1;
  q[1] = qROS.position.a2;
  q[2] = qROS.position.a3;
  q[3] = qROS.position.a4;
  q[4] = qROS.position.a5;
  q[5] = qROS.position.a6;
  q[6] = qROS.position.a7;

  global_histq.push_back(q);
}

VectorXd getConfig()
{
  if(global_SimulationMode)
    return global_iiwa.q;
  else
    return global_histq[global_histq.size()-1];
}

void setConfig(VectorXd q)
{
  if(global_SimulationMode)
    global_iiwa.setConfig(q);
  else
  {
      iiwa_msgs::JointPosition qROS;
      qROS.position.a1 = q[0];
      qROS.position.a2 = q[1];
      qROS.position.a3 = q[2];
      qROS.position.a4 = q[3];
      qROS.position.a5 = q[4];
      qROS.position.a6 = q[5];
      qROS.position.a7 = q[6];
      global_jointPub->publish(qROS);
  }

}


int main(int argc, char** argv)
{
  // Initialize ROS variables
  ros::init(argc, argv, "iiwa_control_generic");
  ros::NodeHandle n;

   

  //Simulation mode or real robot mode
  global_SimulationMode = false;

  //Initialize subscriber to measure the joint position
  ros::Subscriber qSub = n.subscribe("/iiwa/state/JointPosition", 100, callbackConfig);
  //Initialize subscriber to publish the joint position
  ros::Publisher jointPub = n.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 100);
  global_jointPub = &jointPub;

  //Select target pose for the end effector (eef) to move
  //As an example, the target pose is the starting pose:
  //Translated 30cm in X and -30cm in Z (world frame)
  //Rotated 90 degrees in Y (end-effector frame)
  //The global_iiwa.fk(q).htmTool function gives the forward kinematics for the
  //end effector (as a 4x4 homogeneous transformation matrix) at the configuration q.
  //Everything is measured in the world frame.

  

  VectorXd q0 = global_iiwa.q;
  Matrix4d HTM0 = global_iiwa.fk(q0).htmTool;
  Matrix4d taskHTM =  HTM0 * Utils::rotz(M_PI);
  //Utils::trn(0.4,0.3,-0.7) * HTM0 * Utils::roty(M_PI);

 

  //Create a control rate
  //Example: 0.1 seconds
  double T = 0.1; 
  double time = 0;
  ros::Rate loop_rate(1.0/T);

  //File to write the simulation
  ofstream simFile;
  simFile.open ("/home/cair1/PycharmProjects/uaibot_vinicius/test/mexp/data.txt");

  simFile << "tgPose.add_ani_frame(0,np.matrix(" << Utils::printMatrixPython(taskHTM)<<"))"<<std::endl;

  

  bool continueLoop = true;
  while (ros::ok() && continueLoop)
  {
    ros::spinOnce();


    //Compute the current joint velocity to reach the target
    //It is a function of the current configuration and the target pose.
    //"constControl" stands for "Constrained Control", because the controller
    //takes into consideration constraints as
    //(1) Joint velocity limits
    //(2) Joint limits
    //(3) Obstacles
    //In this example, joint velocities limits and joint limits are active, but there are
    //no obstacles.
    //These can be eneabled/disabled  by the "param" input of the constControl function (since it does
    //not appear here, it is set to the default).
    ConstControlResult ccr = global_iiwa.constControl(getConfig(), taskHTM);
    
    //Compute next configuration and send it to the robot by a simple numerical integration 
    //of the velocity. Here we user 1st order Euler explicit with timestep "T"
    VectorXd qdot = ccr.action;
    VectorXd qnext = getConfig() + qdot*T;
    setConfig(qnext);

    //Update time (necessary only for simulation)
    time+=T;

    //Print some information about the task
    TaskResult tr = ccr.taskResult;
    ROS_INFO_STREAM("-------------------------");
    ROS_INFO_STREAM("Position error (m): "<<tr.maxErrorPos);
    ROS_INFO_STREAM("Orientation error (degrees): "<<tr.maxErrorOri);

    //Continue the controler until the maximum position error is below 1 cm and
    //the orientation error is below 2 degrees
    continueLoop = tr.maxErrorPos>0.01 || tr.maxErrorOri > 2.0; 

    


    //Write the configuration to a file with the timestamp
    Utils::sendToSimulation(&simFile, time, getConfig());


    
    if(!continueLoop)
      ROS_INFO_STREAM("Pose converged!");


    loop_rate.sleep();
  }


}
