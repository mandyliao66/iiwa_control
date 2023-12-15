#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
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

using namespace std;
//Global variables
vector<VectorXd> global_histq;
ros::Publisher* global_jointPub;
bool global_SimulationMode = false;
bool continueLoop1 = true; 
    double T = 0.1;
Matrix4d last_tool_position;
Matrix4d aruco_pose;

Manipulator global_iiwa = Manipulator::createKukaIIWA();
bool poseGetting =false;
Eigen::Matrix<double, 4, 4>Tgg;
Eigen::Vector3d angles;
double px=0;
double py=0;
double pz=0;
Eigen::Matrix<double, 3, 3> r;


void callbackConfig(const iiwa_msgs::JointPosition::ConstPtr& msg)
{//ROS_INFO_STREAM("1");

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
    //ROS_INFO_STREAM("2");
}

void setConfig(VectorXd q)
{      //ROS_INFO_STREAM("3");

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

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
      //ROS_INFO_STREAM("4");
   px=0;
   py=0;
   pz=0;
    Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    
    r = q.toRotationMatrix();
     px = msg->position.x;
    py = msg->position.y;
    pz = msg->position.z;
    poseGetting =true;


/*
    Eigen::Matrix<double, 3,3> rx90;
		rx90<<1,0,0,
          0,0,-1,
          0,1,0;
    
    Eigen::Matrix<double, 3, 3>r2;
    r2 = r*rx90;*/
       Eigen::Matrix<double, 4, 4> Tca;

    Tca << r(0, 0), r(0, 1), r(0, 2), px,
       r(1, 0), r(1, 1), r(1, 2), py,
       r(2, 0), r(2, 1), r(2, 2), pz,
       0, 0, 0, 1;

    Eigen::Matrix<double, 4, 4> Tgc;
    Tgc << 0.7071068, -0.7071068, 0, -0.055,
            0.7071068, 0.7071068, 0, -0.055,
             0, 0, 1, 0.021,
             0, 0, 0, 1;
std::cout << "Tca: " << std::endl << Tca << std::endl<< std::endl;
    Eigen::Matrix<double, 4, 4> Tag;
    Tag<< 1,0,0,0,
           0,1,0,0,
           0,0,1,-0.1915,
           0,0,0,1;
    
    
    if(px!=0 || py!=0 || pz!=0)
    Tgg = Tgc* Tca* Tag ;
    else
    Tgg = Tca;

  //rotational offset detected
  Eigen::Matrix<double, 3, 3> rTgg;
  rTgg <<Tgg(0, 0), Tgg(0, 1), Tgg(0, 2), 
        Tgg(1, 0), Tgg(1, 1), Tgg(1, 2), 
        Tgg(2, 0), Tgg(2, 1), Tgg(2, 2);
  angles = rTgg.eulerAngles(2,1,0); 


  std::cout << "rotation: "<< angles(2) <<", "<< angles(1)<< "," << angles(0)<< std::endl<<std::endl;
  
	std::cout << "Tgg: " << std::endl << Tgg << std::endl<< std::endl;	
	std::cout<< "----------------------------------"<<std::endl<<std::endl;
      //ROS_INFO_STREAM("4.1");

  VectorXd my_q = getConfig();
   last_tool_position = global_iiwa.fk(my_q).htmTool; //Get currect robot pose
   aruco_pose = Utils::trn(Tgg(0,3), Tgg(1,3), Tgg(2,3))*Utils::rotx(angles(0))*Utils::roty(angles(1))*Utils::rotz(angles(2));

     // ROS_INFO_STREAM("4.2");
}

   
   

int main(int argc, char** argv) {
    ros::init(argc, argv, "iiwa_control_generic");
    ros::NodeHandle n;
 
    //Simulation mode or real robot mode
    global_SimulationMode = false;
    //ROS_INFO_STREAM("1b");
    //Initialize subscriber to measure the joint position
    ros::Subscriber qSub = n.subscribe("/iiwa/state/JointPosition", 100, callbackConfig);
    for(uint8_t i=0;i<10;i++){
      ros::spinOnce(); 
    ros::Duration(0.1).sleep();
    }
    //Initialize subscriber to publish the joint position

    ros::Publisher jointPub = n.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 100);
    for(uint8_t i=0;i<10;i++){
      ros::spinOnce(); 
    ros::Duration(0.1).sleep();
    }
    global_jointPub = &jointPub;
  ros::Subscriber pose = n.subscribe("aruco_detection", 100, poseCallback);

    VectorXd q0 = global_iiwa.q;
    Matrix4d HTM0 = global_iiwa.fk(q0).htmTool;
    Eigen::Matrix4d taskHTM1;
    

    double time = 0;
    ros::Rate loop_rate(1.0 / T);
    ofstream simFile;
    //simFile.open ("/home/cair1/PycharmProjects/uaibot_vinicius/test/mexp/data.txt");

    // simFile << "tgPose.add_ani_frame(0,np.matrix(" << Utils::printMatrixPython(taskHTM)<<"))"<<std::endl;
    


     while (ros::ok() && continueLoop1)
     {
       ros::spinOnce();    
    
       //ROS_INFO_STREAM("1e");

      //taskHTM1= HTM0*Utils::trn(0,0,0);
      taskHTM1 = HTM0 *Utils::trn(0.55,0,-0.98) * Utils::roty(M_PI);
                   //ROS_INFO_STREAM("1f");
       ros::spinOnce();    

       ConstControlResult ccr1 = global_iiwa.constControl(getConfig(), taskHTM1);
        
    //   //Compute next configuration and send it to the robot by a simple numerical integration 
    //   //of the velocity. Here we user 1st order Euler explicit with timestep "T"
       VectorXd qdot = ccr1.action;       
  
       VectorXd qnext = getConfig() + qdot*T;    

       setConfig(qnext);  

    //   //Update time (necessary only for simulation)
      time+=T;

    //   //Print some information about the task
       TaskResult tr = ccr1.taskResult;
       ROS_INFO_STREAM("-------------------------");
       ROS_INFO_STREAM("Position error (m): "<<tr.maxErrorPos);
       ROS_INFO_STREAM("Orientation error (degrees): "<<tr.maxErrorOri);

       //Continue the controler until the maximum position error is below 1 cm and
       //the orientation error is below 2 degrees
       continueLoop1 = tr.maxErrorPos>0.01 || tr.maxErrorOri > 2.0; 

        
    //   //Write the configuration to a file with the timestamp
    //   //Utils::sendToSimulation(&simFile, time, getConfig());
     }
    
    ros::Duration(2).sleep();
        
        Eigen::Matrix<double, 4, 4> tTgc;         
            tTgc << -0.7071068, 0.7071068, 0, -0.055,
            -0.7071068, -0.7071068, 0, -0.055,
             0, 0, 1, 0.021,
             0, 0, 0, 1;

        Eigen::Matrix<double, 4, 4> tTca;
            tTca << 0.721334, -0.692236, -0.0220669, 0.0119186,
                    0.691126, 0.721515, -0.0419512, 0.00341485,
                     0.0449617, 0.0150098, 0.998876, -0.121741,
                     0, 0, 0, 1;

        Eigen::Matrix<double, 4, 4> tTag;
            tTag << 1,0,0,0,
                    0,1,0,0,
                    0,0,1,-0.1915,
                    0,0,0,1;

        Eigen::Matrix<double,4,4> tTgg = tTgc* tTca* tTag;
            

            //rotational offset detected
            Eigen::Matrix<double, 3, 3> trTgg;
            trTgg <<tTgg(0, 0), tTgg(0, 1), tTgg(0, 2), 
                  tTgg(1, 0), tTgg(1, 1), tTgg(1, 2), 
                  tTgg(2, 0), tTgg(2, 1), tTgg(2, 2);
            Eigen::Vector3d tangles = trTgg.eulerAngles(2,1,0);
           // tangles(0)=fmod(tangles(0)+M_PI,M_PI)-M_PI/2;
           // tangles(1)=fmod(tangles(1)+M_PI,M_PI);
           // tangles(2)=fmod(tangles(2)+M_PI,M_PI);

      //std::cout << "rotation: "<< tangles(0) <<", "<< tangles(1) << "," << tangles(2)<< std::endl<<std::endl;
      //std::cout << "Tca: " << std::endl << tTca << std::endl<< std::endl;
      //std::cout << "Tgg: " << std::endl << tTgg << std::endl<< std::endl;	
		  //std::cout<< "----------------------------------"<<std::endl<<std::endl;
       
      //taskHTM2 =  taskHTM1 *Utils::trn(Tgg(0,3), Tgg(1,3), Tgg(2,3))*Utils::roty(0)*Utils::rotx(0)*Utils::rotz(angles(2));
       //taskHTM_Next=  last_tool_position *Utils::trn(tTgg(0,3), tTgg(1,3), tTgg(2,3))*Utils::roty(tangles(1))*Utils::rotx(tangles(0))*Utils::rotz(tangles(2));
    

    while(ros::ok()){

    
ros::spinOnce();
     ros::Duration(0.01).sleep(); 
                  Matrix4d taskHTM_Next;

    if (poseGetting){

         taskHTM_Next= last_tool_position * aruco_pose;

    }ConstControlResult ccr2 = global_iiwa.constControl(getConfig(), taskHTM_Next);
      //Compute next configuration and send it to the robot by a simple numerical integration 
      //of the velocity. Here we user 1st order Euler explicit with timestep "T"
      VectorXd qdot = ccr2.action;    
      VectorXd qnext = getConfig() + qdot*T;
      setConfig(qnext);  
      //ROS_INFO_STREAM("1k");

      //Print some information about the task
      TaskResult tr2 = ccr2.taskResult;

     // if(continueLoop2){
     //Matrix4d taskHTM_Next = global_iiwa.fk(getConfig()).htmTool * Utils::trn(Tgg(0,3), Tgg(1,3), Tgg(2,3))*Utils::roty(0)*Utils::rotx(0)*Utils::rotz(angles(2));
     
      //std::cout << "taskHTM1: " << std::endl << std::endl << taskHTM1<< std::endl << std::endl;
      //std::cout << "taskHTM2: " << std::endl << std::endl << taskHTM2<< std::endl << std::endl;
      //taskHTM = HTM0 *Utils::trn(Tag(0,3), Tag(1,3), Tag(2,3))*Utils::roty(angles(1))*Utils::rotx(angles(0))*Utils::rotz(angles(2));
  /*ConstControlResult ccr2 = global_iiwa.constControl(getConfig(), taskHTM_Next);

      //Compute next configuration and send it to the robot by a simple numerical integration 
      //of the velocity. Here we user 1st order Euler explicit with timestep "T"
      VectorXd qdot = ccr2.action;    
      VectorXd qnext = getConfig() + qdot*T;
      setConfig(qnext);  
      //ROS_INFO_STREAM("1k");

      //Update time (necessary only for simulation)
      time+=T;

      //Print some information about the task
      TaskResult tr2 = ccr2.taskResult;
      */
     
      ROS_INFO_STREAM("-------------------------");
      ROS_INFO_STREAM("Position error (m): "<<tr2.maxErrorPos);
      ROS_INFO_STREAM("Orientation error (degrees): "<<tr2.maxErrorOri); 
      //ROS_INFO_STREAM(qdot.transpose());    
      //ROS_INFO_STREAM(getConfig().transpose());
      
      
      //if (tr2.maxErrorPos>0.01 || tr2.maxErrorOri > 2.0){
     // poseGetting}

     // if(!continueLoop2)
        //ROS_INFO_STREAM("Pose converged!");
      }

      loop_rate.sleep();
}
