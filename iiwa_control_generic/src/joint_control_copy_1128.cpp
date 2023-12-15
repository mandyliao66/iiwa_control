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
Manipulator global_iiwa = Manipulator::createKukaIIWA();
Eigen::Matrix<double, 4, 4>Tgg;
Eigen::Matrix<double, 3, 3> r;
Eigen::Vector3d angles;
double px=0;

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
    
    Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    r = q.toRotationMatrix();
     px = msg->position.x;
    double py = msg->position.y;
    double pz = msg->position.z;

    Eigen::Matrix<double, 3,3> rx90;
		rx90<<1,0,0,
          0,0,-1,
          0,1,0;
    
    Eigen::Matrix<double, 3, 3>r2;
    r2 = r;
    //*rx90;
   
    Eigen::Matrix<double, 4, 4> Tca;
    Tca << r2(0, 0), r2(0, 1), r2(0, 2), px,
       r2(1, 0), r2(1, 1), r2(1, 2), py,
       r2(2, 0), r2(2, 1), r2(2, 2), pz,
       0, 0, 0, 1;

  
       
       
    //receivedTca = true;
    //std::cout << "Tca: " << std::endl << std::endl << Tca << std::endl << std::endl;

    Eigen::Matrix<double, 4, 4> Tgc;

    Tgc << -0.7071068, 0.7071068, 0, -0.055,
            -0.7071068, -0.7071068, 0, -0.055,
             0, 0, 1, 0.021,
             0, 0, 0, 1;

    Eigen::Matrix<double, 4, 4> Tag;
    Tag<< 1,0,0,0,
           0,1,0,0,
           0,0,1,-0.1915,
           0,0,0,1;
if(px!=0){
    Tgg = Tgc* Tca* Tag ;

    
    /*cout << "rotation" << endl << "x: " << Tag(0, 0) << ", " << Tag(0, 1) << ", " << Tag(0, 2) << endl
            << "y: " << Tag(1, 0) << ", " << Tag(1, 1) << ", " << Tag(1, 2) << endl
            << "z: " << Tag(2, 0) << ", " << Tag(2, 1) << ", " << Tag(2, 2) << endl;*/
      //cout << "translation" << endl << "x " << Tgg(0,3) << endl << "y " << Tgg(1,3) << endl << "z " << Tgg(2,3) << endl;
      

  //rotational offset detected
  Eigen::Matrix<double, 3, 3> rTgg;
  rTgg <<Tgg(0, 0), Tgg(0, 1), Tgg(0, 2), 
        Tgg(1, 0), Tgg(1, 1), Tgg(1, 2), 
        Tgg(2, 0), Tgg(2, 1), Tgg(2, 2);
  angles = rTgg.eulerAngles(2,1,0); 
}
  cout << "rotation: "<< angles(0) <<", "<< angles(1) << ", " << angles(2)<< endl;    
  
  cout << "-----------------------------------------------------------------"<< endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "iiwa_control_generic");
    ros::NodeHandle n;
    ROS_INFO_STREAM("1a");


    //Simulation mode or real robot mode
    global_SimulationMode = false;
    //ROS_INFO_STREAM("1b");

    //Initialize subscriber to measure the joint position
    ros::Subscriber qSub = n.subscribe("/iiwa/state/JointPosition", 100, callbackConfig);
    //Initialize subscriber to publish the joint position
    ros::Subscriber pose = n.subscribe("aruco_detection", 100, poseCallback);

    ros::Publisher jointPub = n.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 100);
    for(uint8_t i=0;i<10;i++){
      ros::spinOnce(); 
    ros::Duration(0.1).sleep();
    }
    global_jointPub = &jointPub;


    //ROS_INFO_STREAM("1c");


    VectorXd q0 = global_iiwa.q;
    Matrix4d HTM0 = global_iiwa.fk(q0).htmTool;
    Eigen::Matrix4d taskHTM1;
    
    
    //ROS_INFO_STREAM("1d");

    double T = 0.1;
    double time = 0;
    ros::Rate loop_rate(1.0 / T);
    ofstream simFile;
    //simFile.open ("/home/cair1/PycharmProjects/uaibot_vinicius/test/mexp/data.txt");

    // simFile << "tgPose.add_ani_frame(0,np.matrix(" << Utils::printMatrixPython(taskHTM)<<"))"<<std::endl;
    

    bool continueLoop1 = true;    
    bool continueLoop2 = true;


     while (ros::ok() && continueLoop1)
     {
       ros::spinOnce();    
    
       //ROS_INFO_STREAM("1e");

    //taskHTM1= HTM0*Utils::trn(0,0,0);
     taskHTM1 = HTM0 *Utils::trn(0.3,-0.2,-0.7) * Utils::roty(M_PI);
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

      
    //   //continueLoop=true;
  
    //   //Write the configuration to a file with the timestamp
    //   //Utils::sendToSimulation(&simFile, time, getConfig());
     }
    
    ros::Duration(2).sleep();

    VectorXd qs = getConfig();

    
    while(ros::ok() && continueLoop2){
        
        ros::spinOnce();
        ros::Duration(0.01).sleep();   

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
            Eigen::Vector3d tangles_norm;
            tangles_norm(0)=fmod(tangles(0)+M_PI,M_PI)-M_PI/2;
            tangles_norm(1)=fmod(tangles(1)+M_PI,M_PI);
            tangles_norm(2)=fmod(tangles(2)+M_PI,M_PI);

      std::cout << "rotation: "<< tangles_norm(0) <<", "<< tangles_norm(1) << "," << tangles_norm(2)<< std::endl<<std::endl;

std::cout << "Tca: " << std::endl << tTca << std::endl<< std::endl;

	std::cout << "Tgg: " << std::endl << tTgg << std::endl<< std::endl;	
		std::cout<< "----------------------------------"<<std::endl<<std::endl;
       
      //taskHTM2 =  taskHTM1 *Utils::trn(Tgg(0,3), Tgg(1,3), Tgg(2,3))*Utils::roty(0)*Utils::rotx(0)*Utils::rotz(angles(2));

      //////////////////////NE///////////////////////////////////////////////////
      
     VectorXd my_q =getConfig(); //Get currect joint angles
     Matrix4d last_tool_position = global_iiwa.fk(my_q).htmTool; //Get currect robot pose
     Matrix4d aruco_pose = Utils::trn(Tgg(0,3), Tgg(1,3), Tgg(2,3))*Utils::rotz(tangles(2));
     //Utils::roty(0)*Utils::rotx(0)
     //
    //aruco_pose = Utils::trn(0,0,-0.10);
     //
    Matrix4d taskHTM_Next;
          
    taskHTM_Next=  last_tool_position *Utils::trn(tTgg(0,3), tTgg(1,3), tTgg(2,3))*Utils::roty(tangles_norm(1))*Utils::rotx(tangles_norm(0))*Utils::rotz(tangles_norm(2));
    
    //taskHTM_Next = last_tool_position * aruco_pose;

    // ROS_INFO_STREAM(last_tool_position);
    //ROS_INFO_STREAM("AAAA");
    //ROS_INFO_STREAM(my_q);
    //ROS_INFO_STREAM(aruco_pose);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
     //Matrix4d taskHTM_Next = global_iiwa.fk(getConfig()).htmTool * Utils::trn(Tgg(0,3), Tgg(1,3), Tgg(2,3))*Utils::roty(0)*Utils::rotx(0)*Utils::rotz(angles(2));
     
      //std::cout << "taskHTM1: " << std::endl << std::endl << taskHTM1<< std::endl << std::endl;
      //std::cout << "taskHTM2: " << std::endl << std::endl << taskHTM2<< std::endl << std::endl;
      //taskHTM = HTM0 *Utils::trn(Tag(0,3), Tag(1,3), Tag(2,3))*Utils::roty(angles(1))*Utils::rotx(angles(0))*Utils::rotz(angles(2));
  
      ConstControlResult ccr2 = global_iiwa.constControl(getConfig(), taskHTM_Next);
      //ConstControlResult ccr2 = global_iiwa.constControl(qs, taskHTM_Next);
      
      ROS_INFO_STREAM("1h");

      //Compute next configuration and send it to the robot by a simple numerical integration 
      //of the velocity. Here we user 1st order Euler explicit with timestep "T"
      VectorXd qdot = ccr2.action;    


      //ROS_INFO_STREAM("1i");
  
      VectorXd qnext = getConfig() + qdot*T;
      //qs = qs + qdot*T;    

      //ROS_INFO_STREAM("1j");

      setConfig(qnext);  
      //ROS_INFO_STREAM("1k");

      //Update time (necessary only for simulation)
      time+=T;

      //Print some information about the task
      TaskResult tr2 = ccr2.taskResult;
      
     
      ROS_INFO_STREAM("-------------------------");
      ROS_INFO_STREAM("Position error (m): "<<tr2.maxErrorPos);
      ROS_INFO_STREAM("Orientation error (degrees): "<<tr2.maxErrorOri); 
      ROS_INFO_STREAM(qdot.transpose());    
      ROS_INFO_STREAM(getConfig().transpose());
      }
      continueLoop2=true;


      if(!continueLoop1)
        ROS_INFO_STREAM("Pose converged!");


      loop_rate.sleep();
  }


