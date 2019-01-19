#include <ros/ros.h>

#include <Eigen/Dense>
#include <math.h>
#include <time.h>

#include "nearlab_msgs/energy_optimal_traj.h"
#include "nearlab_msgs/attitude_traj.h"
#include "quatMath.h"

Eigen::Vector3d r;
Eigen::Vector3d v;
Eigen::Vector4d q;
Eigen::Vector3d w;
bool updated;

void stateCallback(const nearlab_msgs::StateStamped msg){
  for(int i=0;i<3;i++){
    r(i) = msg.r[i];
    v(i) = msg.v[i];
    q(i) = msg.q[i];
    w(i) = msg.w[i];
  }
  q(3) = msg.q(3);
  updated = true;
}

nearlab_msgs::energy_optimal_traj setupTrajRequest(const NodeHandle& nh){
  double dt,m,tau,nu,F,mu,r[3],v[3],R[3];
  
  nearlab_msgs::energy_optimal_traj srv;
  // Get parameters
  nh.getParam("dt",dt);
  nh.getParam("final_time",srv.request.tEnd);
  nh.getParam("sc_mass", srv.request.sc_mass);
  nh.getParam("time_const",srv.request.time_const);
  nh.getParam("dist_const",srv.request.dist_const);
  nh.getParam("sc_thrust", srv.request.sc_thrust);
  nh.getParam("grav_param",srv.request.grav_param);
  nh.getParam("final_radius_x",srv.request.rEnd[0]);
  nh.getParam("final_radius_y",srv.request.rEnd[1]);
  nh.getParam("final_radius_z",srv.request.rEnd[2]);
  nh.getParam("final_velocity_x",srv.request.vEnd[0]);
  nh.getParam("final_velocity_y",srv.request.vEnd[1]);
  nh.getParam("final_velocity_z",srv.request.vEnd[2]);
  nh.getParam("orbital_radius_x",srv.request.rOrb[0]);
  nh.getParam("orbital_radius_x",srv.request.rOrb[1]);
  nh.getParam("orbital_radius_x",srv.request.rOrb[2]);

  srv.request.intervals = (int)round(srv.request.tEnd/dt)+1;
  srv.request.tStart = 0;

  return srv;
}

nearlab_msgs::attitude_traj setupAttrRequest(const NodeHandle& nh){
  double dt,tf,q[4];
  nearlab_msgs::attitude_traj srv;
  // Get parameters
  nh.getParam("dt",dt);
  nh.getParam("final_time",srv.request.tEnd);
  nh.getParam("final_quat_x",srv.request.qEnd[0]);
  nh.getParam("final_quat_y",srv.request.qEnd[1]);
  nh.getParam("final_quat_z",srv.request.qEnd[2]);
  nh.getParam("final_quat_w",srv.request.qEnd[3]);
  
  srv.request.intervals = (int)round(srv.request.tEnd/dt)+1;
  srv.request.tStart = 0;

  for(int i=0;i<4;i++){
     = q[i];
  }

  return srv;
}

int main(int argc, char** argv){
  ros::init(argc,argv,"sc_controller");
  ros::NodeHandle nh;
  updated = false;
  

  // Subscribers
  subState = nh.subscribe("/orbot/space/state/estimate",2,stateCallback);

  // Publishers
  pubControl = nh.advertise<geometry_msgs::Vector3Stamped>("/orbot/space/control"),1000);
  
  // setup trajectory clients
  ros::ServiceClient traj_client = nh.serviceClient<nearlab_msgs::energy_optimal_traj>("/orbot/space/energy_optimal_traj");
  ros::ServiceClient att_client = nh.serviceClient<nearlab_msgs::attitude_traj>("/orbot/space/attitude_traj");
  nearlab_msgs::energy_optimal_traj traj_srv = setupTrajRequest(nh);
  nearlab_msgs::attitude_traj att_srv = setupAttRequest(nh);
  Eigen::MatrixXd rStar, vStar, qStar, tStar;

  // Loop
  ROS_INFO("Controller Listening for Initial Estimate");
  ros::Rate loop_rate(100);
  bool initialized = false;
  ros::Time tStart;

  while(ros::ok()){
    while(!intialized){
      ros::spinOnce();
      
      if(updated){
        // Get initial state
        ROS_INFO("Got Initial State");
        
        for(int i=0;i<3;i++){
          traj_srv.request.rStart[i] = r[i];
          traj_srv.request.vStart[i] = v[i];
          att_srv.request.qStart[i] = q[i];
        }
        att_srv.request.qStart[3] = q[3];
        
        // Generate trajectories
        ROS_INFO("Generating Trajectories");
        if(traj_client.call(traj_srv)){
          ROS_INFO("Successfully generated optimal trajectory to target location.");
          rStar = Eigen::MatrixXd::Zero(3,traj_srv.request.intervals);
          vStar = Eigen::MatrixXd::Zero(3,traj_srv.request.intervals);
          tStar = Eigen::VectorXd::Zero(traj_srv.request.intervals);
          for(int i=0;i<intervals;i++){
            rStar(0,i) = traj_srv.response.rx[i];
            rStar(1,i) = traj_srv.response.ry[i];
            rStar(2,i) = traj_srv.response.rz[i];
            vStar(0,i) = traj_srv.response.rx[i];
            vStar(1,i) = traj_srv.response.vy[i];
            vStar(2,i) = traj_srv.response.vz[i];
            tStar(i) = traj_srv.response.times[i];
          }
        }else{
          ROS_INFO("Error in generating optimal trajectory to target location.");
          continue;
        }
        if(att_client.call(att_srv)){
          ROS_INFO("Successfully generated trajectory to target attitude.");
          vStar = Eigen::MatrixXd::Zero(3,traj_srv.request.intervals);
          tStar = Eigen::VectorXd::Zero(traj_srv.request.intervals);
          for(int i=0;i<intervals;i++){
            qStar(0,i) = traj_srv.response.qx[i];
            qStar(1,i) = traj_srv.response.qy[i];
            qStar(2,i) = traj_srv.response.qz[i];
            qStar(3,i) = traj_srv.response.qw[i];
          }
        }else{
          ROS_INFO("Error in generating trajectory to target attitude.");
          continue;
        }
        tStart = ros::Time::now();
        initialized = true;
        updated = false;
      }
    }
    // With current state/time, figure out goal pose
    double tCurr = (ros::Time::now()-tStart).toSec();


    // Use PID controller to get attitude control output
    // Use PID controller to get linear control output
    // Combine the two
    // Publish them



    ros::spinOnce();
    loop_rate.sleep();
  }



  
}
