#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <deque>

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>



#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include <carla/client/ActorList.h>
#include <carla/geom/Vector3D.h>
#include <carla/client/Vehicle.h>
#include <carla/rpc/VehicleAckermannControl.h>



#include "obstacle.h"
#include "reference_point.h"
#include "planning/host_state.pb.h"
#include "em_planner.h"
#include "trajectory_history.h"
#include "mpc_controller.h"
#include "planning_util.h"

#include "ReferenceLineReader.hpp"
#include "planning_util.cpp"



using namespace carla::client;
using AckermannControl = carla::rpc::VehicleAckermannControl;

std::ofstream fsss2("host_state.txt");
std::ofstream fsss_raw_control("raw_control_input.txt");
std::ofstream fsss_smooth_control("smooth_control.txt");
std::ofstream fss_ds_accel("desired_accel.txt");


// Create an object of ReferenceLineReader class
ReferenceLineReader ref_reader;  

// 
// struct ControlCommand{
//   double speed;
//   double acceleration;
//   double steer;
// }control_command;

// some global variable 
  // em_planner
caros::planning::DpPathConfig dp_path_config;
caros::planning::QpPathConfig qp_path_config;
caros::planning::DpSpeedConfig dp_speed_config;
caros::planning::QpSpeedConfig qp_speed_config;

// prepare inputs
caros::planning::HostState host_state;
std::vector<caros::common::Obstacle> obstacles;

// planning result and mutex
std::mutex planning_mutex;
caros::planning::Trajectory planning_result;

// getting the planning result 
caros::planning::Trajectory GetPlanningResult(const carla::SharedPtr<carla::client::Vehicle> &vehicle,
                                              caros::planning::Trajectory& pre_traj){
    
    int max_count = 3200;
    int count = 0;
    double dt = 0.01;
    while(count < max_count){
      double relative_time = count * dt;
      if(count % 100 == 0 ){
        // do the planning here
        //std::ofstream file("planning_time.txt", std::ios_base::app);
        static bool first_execute = true;
        if(first_execute){
          GetHostStateInfo(boost::static_pointer_cast<Vehicle>(vehicle),host_state,first_execute);
          first_execute = false;
        }else{
          GetHostStateInfo(boost::static_pointer_cast<Vehicle>(vehicle),host_state,false);
        }
        
        auto reference_line = ref_reader.GetReferenceLine(host_state.x(),host_state.y());
        pre_traj.Clear();
        pre_traj = caros::planning::TrajectoryHistory::GetLastTrajectory();
        obstacles.clear();

        auto pre_size = pre_traj.NumOfPoints();
        caros::planning::EMPlanner em_planner(dp_path_config,qp_path_config,dp_speed_config,qp_speed_config);
        auto start_time = std::chrono::high_resolution_clock::now();
        em_planner.OnPlan(host_state,reference_line,obstacles,pre_traj,relative_time);

        auto size = reference_line.size();
        auto pre_traj_size = pre_traj.NumOfPoints();
        auto end_time = std::chrono::high_resolution_clock::now(); 
        std::chrono::duration<double> diff = end_time - start_time;
        std::cout << "Planning took " << diff.count() << " seconds" << std::endl; 

        // lock starts here after planning,we dont planning to block Controlling
        std::unique_lock<std::mutex> scope_locker(planning_mutex);
        planning_result =  em_planner.GetPlanningResult();  // here got the planning result
        // file << "Planning took " << diff.count() << " seconds\n" << "reference line size: " << size << std::endl << "pre_traj size: " << pre_traj_size << std::endl
        //       << "size of trajectory: " << planning_result.GetTrajectory().size() << std::endl;

        // SaveFinalTraj(planning_result);
        // file.close();
       
      }
      else{
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
}


/**
 * func name:ExecuteVehicleControl
 * 
 * Desc: Control the ego vehicle with plnning result
 * this function use the planning result, and feed it to the mpc controller,
 * and then mpc controller will culculate and output a Control Command(speed,acceleration,steer)
 * and this command will apply to the ego vehicle within this function
 * 
   needed parameter:
                      vehicle
                      planning result
                      relative time
                      mpc controller       
**/ 
void ExecuteVehicleControl(const carla::SharedPtr<carla::client::Vehicle> &vehicle,
                          caros::control::MpcController mpc_controller){
  int max_count = 3200;
  int count     = 0;
  double dt     = 0.01;

  while(count < max_count){
    double relative_time = count * dt;
    static bool velocity_init = true;
    AckermannControl ackermann_control;
    Eigen::VectorXd raw_control_input;
    Eigen::VectorXd smoothed_control_input;
    auto start = std::chrono::high_resolution_clock::now();
    // planning_result!!!!   and locking
    {
      std::unique_lock<std::mutex> scope_locker(planning_mutex);
      GetHostStateInfo(boost::static_pointer_cast<Vehicle>(vehicle),host_state,false);
      mpc_controller.Control(planning_result,host_state,relative_time,raw_control_input,smoothed_control_input);
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "control step takes time: " << duration.count() << " ms" << std::endl;
    caros::common::TrajectoryPoint control_point = mpc_controller.GetControlPoint();
    double a_ref = control_point.a();
    double v_ref = control_point.v();
    double front_wheel_angle = smoothed_control_input[0];
    double desired_accel = smoothed_control_input[1];
    // setting ackermann control
    ackermann_control.acceleration = desired_accel;
    ackermann_control.speed        = v_ref;
    double steering_command = ToCarlaSteerControl(front_wheel_angle); // 
    ackermann_control.steer        = steering_command / 2; 
    // threshold for steer
    if(std::abs(ackermann_control.steer) < 0.01) {
      ackermann_control.steer =0;
    } 
    
    // apply control to vehicle in carla
    boost::static_pointer_cast<Vehicle>(vehicle)->ApplyAckermannControl(ackermann_control);
    // this while loop is to elimate this 0.5 second delay happened when first command the ego vehilce  
    while(velocity_init){
      GetHostStateInfo(boost::static_pointer_cast<Vehicle>(vehicle),host_state,false);
      if(host_state.speed() > 1.01) {
        velocity_init = false;
      }
    }
    std::cout << "vref: " << v_ref << " aref: " << a_ref << std::endl;
    auto error = mpc_controller.GetError(); 
    std::cout << "error: " << std::endl;
    std::cout << error.transpose() << std::endl;
    std::cout << "delta_f: " << front_wheel_angle << std::endl;

    // showing acceleration information
    std::cout << "disired acceleration: " << desired_accel << std::endl;
    std::cout << "actual  acceleration: " << host_state.accel() << std::endl  << std::endl;
    std::cout << host_state.DebugString() << std::endl;
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  } 
  // write some value information for debug and draw plot
  // fss_ds_accel << desired_accel << std::endl; // writing desired acceleration to the file 
  // fsss2 << host_state.x() << "," << host_state.y() <<"," <<host_state.speed() <<"," << host_state.accel()<< "," << host_state.heading() << "," << host_state.yawrate() << std::endl;
  // fsss_raw_control << raw_control_input[0]    <<     "," << raw_control_input[1] << std::endl;
  // fsss_smooth_control <<  smoothed_control_input[0] <<  "," <<  smoothed_control_input[1] << std::endl;
}


int main(){
 // carla setup
  auto client = Client("localhost",2000);
  
  auto world = client.GetWorld();
  auto vehicle = SearchVehicle(world);
  auto settings = world.GetSettings();
  settings.substepping = true;
  settings.max_substep_delta_time = 0.001;
  settings.max_substeps = 100;
  settings.fixed_delta_seconds = 0.01;
  carla::time_duration timeout = std::chrono::seconds(1);
  world.ApplySettings(settings, timeout); 
  
  
  // prepare for mpc controller
  caros::control::VehicleConfig vehicle_config = SetVehicleConfig();
  caros::control::MpcConfig mpc_config =SetMpcConfig();

  caros::control::MpcController mpc_controller(vehicle_config,mpc_config);
  Eigen::MatrixXd  u_lower;
  Eigen::MatrixXd  u_upper;
  Eigen::MatrixXd  x_lower;
  Eigen::MatrixXd  x_upper;
  Eigen::MatrixXd   du_min;
  Eigen::MatrixXd   du_max;
  SetMpcBoundary(u_lower,u_upper,x_lower,x_upper,du_min,du_max);
  mpc_controller.Init(u_lower,    u_upper,    
                      x_lower,    x_upper,
                      du_min,    du_max);


  // reference line
  std::string filename = "../reference_line.txt";
     // Open the reference file
  if (!ref_reader.OpenFile("../reference_line.txt")) {
      return -1;
      std::cout << "fail to open reference file,terminating the program...." << std::endl;
  }


  // // get the obstacle
  obstacles = GetObstacleInfo(world,boost::static_pointer_cast<Vehicle>(vehicle),80);
  ShowObstacleInfo(obstacles);
  std::ofstream fsss("corners.txt");
    for(auto obstacle : obstacles){
          std::vector<caros::common::math::Vec2d> all_corner = obstacle.PerceptionBoudingBox().GetAllCorners();
          for(int idx = 0; idx < all_corner.size(); idx++){
          fsss << std::to_string(all_corner[idx].x()-host_state.x()) << ","<< std::to_string(all_corner[idx].y()-host_state.y())<< "\n";
        }
    }
  fsss.close();
  //int times = 0;
    carla::geom::Vector3D target_velocity(0,1,0);
    vehicle->SetTargetVelocity(target_velocity);
    caros::planning::Trajectory planning_result;
     caros::planning::Trajectory pre_traj = caros::planning::TrajectoryHistory::GetLastTrajectory();

    // planning thread
    std::thread process_planning(GetPlanningResult,boost::static_pointer_cast<Vehicle>(vehicle),std::ref(pre_traj));
    // vehicle control thread
    std::thread process_vehicle_control( ExecuteVehicleControl,boost::static_pointer_cast<Vehicle>(vehicle),std::ref(mpc_controller));

    if(process_planning.joinable())
      process_planning.join();

    if( process_vehicle_control.joinable())
      process_vehicle_control.join();

    // so far this one loop for planning and control
    std::cout << "Iam outside the while ..........." << std::endl;
    ref_reader.CloseFile();  // Close the reference file
    return 0;
}