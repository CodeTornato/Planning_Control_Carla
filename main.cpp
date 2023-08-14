#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <deque>

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


using namespace carla::client;
using AckermannControl = carla::rpc::VehicleAckermannControl;


double Degree2Rad(double degree) {
  return degree/180 * M_PI;
}
caros::control::VehicleConfig SetVehicleConfig() {
    caros::control::VehicleConfig vehicle_config;
    vehicle_config.mass = 1845;
    vehicle_config.cornering_stiffness = 110000;
    vehicle_config.mass_center_front_wheel =1.05;
    vehicle_config.wheelbase = 3;
    vehicle_config.yaw_inertia = 1536.7;
    return vehicle_config;

}
caros::control::MpcConfig SetMpcConfig() {
    caros::control::MpcConfig mpc_config;
    mpc_config.eps_abs = 0.01;
    mpc_config.max_iteration = 1500;
    mpc_config.nx = 6;
    mpc_config.nu = 2;
    mpc_config.pred_horizon =10;
    return mpc_config;

}

void SetMpcBoundary(Eigen::MatrixXd&  u_lower,    Eigen::MatrixXd&  u_upper,    
                    Eigen::MatrixXd&  x_lower,    Eigen::MatrixXd&  x_upper,
                    Eigen::MatrixXd&   du_min,    Eigen::MatrixXd&   du_max) {
    // 控制变量的下限
    double max_deceleration = -8;
    Eigen::MatrixXd lower_bound(2, 1);
    lower_bound << -M_PI/3, max_deceleration;
    u_lower = lower_bound;

    // 控制变量的上限
    double max_acceleration = 8;
    Eigen::MatrixXd upper_bound(2, 1);
    upper_bound << M_PI/3, max_acceleration;
    u_upper = upper_bound;

    // 状态变量的上下限
    const double max = std::numeric_limits<double>::max();
    Eigen::MatrixXd lower_state_bound(6, 1);
    Eigen::MatrixXd upper_state_bound(6, 1);

    // lateral_error, lateral_error_rate, heading_error, heading_error_rate
    // station_error, station_error_rate
    lower_state_bound << -1.0 * max, -1.0 * max, -1.0 * M_PI, -1.0 * max,
        -1.0 * max, -1.0 * max;
    upper_state_bound << max, max, M_PI, max, max, max;
    x_lower = lower_state_bound;
    x_upper = upper_state_bound;
    //control input rate of change boundary
    Eigen::MatrixXd delta_u_min(2,1);
    Eigen::MatrixXd delta_u_max(2,1);
    delta_u_min << -0.05, -0.5;
    delta_u_max <<  0.05,  0.5;
    du_min  = delta_u_min;
    du_max  = delta_u_max;

}


auto SearchVehicle(carla::client::World world){
    auto actorList = world.GetActors();
    std::cout << "ActorList is Empty? " << actorList->empty() << std::endl;
    int actors_size = actorList->size();
    for(int index = 0;index < actors_size; index++){
      std::cout <<"ID is: " << (int)actorList->at(index)->GetId() << std::endl;
      auto attributeList = actorList->at(index)->GetAttributes();
      for(auto & attribute : attributeList ){
        if(attribute.GetValue() == "hero"){
          std::cout << "Found the Car!"; 
          std::cout << "Actor id of this car is: " << attribute.GetId() << std::endl;
          return actorList->at(index);
        }
 //  std::cout << "Actor Attribute Value is: " << attribute.GetValue() << std::endl;
    }
   }  
  auto actor = world.GetActor(42);
  return actor;
}


auto SearchObstacle(carla::client::World world){
    auto actorList = world.GetActors();
    std::cout << "ActorList is Empty? " << actorList->empty() << std::endl;
    int actors_size = actorList->size();
    for(int index = 0;index < actors_size; index++){
      std::cout <<"ID is: " << (int)actorList->at(index)->GetId() << std::endl;
      auto attributeList = actorList->at(index)->GetAttributes();
      for(auto & attribute : attributeList ){
        if(attribute.GetValue() == "my_obstacle"){
          std::cout << "Found the Obstacle!"; 
          std::cout << "Actor id of this obstacle is: " << attribute.GetId() << std::endl;
          return actorList->at(index);
        }
 //  std::cout << "Actor Attribute Value is: " << attribute.GetValue() << std::endl;
    }
   }  
  auto actor = world.GetActor(42);
  return actor;
}


std::vector<caros::planning::ReferencePoint> GetReferenceLine(const std::string &filename) {
  
    std::vector<caros::planning::ReferencePoint> reference_points;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Failed to open the file: " << filename << std::endl;
        return reference_points;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        double x, y, heading, kappa;
        char delimiter;

        if (ss >> x >> delimiter >> y >> delimiter >> heading >> delimiter >> kappa) {
           
            caros::planning::ReferencePoint reference_point;
            reference_point.set_x(x);
            reference_point.set_y(y);
            reference_point.set_heading(heading);
            reference_point.set_kappa(kappa);
            reference_points.push_back(reference_point);
        }
    }
    file.close();
    return reference_points;
}

void showReferencePoint(const std::vector<caros::planning::ReferencePoint>& reference_line){
    std::cout << std::endl << "Reference Line: " << std::endl << std::endl;
    int ref_size = reference_line.size();
    for(int index = 0;index < ref_size; index++){
        std::cout << " reference line[" << index << "]: " << reference_line[index].x() << ","<< reference_line[index].y()
          << ","<< reference_line[index].heading() <<"," << reference_line[index].kappa() << std::endl;
    }
}

std::vector<caros::common::Obstacle> GetObstacleInfo(carla::client::World world,const carla::SharedPtr<carla::client::Vehicle> &vehicle, double distance_threshold = 80){

  auto vehicle_location = vehicle->GetLocation();
  auto vehicle_transform = vehicle->GetTransform();
  auto actors = world.GetActors();
  std::vector<caros::common::Obstacle> obstacles;

  for (const auto &actor : *actors) {
    std::string type_id = actor->GetTypeId();
    if ((type_id.find("vehicle.") == 0 || type_id.find("static.") == 0 || type_id.find("obstacle.") == 0) && actor->GetId() != vehicle->GetId()) {
      double distance = vehicle_location.Distance(actor->GetLocation());
      if (distance <= distance_threshold) {
        auto id = actor->GetId();
        auto type_id = actor->GetTypeId();
        std::cout << "type_id is " << type_id << std::endl << std::endl;
        auto obstacle_transform = actor->GetTransform();
        auto obstacle_velocity = actor->GetVelocity();
       // auto obstacle_angular_velocity = actor->GetAngularVelocity();
        auto obstacle_bounding_box = actor->GetBoundingBox();
        double heading = obstacle_transform.rotation.yaw;

        caros::perception::PerceptionObstacle perception_obstacle;
        perception_obstacle.mutable_position()->set_x(obstacle_transform.location.x);
        perception_obstacle.mutable_position()->set_y(obstacle_transform.location.y);
        perception_obstacle.mutable_velocity()->set_x(obstacle_velocity.x);
        perception_obstacle.mutable_velocity()->set_y(obstacle_velocity.y);
        perception_obstacle.mutable_velocity()->set_z(0);
        perception_obstacle.set_length(obstacle_bounding_box.extent.x * 2 );
        perception_obstacle.set_width( obstacle_bounding_box.extent.y * 2 );
        perception_obstacle.set_height(obstacle_bounding_box.extent.z * 2 );
        perception_obstacle.set_theta(heading);
        caros::common::Obstacle obstacle(std::to_string(id),perception_obstacle);
        obstacles.push_back(obstacle);
      }
    }
  }
  return obstacles;
}

void ShowObstacleInfo(const std::vector<caros::common::Obstacle> &obstacles) {
  for (auto obstacle : obstacles) {
    const auto position = obstacle.Perception().position();
    const auto velocity = obstacle.Perception().velocity();
    double heading = obstacle.Perception().theta();
    double length = obstacle.Perception().length();
    double width = obstacle.Perception().width();
    double height = obstacle.Perception().height();

    std::cout << "Obstacle ID: " << obstacle.Id() << std::endl;
    std::cout << "Position: (" << position.x() << ", " << position.y() << ")" << std::endl;
    std::cout << "Velocity: (" << velocity.x() << ", " << velocity.y() << ", " << velocity.z() << ")" << std::endl;
    std::cout << "Heading: " << heading << " degrees" << std::endl;
    std::cout << "Dimensions: " << length << "m (length) x " << width << "m (width) x " << height << "m (height)" << std::endl;
    std::cout << std::endl;
  }
}

// apparently there is some issue with the acceleration carla,like the value of accel reaching 20
// so we need a filter to filter out some bad value and also use moving average method
double FilterAndSmoothAcceleration(double acceleration){
  
  static std::deque<double> last_5_accel;
  double max_accel = 6;     // the maxium acceleration can be set
  double accel_moving_avg = 0;

  // check if acceleration is bigger than maxium acceleration  
  if(acceleration > max_accel){
    acceleration = max_accel;
  }
  
  last_5_accel.push_back(acceleration);

  if( last_5_accel.size() > 5){
    last_5_accel.pop_front();
  }

  for( const auto& accel : last_5_accel ){
    accel_moving_avg += accel; 
  }
  
  accel_moving_avg /= last_5_accel.size();
  return accel_moving_avg;
}


 
 double dotProduct(carla::geom::Vector3D velocity, carla::geom::Vector3D accel){
    double dot_product = velocity.x * accel.x + velocity.y * accel.y + velocity.z * accel.z;
    return dot_product;
 }

 void GetHostStateInfo(const carla::SharedPtr<carla::client::Vehicle> &vehicle,caros::planning::HostState& host_state,bool is_first_call){

  carla::geom::Transform transform = vehicle->GetTransform();
  carla::geom::Vector3D location   = transform.location;
  carla::geom::Rotation rotation   = transform.rotation;
  carla::geom::Vector3D velocity   = vehicle->GetVelocity();
  carla::geom::Vector3D  accel     = vehicle->GetAcceleration();
  carla::geom::Vector3D extend     = vehicle->GetBoundingBox().extent;
  carla::geom::Vector3D angular_velocity = vehicle->GetAngularVelocity();

  double speed        = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y );
  double smooth_accel = std::sqrt(accel.x * accel.x + accel.y * accel.y);
  double yaw_rate     = angular_velocity.z;

  host_state.set_x(location.x);
  host_state.set_y(location.y);
  host_state.set_vx(velocity.x);

  host_state.set_vy(velocity.y);
  host_state.set_ax(accel.x);
  host_state.set_ay(accel.y);
  double heading = Degree2Rad(rotation.yaw);
  host_state.set_heading(heading);
  host_state.set_speed(speed);
  host_state.set_yawrate(yaw_rate);
  std::cout << "velocity in gethost function: " <<  velocity.x << ", " << velocity.y << ", " << velocity.z << ")"  << std::endl;
  // std::cout << "acceleration in gethost function: " << acceleration << std::endl;
  smooth_accel = FilterAndSmoothAcceleration(smooth_accel);

  if( dotProduct(velocity,accel) < 0){
    std::cout << smooth_accel << std::endl;
    smooth_accel = -smooth_accel;
    std::cout << "smooth_accel is being negative: " << std::endl;
    std::cout << smooth_accel << std::endl;
  }
   double max_accel = 8;     // the maxium acceleration can be set

  // check if acceleration is bigger than maxium acceleration  
  if(smooth_accel > max_accel){
     smooth_accel = max_accel;
  }

  if(smooth_accel < -8){
    smooth_accel  = -8;
  }

  host_state.set_accel(smooth_accel); 

  // when planning for the first time, 
  if( is_first_call){
    host_state.set_vx(-1);
    host_state.set_vy(0);
    host_state.set_speed(-1);
    host_state.set_ax(0);
    host_state.set_ay(0);
    host_state.set_accel(0);
    host_state.set_yawrate(0);
  }    
}

void ShowAcceleration(double plan_accel,double raw_accel, double smooth_accel,double actual_accel){
   std::cout << "------------------------------Acceleration------------------------------" << std::endl;
   std::cout << "Planned     Acceleration: " << plan_accel << std::endl;
   std::cout << "Raw Control Acceleration: " << raw_accel  << std::endl;
   std::cout << "Smooth      Acceleration: "  << smooth_accel << std::endl;
   std::cout << "Actual      Acceleration: " << actual_accel << std::endl;
   std::cout << "---------------------------------------------------------------------" << std::endl;
   std::cout << std::endl;
}



double ToCarlaSteerControl(double delta_f){
  double steering_command = -(delta_f /M_PI *180) /70;
  return steering_command;
}

int main(){
 // carla setup
  auto client = Client("localhost",2000);
  
  auto world = client.GetWorld();
  auto vehicle = SearchVehicle(world);
  // auto settings = world.GetSettings();
  // settings.substepping = true;
  // settings.max_substep_delta_time = 0.001;
  // settings.max_substeps = 100;
  // settings.fixed_delta_seconds = 0.01;
  // carla::time_duration timeout = std::chrono::seconds(1);
  // world.ApplySettings(settings, timeout); 


  

  // reference line
  std::string filename = "../reference_line.txt";


  // em_planner
  caros::planning::DpPathConfig dp_path_config;
  caros::planning::QpPathConfig qp_path_config;
  caros::planning::DpSpeedConfig dp_speed_config;
  caros::planning::QpSpeedConfig qp_speed_config;

  // prepare inputs
  caros::planning::HostState host_state;
  std::vector<caros::planning::ReferencePoint> reference_line;
  std::vector<caros::common::Obstacle> obstacles;
  

  std::ofstream fsss2("host_state.txt");
  std::ofstream fsss_raw_control("raw_control_input.txt");
  std::ofstream fsss_smooth_control("smooth_control.txt");
  std::ofstream fss_ds_accel("desired_accel.txt");
  // get the obstacle
  // obstacles = GetObstacleInfo(world,boost::static_pointer_cast<Vehicle>(vehicle),80);
  // ShowObstacleInfo(obstacles);
  // std::ofstream fsss("corners.txt");
  //   for(auto obstacle : obstacles){
  //         std::vector<caros::common::math::Vec2d> all_corner = obstacle.PerceptionBoudingBox().GetAllCorners();
  //         for(int idx = 0; idx < all_corner.size(); idx++){
  //         fsss << std::to_string(all_corner[idx].x()-host_state.x()) << ","<< std::to_string(all_corner[idx].y()-host_state.y())<< "\n";
  //       }
  //   }
  // fsss.close();
  //int times = 0;

    //while( vehicle->IsActive() ){
    reference_line = GetReferenceLine(filename);


    // set vehicle starting speed
    carla::geom::Vector3D target_velocity(-1,0,0);
    vehicle->SetTargetVelocity(target_velocity);
    AckermannControl ackermann_control;
   
    GetHostStateInfo(boost::static_pointer_cast<Vehicle>(vehicle),host_state,true);
    
    caros::planning::Trajectory pre_traj = caros::planning::TrajectoryHistory::GetLastTrajectory();
    caros::planning::EMPlanner em_planner(dp_path_config,qp_path_config,dp_speed_config,qp_speed_config);
    
    std::cout << host_state.DebugString() << std::endl;
    em_planner.OnPlan(host_state,reference_line,obstacles,pre_traj,caros::common::time::ToSecond(Clock::Now()));
    em_planner.GetDebugInfo();
    
    // // find and destroy obstacle
    // auto obs_actor = SearchObstacle(world);
    // obs_actor->Destroy();
    
    caros::planning::Trajectory planning_result =  em_planner.GetPlanningResult();
    
     
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
    Eigen::VectorXd raw_control_input;
    Eigen::VectorXd smoothed_control_input;


    int count = 0;
    double dt = 0.01;
    bool check_velocity = true;
    while (count < 800 ){
      if(host_state.y()  > 90 ){
        break;
      }
      std::cout << "------------------------------------------------------------" << std::endl;

      // if( count % 800 == 0 && count != 0){
      //   GetHostStateInfo(boost::static_pointer_cast<Vehicle>(vehicle),host_state,false);
      //    caros::planning::Trajectory pre_traj = caros::planning::TrajectoryHistory::GetLastTrajectory();
      //     em_planner.OnPlan(host_state,reference_line,obstacles,pre_traj,caros::common::time::ToSecond(Clock::Now()));
      //     em_planner.GetDebugInfo();
      //     planning_result =  em_planner.GetPlanningResult();
      // }
      
      double relative_time = count * dt;
      std::cout << "relative_time: " << relative_time << std::endl;
      auto start = std::chrono::high_resolution_clock::now();
      mpc_controller.Control(planning_result,host_state,relative_time,raw_control_input,smoothed_control_input);
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
      ackermann_control.steer        = steering_command; 
      // threshold for steer
      if(std::abs(ackermann_control.steer) < 0.01) {
        ackermann_control.steer =0;
      } 
       // apply control to vehicle in carla
      boost::static_pointer_cast<Vehicle>(vehicle)->ApplyAckermannControl(ackermann_control);

      GetHostStateInfo(boost::static_pointer_cast<Vehicle>(vehicle),host_state,false);
      while(check_velocity){
        GetHostStateInfo(boost::static_pointer_cast<Vehicle>(vehicle),host_state,false);
        if(host_state.speed() < -0.101) {
          check_velocity = false;
        }
      }
      
      
      std::cout << "vref: " << v_ref << " aref: " << a_ref << std::endl;
      auto error = mpc_controller.GetError();
      std::cout << "error: " << std::endl;
      std::cout << error.transpose() << std::endl;
      std::cout << "delta_f: " << front_wheel_angle << std::endl;

      // showing acceleration information
      std::cout << "disired acceleration: " << desired_accel << std::endl;
      std::cout << "actual  acceleration: " << host_state.accel() << std::endl;

      std::cout << host_state.DebugString() << std::endl;
     
      std::cout << std::endl;
      
      // write some value information for debug and draw plot
      fss_ds_accel << desired_accel << std::endl; // writing desired acceleration to the file 
      fsss2 << host_state.x() << "," << host_state.y() <<"," <<host_state.speed() <<"," << host_state.accel()<< "," << host_state.heading() << "," << host_state.yawrate() << std::endl;
      fsss_raw_control << raw_control_input[0]    <<     "," << raw_control_input[1] << std::endl;
      fsss_smooth_control <<  smoothed_control_input[0] <<  "," <<  smoothed_control_input[1] << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      count++;
    }
    std::cout << "Iam outside the while ..........." << std::endl;



  return 0;
}
