/* 

Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the usv_gazebo_dynamics_plugin package, known as this Package.

This Package free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This Package s distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

*/


#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <ros/time.h>

#include <usv_gazebo_plugins/usv_gazebo_dynamics_plugin.h>

#define GRAVITY 9.815
#define AREA 0.48 //1.2m in length, 0.2m in width, 2 pontoons.
#define METALENGTH 0.2
#define METAWIDTH 0.2

#define MAX_OUTPUT 1.0
#define MAX_FWD_THRUST 100.0
#define MAX_BCK_THRUST 100.0
#define BOAT_WIDTH 1.0

using namespace gazebo;

UsvPlugin::UsvPlugin()
{
}

UsvPlugin::~UsvPlugin()
{
  rosnode_->shutdown();
  spinner_thread_->join();
  delete rosnode_;
  delete spinner_thread_;
}

void UsvPlugin::FiniChild()
{
}
    
void UsvPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  model_ = _parent;
  world_ = model_->GetWorld();

  node_namespace_ = "";
  if (_sdf->HasElement("robotNamespace")) 
  {
    node_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }
 if (!_sdf->HasElement("bodyName") || !_sdf->GetElement("bodyName")->GetValue())
  {
    link = model_->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    link = model_->GetLink(link_name_);
  }

  if (!link)
  {
    ROS_FATAL("gazebo_ros_kingfisiher plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("maxForce") || !_sdf->GetElement("maxForce")->GetValue()) {
    max_force_ = 62.0;
  } 
  else {
    max_force_ = _sdf->GetElement("maxForce")->Get<double>();
  }

  if (!_sdf->HasElement("xyzDampingFactor") || !_sdf->GetElement("xyzDampingFactor")->GetValue()) {
    xyz_damping_ = 20.0;
  } 
  else {
    xyz_damping_ = _sdf->GetElement("xyzDampingFactor")->Get<double>();
  }

  if (!_sdf->HasElement("yawDampingFactor") || !_sdf->GetElement("yawDampingFactor")->GetValue()) {
    yaw_damping_ = 20.0;
  } 
  else {
    yaw_damping_ = _sdf->GetElement("yawDampingFactor")->Get<double>();
  }

  if (!_sdf->HasElement("rolPitDampingFactor") || !_sdf->GetElement("rolPitDampingFactor")->GetValue()) {
    rp_damping_ = 5.0;
  } 
  else {
    rp_damping_ = _sdf->GetElement("yawDampingFactor")->Get<double>();
  }

  if (!_sdf->HasElement("waterLevel") || !_sdf->GetElement("waterLevel")->GetValue()) {
    water_level_ = 0.5;
  } 
  else {
    water_level_ = _sdf->GetElement("waterLevel")->Get<double>();
  }

  if (!_sdf->HasElement("waterDensity") || !_sdf->GetElement("waterDensity")->GetValue()) {
    water_density_ = 997.7735;
  } 
  else {
    water_density_ = _sdf->GetElement("waterDensity")->Get<double>();
  }

  if (!_sdf->HasElement("cmdTimeout") || !_sdf->GetElement("cmdTimeout")->GetValue()) {
    cmd_timeout_ = 1.0; // how long to allow no input on cmd_drive
  } 
  else {
    cmd_timeout_ = _sdf->GetElement("cmdTimeout")->Get<double>();
  }
  
  // Get inertia and mass of vessel
  inertia = link->GetInertial()->GetPrincipalMoments();
  mass = link->GetInertial()->GetMass();

  //initialize time and odometry position
  prev_update_time_ = last_cmd_drive_time_ = this->world_->GetSimTime();

  // Initialize the ROS node and subscribe to cmd_drive
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "usv_gazebo", 
    ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle( node_namespace_ );

  cmd_drive_sub_ = rosnode_->subscribe("cmd_drive", 1, &UsvPlugin::OnCmdDrive, this );

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->spinner_thread_ = new boost::thread( boost::bind( &UsvPlugin::spin, this) );
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&UsvPlugin::UpdateChild, this));

  // Initialize Added Mass Matrix
  Ma = Eigen::MatrixXd(6,6);
  Ma << 5,   0,   0, 0, 0, 0,
        0,   5,   0, 0, 0, 0,
        0,   0,   1, 0, 0, 0,
        0,   0,   0, 1, 0, 0, 
        1,   0,   0, 0, 1, 0,  
        0,   0,   0, 0, 0, 1;
}


void UsvPlugin::UpdateChild()
{
  common::Time time_now = this->world_->GetSimTime();
  common::Time step_time = time_now - prev_update_time_;
  
  double dt = step_time.Double();
  prev_update_time_ = time_now;

  // Enforce command timeout
  common::Time cmd_time = time_now - last_cmd_drive_time_;
  double dcmd = cmd_time.Double();
  if ( dcmd > cmd_timeout_ )
  {
    thrust = 0.0;
    turntq = 0.0;
  }
      
  // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  pose = link->GetWorldPose();
  angular_acceleration = (link->GetWorldAngularVel() - angular_velocity) / dt;
  angular_velocity = link->GetWorldAngularVel();
  euler = pose.rot.GetAsEuler();
  acceleration = (link->GetWorldLinearVel() - velocity) / dt;
  velocity = link->GetWorldLinearVel();

  // Get gravity
  math::Vector3 gravity_body = pose.rot.RotateVector(world_->GetPhysicsEngine()->GetGravity());
  double gravity = gravity_body.GetLength();
  double load_factor = gravity * gravity / world_->GetPhysicsEngine()->GetGravity().Dot(gravity_body);

  // Rotate vectors to coordinate frames relevant for control
  math::Quaternion heading_quaternion(cos(euler.z/2), 0, 0, sin(euler.z/2));
  math::Vector3 velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
  math::Vector3 acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
  math::Vector3 angular_velocity_body = pose.rot.RotateVectorReverse(angular_velocity);

  // update external forces
  // - input_force: sum of left and right thrust
  // - input_torque: torque generated by the moment differences between the thrusters about the CoG
  // - amass_force: the force induced by added mass hydrodynamic effect
  // - hdamp_force: the hydrodynamic damping force caused by skin friction
  // - resto_force: restoration forces, consisting of buoyancy + gravity.
  //                gravity accounted for by Gazebo itself, and is dropped here.

  math::Vector3 input_force(thrust, 0, 0);
  math::Vector3 hdamp_force(-velocity.x*xyz_damping_, 
    -velocity.y*xyz_damping_, -velocity.z*xyz_damping_);
  math::Vector3 hdamp_torque(-angular_velocity_body.x*rp_damping_, 
    -angular_velocity_body.y*rp_damping_, 
    -angular_velocity_body.z*yaw_damping_);

  Eigen::VectorXd state(6);
  state << acceleration_xy.x, acceleration_xy.y, acceleration_xy.z, 
    angular_acceleration.x, angular_acceleration.y, angular_acceleration.z;
  Eigen::VectorXd amass_force_eigen = Ma*state;

  math::Vector3 amass_force(-amass_force_eigen(0), 
    -amass_force_eigen(1), -amass_force_eigen(2));
  math::Vector3 amass_torque(-amass_force_eigen(3), 
    -amass_force_eigen(4), -amass_force_eigen(5));

  double buoy_force = (water_level_ - pose.pos.z)*AREA*GRAVITY*water_density_;
  math::Vector3 resto_force(0, 0, buoy_force);
  math::Vector3 resto_torque(-METAWIDTH/2*sin(euler.x)*buoy_force, 
    -METALENGTH/2*sin(euler.y)*buoy_force, 0);

  math::Vector3 force = input_force + amass_force + 
    heading_quaternion.RotateVectorReverse(hdamp_force + resto_force);
  math::Vector3 input_torque(0.0, 0.0, turntq);
  math::Vector3 torque = input_torque + hdamp_torque + amass_torque + resto_torque;

  // add force and torque in body-fixed frame in gazebo
  link->AddRelativeForce(force);
  link->AddRelativeTorque(torque);
}

void UsvPlugin::OnCmdDrive( const kingfisher_msgs::DriveConstPtr &msg)
{
    last_cmd_drive_time_ = this->world_->GetSimTime();

    double real_left_thrust = 0;
    double real_right_thrust = 0;
    if (msg->left > 0)
        real_left_thrust = msg->left/MAX_OUTPUT*MAX_FWD_THRUST;
    else if (msg->left < 0)
        real_left_thrust = msg->left/MAX_OUTPUT*MAX_BCK_THRUST;

    if (msg->right > 0)
        real_right_thrust = msg->right/MAX_OUTPUT*MAX_FWD_THRUST;
    else if (msg->right < 0)
        real_right_thrust = msg->right/MAX_OUTPUT*MAX_BCK_THRUST;

    thrust = real_left_thrust + real_right_thrust;
    turntq = (real_right_thrust - real_left_thrust)*BOAT_WIDTH;
}

void UsvPlugin::spin()
{
    while(ros::ok()) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(UsvPlugin);

