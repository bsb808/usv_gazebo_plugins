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
				    //#define AREA 0.48 //1.2m in length, 0.2m in width, 2 pontoons.
				    //#define METALENGTH 0.2
				    //#define METAWIDTH 0.2

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
  ROS_INFO("Loading usv_gazebo_dynamics_plugin");
  model_ = _parent;
  world_ = model_->GetWorld();

  // Retrieve model paramters from SDF
  // Set default values
  node_namespace_ = "";
  param_X_dot_u_ = 5;  // Added mass
  param_Y_dot_v_ = 5;
  param_N_dot_r_ = 1;
  param_X_u_ = 20;  // linear drag
  param_X_uu_ = 0.0;
  param_Y_v_ = 20;
  param_Y_vv_ = 0.0;
  param_N_r_ = 20;
  param_N_rr_ = 0.0;
  param_max_cmd_ = 1.0;
  param_max_force_fwd_ = 100.0;
  param_max_force_rev_ = -100.0;
  param_boat_width_ = 1.0;
  param_metacentric_length_ = 0.2 ; // From clearpath
  param_metacentric_width_ = 0.2;  // ditto
  param_boat_area_ = 0.48;  // Clearpath: 1.2m in length, 0.2m in width, 2 pontoons.

  xyz_damping_ = 20.0;
  yaw_damping_ = 20.0;
  rp_damping_ = 5.0;
  water_level_ = 0.5;
  water_density_ = 997.7735;
  cmd_timeout_ = 1.0; // how long to allow no input on cmd_drive

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
    ROS_FATAL("usv_gazebo_dynamics_plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  if (_sdf->HasElement("xyzDampingFactor") && _sdf->GetElement("xyzDampingFactor")->GetValue()) 
  {
    xyz_damping_ = _sdf->GetElement("xyzDampingFactor")->Get<double>();
  }
  
  if (_sdf->HasElement("yawDampingFactor") && _sdf->GetElement("yawDampingFactor")->GetValue()) 
  {
    yaw_damping_ = _sdf->GetElement("yawDampingFactor")->Get<double>();
  }

  if (_sdf->HasElement("rolPitDampingFactor") && _sdf->GetElement("rolPitDampingFactor")->GetValue()) 
  {
    rp_damping_ = _sdf->GetElement("rolPitDampingFactor")->Get<double>();
  }

  if (_sdf->HasElement("waterLevel") && _sdf->GetElement("waterLevel")->GetValue()) 
  {
    water_level_ = _sdf->GetElement("waterLevel")->Get<double>();
  }

  if (_sdf->HasElement("waterDensity") && _sdf->GetElement("waterDensity")->GetValue()) 
  {
    water_density_ = _sdf->GetElement("waterDensity")->Get<double>();
  }

  if (_sdf->HasElement("cmdTimeout") && _sdf->GetElement("cmdTimeout")->GetValue()) 
  {
    cmd_timeout_ = _sdf->GetElement("cmdTimeout")->Get<double>();
  }
  
  // Get inertia and mass of vessel
  math::Vector3 inertia = link->GetInertial()->GetPrincipalMoments();
  double mass = link->GetInertial()->GetMass();

  // Report some of the pertinent parameters for verification
  ROS_INFO("USV Dynamics Parameters: From URDF XACRO model definition");
  ROS_INFO_STREAM("Vessel Mass (rigid-body): " << mass);
  ROS_INFO_STREAM("Vessel Inertia Vector (rigid-body): X:" << inertia[0] <<
		  " Y:"<<inertia[1] <<
		  " Z:"<<inertia[2]);
  ROS_INFO("USV Dynamics Parameters: Plugin Parameters");

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
  Ma_ = Eigen::MatrixXd(6,6);
  Ma_ << param_X_dot_u_ ,   0,   0, 0, 0, 0,
        0,   param_Y_dot_v_,   0, 0, 0, 0,
        0,   0,   1, 0, 0, 0,
        0,   0,   0, 1, 0, 0, 
        0,   0,   0, 0, 1, 0,  
        0,   0,   0, 0, 0, param_N_dot_r_ ;
}

double UsvPlugin::scaleThrustCmd(double cmd, double max_cmd, double max_pos, double max_neg)
{
  double val = 0.0;
  if (cmd >= 0.0){
    val = cmd/max_cmd*max_pos;
    if (val > max_pos)
    {
      val = max_pos;
    }
  }
  else  // cmd is less than zero
  {
    val = -1.0*std::abs(cmd)/max_cmd*std::abs(max_neg);
    if (std::abs(val) > std::abs(max_neg))
    {
      val = -1.0*std::abs(max_neg);  // ensure it is negative
    }
  }
  return val;  
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
    ROS_WARN_STREAM_THROTTLE(1.0,"Command timeout!");
    last_cmd_drive_left_ = 0.0;
    last_cmd_drive_right_ = 0.0;
  }
  // Scale commands to thrust and torque forces
  ROS_DEBUG_STREAM_THROTTLE(1.0,"Last cmd: left:" << last_cmd_drive_left_
		   << " right: " << last_cmd_drive_right_);
  double thrust_left = scaleThrustCmd(last_cmd_drive_left_, param_max_cmd_,
				      param_max_force_fwd_, 
				      param_max_force_rev_);

  double thrust_right = scaleThrustCmd(last_cmd_drive_right_, param_max_cmd_,
				      param_max_force_fwd_, 
				      param_max_force_rev_);
  ROS_DEBUG_STREAM_THROTTLE(1.0,"Thrust: left:" << thrust_left
		   << " right: " << thrust_right);
  double thrust = thrust_left+thrust_right;
  double torque = (thrust_right - thrust_right)*param_boat_width_;
      
  // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  pose = link->GetWorldPose();
  
  angular_acceleration_ = (link->GetWorldAngularVel() - angular_velocity_) / dt;
  angular_velocity_ = link->GetWorldAngularVel();
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
  math::Vector3 angular_velocity_body = pose.rot.RotateVectorReverse(angular_velocity_);

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
    angular_acceleration_.x, angular_acceleration_.y, angular_acceleration_.z;
  Eigen::VectorXd amass_force_eigen = Ma_*state;

  math::Vector3 amass_force(-amass_force_eigen(0), 
    -amass_force_eigen(1), -amass_force_eigen(2));
  math::Vector3 amass_torque(-amass_force_eigen(3), 
    -amass_force_eigen(4), -amass_force_eigen(5));

  double buoy_force = (water_level_ - pose.pos.z)*param_boat_area_*GRAVITY*water_density_;
  math::Vector3 resto_force(0, 0, buoy_force);
  math::Vector3 resto_torque(-param_metacentric_width_/2*sin(euler.x)*buoy_force, 
    -param_metacentric_length_/2*sin(euler.y)*buoy_force, 0);

  math::Vector3 force_vec = input_force + amass_force + 
    heading_quaternion.RotateVectorReverse(hdamp_force + resto_force);
  math::Vector3 input_torque_vec(0.0, 0.0, torque);
  math::Vector3 torque_vec = input_torque_vec + hdamp_torque + amass_torque + resto_torque;

  // add force and torque in body-fixed frame in gazebo
  link->AddRelativeForce(force_vec);
  link->AddRelativeTorque(torque_vec);
}

void UsvPlugin::OnCmdDrive( const kingfisher_msgs::DriveConstPtr &msg)
{
    last_cmd_drive_time_ = this->world_->GetSimTime();
    last_cmd_drive_left_ = msg->left;
    last_cmd_drive_right_ = msg->right;
}

void UsvPlugin::spin()
{
    while(ros::ok()) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(UsvPlugin);

