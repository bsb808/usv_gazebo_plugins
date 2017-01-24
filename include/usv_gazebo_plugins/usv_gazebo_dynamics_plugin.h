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


#ifndef USV_GAZEBO_DYNAMICS_H
#define USV_GAZEBO_DYNAMICS_H

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <kingfisher_msgs/Drive.h>

#include <Eigen/Core>
				    //#include <tf/transform_broadcaster.h>
#include <ros/ros.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

namespace gazebo
{
  class UsvPlugin : public ModelPlugin
  {
  public:
    UsvPlugin();
    virtual ~UsvPlugin();
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  protected:
    virtual void UpdateChild();
    virtual void FiniChild();
  private:
    void OnContact(const std::string &name, const physics::Contact &contact);
    void OnCmdDrive( const kingfisher_msgs::DriveConstPtr &msg);
    
    /// Parameters
    std::string node_namespace_;
    std::string link_name_;
    
    ros::NodeHandle *rosnode_;
    
    ros::Publisher sensor_state_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher joint_state_pub_;
    
    ros::Subscriber cmd_drive_sub_;
    
    physics::WorldPtr world_;
    physics::ModelPtr model_;  
    physics::LinkPtr link;
    sensors::SensorPtr parent_sensor_;
    
    /// Speeds of the wheels
    float wheel_speed_[4];
    
    // Simulation time of the last update
    common::Time prev_update_time_;
    common::Time last_cmd_drive_time_;  
    math::Pose pose;
    math::Vector3 euler, velocity, acceleration, angular_velocity, angular_acceleration;
    math::Vector3 inertia;
    double thrust, turntq;
    double mass;
    double max_force_, xyz_damping_, yaw_damping_, rp_damping_, water_level_, water_density_, cmd_timeout_;
    Eigen::MatrixXd Ma;
    
    //tf::TransformBroadcaster transform_broadcaster_;
    sensor_msgs::JointState js_;
    
    void spin();
    boost::thread *spinner_thread_;
    
    event::ConnectionPtr contact_event_;
    
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
  };  // class UsvPlugin
} // namespace gazebo

#endif //USV_GAZEBO_DYNAMICS_H
