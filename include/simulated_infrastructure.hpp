/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Thomas Lobig
 *    Mikkel Skov Maarssø
 *    Sanath Himasekhar Konthala
 *    Marko Mizdrak
 ********************************************************************************/

#pragma once

#include <adore_math/point.h>
#include <adore_math/polygon.h>
#include <chrono>
#include <map>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "adore_dynamics_conversions.hpp"
#include "adore_ros2_msgs/msg/gear_state.hpp"
#include "adore_ros2_msgs/msg/state_monitor.hpp"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"
#include "adore_ros2_msgs/msg/vehicle_command.hpp"

#include "adore_ros2_msgs/msg/vehicle_info.hpp"
#include "dynamics/integration.hpp"
#include "dynamics/physical_vehicle_model.hpp"
#include "dynamics/vehicle_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "adore_ros2_msgs/msg/infrastructure_info.hpp"

using namespace std::chrono_literals;

namespace adore
{
namespace simulated_infrastructure
{
class SimulatedInfrastructure : public rclcpp::Node
{
public:

  explicit SimulatedInfrastructure(const rclcpp::NodeOptions & options);

private:

  void load_parameters();
  void create_subscribers();
  void create_publishers();
  void timer_callback();

  void update_dynamic_subscriptions();

  /******************************* PUBLISHERS ************************************************************/
  rclcpp::Publisher<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr publisher_traffic_participant_set;
  rclcpp::Publisher<adore_ros2_msgs::msg::InfrastructureInfo>::SharedPtr publisher_infrastructure_info;
  rclcpp::Publisher<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr publisher_traffic_participant_set_with_trajectories;

  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr subscriber_traffic_participants_with_trajectories;
  using StateSubscriber = rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipant>::SharedPtr;
  std::unordered_map<std::string, StateSubscriber> other_vehicle_traffic_participant_subscribers;

  void publish_traffic_participants();
  void other_vehicle_traffic_participant_callback( const adore_ros2_msgs::msg::TrafficParticipant& msg,
                                                   const std::string&                              vehicle_namespace );
  rclcpp::TimerBase::SharedPtr main_timer;
  rclcpp::TimerBase::SharedPtr dynamic_subscription_timer;

  /******************************* OTHER MEMBERS ************************************************************/
  adore::math::Point2d infrastructure_position;
  std::optional<adore::math::Polygon2d> infrastructure_validity_area;
  double infrastructure_heading = 0.0;
  std::unordered_map<std::string, dynamics::TrafficParticipant> other_vehicles;
  void traffic_participants_with_trajectories_callback( const adore_ros2_msgs::msg::TrafficParticipantSet& msg );
};

} // namespace simulated_infrastructure
} // namespace adore
