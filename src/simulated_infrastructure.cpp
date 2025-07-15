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
#include "simulated_infrastructure.hpp"

#include "adore_ros2_msgs/msg/infrastructure_info.hpp"
#include "adore_ros2_msgs/msg/point2d.hpp"
#include "adore_ros2_msgs/msg/polygon2d.hpp"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"

namespace adore
{
namespace simulated_infrastructure
{
SimulatedInfrastructure::SimulatedInfrastructure( const rclcpp::NodeOptions& options ) :
  Node( "simulated_infrastructure", options )
{
  load_parameters();
  create_publishers();
  create_subscribers();
}

void
SimulatedInfrastructure::load_parameters()
{
  double infrastructure_position_x;
  declare_parameter( "infrastructure_position_x", 0.0 );
  get_parameter( "infrastructure_position_x", infrastructure_position_x );

  double infrastructure_position_y;
  declare_parameter( "infrastructure_position_y", 0.0 );
  get_parameter( "infrastructure_position_y", infrastructure_position_y );

  declare_parameter( "infrastructure_yaw", 0.0 );
  get_parameter( "infrastructure_yaw", infrastructure_heading );
  infrastructure_position.x = infrastructure_position_x;
  infrastructure_position.y = infrastructure_position_y;

  std::vector<double> validity_area_points; // request assistance polygon
  declare_parameter( "validity_polygon", std::vector<double>{} );
  get_parameter( "validity_polygon", validity_area_points );

  // Convert the parameter into a Polygon2d
  if( validity_area_points.size() >= 6 ) // minimum 3 x, 3 y
  {
    math::Polygon2d validity_area;
    validity_area.points.reserve( validity_area_points.size() / 2 );

    for( size_t i = 0; i < validity_area_points.size(); i += 2 )
    {
      double x = validity_area_points[i];
      double y = validity_area_points[i + 1];
      validity_area.points.push_back( { x, y } );
    }
    infrastructure_validity_area = validity_area;
  }
}

void
SimulatedInfrastructure::create_publishers()
{
  publisher_traffic_participant_set = create_publisher<adore_ros2_msgs::msg::TrafficParticipantSet>( "traffic_participants", 10 );
  publisher_infrastructure_info     = create_publisher<adore_ros2_msgs::msg::InfrastructureInfo>( "infrastructure_info", 10 );
  publisher_traffic_participant_set_with_trajectories
    = create_publisher<adore_ros2_msgs::msg::TrafficParticipantSet>( "/global/infrastructure_calculated_traffic_participants", 10 );
}

void
SimulatedInfrastructure::create_subscribers()
{
  subscriber_traffic_participants_with_trajectories = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>(
    "traffic_participants_with_trajectories", 10,
    std::bind( &SimulatedInfrastructure::traffic_participants_with_trajectories_callback, this, std::placeholders::_1 ) );
  main_timer                 = create_wall_timer( 100ms, std::bind( &SimulatedInfrastructure::timer_callback, this ) );
  dynamic_subscription_timer = create_wall_timer( 1s, std::bind( &SimulatedInfrastructure::update_dynamic_subscriptions, this ) );
}

void
SimulatedInfrastructure::timer_callback()
{
  publish_traffic_participants();
  // update_dynamic_subscriptions();
  // latest_traffic_participant_set.remove_old_participants( 1.0, this->now().seconds() );

  // publisher_traffic_participant_set->publish( dynamics::conversions::to_ros_msg( latest_traffic_participant_set ) );

  adore_ros2_msgs::msg::InfrastructureInfo infrastructure_info_msg;
  infrastructure_info_msg.position_x = infrastructure_position.x;
  infrastructure_info_msg.position_y = infrastructure_position.y;
  infrastructure_info_msg.yaw        = infrastructure_heading;

  adore_ros2_msgs::msg::Polygon2d validity_area_msg;
  if( infrastructure_validity_area.has_value() )
  {
    for( auto point : infrastructure_validity_area.value().points )
    {
      adore_ros2_msgs::msg::Point2d point_msg;
      point_msg.x = point.x;
      point_msg.y = point.y;

      validity_area_msg.points.push_back( point_msg );
    }
    infrastructure_info_msg.validity_area = validity_area_msg;
  }
  publisher_infrastructure_info->publish( infrastructure_info_msg );
}

void
SimulatedInfrastructure::update_dynamic_subscriptions()
{
  auto       topic_names_and_types = get_topic_names_and_types();
  std::regex valid_topic_regex( R"(^/([^/]+)/traffic_participant$)" );
  std::regex valid_type_regex( R"(^adore_ros2_msgs/msg/TrafficParticipant$)" );

  for( const auto& topic : topic_names_and_types )
  {
    const std::string&              topic_name = topic.first;
    const std::vector<std::string>& types      = topic.second;

    std::smatch match;
    if( std::regex_match( topic_name, match, valid_topic_regex )
        && std::any_of( types.begin(), types.end(),
                        [&]( const std::string& type ) { return std::regex_match( type, valid_type_regex ); } ) )
    {
      std::string vehicle_namespace = match[1].str();

      // Skip subscribing to own namespace
      if( vehicle_namespace == std::string( get_namespace() ).substr( 1 ) )
      {
        continue;
      }

      // Check if already subscribed
      if( other_vehicle_traffic_participant_subscribers.count( vehicle_namespace ) > 0 )
      {
        continue;
      }

      // Create a new subscription
      auto subscription = create_subscription<adore_ros2_msgs::msg::TrafficParticipant>(
        topic_name, 10, [this, vehicle_namespace]( const adore_ros2_msgs::msg::TrafficParticipant& msg ) {
        other_vehicle_traffic_participant_callback( msg, vehicle_namespace );
      } );

      other_vehicle_traffic_participant_subscribers[vehicle_namespace] = subscription;

      RCLCPP_INFO( get_logger(), "Subscribed to new vehicle namespace: %s", vehicle_namespace.c_str() );
    }
  }
}

void
SimulatedInfrastructure::publish_traffic_participants()
{
  dynamics::TrafficParticipantSet traffic_participants;

  for( const auto& [vehicle_namespace, other_vehicle] : other_vehicles )
  {
    double sensor_range = 1000;
    double distance     = adore::math::distance_2d( other_vehicle.state, infrastructure_position );
    if( distance > sensor_range )
      continue;

    traffic_participants.participants[other_vehicle.id] = other_vehicle;
  }

  traffic_participants.validity_area = infrastructure_validity_area;

  // adore_ros2_msgs::msg::Polygon2d validity_area_msg;
  // if ( infrastructure_validity_area.has_value() )
  // {
  //   for (auto point : infrastructure_validity_area.value().points)
  //   {
  //       adore_ros2_msgs::msg::Point2d point_msg;
  //       point_msg.x = point.x;
  //       point_msg.y = point.y;

  // validity_area_msg.points.push_back(point_msg);
  // }
  // infrastructure_info_msg.validity_area = validity_area_msg;
  publisher_traffic_participant_set->publish( dynamics::conversions::to_ros_msg( traffic_participants ) );
}

void
SimulatedInfrastructure::other_vehicle_traffic_participant_callback( const adore_ros2_msgs::msg::TrafficParticipant& msg,
                                                                     const std::string&                              vehicle_namespace )
{
  other_vehicles[vehicle_namespace] = dynamics::conversions::to_cpp_type( msg );
}

void
SimulatedInfrastructure::traffic_participants_with_trajectories_callback( const adore_ros2_msgs::msg::TrafficParticipantSet& msg )
{
  publisher_traffic_participant_set_with_trajectories->publish( msg );
}

} // namespace simulated_infrastructure
} // namespace adore

int
main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<adore::simulated_infrastructure::SimulatedInfrastructure>( rclcpp::NodeOptions{} ) );
  rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::simulated_infrastructure::SimulatedInfrastructure )
