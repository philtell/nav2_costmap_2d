#include "nav2_costmap_2d/decaying_obstacle_layer.hpp"

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include "nav2_costmap_2d/cost_values.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

PLUGINLIB_EXPORT_CLASS(decaying_layer::DecayingObstacleLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace decaying_layer
{

void DecayingObstacleLayer::onInitialize()
{
    declareParameter("decay_second_time", rclcpp::ParameterValue(0.0));
    declareParameter("decay_nan_second_time", rclcpp::ParameterValue(500.0));
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }
    node->get_parameter(name_+"."+"decay_second_time", decay_second_time_);
    node->get_parameter(name_+"."+"decay_nan_second_time", decay_nansecond_time_);
    timeout = rclcpp::Duration(decay_second_time_, decay_nansecond_time_);
    ObstacleLayer::onInitialize();
    current_ = true;
    dyn_params_handler_ = node->add_on_set_parameters_callback(
            std::bind(
                    &DecayingObstacleLayer::dynamicParametersCallback,
                    this, std::placeholders::_1));
}

using nav2_costmap_2d::Observation;

void DecayingObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
    RCLCPP_DEBUG(
      logger_, "EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEpq.top().time +  timeout is %f \n decay_second_time_ is %lf decay_nansecond_time_ is %lf",
      pq.top().time.seconds() + timeout.seconds(),decay_second_time_,decay_nansecond_time_);
  ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
}

void DecayingObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
    RCLCPP_WARN(logger_,"decay_second_time_ is %lf decay_nansecond_time_ is %lf",decay_second_time_,decay_nansecond_time_);;
//      RCLCPP_WARN(
//      logger_, "EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEpq.top().time +  timeout is %f \n decay_second_time_ is %lf decay_nansecond_time_ is %lf",
//      pq.top().time.seconds() + timeout.seconds(),decay_second_time_,decay_nansecond_time_);
  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  if (!enabled_) return;
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  //get the marking observations
  current = current && getMarkingObservations(observations);

  //get the clearing observations
  current = current && getClearingObservations(clearing_observations);

  //update the global current status
  current_ = current;
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::Time time = clock->now();
  TimeCoord tc(time);

  //raytrace freespace
  for (unsigned int i = 0; i < clearing_observations.size(); ++i) {
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  //place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end();
       ++it) {
    const Observation & obs = *it;

    sensor_msgs::msg::PointCloud2 & cloud = *(obs.cloud_);

    double sq_obstacle_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    unsigned int cloud_size = cloud.height * cloud.width;

    for (unsigned int i = 0; i < cloud_size; ++i) {
      double px = iter_x[i], py = iter_y[i], pz = iter_z[i];

      //if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_) {
        RCLCPP_DEBUG(logger_, "The point is too high");
        continue;
      }

      //compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) +
                       (py - obs.origin_.y) * (py - obs.origin_.y) +
                       (pz - obs.origin_.z) * (pz - obs.origin_.z);

      //if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_range) {
        RCLCPP_DEBUG(logger_, "The point is too far away");
        continue;
      }

      //now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      if (!worldToMap(px, py, mx, my)) {
        RCLCPP_DEBUG(logger_, "Computing map coords failed");
        continue;
      }

      unsigned int index = getIndex(mx, my);
      costmap_[index] = LETHAL_OBSTACLE;
      tc.coord = index;
      tc.x = px;
      tc.y = py;
      pq.push(tc);
      status_map[index] = tc.time;

      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }
  RCLCPP_DEBUG(
      logger_, "EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEpq.top().time +  timeout is %f  tc.time is %f \n decay_second_time_ is %lf decay_nansecond_time_ is %lf",
      pq.top().time.seconds() + timeout.seconds(), tc.time.seconds(),decay_second_time_,decay_nansecond_time_);
  while (pq.size() > 0 && pq.top().time + timeout < tc.time) {
    TimeCoord x = pq.top();
    pq.pop();
    int index = x.coord;
    if (costmap_[index] == LETHAL_OBSTACLE && status_map[index] == x.time) {
      costmap_[index] = nav2_costmap_2d::NO_INFORMATION;
      touch(x.x, x.y, min_x, min_y, max_x, max_y);
    }
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

    rcl_interfaces::msg::SetParametersResult
    DecayingObstacleLayer::dynamicParametersCallback(std::vector <rclcpp::Parameter> parameters)
    {
        std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
        rcl_interfaces::msg::SetParametersResult result;

        for (auto parameter : parameters) {
            const auto & param_type = parameter.get_type();
            const auto & param_name = parameter.get_name();

            if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
                if (param_name == name_ + "." + "decay_nan_second_time") {
                    decay_nansecond_time_ = parameter.as_double();
                } else if (param_name == name_ + "." + "decay_second_time") {
                    decay_second_time_ = parameter.as_double();
                }
            }
//            else if (param_type == ParameterType::PARAMETER_BOOL) {
//                if (param_name == name_ + "." + "enabled") {
//                    enabled_ = parameter.as_bool();
//                } else if (param_name == name_ + "." + "footprint_clearing_enabled") {
//                    footprint_clearing_enabled_ = parameter.as_bool();
//                }
//            } else if (param_type == ParameterType::PARAMETER_INTEGER) {
//                if (param_name == name_ + "." + "combination_method") {
//                    combination_method_ = parameter.as_int();
//                }
//            }
        }

        result.successful = true;
        return result;
    }
}  // namespace decaying_layer