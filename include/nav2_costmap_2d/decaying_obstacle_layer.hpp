#ifndef DECAYING_LAYER_HPP_
#define DECAYING_LAYER_HPP_
#include <chrono>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include "layer.hpp"
#include "layered_costmap.hpp"
#include "observation.hpp"
#include "obstacle_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace decaying_layer
{

// 定义时间坐标系
class TimeCoord
{
public:
  TimeCoord(rclcpp::Time t) : time(t), x(0.0), y(0.0), coord(-1) {}
  rclcpp::Time time;
  double x, y;
  int coord;
};

// 定义不较模板类
class Compare
{
public:
  bool operator()(TimeCoord a, TimeCoord b) { return a.time > b.time; }
};

class DecayingObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  DecayingObstacleLayer() : timeout(std::chrono::milliseconds(500)),last_robot_x_(0.0),last_robot_y_(0.0),decay_second_time_(0),decay_nansecond_time_(500)
  {
    last_index_ = 0;
    // 创建一个半秒钟的持续时间
    RCLCPP_DEBUG(logger_, "DecayingObstacleLayer I Initialized !!!!");
  };
  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);
  rclcpp::Duration timeout;  // 最大超时时间
private:
    /**
     * @brief Callback executed when a parameter change is detected
     * @param event ParameterEvent message
     */
    rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector <rclcpp::Parameter> parameters);

    std::priority_queue <TimeCoord, std::vector<TimeCoord>, Compare> pq;  // 时间优先队列
    std::map<int, rclcpp::Time> status_map;
    double last_robot_x_, last_robot_y_;
    int last_index_;
    double decay_second_time_, decay_nansecond_time_;

};
}  // namespace decaying_layer
#endif
