#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;


using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      rclcpp::QoS qos(rclcpp::KeepLast(1)); // Match the history and depth
      qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      subscription_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "/map_borders", qos, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const geometry_msgs::msg::Polygon::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->points[0].x);
    }
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  plt::plot({1,3,2,4});
  plt::show();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}