#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp" // 提供消息接口
#include "tf2_ros/static_transform_broadcaster.h" // 静态变换广播器
#include "tf2/LinearMath/Quaternion.h" // 四元数计算
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // 消息类型转换

class StaticTFBroadcasterNode : public rclcpp::Node
{
private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

public:
  StaticTFBroadcasterNode() : Node("static_tf_broadcaster")
  {
    this->static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->publish_tf();
  }

  void publish_tf()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "target_point";
    transform.transform.translation.x = 5.0;
    transform.transform.translation.y = 3.0;
    transform.transform.translation.z = 0.0;

    // 设置旋转（四元数）
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, 60*M_PI / 180.0); // 无旋转
    transform.transform.rotation = tf2::toMsg(quat); // 转换为消息类型
    this->static_broadcaster_->sendTransform(transform);
  }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTFBroadcasterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown(); 
    return 0;
}
