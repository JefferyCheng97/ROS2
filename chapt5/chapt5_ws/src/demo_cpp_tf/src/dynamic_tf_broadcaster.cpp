#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "chrono"
#include "tf2_ros/transform_broadcaster.h" // 动态坐标广播器类

using namespace std::chrono_literals; // 使用s, ms

class TFBroadcaster : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    TFBroadcaster() : Node("tf_broadcaster")
    {
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&TFBroadcaster::publish_tf, this));
    }
    void publish_tf()
    {
        // 创建一个TransformStamped类型的消息对象transform，用于填充坐标变换的详细信息
        geometry_msgs::msg::TransformStamped transform;
        // 设置消息头的时间戳为当前节点的时钟时间
        transform.header.stamp = this->get_clock()->now();
        // 设置坐标变换的父坐标系、子坐标系，表明从map到target_point坐标系的变换
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = 2.0;
        transform.transform.translation.y = 3.0;
        transform.transform.translation.z = 0.0;
        // 创建一个tf2的四元数对象q
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 30 * M_PI / 180.0);
        // 通过tf2::toMsg(q)将tf2的四元数转换为geometry_msgs中对应的旋转消息类型
        transform.transform.rotation = tf2::toMsg(q);
        // 发布消息
        this->broadcaster_->sendTransform(transform);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // 创建TFBroadcaster类的智能指针node，实例化节点
    auto node = std::make_shared<TFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}