#include "chrono"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"  // 提供了四元数转欧拉角
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"              // 提供buffer_存储坐标信息
#include "tf2_ros/transform_listener.h"  // 坐标监听类

using namespace std::chrono_literals;    // 使用s, ms

class TFListener : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;

public:
    TFListener() : Node("tf_listener")
    {
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        // tf2_ros::TransformListener会自从订阅静态和动态话题，并保存到buffer_中
        this->listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this);
        timer_ = this->create_wall_timer(1s, std::bind(&TFListener::getTransform, this));
    }

    void getTransform()
    {
        try
        {
            // 查询坐标关系
            const auto transform = buffer_->lookupTransform("base_link", "target_point", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0f));
            
            // 获取查询结果
            auto translation = transform.transform.translation;
            auto rotation = transform.transform.rotation;
            double y, p, r;

            // 将四元数转换为欧拉角
            tf2::getEulerYPR(rotation, y, p, r);
            RCLCPP_INFO(this->get_logger(), "目标坐标系在基坐标系下的位置关系(平移): x=%.2f, y=%.2f, z=%.2f", 
                        translation.x, translation.y, translation.z);
            RCLCPP_INFO(this->get_logger(), "目标坐标系在基坐标系下的姿态关系(旋转): yaw=%.2f, pitch=%.2f, roll=%.2f", 
                        y, p, r);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "转换失败: %s", e.what());
        }
        
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // 创建TFListener类的智能指针node，实例化节点
    auto node = std::make_shared<TFListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}