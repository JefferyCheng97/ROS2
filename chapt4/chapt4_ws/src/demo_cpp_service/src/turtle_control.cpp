#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"

using Patrol = chapt4_interfaces::srv::Patrol;

class TurtleControlNode: public rclcpp::Node
{
private:
    rclcpp::Service<Patrol>::SharedPtr patrol_service_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
    double target_x_{1.0};
    double target_y_{1.0};
    double k_{1.0};
    double max_speed_{3.0};

public:
    explicit TurtleControlNode(const std::string& node_name): Node(node_name)
    {
        patrol_service_ = this->create_service<Patrol>(
            "patrol",
            [this](const std::shared_ptr<Patrol::Request> request,
                   std::shared_ptr<Patrol::Response> response) -> void
            {
                if ((request->target_x > 0.0 && request->target_x < 12.0) &&
                    (request->target_y > 0.0 && request->target_y < 12.0))
                {
                    this->target_x_ = request->target_x;
                    this->target_y_ = request->target_y;
                    response->result = Patrol::Response::SUCCESS;
                }
                else
                {
                    response->result = Patrol::Response::FAIL;
                }
            });

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 
            10, std::bind(&TurtleControlNode::on_pose_received, this, std::placeholders::_1));
    }

    void on_pose_received(const turtlesim::msg::Pose::SharedPtr pose)
    {
        // 获取当前位置
        auto current_x = pose->x;
        auto current_y = pose->y;
        RCLCPP_INFO(get_logger(), "当前坐标: (%.2f, %.2f)", current_x, current_y);

        // 计算到目标点的距离
        auto error_x = target_x_ - current_x;
        auto error_y = target_y_ - current_y;
        auto distance = std::sqrt(error_x * error_x + error_y * error_y);
        auto angle = std::atan2(error_y, error_x) - pose->theta;

        // 控制策略
        auto msg = geometry_msgs::msg::Twist();
        if (distance > 0.1)
        {
            if (fabs(angle) > 0.2)
            {
                msg.angular.z = fabs(angle);       
            }
            else
            {
                msg.linear.x = k_ * distance;
            }
        }

        // 限制最大速度
        if(msg.linear.x > max_speed_)
        {
            msg.linear.x = max_speed_;
        }
        
        publisher_->publish(msg);        
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControlNode>("turtle_control");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}