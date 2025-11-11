#include "rclcpp/rclcpp.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"
#include <chrono>

using Patrol = chapt4_interfaces::srv::Patrol;
using namespace std::chrono_literals;

class PatrolClient: public rclcpp::Node
{
private:
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    PatrolClient(const std::string& node_name): Node(node_name)
    {
        srand(time(NULL));
        patrol_client_ = this->create_client<Patrol>("patrol");
        timer_ = this->create_wall_timer(10s, [&]() -> void
        {
            while (!this->patrol_client_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "rclcpp挂了，重启服务吧...");
                    return;
                }

                RCLCPP_ERROR(this->get_logger(), "等待服务上线中...");
            }

            auto request = std::make_shared<Patrol::Request>();
            request->target_x = rand() % 15;
            request->target_y = rand() % 15;
            RCLCPP_INFO(this->get_logger(), "准备好目标点: (%g, %g)", request->target_x, request->target_y);
            
            this->patrol_client_->async_send_request(request,
                [&](rclcpp::Client<Patrol>::SharedFuture future) -> void
                {
                    auto response = future.get();
                    if (response->result == Patrol::Response::SUCCESS)
                    {
                        RCLCPP_INFO(this->get_logger(), "巡逻任务接受成功！");
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "巡逻任务接受失败，目标点超出范围！");
                    }
                });
        });
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClient>("turtle_control");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}