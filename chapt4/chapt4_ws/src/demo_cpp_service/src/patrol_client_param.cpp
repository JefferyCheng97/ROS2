#include "rclcpp/rclcpp.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include <chrono>

using Patrol = chapt4_interfaces::srv::Patrol;
using namespace std::chrono_literals;
using SetP = rcl_interfaces::srv::SetParameters;

class PatrolClient: public rclcpp::Node
{
private:
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    PatrolClient(const std::string& node_name): Node(node_name)
    {
        srand(time(NULL));
        patrol_client_ = this->create_client<Patrol>("patrol_param");
        timer_ = this->create_wall_timer(10s, [&]() -> void
        {
            while (!this->patrol_client_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "rclcpp挂了, 重启服务吧...");
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

    SetP::Response::SharedPtr call_set_parameter(const rcl_interfaces::msg::Parameter &param)
    {
        auto patrol_client_ = this->create_client<SetP>("/turtle_controller/set_parameters");

        while (!this->patrol_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "rclcpp挂了, 重启服务吧...");
                return nullptr;
            }

            RCLCPP_ERROR(this->get_logger(), "等待服务上线中...");
        }

        auto request = std::make_shared<SetP::Request>();
        request->parameters.push_back(param);

        auto future = patrol_client_->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
        auto response = future.get();

        return response;
    }

    void update_service_param_k(double k)
    {
        auto param = rcl_interfaces::msg::Parameter();
        param.name = "k";

        auto param_value = rcl_interfaces::msg::ParameterValue();
        param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_value.double_value = k;
        param.value = param_value;

        auto response = this->call_set_parameter(param);

        if (response == NULL) 
        {
            RCLCPP_ERROR(this->get_logger(), "设置参数失败！");

            return;
        }
        else
        {
            for (auto result : response->results)
            {
                if (result.successful)
                {
                    RCLCPP_INFO(this->get_logger(), "参数设置成功！");
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "参数设置失败！%s", result.reason.c_str());
                }
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClient>("turtle_controller");
    node->update_service_param_k(4.0);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}