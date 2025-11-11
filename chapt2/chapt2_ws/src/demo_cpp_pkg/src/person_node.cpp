#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace std;

class PersonNode: public rclcpp::Node
{
private:
    string _name;
    int _age;

public:
    PersonNode(const string& node_name, const string& name, const int& age): Node(node_name)
    {
        this->_name = name;
        this->_age = age;
    }

    void eat(const string& food_name)
    {
        // RCLCPP_INFO(this->get_logger(), 
        //     "我是%s, %d岁, 爱吃%s", 
        //     this->_name.c_str(), 
        //     this->_age, 
        //     food_name.c_str());
        RCLCPP_INFO_STREAM(this->get_logger(),
            "我是" << this->_name << ", " << this->_age << "岁, 爱吃" << food_name);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = make_shared<PersonNode>("person_node", "张三", 20);
    RCLCPP_INFO(node->get_logger(), "Hello, I am a person node.");
    node->eat("苹果");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}