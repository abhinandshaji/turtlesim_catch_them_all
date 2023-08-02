#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/kill.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"

class TurtleKillClientNode : public rclcpp::Node
{
public:
    TurtleKillClientNode() : Node("turtle_kill_client_node"), reach_(0), turtle_(2)
    {
        subscriber_ = this->create_subscription<std_msgs::msg::Bool>("turtle_status", 10, std::bind(&TurtleKillClientNode::callbackTurtleStatus, this, std::placeholders::_1));

        client_ = this->create_client<turtlesim::srv::Kill>("kill");
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for kill service!");
        }

        clr_client_ = this->create_client<std_srvs::srv::Empty>("clear");
        while (!clr_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for clear service!");
        }

        RCLCPP_INFO(this->get_logger(), "Turtle kill client node has been started!");
    }

    void turtlekiller()
    {
        auto request_ = std::make_shared<turtlesim::srv::Kill::Request>();
        auto clr_request =  std::make_shared<std_srvs::srv::Empty::Request>();

        request_->name = "turtle" + std::to_string(turtle_);

        auto future = client_->async_send_request(request_);
        auto clr_future = clr_client_->async_send_request(clr_request);
        
        turtle_++;
        reach_ = 1;
    }

private:
    void callbackTurtleStatus(const std_msgs::msg::Bool::SharedPtr status_)
    {

        if (status_->data == true && reach_ == 0)
        {
            turtlekiller();
        }

        else if (status_->data == false)
        {
            reach_ = 0;
        }
    }

    // shows if we reached or departed
    int reach_;

    // numberr of the turtle
    int turtle_;

    // Subscriber for the turtle_status
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;

    // Client to kill turtle
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client_;

    //Client to clear
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clr_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleKillClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}