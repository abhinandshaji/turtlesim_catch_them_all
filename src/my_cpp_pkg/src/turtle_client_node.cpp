#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/msg/pose.hpp"
#include <random>

class TurtleClientNode : public rclcpp::Node
{
public:
    TurtleClientNode() : Node("turtle_client_node")
    {

        client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for service!");
        }

        publisher_ = this->create_publisher<turtlesim::msg::Pose>("turtle_position", 10);

        timer_ = this->create_wall_timer(std::chrono::seconds(7), std::bind(&TurtleClientNode::TurtleSpawner, this));

        RCLCPP_INFO(this->get_logger(), "Turtle client node has started!");
    }

    void TurtleSpawner()
    {

        auto request_ = std::make_shared<turtlesim::srv::Spawn::Request>();

        // Create a random number generator and distribution
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dist(1, 10);
        std::uniform_real_distribution<float> theta_dist(-3.14, 3.14);

        request_->x = dist(gen);
        request_->y = dist(gen);
        request_->theta = theta_dist(gen);

        // Position of turtle to be published to the conttrol node using the pose msg from turtlesim/msg
        auto pos_ = turtlesim::msg::Pose();
        // Since pos_is not a shared pointer we have to use the dot syntax instead of -> syntax
        pos_.x = request_->x;
        pos_.y = request_->y;
        pos_.theta = request_->theta;
        pos_.linear_velocity = 0.0;
        pos_.angular_velocity = 0.0;

        // publisher the position
        publisher_->publish(pos_);

        auto future = client_->async_send_request(request_);
        try
        {
            // RCLCPP_INFO(this->get_logger(), "Spawned turtle!");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

private:
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_; // client_  is shared pointer of client class from the rclcpp lib sending std::string type data
    rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_; // We use timer to initialize the thread
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}