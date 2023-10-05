#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TurtleControlNode : public rclcpp::Node
{
public:
    TurtleControlNode() : Node("turtle_control_node")
    {
        // Subscribe to turtle_position
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>("turtle_position", 10,
                                                                      std::bind(&TurtleControlNode::callbackGoal,
                                                                                this, std::placeholders::_1));

        parent_sub_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,
                                                                      std::bind(&TurtleControlNode::callbackPos,
                                                                                this, std::placeholders::_1));

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        status_publisher_ = this->create_publisher<std_msgs::msg::Bool>("turtle_status", 10);

        // Initialize shared pointers
        er_ = std::make_shared<turtlesim::msg::Pose>();
        goal_ = std::make_shared<turtlesim::msg::Pose>();

        RCLCPP_INFO(this->get_logger(), "Turtle control node has been started!!");
    }

private:
    void callbackGoal(const turtlesim::msg::Pose::SharedPtr pos_)
    {
        goal_ = pos_;
    }

    void callbackPos(const turtlesim::msg::Pose::SharedPtr pos_)
    {
        // Get to goal
        // pose runs from pi to -pi
        // goal from pi to -pi
        er_->theta = goal_->theta - pos_->theta;
        er_->x = goal_->x - pos_->x;
        er_->y = goal_->y - pos_->y;

        dt_ = 0.016; // as rate of publishing will be same as rate of publishing of pose topic

        // if only first turtle there we dont let it move
        if (goal_->x == 0) // We made it so that we get only 1-10 values for goal so this condition enough to fix that this is the first turtle
        {
            V_x = 0.0f;
            V_y = 0.0f;
            omega_ = 0.0f;
        }

        // Check if next iteration required
        else if (er_->x < 0.1 && er_->y < 0.1 && er_->theta < 0.3 && er_->x > -0.1 && er_->y > -0.1 && er_->theta > -0.3)
        {
            auto status_ = std_msgs::msg::Bool();
            status_.data = true;
            status_publisher_->publish(status_);
            status_.data = false;

            // Since next iteration is not needed set values to zero
            V_x = 0.0f;
            V_y = 0.0f;
            omega_ = 0.0f;
        }

        else
        {   
            auto status_ = std_msgs::msg::Bool();
            status_.data = false;
            status_publisher_->publish(status_);
            V_x = er_->x / dt_;
            V_y = er_->y / dt_;
            omega_ = er_->theta / dt_;
        }

        send_pos_.angular.x = 0.0f;
        send_pos_.angular.y = 0.0f;
        send_pos_.linear.z = 0.0f;

        // Using PID control set value of input pose_
        // goal x and goal y are interchanged by the velicty axis and axis of the turtle
        send_pos_.angular.z = 0.006 * (omega_) + 0.00045 * omega_ * dt_ + 0.00003 * (omega_ / dt_);

        /* Using rotation matrix equations use
        the Vx, Vy as the global x_dot and y_dot and
        what we send as the body velocities u, v, r*/
        // this tuning for the PID control works best till now
        //Poles of this system is has negative real parts and one zero at origin so this system is stable
        send_pos_.linear.x = cos(pos_->theta) * ((0.1 * (V_x) + 0.0006 * V_x * dt_ + 0.00002 * (V_x / dt_))) + sin(pos_->theta) * ((0.1 * (V_y) + 0.0006 * V_y * dt_ + 0.00002 * (V_y / dt_)));
        send_pos_.linear.y = -sin(pos_->theta) * ((0.1 * (V_x) + 0.0006 * V_x * dt_ + 0.00002 * (V_x / dt_))) + cos(pos_->theta) * ((0.1 * (V_y) + 0.0006 * V_y * dt_ + 0.00002 * (V_y / dt_)));

        pub_->publish(send_pos_);
    }
    // subscriber sharedptr variable to get position of newly spawned turtle
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
    std::shared_ptr<turtlesim::msg::Pose> goal_; // positiion for first turtle to reach

    // subscriber sharedptr varibale of type pose in msg turtlesim for getting position of turtle
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr parent_sub_;

    // set variable for error and time
    // std::shared_ptr<turtlesim::msg::Pose> er_;
    float dt_;

    // Publisher varibale and the variable to publish
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    geometry_msgs::msg::Twist send_pos_;

    //variables to be send through topics
    float V_x;
    float V_y;
    float omega_;

    // finish status publisher
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
