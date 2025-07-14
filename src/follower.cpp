#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"


class TurtleFollower : public rclcpp::Node
{
public:
    TurtleFollower() : Node("turtle_follower")
    {

         // Initialize gains
        kp_v_ = 1.5;
        kp_omega_ = 5.0;
        kp_lat_ = 2.0;

        sub_turtle1_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&TurtleFollower::turtle1_pose_callback, this, std::placeholders::_1));

        sub_turtle2_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle2/pose", 10,
            std::bind(&TurtleFollower::turtle2_pose_callback, this, std::placeholders::_1));


        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
    }

private:
    
    double kp_v_;
    double kp_omega_;
    double kp_lat_;

    void turtle1_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        turtle1_pose_ = *msg;
        //std::cout<<"[turtle1_pos_x: "<<turtle1_pose_.x<<"] ";
        follow();
    }
    void turtle2_pose_callback(const turtlesim::msg::Pose::SharedPtr msg1)
    {
        turtle2_pose_ = *msg1;
        //std::cout<<"[turtle2_pos_x"<<turtle2_pose_.x<<"]"<<std::endl;
    }

    void follow()
    {
        // Compute errors
        double ex = turtle1_pose_.x - turtle2_pose_.x;
        double ey = turtle1_pose_.y - turtle2_pose_.y;
        double e_theta = normalizeAngle(turtle1_pose_.theta - turtle2_pose_.theta);

        // Transform to robot (turtle2) frame
        double cos_theta = cos(turtle2_pose_.theta);
        double sin_theta = sin(turtle2_pose_.theta);

        double ex_r = cos_theta * ex + sin_theta * ey;
        double ey_r = -sin_theta * ex + cos_theta * ey;

        // Compute control
        double v = kp_v_ * ex_r;
        double omega = kp_omega_ * e_theta + kp_lat_ * ey_r;
        
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = v;     
        cmd.angular.z = omega;    
        pub_->publish(cmd);
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_turtle1_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_turtle2_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

    turtlesim::msg::Pose turtle1_pose_;
    turtlesim::msg::Pose turtle2_pose_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleFollower>());
    rclcpp::shutdown();
    return 0;
}
