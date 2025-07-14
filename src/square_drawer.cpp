#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class SquareDrawer : public rclcpp::Node
{
public:
    SquareDrawer() : Node("square_drawer")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        draw_square();
    }

private:
    void draw_square()
    {
        for (int i = 0; i < 4; ++i)
        {
            auto move = geometry_msgs::msg::Twist();
            move.linear.x = 0.0;
            move.angular.z = 0.0;
            publisher_->publish(move);
            std::this_thread::sleep_for(2s);
            move.linear.x = 2.0;
            move.angular.z = 0.0;
            publisher_->publish(move);

            RCLCPP_INFO(this->get_logger(), "Moving forward");
            std::this_thread::sleep_for(2s);

            move.linear.x = 0.0;
            publisher_->publish(move);

            auto turn = geometry_msgs::msg::Twist();
            turn.angular.z = 1.57;
            publisher_->publish(turn);
            RCLCPP_INFO(this->get_logger(), "Turning");
            std::this_thread::sleep_for(1s);

            turn.angular.z = 0.0;
            publisher_->publish(turn);
        }
        RCLCPP_INFO(this->get_logger(), "Finished drawing square");
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SquareDrawer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
