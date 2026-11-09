#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sura_msgs/msg/sura_velocity_command.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

class Nav2VelocityAdapter : public rclcpp::Node
{
public:
    Nav2VelocityAdapter()
        : Node("nav2_velocity_adapter")
    {
        requester_ = declare_parameter<std::string>(
            "requester", "nav2");

        controller_ = declare_parameter<std::string>(
            "controller", "body_velocity");

        frame_id_ = declare_parameter<std::string>(
            "frame_id", "blueboat/base_link");

        const int priority = declare_parameter<int>(
            "priority", 80);

        if (priority < 1 || priority > 100) {
            throw std::invalid_argument("Priority must be between 1 and 100");
        }

        priority_ = static_cast<uint8_t>(priority);

        // Publicador hacia SURA
        publisher_ = create_publisher<
            sura_msgs::msg::SuraVelocityCommand>(
                "/blueboat/controller/arbitrator/velocity",
                rclcpp::QoS(1).reliable()
            );

        // Suscriptor de Nav2
        subscription_ = create_subscription<
            geometry_msgs::msg::Twist>(
                "/blueboat/nav2/cmd_vel",
                rclcpp::QoS(1).reliable(),
                std::bind(
                    &Nav2VelocityAdapter::callback,
                    this,
                    std::placeholders::_1
                )
            );

        RCLCPP_INFO(
            get_logger(),
            "Nav2 velocity adapter ready"
        );
    }

private:
    void callback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        sura_msgs::msg::SuraVelocityCommand command;

        command.header.stamp = now();
        command.header.frame_id = frame_id_;

        command.requester = requester_;
        command.controller = controller_;
        command.priority = priority_;

        // El giro positivo en ENU tiene signo contrario en el control NED.
        command.velocity = *msg;
        command.velocity.angular.z = -msg->angular.z;

        publisher_->publish(command);
    }

    std::string requester_;
    std::string controller_;
    std::string frame_id_;
    uint8_t priority_;

    rclcpp::Publisher<
        sura_msgs::msg::SuraVelocityCommand
    >::SharedPtr publisher_;

    rclcpp::Subscription<
        geometry_msgs::msg::Twist
    >::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav2VelocityAdapter>());
    rclcpp::shutdown();
    return 0;
}
