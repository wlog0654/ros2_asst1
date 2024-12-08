#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include <vector>
#include <cmath>

class node : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target, catcher;
    bool reach_flag;

    // PID控制器变量
    double Kp_linear, Ki_linear, Kd_linear;
    double Kp_angular, Ki_angular, Kd_angular;

    double prev_error_linear, prev_error_angular;
    double integral_linear, integral_angular;

    double relative_x, relative_y, relative_theta;  // 相对位置与角度

    // 角度规范化到[-pi, pi]
    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_->lookupTransform(catcher, target, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", catcher.c_str(), target.c_str(), ex.what());
            return;
        }

        auto t = tf.transform;
        auto message = geometry_msgs::msg::Twist();

        // 计算海龟与目标海龟的相对位置
        double error_x = t.translation.x - relative_x;
        double error_y = t.translation.y - relative_y;
        double error_linear = hypot(error_x, error_y);
        double target_theta = atan2(error_y, error_x);
        double error_angular = normalize_angle(target_theta - relative_theta);

        // PID控制计算
        double control_linear = Kp_linear * error_linear + Kd_linear * (error_linear - prev_error_linear);
        double control_angular = Kp_angular * error_angular + Kd_angular * (error_angular - prev_error_angular);

        // 更新积分项和前一个误差
        integral_linear += error_linear;
        integral_angular += error_angular;
        prev_error_linear = error_linear;
        prev_error_angular = error_angular;

        // 如果误差小于阈值，停止
        if (error_linear < 0.1)
        {
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            publisher_->publish(message);
            if (!reach_flag) RCLCPP_INFO(this->get_logger(), "Reached target");
            reach_flag = true;
            return;
        }
        else
        {
            reach_flag = false;
        }

        // 更高的速度限制
        double max_linear_speed = 5.0;  // 适当设置更小的线速度限制
        double max_angular_speed = 5.0;  // 适当设置更小的角速度限制

        // 防止过度速度控制，应用速度限制
        if (control_linear > max_linear_speed) {
            control_linear = max_linear_speed;
        } else if (control_linear < -max_linear_speed) {
            control_linear = -max_linear_speed;
        }

        if (control_angular > max_angular_speed) {
            control_angular = max_angular_speed;
        } else if (control_angular < -max_angular_speed) {
            control_angular = -max_angular_speed;
        }

        // 发布控制命令
        message.linear.x = control_linear;
        message.angular.z = control_angular;
        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x: '%f', angular.z: '%f'", message.linear.x, message.angular.z);
        publisher_->publish(message);
    }

public:
    node(std::string target, std::string catcher, double relative_x, double relative_y, double relative_theta)
        : Node("follower"), target(target), catcher(catcher),
          relative_x(relative_x), relative_y(relative_y), relative_theta(relative_theta)
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&node::timer_callback, this));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(catcher + "/cmd_vel", 10);

        // 初始化PID控制参数，适当调整以避免过大的响应
        Kp_linear = 4.0; Ki_linear = 0.0; Kd_linear = 0.5;  // PID调节线性速度
        Kp_angular = 4.0; Ki_angular = 0.0; Kd_angular = 1.0;  // PID调节角度速度

        prev_error_linear = 0.0;
        prev_error_angular = 0.0;
        integral_linear = 0.0;
        integral_angular = 0.0;

        RCLCPP_INFO(this->get_logger(), "Hello, world");
    }

    ~node()
    {
        RCLCPP_INFO(this->get_logger(), "Goodbye, world");
    }
};

class ROS_EVENT_LOOP
{
public:
    ROS_EVENT_LOOP(int argc, char *argv[], std::string target, std::vector<std::string> catchers)
    {
        rclcpp::init(argc, argv);

        rclcpp::executors::SingleThreadedExecutor executor;

        // 固定的相对位置与角度
        std::vector<std::tuple<double, double, double>> relative_positions = {
            {0.5, 0.0, 0.0}, // 第一个海龟相对于目标的位置
            {0.5, 0.5, 0.0}, // 第二个海龟相对于目标的位置
            {0.0, 0.5, 0.0}, // 第三个海龟相对于目标的位置
            // 你可以继续添加更多海龟的相对位置
        };

        // 设置跟随者之间的相对位置
        std::vector<std::tuple<double, double>> relative_positions_between_followers = {
            {0.0, 1.0}, // 第二个海龟相对于第一个海龟的位置
            {0.0, 1.0}, // 第三个海龟相对于第二个海龟的位置
            // 你可以继续添加更多海龟之间的相对位置
        };

        for (size_t i = 0; i < catchers.size(); ++i)
        {
            auto node_ptr = std::make_shared<node>(target, catchers[i],
                                                   std::get<0>(relative_positions[i]),
                                                   std::get<1>(relative_positions[i]),
                                                   std::get<2>(relative_positions[i]));
            node_ptrs.push_back(node_ptr);
            executor.add_node(node_ptr);
        }

        executor.spin();
    }

    ~ROS_EVENT_LOOP()
    {
        rclcpp::shutdown();
    }

private:
    std::vector<std::shared_ptr<node>> node_ptrs;
};



int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: follow target catcher1 [catcher2 ...]");
        return 1;
    }

    std::vector<std::string> catchers;
    for (int i = 2; i < argc; ++i)
    {
        catchers.push_back(argv[i]);
    }

    ROS_EVENT_LOOP(argc, argv, argv[1], catchers);
    return 0;
}

