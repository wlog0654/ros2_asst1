#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose.hpp"
#include "turtlesim/msg/pose.hpp"

class node: public rclcpp::Node
{
private:
    tf2_ros::TransformBroadcaster tfb;
    std::string turtleName;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = turtleName;
        t.transform.translation.x = msg->x;
        t.transform.translation.y = msg->y;
        t.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tfb.sendTransform(t);
    }

public:
    node(std::string st): Node(st + "_boardcaster"), tfb(this), turtleName(st)
    {
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            turtleName + "/pose", 10, std::bind(&node::pose_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Started node for turtle: %s", turtleName.c_str());
    }
    ~node()
    {
        RCLCPP_INFO(this->get_logger(), "Goodbye, world from turtle: %s", turtleName.c_str());
    }
};

class ROS_EVENT_LOOP
{
public:
    ROS_EVENT_LOOP(int argc, char *argv[], std::vector<std::string> turtle_names)
    {
        rclcpp::init(argc, argv);
        
        // 为每只海龟启动一个单独的节点
        for (const auto& turtle_name : turtle_names)
        {
            auto node_ptr = std::make_shared<node>(turtle_name);
            node_ptrs.push_back(node_ptr);
        }

        // 启动所有节点
        rclcpp::executors::SingleThreadedExecutor exec; // 单线程执行器
        for (auto& node_ptr : node_ptrs)
        {
            exec.add_node(node_ptr); // 将每个节点加入执行器
        }
        exec.spin(); // 执行所有节点
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
    if (argc < 2)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: tf turtle_name1 [turtle_name2 ...]");
        return 1;
    }
    
    std::vector<std::string> turtle_names;
    for (int i = 1; i < argc; ++i)
    {
        turtle_names.push_back(argv[i]);
    }

    ROS_EVENT_LOOP(argc, argv, turtle_names);
    return 0;
}

