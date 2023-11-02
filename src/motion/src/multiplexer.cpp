#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <list>

using rclcpp::Node;
class Multiplexer : public Node
{
public:
    Multiplexer() : Node("multiplexer")
    {
        auto relay = std::make_shared<RelayNode>();
        rclcpp::spin(relay);
    }

private:

    class RelayNode : public Node
    {
        // Aliasing so that if we change the name it should prevent many lines of updates
        typedef std_msgs::msg::String TopicType;

    public:
        RelayNode() : Node("relay")
        {
            _publisher = this->create_publisher<TopicType>("goals", 10);
            // auto sub = this->create_subscription<TopicType>("a",1,0);
            _timer = this->create_wall_timer(
                std::chrono::milliseconds(500), std::bind(&RelayNode::publishMessage, this));
        }

    private:
        void publishMessage()
        {
            auto message = std_msgs::msg::String();
            message.data = "Hello, world! " + std::to_string(_count++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            _publisher->publish(message);
        }
        size_t _count;
        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<TopicType>::SharedPtr _publisher;
    };
    
};

int main(int argc, char *argv[])
{
    // Initialize the ROS2 system
    rclcpp::init(argc, argv);
    
    // Instantiate muxer
    auto muxer = std::make_shared<Multiplexer>();
    //Execute until shutdown
    rclcpp::spin(muxer);
    // Shut down the ROS2 system
    rclcpp::shutdown();
    return 0;
}