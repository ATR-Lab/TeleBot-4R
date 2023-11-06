#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "telebot_interfaces/msg/motor_goals.hpp"
#include <exception>
#include <memory>
#include <unordered_map>
#include <list>
#include <mutex>
#include "motion/control_source_handler.hpp"
using rclcpp::Node;
using telebot_interfaces::msg::MotorGoals;
typedef std::list<rclcpp::Subscription<MotorGoals>> SubscriberList;
using std::placeholders::_1;

class Multiplexer : public Node
{
public:
    Multiplexer() : Node("multiplexer")
    {
        // Set up our control source topic settings
        rclcpp::QoS subQos(rclcpp::KeepLast(1)); // Setting queue size
        subQos.reliable();                       // Setting communication to reliable. All messages will be received, publishers to this topic must also be reliable.
        subQos.transient_local();                // This means that this topic will grab the last message upon subscribing. The publisher must also be transient local.
        auto sourceListener = this->create_subscription<std_msgs::msg::String>("control_source", subQos, std::bind(&Multiplexer::changeControlSource, this, _1));//The discard is important here

        // Init publisher
        rclcpp::QoS pubQos(rclcpp::KeepLast(1));
        _publisher = this->create_publisher<MotorGoals>("motor_goals", pubQos);

        // Create publish loop
        _timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Multiplexer::relay, this));
    }

    inline ControlSourceResult registerSource(const std::string &sourceName, const topicVector &topics)
    {
        return _sourceHandler.registerSource(sourceName, topics);
    }

private:
    void changeControlSource(const std_msgs::msg::String &sourceName)
    {
        auto success = _sourceHandler.setActiveSource(sourceName.data);
        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set active source to: %s. This source is not registered.", sourceName.data.c_str());
            return;
        }
        //Clear subscriptions
        for (auto &sub : _activeSubscriptions)
        {
            // Gets rid of the shared pointers references and deallocates ob
            sub.get_subscription_handle().reset();
        }
        _sourceHandler.getActiveSourceTopics();
        RCLCPP_INFO(this->get_logger(), "Successfully set source to: %s", sourceName.data.c_str());
    }
    void relay()
    {
        //Lock buffer
        _bufferLocked = true;
        try
        {
            //Compiling messages
            MotorGoals msg;
            for (const auto &it : _motorGoalBuffer)
            {
                msg.motor_id.push_back(it.first);
                msg.motor_goal.push_back(it.second.first);
                msg.movement_type.push_back(it.second.second);
            }
            //Publishing new message
            _publisher->publish(msg);

            //Clear buffer
            _motorGoalBuffer.clear();
        }
        catch (std::exception& ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Relay failed: %s",ex.what());
        }
        _bufferLocked = false;
    }
    void listen(const MotorGoals &msg)
    {
        size_t motorGoalsLength = msg.motor_goal.size();
        if (motorGoalsLength != msg.motor_id.size() || motorGoalsLength !=msg.movement_type.size())
        {
            RCLCPP_ERROR(this->get_logger(), "Multiplexer refused relay! Unequal count of entries in motor goals message.");
            return;
        }
        for (size_t i = 0; i < motorGoalsLength; i++)
        {
            while (_bufferLocked)
            {
                sleep(0.05);
            }
            auto pair=std::make_pair(msg.motor_id[i],std::make_pair(msg.motor_goal[i],msg.movement_type[i]));
            auto res=_motorGoalBuffer.insert(pair);
            if(!res.second){
                res.first->second=pair.second;
            }
        }
    }

    //Data
    ControlSourceHandler _sourceHandler;
    SubscriberList _activeSubscriptions;
    // This forces subscribers to wait for the buffer to finish its read of the motor goals
    bool _bufferLocked = false;
    // Maps a motor id to a pair of float and a boolean for if it should be written
    std::unordered_map<int16_t, std::pair<float,std::string>> _motorGoalBuffer;
    rclcpp::Publisher<MotorGoals>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char *argv[])
{
    // Initialize the ROS2 system
    rclcpp::init(argc, argv);

    // Instantiate muxer
    auto muxer = std::make_shared<Multiplexer>();
    
    // Execute until shutdown
    rclcpp::spin(muxer);
    // Shut down the ROS2 system
    rclcpp::shutdown();
    return 0;
}