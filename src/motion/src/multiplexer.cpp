//All topics, remap these in a launch file
/*
Published topic for the goals for entire robot:
    private/motor_goals 
Subscribed topic for what control source to set active:
    public/control_source       
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "telebot_interfaces/msg/motor_goal.hpp"
#include "telebot_interfaces/msg/motor_goal_list.hpp"
#include "telebot_interfaces/msg/motor_state.hpp"
#include <exception>
#include <algorithm>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include <mutex>
#include "motion/control_source_handler.hpp"
#include "motion/topic_prefixes.hpp"
// #include "motion/driver.hpp"
using rclcpp::Node;
using telebot_interfaces::msg::MotorGoal;
using telebot_interfaces::msg::MotorGoalList;
using telebot_interfaces::msg::MotorState;
typedef std::list<std::shared_ptr<rclcpp::Subscription<MotorGoalList>>> SubscriberList;
using std::placeholders::_1;

// class Driver2:public Driver<MotorGoals,MotorState>{
//     Driver2():Driver("driver_upper","tmp","out"){
//         _publisher
//     }
// };
class Multiplexer : public Node
{
public:
    Multiplexer() : Node("multiplexer")
    {
        declareParams();
        auto idVec=get_parameter("invert_motor_goal_ids").as_integer_array();

        std::for_each(idVec.begin(),idVec.end(),[this](int val){_motorIDsToInvert.insert(val);});
        
        initializeSources();
    
        // Set up our control source topic settings
        rclcpp::QoS subQos(rclcpp::KeepLast(1)); // Setting queue size
        subQos.reliable();                       // Setting communication to reliable. All messages will be received, publishers to this topic must also be reliable.
        subQos.transient_local();                // This means that this topic will grab the last message upon subscribing. The publisher must also be transient local.

        //
        _sourceListener = this->create_subscription<std_msgs::msg::String>("public/control_source", subQos, std::bind(&Multiplexer::changeControlSource, this, _1)); // The discard is important here
        // Init publisher
        rclcpp::QoS pubQos(rclcpp::KeepLast(1));
        _publisher = this->create_publisher<MotorGoalList>("private/motor_goals", pubQos);

        // Create publish loop
        _timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Multiplexer::relay, this));
    }

    inline ControlSourceResult registerSource(const std::string &sourceName, const topicVector &topics)
    {
        return _sourceHandler.registerSource(sourceName, topics);
    }

private:
    void initializeSources()
    {
        //THiS IS ONLY GOING TO WORK ON CHRIS' MACHINE!!!
        _sourceHandler.loadFromFile("/home/chris/TeleBot-4R-ROS-2-1/src/motion/config/.ctrlsrcs");

        // Load from file
        //  std::vector<std::vector<std::string>> sources;
        //  get_parameter("control_sources",sources);
        //  for(auto& source:sources){
        //      std::vector<std::string> topics(source.begin() + 1, source.end());//Copy just the topics
        //      _sourceHandler.registerSource(source[0],topics);
        //  }

        auto defaultSrc = get_parameter("default_source").as_string();
        if (defaultSrc != "")
        {
            std_msgs::msg::String str;
            str.data = defaultSrc;
            changeControlSource(str);
        }
    }
    void declareParams()
    {
        this->declare_parameter("default_source", "");
        this->declare_parameter("invert_motor_goal_ids", std::vector<int>{});
    }
    void changeControlSource(const std_msgs::msg::String &sourceName)
    {
        auto success = _sourceHandler.setActiveSource(sourceName.data);
        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set active source to: %s. This source is not registered.", sourceName.data.c_str());
            return;
        }

        // Clear subscriptions
        for (auto &sub : _activeSubscriptions)
        {
            // Gets rid of the shared pointers references and deallocates ob
            sub->get_subscription_handle().reset();
        }
        _activeSubscriptions.clear();

        // Register new subscribers
        const rclcpp::QoS subQos(rclcpp::KeepLast(1));
        for (auto &topic : _sourceHandler.getActiveSourceTopics())
        {
            _activeSubscriptions.push_front(this->create_subscription<MotorGoalList>(topic, subQos, std::bind(&Multiplexer::listen, this, _1)));
        }

        RCLCPP_INFO(this->get_logger(), "Successfully set source to: %s", sourceName.data.c_str());
    }
    void relay()
    {
        const int NO_LOG_YET = 0;
        const int IGNORE_LOG = 1;
        const int RELAY_LOG = 2;
        static int lastLog = 0;
        // Lock buffer
        _bufferLocked = true;

        if (_motorGoalBuffer.empty())
        {
            _bufferLocked = false;
            if (lastLog == NO_LOG_YET || lastLog == RELAY_LOG)
            {
                RCLCPP_INFO(this->get_logger(), "Relaying ignored!");
                lastLog = IGNORE_LOG;
            }
            return;
        }
        try
        {
            if (lastLog == NO_LOG_YET || lastLog == IGNORE_LOG)
            {
                RCLCPP_INFO(this->get_logger(), "Relaying...");
                lastLog=RELAY_LOG;
            }
            // Compiling messages
            MotorGoalList msg;
            for (const auto &it : _motorGoalBuffer)
            {
                MotorGoal tmp;
                tmp.motor_id = it.first;
                if(_motorIDsToInvert.find(tmp.motor_id)!=_motorIDsToInvert.end()){
                    RCLCPP_INFO(this->get_logger(), "Motor in ids");
                }
                //Invert motors that should be

                tmp.motor_goal = (_motorIDsToInvert.find(tmp.motor_id)!=_motorIDsToInvert.end())?it.second.first*-1.0:it.second.first;
                tmp.movement_type = it.second.second;
                msg.motor_goals.push_back(tmp);
            }
            // Publishing new message
            _publisher->publish(msg);

            // Clear buffer
            _motorGoalBuffer.clear();
        }
        catch (std::exception &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Relay failed: %s", ex.what());
        }
        _bufferLocked = false;
    }
    void listen(const MotorGoalList &msg)
    {
        for (auto &goal : msg.motor_goals)
        {
            while (_bufferLocked)
            {
                sleep(0.05);
            }
            auto pair = std::make_pair(goal.motor_id, std::make_pair(goal.motor_goal, goal.movement_type));
            auto res = _motorGoalBuffer.insert(pair);
            if (!res.second)
            {
                res.first->second = pair.second;
            }
        }
    }

    // Data
    ControlSourceHandler _sourceHandler;
    SubscriberList _activeSubscriptions;
    // This forces subscribers to wait for the buffer to finish its read of the motor goals
    bool _bufferLocked = false;
    std::unordered_set<int> _motorIDsToInvert;
    // Maps a motor id to a pair of float and a boolean for if it should be written
    std::unordered_map<int16_t, std::pair<float, std::string>> _motorGoalBuffer;
    rclcpp::Publisher<MotorGoalList>::SharedPtr _publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sourceListener;
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