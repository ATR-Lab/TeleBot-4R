#include "rclcpp/rclcpp.hpp"
#include "telebot_interfaces/msg/motor_state_list.hpp" // Replace with the actual message type you want to mirror
#include "telebot_interfaces/msg/motor_state.hpp"
#include "telebot_interfaces/msg/motor_goal_list.hpp" // Replace with the actual message type you want to mirror
#include "telebot_interfaces/msg/motor_goal.hpp"
#include <unordered_map>
using std::unordered_map;
using namespace telebot_interfaces::msg; // Possibly bad practice but i dont care
class MinibotMirror : public rclcpp::Node
{
public:
    MinibotMirror() : Node("minibot_mirror")
    {
        initMap();
        rclcpp::QoS qos(3);
        qos.reliable();
        qos.transient_local();
        _publisher = this->create_publisher<MotorGoalList>("mirror_minibot", qos);
        _subscription = create_subscription<MotorStateList>(
            "telebot/L_1/motion/public/upper_state", 10, std::bind(&MinibotMirror::subscriptionCallback, this, std::placeholders::_1));

        timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&MinibotMirror::publishCallback, this));
    }

private:
    // Implement this in the lab. I am unsure of mappings atm.
    void initMap()
    {
        // Map minibot ids to their telebot equivalent
        _minibotToTelebot = {
            {1, 4},   // right_shoulder_x
            {2, 5},   // right_shoulder_y
            {3, 6},   // right_shoulder_z
            {4, 7},   // right_arm_x
            {5, 8},   // right_arm_z
            {6, 9},   // right_wrist_y
            {7, 10},  // right_gripper
            {11, 11}, // left_shoulder_x
            {12, 12}, // left_shoulder_y
            {13, 17}, // left_shoulder_z
            {14, 21}, // left_arm_x
            {15, 22}, // left_arm_z
            {16, 23}, // left_wrist_y
            {17, 24}  // left_gripper
        };
    }

    void subscriptionCallback(const MotorStateList::SharedPtr msg)
    {
        // Unpack data into unordered map
        for (auto &state : msg->motor_states)
        {
            auto res = _motorStateBuffer.find(state.id);
            // Check if it is in buffer
            if (res == _motorStateBuffer.end())
            {
                _motorStateBuffer.insert(std::make_pair(state.id, state));
            }
            else
            {
                // Update if already in buffer
                res->second = state;
            }
        }
    }
    bool isClawMotor(int id)
    {
        const int LEFT_MINI_CLAW = 7;
        const int RIGHT_MINI_CLAW = 17;
        switch (id)
        {
        // Waterfall through to true return in case of claw motor
        case RIGHT_MINI_CLAW:
        case LEFT_MINI_CLAW:
            return true;
        default:
            return false;
        }
    }
    // Non linear control mapping for more precision towards -3.14
    float clawTransform(float minibotClawPosition)
    {
        const float A = 0.281;
        const float B = 1.77;
        const float C = 2.78;
        return (A * (minibotClawPosition * minibotClawPosition)) + (minibotClawPosition * B) + C;
    }
    // Constructs and adds the messages for the claws
    bool constructClawMessage(const MotorState &minibotState, MotorGoalList &goals)
    {
        const int LEFT_MINI_CLAW = 7;
        const int RIGHT_MINI_CLAW = 17;

        MotorGoal outerPincer, innerPincer;
        outerPincer.movement_type = "P";
        innerPincer.movement_type = "P";

        if (minibotState.id == LEFT_MINI_CLAW)
        {
            // Calculate claw positions for telebot left motors
            outerPincer.motor_id = 23; // IDK if this is actually outer, need real testing
            innerPincer.motor_id = 24; // IDK if this is actually inner, need real testing

            outerPincer.motor_goal = clawTransform(minibotState.position);        // Motor 23 approaches 0 from +1.5 to close
            innerPincer.motor_goal = -1.0 * clawTransform(minibotState.position); // Motor 24 appoaches 0 from -1.5 to close

            goals.motor_goals.push_back(outerPincer);
            goals.motor_goals.push_back(innerPincer);
            return true;
        }
        // I didn't measure these, bc connections were loose, will need to verify or re attach to make match the pattern
        else if (minibotState.id == RIGHT_MINI_CLAW)
        {
            // Calculate claw positions for telebot right motors
            outerPincer.motor_id = 21; // IDK if this is actually outer, need real testing
            innerPincer.motor_id = 22; // IDK if this is actually inner, need real testing

            outerPincer.motor_goal = clawTransform(minibotState.position);        // Motor 23 approaches 0 from +1.5 to close
            innerPincer.motor_goal = -1.0 * clawTransform(minibotState.position); // Motor 24 appoaches 0 from -1.5 to close

            goals.motor_goals.push_back(outerPincer);
            goals.motor_goals.push_back(innerPincer);

            return true;
        }
        return false;
    }
    MotorGoalList constructMessage()
    {

        MotorGoalList msg;

        for (auto &statePair : _motorStateBuffer)
        {
            auto rawState = statePair.second;
            if (constructClawMessage(rawState, msg))
            {
                continue;
            }

            // Make sure motor has a destination to mirror to. Ignore it if not
            if (_minibotToTelebot.count(rawState.id) < 1)
            {
                RCLCPP_INFO_ONCE(this->get_logger(), "Not all motors from subscription are being relayed. This can be ignored if there are only certain motors you want to mirror.");
                continue;
            }

            //Mirror the motor
            MotorGoal transformedGoal;
            transformedGoal.motor_goal = rawState.goal;
            transformedGoal.motor_id = _minibotToTelebot.at(rawState.id);
            transformedGoal.movement_type = "P";

            msg.motor_goals.push_back(transformedGoal);
        }
        return msg;
    }
    void publishCallback()
    {

        _publisher->publish(constructMessage());
    }
    unordered_map<int32_t, int32_t> _minibotToTelebot;    // Maps minibot ids to their telebot equivalent
    unordered_map<int32_t, MotorState> _motorStateBuffer; // Maps minibot ids to their telebot equivalent
    rclcpp::Subscription<MotorStateList>::SharedPtr _subscription;
    rclcpp::Publisher<MotorGoalList>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinibotMirror>());
    rclcpp::shutdown();
    return 0;
}
