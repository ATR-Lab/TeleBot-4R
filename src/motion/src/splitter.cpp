//All topics, remap these in a launch file
/*
Publish topic for the goals for upper:
    private/upper_goals 
Publish topic for the goals for lower:
    private/lower_goals 
Topic to subscribe to:
    private/motor_goals       
*/

#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "telebot_interfaces/msg/motor_goal.hpp"
#include "telebot_interfaces/msg/motor_goal_list.hpp"
#include "motion/topic_prefixes.hpp"

using std::vector;
using std::find;
using std::placeholders::_1;

using rclcpp::Node;
using telebot_interfaces::msg::MotorGoal;
using telebot_interfaces::msg::MotorGoalList;
using rclcpp::Publisher;
using rclcpp::Subscription;

class Splitter : public Node {
public:
    Splitter() : Node("splitter") {
        this->declare_parameter("motor_ids.upper_ids", vector<int>());
        this->declare_parameter("motor_ids.lower_ids", vector<int>());

        this->upperPublisher_ = this->create_publisher<MotorGoalList>       ("private/upper_goals", 10);
        this->lowerPublisher_ = this->create_publisher<MotorGoalList>       ("private/lower_goals", 10);
        this->sourceSubscription_ = this->create_subscription<MotorGoalList>("private/motor_goals", 10, std::bind(&Splitter::goalCallback, this, _1));
    }
private:
    void goalCallback(const MotorGoalList &goals) const {
        const auto upperIds = this->get_parameter("motor_ids.upper_ids").as_integer_array();
        const auto lowerIds = this->get_parameter("motor_ids.lower_ids").as_integer_array();

        // Messages to be sent to corresponding topics
        MotorGoalList upperGoals = MotorGoalList();
        MotorGoalList lowerGoals = MotorGoalList();

        // Iterate through each goal and sort them into upperGoals,lowerGoals, or discard
        for (auto it = goals.motor_goals.begin(); it != goals.motor_goals.end(); ++it) {
            if (find(upperIds.begin(), upperIds.end(), it->motor_id) != upperIds.end()) {
                upperGoals.motor_goals.push_back(*it);
            }
            else if (find(lowerIds.begin(), lowerIds.end(), it->motor_id) != lowerIds.end()) {
                lowerGoals.motor_goals.push_back(*it);
            }
            else {
                RCLCPP_WARN(this->get_logger(), "Invalid Motor ID: %d! Discarding Goal...", it->motor_id);
            }
        }

        // Publish sorted messages to be checked by safety
        this->upperPublisher_->publish(upperGoals);
        this->lowerPublisher_->publish(lowerGoals);
    }

    Publisher<MotorGoalList>::SharedPtr upperPublisher_;
    Publisher<MotorGoalList>::SharedPtr lowerPublisher_;
    Subscription<MotorGoalList>::SharedPtr sourceSubscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Splitter>());
    rclcpp::shutdown();
    return 0;
}