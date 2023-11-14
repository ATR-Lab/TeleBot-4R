#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "telebot_interfaces/msg/motor_goal_list.hpp"
#include "telebot_interfaces/msg/motor_goal.hpp"
using namespace std::chrono_literals;
using telebot_interfaces::msg::MotorGoalList;
using telebot_interfaces::msg::MotorGoal;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Dummy : public rclcpp::Node
{
  public:
    Dummy()
    : Node("Dummy"), count_(0)
    {
      publisher_ = this->create_publisher<MotorGoalList>("motor_goals_safe/upper", 10);
      timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&Dummy::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      MotorGoalList message;
      MotorGoal goal;
      goal.motor_id=1;
      goal.motor_goal=2;
      goal.movement_type="p";
      message.motor_goals.push_back(goal);
      RCLCPP_INFO(this->get_logger(), "Publishing...");
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<MotorGoalList>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto dummy=std::make_shared<Dummy>();
  rclcpp::spin(dummy);
  rclcpp::shutdown();
  return 0;
}