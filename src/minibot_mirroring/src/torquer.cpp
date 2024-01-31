#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "telebot_interfaces/msg/motor_goal.hpp"
#include "telebot_interfaces/msg/motor_goal_list.hpp"

class MyPublisher : public rclcpp::Node
{
public:
  MyPublisher()
  : Node("my_publisher")
  {
    publisher_ = this->create_publisher<telebot_interfaces::msg::MotorGoalList>("torque", 10);
  }

  void publish_motor_goal(const int id, const int32_t torque)
  {
    telebot_interfaces::msg::MotorGoalList msg;
    telebot_interfaces::msg::MotorGoal torqueMotor;
    torqueMotor.motor_goal = torque;
    torqueMotor.motor_id = id;
    torqueMotor.movement_type = "Q";
    msg.motor_goals.push_back(torqueMotor);
    publisher_->publish(std::move(msg));
    // RCLCPP_INFO(this->get_logger(), "Published MotorGoal: speed=%f, position=%f", speed, position);
  }

private:
  rclcpp::Publisher<telebot_interfaces::msg::MotorGoalList>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyPublisher>();

  while (true)
  {
    std::string input;
    std::cout << "Enter 'q' to quit or anything else to continue: ";
    std::getline(std::cin, input);
    if (input == "q")
    {
      break;
    }

    int32_t id;
    std::cout << "Enter motor id: ";
    std::cin>>id;

    int32_t torque;
    std::cout << "Enter 1 to torque, -1 to untorque: ";
    std::cin >> torque;

    node->publish_motor_goal(id, torque);

    rclcpp::spin_some(node);
    std::cin.ignore();  // Clear the newline from the buffer
  }

  rclcpp::shutdown();
  return 0;
}
