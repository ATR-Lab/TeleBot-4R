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
    : Node("dummy"), count_(0)
    {
      
      declare_parameter("str_param","default");
      declare_parameter("list_param",std::vector<int>());
      declare_parameter("int_param",40);
      std::cout<<get_parameter("str_param").as_string()<<std::endl;
      std::cout<<get_parameter("int_param").as_int()<<std::endl;
      //For some reason your parameter needs to be loaded into a variable before being iterated.
      auto a=get_parameter("list_param").as_integer_array();
      for(auto i:a){
        std::cout<<i<<", ";
      }
      std::cout<<std::endl;
      publisher_ = this->create_publisher<MotorGoalList>("motor_goals_safe/upper", 10);
      timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&Dummy::timer_callback, this));
    }

  private:
    float last_val=-3.14;
    void timer_callback()
    {
      MotorGoalList message;
      MotorGoal goal;
      goal.motor_id=2;
      goal.motor_goal=last_val;
      goal.movement_type="P";
      message.motor_goals.push_back(goal);
      RCLCPP_INFO(this->get_logger(), "Publishing... %3.2f",last_val);
      last_val+=0.1;
      if(last_val>3.14){
        last_val=-3.14;
      }
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