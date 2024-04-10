#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "telebot_interfaces/msg/motor_goal_list.hpp"
#include "telebot_interfaces/msg/motor_goal.hpp"
#include "telebot_interfaces/msg/motor_state_list.hpp"
#include "telebot_interfaces/msg/motor_state.hpp"
#include "motion/topic_prefixes.hpp"
using namespace std::chrono_literals;
using telebot_interfaces::msg::MotorGoalList;
using telebot_interfaces::msg::MotorGoal;
using telebot_interfaces::msg::MotorStateList;
using telebot_interfaces::msg::MotorState;
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
      publisher_ = this->create_publisher<MotorGoalList>("dbg", 10);
      timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000), std::bind(&Dummy::timer_callback, this));
      subscriber_ = this->create_subscription<MotorStateList>(
            "/telebot/L_1/motion/public/upper_state", 10, std::bind(&Dummy::subscriber_callback, this, std::placeholders::_1));
    }

  private:
    float last_val=-2;
    rclcpp::Subscription<telebot_interfaces::msg::MotorStateList>::SharedPtr subscriber_;
    
    void subscriber_callback(const MotorStateList::SharedPtr msg)
    {
      if(msg->motor_states.size()<1){
        return;
      }
        MotorGoalList message;
        MotorGoal goal;
        goal.motor_id=13;
        goal.motor_goal=msg->motor_states[0].position;
        goal.movement_type="P";
        message.motor_goals.push_back(goal);
        RCLCPP_INFO(this->get_logger(), "Echoing to 13....");
    
        publisher_->publish(message);
    }
    void timer_callback()
    { 
      MotorGoalList message;
      MotorGoal goal;
      goal.motor_id=3;
      goal.motor_goal=last_val;
      goal.movement_type="P";
      message.motor_goals.push_back(goal);
      RCLCPP_INFO(this->get_logger(), "Publishing... %3.2f",last_val);
      
      if(last_val==-2){
        last_val=-1;
      }
      else{
        last_val=-2;
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