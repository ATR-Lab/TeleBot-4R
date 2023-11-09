
#ifndef NODE_UTILS_HPP
#define NODE_UTILS_HPP
#include "rclcpp/rclcpp.hpp"
#include <memory>
using rclcpp::Node;
using std::placeholders::_1;
namespace NodeUtils
{
    bool validateMessage(const MotorGoals& msg){
        return msg.motor_goal.size() == msg.motor_id.size() && msg.motor_goal.size() ==msg.movement_type.size();
    }
    /// @brief Creates a subscription for a node
    /// @tparam NodeT Your nodes type
    /// @tparam SubscriptionT The type of the subscription
    /// @param nodeObject This should be "this" inside of a node, but you can pass the node itself from outside.
    /// @param topic The name of your subscribers topic
    /// @param callback The callback for you nodes subscriber
    /// @param qos Optional qos parameter, will use default if left empty
    template <typename NodeT, typename SubscriptionT>
    auto createSubscription(NodeT &nodeObject, const char *topic, void (NodeT::*callback)(const std::shared_ptr<SubscriptionT> &), rclcpp::QoS *qos = nullptr)
    {
        rclcpp::QoS subQos;
        if (qos != nullptr)
        {
            subQos = *qos;
        }
        return nodeObject.create_subscription(topic, subQos, std::bind(callback, &nodeObject, _1));
    }
}

#endif