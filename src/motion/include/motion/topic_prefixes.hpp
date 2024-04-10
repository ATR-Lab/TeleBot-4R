#ifndef TOPIC_PREFIXES_HPP
#define TOPIC_PREFIXES_HPP
#include <string>
namespace TopicPrefixes{
    const std::string SUBSYS_PREFIX="telebot/L_1/motion/";
    /// @brief Creates the properly styled topic name for a public topic in the motion package.
    /// @param topicName The desired name for the topic, may not start with a "/"
    /// @return Your formatted topic string
    std::string getPublicTopicName(std::string topicName){
        return SUBSYS_PREFIX+"public/"+topicName;
    }
    /// @brief Creates the properly styled topic name for a private topic in the motion package.
    /// @param topicName The desired name for the topic, may not start with a "/"
    /// @return Your formatted topic string
    std::string getPrivateTopicName(std::string topicName){
        return SUBSYS_PREFIX+"private/"+topicName;
    }
}
#endif