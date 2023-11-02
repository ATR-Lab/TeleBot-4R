#ifndef CONTROL_SOURCE_HANDLER_HPP
#define CONTROL_SOURCE_HANDLER_HPP
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>
#include <list>

typedef std::vector<std::string> topicVector;

enum ControlSourceRegisterResult{
    FAILED,
    SUCCEEDED,
    ALREADY_REGISTERED
};

const char* NO_CONTROL_SOURCE="";

class ControlSourceHandler{
public:
    ControlSourceHandler(){}//Done

    const std::string getActiveSourceName()const{return _activeSource;}//Done
    const topicVector getActiveSourceTopics()const{return _activeTopics;}//Done

    //Setters
    bool setActiveSource(const std::string& sourceName);//Done
    ControlSourceRegisterResult registerSource(const std::string&,const topicVector&);
private:
    std::unordered_map<std::string,topicVector> _controlSources;
    topicVector _activeTopics;
    std::string _activeSource=NO_CONTROL_SOURCE;
};
#endif