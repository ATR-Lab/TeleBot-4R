#ifndef CONTROL_SOURCE_HANDLER_HPP
#define CONTROL_SOURCE_HANDLER_HPP
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>
#include <list>

typedef std::vector<std::string> topicVector;

enum ControlSourceResult{
    FAILED,
    SUCCEEDED,
    ALREADY_REGISTERED
};

const char* NO_CONTROL_SOURCE="no_source";

class ControlSourceHandler{
public:
    ControlSourceHandler(){
        //Initialize the default empty source
        _controlSources.insert(std::make_pair(NO_CONTROL_SOURCE,_activeTopics));
        _activeSource=NO_CONTROL_SOURCE;
    }//Done

    const std::string getActiveSourceName()const{return _activeSource;}//Done
    const topicVector getActiveSourceTopics()const{return _activeTopics;}//Done

    //Setters
    bool setActiveSource(const std::string&);//Done
    ControlSourceResult registerSource(const std::string&,const topicVector&);
private:
    std::unordered_map<std::string,topicVector> _controlSources;
    topicVector _activeTopics;
    std::string _activeSource;
};
#endif