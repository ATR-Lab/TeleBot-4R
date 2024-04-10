#ifndef CONTROL_SOURCE_HANDLER_HPP
#define CONTROL_SOURCE_HANDLER_HPP

#ifndef NO_CONTROL_SOURCE
#define NO_CONTROL_SOURCE "no_source"
#endif

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
    int loadFromFile(const std::string&);
private:
    std::unordered_map<std::string,topicVector> _controlSources;
    topicVector _activeTopics;
    std::string _activeSource;
};
#endif