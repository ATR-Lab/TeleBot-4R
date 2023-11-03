#include "motion/control_source_handler.hpp"
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <exception>

/// @brief Sets a given source name to active
/// @param sourceName The unique name of a control source
/// @return True on successfully setting it, false on failure
bool ControlSourceHandler::setActiveSource(const std::string& sourceName)
{
    auto res=_controlSources.find(sourceName);
    if(res!=_controlSources.end()){
        _activeSource=sourceName;
        _activeTopics=res->second;
        return true;
    }
    return false;

}

const ControlSourceResult& ControlSourceHandler::registerSource(const std::string& name,const topicVector& topics){
    //Check that we have an actual name
    if(name.length()<1){
        std::cerr<<"Registration Failed: Source name can't be empty";
        return ControlSourceResult::FAILED;
    }
    //Empty sources not allowed
    if(topics.size()<1){
        std::cerr<<"Registration Failed: You may not register a source with no topics";
        return ControlSourceResult::FAILED;
    }
    //Actually insert
    auto res=_controlSources.insert(std::make_pair(name,topics));
    try{
        //Check result of the insertion
        if(res.second){
            return ControlSourceResult::SUCCEEDED;
        }
        return ControlSourceResult::ALREADY_REGISTERED;
    }catch(...){
        return ControlSourceResult::FAILED;
    } 
}
