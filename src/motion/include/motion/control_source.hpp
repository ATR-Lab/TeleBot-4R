#ifndef CONTROL_SOURCES_HPP
#define CONTROL_SOURCES_HPP

#include <unordered_map>
#include <string.h>
#include <vector>
#include <list>

//These ids should be updated along with the config file in order to add any control source into multiplexer
enum ControlSourceID{
    NotActive=-1,
    MinibotTeleop,
    GUIControl,
    LAST
};

class ControlSource{
private:
    //These values are only set through constructor, all other accesses should be read only
    ControlSourceID _sourceID=NotActive;
    std::list<std::string> _topics;
    std::string _name;
public:
    // Constructors
    /// @brief Constructs an inactive control source with no topics or name
    ControlSource(){}
    ControlSource(ControlSourceID,std::list<std::string>,std::string);

    //Getters
    const ControlSourceID getID()const{return _sourceID;}
    const std::list<std::string> getTopics()const{return _topics;}
    const std::string getName()const{return _name;}
};


class ControlSourceHandler{
private:
    std::vector<ControlSource> loadSourcesFromFile(const std::string&); //Done, untested
    const ControlSource* getSourceById(const ControlSourceID)const; //Done
    std::string _sourceConfigPath; //Reflects the last loads filepath.
    std::vector<ControlSource> _controlSources;
    ControlSource* _activeSource=nullptr;
    
public:

    ControlSourceHandler(const std::string&);//Done

    //Getters
    std::string getConfigPath()const {return _sourceConfigPath;}//Done
    const ControlSourceID getActiveSourceID()const{return _activeSource->getID();}//Done
    const std::string getActiveSourceName()const{return _activeSource->getName();}//Done
    const std::list<std::string> getActiveSourceTopics()const{return _activeSource->getTopics();}//Done
    
    //Setters
    // bool refreshSources();//Needs done
    // bool updateSourceConfigPath(std::string&);//needs done
    void setActiveSource(ControlSourceID sourceID);//Done
};


#endif