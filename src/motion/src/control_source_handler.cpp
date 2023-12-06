#include "motion/control_source_handler.hpp"
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <exception>
#include <stdexcept>
#include <algorithm>
#include <iterator>
/// @brief Sets a given source name to active
/// @param sourceName The unique name of a control source
/// @return True on successfully setting it, false on failure
bool ControlSourceHandler::setActiveSource(const std::string &sourceName)
{
    auto res = _controlSources.find(sourceName);
    if (res != _controlSources.end())
    {
        _activeSource = sourceName;
        _activeTopics = res->second;
        return true;
    }
    return false;
}

ControlSourceResult ControlSourceHandler::registerSource(const std::string &name, const topicVector &topics)
{
    // Check that we have an actual name
    if (name.length() < 1)
    {
        std::cerr << "Registration Failed: Source name can't be empty";
        return ControlSourceResult::FAILED;
    }
    // Empty sources not allowed
    if (topics.size() < 1)
    {
        std::cerr << "Registration Failed: You may not register a source with no topics";
        return ControlSourceResult::FAILED;
    }
    // Actually insert
    auto res = _controlSources.insert(std::make_pair(name, topics));
    try
    {
        // Check result of the insertion
        if (res.second)
        {
            return ControlSourceResult::SUCCEEDED;
        }
        return ControlSourceResult::ALREADY_REGISTERED;
    }
    catch (...)
    {
        return ControlSourceResult::FAILED;
    }
}

bool ControlSourceHandler::loadFromFile(const std::string &controlSourceFilePath)
{
    std::ifstream sourcesFile(controlSourceFilePath);
    // Check if the file is opened successfully
    if (!sourcesFile.is_open())
    {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    // Read the file character by character
    // char ch;
    // std::string source;
    // topicVector topics;
    // std::string workingTopic;
    // bool sourceParsed=false;

    char buf[1000];
    while (sourcesFile.getline(buf, 1000))
    {
        std::string line(buf);
        auto colonPos = line.find(':');
        if (colonPos != std::string::npos)
        {
            auto source = line.substr(0, colonPos);
            // Remove leading whitespace
            source.erase(source.begin(), std::find_if(source.begin(), source.end(), [](unsigned char ch)
                                                      { return !std::isspace(ch); }));

            // Split the string by spaces after the first colon
            std::istringstream iss(line.substr(colonPos + 1));
            std::vector<std::string> topics{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};

            registerSource(source,topics);
        }
    }

    // while (!sourcesFile.eof()) {
    //     //Get char
    //     sourcesFile.get(ch);
    //     // Colon represents end of topic name
    //     sourceParsed=(ch==':');
    //     switch (ch){
    //         //When newline encountered, register source
    //         case '\n':{
    //             //If we have an empty topic, throw error
    //             if(workingTopic.length()==0){
    //                 std::logic_error e("Failed to parse control sources! Please ensure no whitespace is after your final topic in the control source file.");
    //                 throw e;
    //             }
    //             //Register source and prepare for new source
    //             topics.push_back(workingTopic);
    //             registerSource(source,topics);

    //             //Reset state
    //             sourceParsed=false;
    //             workingTopic.clear();
    //             source.clear();
    //             topics.clear();
    //             break;
    //         }
    //         case '\t':{
    //             //Ignore tabs too
    //         }
    //         case ' ':{
    //             //If space add topic to list
    //             if (sourceParsed){
    //                 topics.push_back(workingTopic);
    //                 workingTopic.clear();
    //             }
    //             break;
    //         }
    //         default:{
    //             if (sourceParsed){
    //                 workingTopic+=ch;
    //             }
    //             else{
    //                 source+=ch;
    //             }
    //         }

    //     }
    // }

    sourcesFile.close();
}