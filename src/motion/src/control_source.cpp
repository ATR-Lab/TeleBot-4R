#include "motion/control_source.hpp"
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <exception>
/// @brief Creates a control source object
/// @param id The unique idenftifier of the source
/// @param topics List of necessary topics to subscribe to
/// @param name Name of the control source, used largely for readability and help debugging
ControlSource::ControlSource(ControlSourceID id, std::list<std::string> topics, std::string name)
{
    _sourceID = id;
    _topics = topics;
    _name = name;
}

/// @brief Loads the source objects from file. Size min is 1 for only the not active source.
/// @param path Path to file containing the ControlSource objects
/// @return List of parsed control source objects
std::vector<ControlSource> ControlSourceHandler::loadSourcesFromFile(const std::string &path)
{
    std::vector<ControlSource> sources; // Static vector to hold parsed control sources

    std::ifstream inputFile(path);
    if (!inputFile.is_open())
    {
        std::cerr << "Error: Unable to open file " << path << std::endl;
        return sources; // Return an empty vector
    }

    sources.clear(); // Clear the vector in case this function is called multiple times

    std::string line;
    int lineNum = 1;
    while (std::getline(inputFile, line))
    {
        try
        {
            std::istringstream iss(line);

            char delimiter;
            int id;
            std::list<std::string> topics;
            std::string name;

            // Parse the line using the specified format
            if (iss >> id >> delimiter && delimiter == ':')
            {
                char comma;
                std::string topic;
                while (iss >> delimiter >> topic >> comma && delimiter == '"' && comma == ',')
                {
                    topics.push_back(topic);
                }

                std::string sourceName;
                if (iss >> delimiter >> sourceName && delimiter == '"' && iss.get() == '\n')
                {
                    sources.emplace_back(static_cast<ControlSourceID>(id), topics, sourceName);
                }
            }
        }
        catch (...)
        {
            std::cerr << "Failed to parse line" << lineNum << std::endl
                      << "Please verify that the control source config file is formatted properly.";
        }
        lineNum++;
    }

    inputFile.close();
    return sources;
}

// ControlSource Handler Code \/\/\/

/// @brief Initialize a control source handler from a file 
/// @param configFilePath Path where we load control sources from
ControlSourceHandler::ControlSourceHandler(const std::string &configFilePath)
{
    _sourceConfigPath = configFilePath;

    // Load sources, includes not-active source by default
    _controlSources = loadSourcesFromFile(_sourceConfigPath);

    // Set to not active 
    _activeSource = &_controlSources[NotActive];
}

/// @brief Reloads the file and updates any changes made to the sources
/// @return True if successfully refreshed, false otherwise
// bool ControlSourceHandler::refreshSources()
// {
//     try
//     {
//         _controlSources = loadSourcesFromFile(_sourceConfigPath);
//     }
//     catch (std::exception &e)
//     {
//         // Place holder for my exception that I will throw
//         // Log exception
//         return false;
//     }
//     catch (std::ifstream::failure &e)
//     {
//     }
//     return true;
// }

/// @brief Updates path to source config and loads sources from it
/// @param path New file path for the sources
/// @return True on successful update and refresh, false otherwise
// bool ControlSourceHandler::updateSourceConfigPath(std::string &path)
// { // Not expected to be needed really
//     _sourceConfigPath = path;
//     refreshSources();
// }

const ControlSource* ControlSourceHandler::getSourceById(const ControlSourceID sourceID)const
{
    return &_controlSources[sourceID];
}

void ControlSourceHandler::setActiveSource(ControlSourceID sourceID)
{
    _activeSource = &_controlSources[sourceID];
}