#include "node_utils/node_utils.hpp"
#include <iostream>
#include <cstdlib>
#include <sstream>
#include <string>

namespace NodeUtils{


/// @brief  NOT INTENDED FOR USE, MEANT TO BE AN INTERNAL HELPER FUNCTION IN THE NAMESPACE. USE getPortBySerialID
/// @param udevInfoOutput Output to search for serial ID
/// @return Serial id for a given port
std::string getSerialNumber(const std::string& udevInfoOutput) {
    std::istringstream iss(udevInfoOutput);
    std::string line;

    while (std::getline(iss, line)) {
        size_t pos = line.find("ID_SERIAL_SHORT=");
        if (pos != std::string::npos) {
            return line.substr(pos + 16); // Length of "ID_SERIAL_SHORT=" is 16
        }
    }

    return ""; // Return empty string if not found
}

// Function to search for a device by serial number
// std::string getPortBySerialID(const std::string& serialID) {
//     // Build the command string
//     std::string command = "/bin/udevadm info --name=/dev/ttyUSB*";

//     // Execute the command and capture the output
//     FILE* pipe = popen(command.c_str(), "r");
//     if (!pipe) {
//         std::cerr << "Error running udevadm info" << std::endl;
//         return "";
//     }

//     // Read udevadm info output
//     char buffer[128];
//     std::string udevInfoOutput;
//     while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
//         udevInfoOutput += buffer;
//     }

//     // Close the pipe
//     pclose(pipe);

//     // Iterate through each matched path
//     std::istringstream pathsStream(udevInfoOutput);
//     std::string path;
//     while (std::getline(pathsStream, path, '\0')) {
//         // Extract serial number from udevadm info output
//         std::string deviceSerialNumber = getSerialNumber(path);

//         // Check if the serial number matches the target
//         if (deviceSerialNumber == serialID) {
//             // Output the serial number and device path
//             std::cout << "Serial: " << deviceSerialNumber << " | Path: " << path << std::endl;
//             return path; // Stop searching after finding the matching device
//         }
//     }

//     // If the function reaches this point, the device with the specified serial number was not found
//     std::cerr << "Device with serial number " << serialID << " not found." << std::endl;
//     return "";
// }

/// @brief Searches all ports for a specific serial Device
/// @param serialID The serial id to search for
/// @return The associated port. Eg /dev/ttyUSB1
std::string getPortBySerialID(const std::string& serialID){
    // Initialize glob structure
    glob_t globResult;
    int globStatus = glob("/dev/ttyUSB*", 0, nullptr, &globResult);

    // Check for errors in glob
    if (globStatus != 0) {
        std::cerr << "Error in glob: " << globStatus << std::endl;
        return "";
    }

    // Iterate through each matched path
    for (size_t i = 0; i < globResult.gl_pathc; ++i) {
        // Get the path of the device
        std::string devicePath = globResult.gl_pathv[i];

        // Build the command string
        std::string command = "/bin/udevadm info --name=" + devicePath;

        // Execute the command and capture the output
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) {
            std::cerr << "Error running udevadm info for " << devicePath << std::endl;
            continue;
        }

        // Read udevadm info output
        char buffer[128];
        std::string udevInfoOutput;
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            udevInfoOutput += buffer;
        }

        // Close the pipe
        pclose(pipe);

        // Extract serial number from udevadm info output
        std::string serialNumber = getSerialNumber(udevInfoOutput);

        // Check if the serial number is in the command-line arguments
        
        if (serialNumber == serialID) {
                // Output the serial number and device path
                std::cout << "Serial: " << serialNumber << " | Path: " << devicePath << std::endl;
                return devicePath;
        }
        
    }

    // Free allocated resources
    globfree(&globResult);
    std::cerr << "Device with serial number " << serialID << " not found." << std::endl;
    return "";
}
}