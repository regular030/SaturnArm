#include "wifi_util.h"
#include <fstream>
#include <iostream>
#include <regex>

bool WiFiUtil::get_credentials(std::string& ssid, std::string& password) {
    const char* config_file = "/etc/wpa_supplicant/wpa_supplicant.conf";
    std::ifstream file(config_file);
    
    if (!file.is_open()) {
        std::cerr << "Error opening WiFi config file: " << config_file << std::endl;
        return false;
    }

    std::regex ssid_regex("ssid=\"([^\"]+)\"");
    std::regex psk_regex("psk=\"([^\"]+)\"");
    std::string line;
    bool found = false;

    while (std::getline(file, line)) {
        std::smatch match;
        
        if (std::regex_search(line, match, ssid_regex)) {
            ssid = match[1];
            found = true;
        }
        else if (std::regex_search(line, match, psk_regex)) {
            password = match[1];
        }
    }

    file.close();
    return found && !password.empty();
}
