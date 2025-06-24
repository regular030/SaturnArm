#include "wifi_util.h"
#include <fstream>
#include <iostream>
#include <regex>

bool WiFiUtil::get_credentials(std::string& ssid, std::string& password) {
    const char* config_file = "/etc/wpa_supplicant/wpa_supplicant.conf";
    std::ifstream file(config_file);
    
    if (!file.is_open()) {
        std::cerr << "ERROR: Could not open WiFi config: " << config_file << std::endl;
        return false;
    }

    std::regex ssid_regex(R"(ssid\s*=\s*\"([^\"]+)\")");
    std::regex psk_regex(R"(psk\s*=\s*\"([^\"]+)\")");
    std::string line;
    bool found_ssid = false;
    bool found_psk = false;

    while (std::getline(file, line)) {
        std::smatch match;
        
        if (!found_ssid && std::regex_search(line, match, ssid_regex)) {
            ssid = match[1];
            found_ssid = true;
        }
        else if (!found_psk && std::regex_search(line, match, psk_regex)) {
            password = match[1];
            found_psk = true;
        }
        
        if (found_ssid && found_psk) break;
    }

    file.close();
    
    if (!found_ssid || !found_psk) {
        std::cerr << "ERROR: WiFi credentials not found in config file" << std::endl;
        return false;
    }
    
    std::cout << "Retrieved WiFi credentials for: " << ssid << std::endl;
    return true;
}
