#pragma once
#include <string>

class WiFiUtil {
public:
    static bool get_credentials(std::string& ssid, std::string& password);
};
