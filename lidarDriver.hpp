#pragma once
#include "serialib.h"
#include <functional>

class LidarDriver
{
public:
    LidarDriver(const char* deviceUrl);
    ~LidarDriver();
    
    void startLoop( std::function<void(uint16_t* angles,uint16_t* distances)> func);
private:
    serialib m_serial;
    bool m_connected;
};