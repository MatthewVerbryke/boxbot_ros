// Serial connection interface class header
//
// Copyright 2022 University of Cincinnati
// All rights reserved. See LICENSE file at:
// https://github.com/MatthewVerbryke/rse_dam
// Additional copyright may be held by others, as reflected in the commit history.

#ifndef SERIALINTERFACE
#define SERIALINTERFACE

#include <numeric>
#include <string>
#include <vector>

#include <libserial/SerialPort.h>

using namespace LibSerial;

class SerialInterface
{
private:

    // General parameters
    std::string name;
    size_t timeout;
    
    // Serial port object
    SerialPort port;
    
    // Data storage variable
    unsigned char read_byte;
    
public:
    
    // Constructor
    SerialInterface();
    
    // Public Functions
    void setupPort(std::string port_name, int baud_rate, int ms_timeout);
    std::vector<int> readPacket();
    std::vector<int> execute(int id, int ins, std::vector<int> params);
    std::vector<int> read(std::vector<int> ids, int start, int length);
    int write(std::vector<int> values, int start, int num_bytes);
};

#endif
