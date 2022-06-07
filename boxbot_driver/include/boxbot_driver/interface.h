// Serial connection interface class header
//
// Copyright 2022 University of Cincinnati
// All rights reserved. See LICENSE file at:
// https://github.com/MatthewVerbryke/rse_dam
// Additional copyright may be held by others, as reflected in the commit history.
//
// TODO: needs to be tested and debugged

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
    SerialInterface(std::string port_name, int baud_rate, size_t time_out);
    
    // Public Functions
    std::vector<int> readPacket();
    std::vector<int> execute(int id, int ins, std::vector<int> params, bool ret);
};

#endif
