// Serial connection interface class file
//
// Copyright 2022 University of Cincinnati
// All rights reserved. See LICENSE file at:
// https://github.com/MatthewVerbryke/rse_dam
// Additional copyright may be held by others, as reflected in the commit history.
//
// TODO: needs to be tested and debugged

#include <iostream>
#include <numeric>
#include <vector>
#include <unistd.h>

#include "boxbot_driver/interface.h"
#include "boxbot_driver/control_table.h"

// Constructor
SerialInterface::SerialInterface(std::string port_name, int baud_rate, size_t ms_timeout){
    
    // Store input parameters
    name = port_name;
    timeout = ms_timeout;
    
    // Try to open the desired serial port
    try{
        port.Open(port_name);
    }
    catch(const OpenFailed&){
        std::cerr << "Failed to open serial port at '" << port_name << "'" << std::endl;
    }
    
    // Set the baudrate
    if (baud_rate == 115200){
        port.SetBaudRate(BaudRate::BAUD_115200);
    }
    // NOTE: Add other bauds here as nessecary
    else{
        std::cerr << "Invalid or uncoded baud rate entered" << std::endl;
    }
    
    // Set data bytr size to 8 bits
    port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    
    // Configure hardware flow control
    port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
    
    // Configure parity
    port.SetParity(Parity::PARITY_NONE);
    
    // Set number of stop bits
    port.SetStopBits(StopBits::STOP_BITS_1);
}

// Read a message packet using the Dynamixel Protocol 1.0
std::vector<int> SerialInterface::readPacket(){
    
    // Initialize variables
    int mode = 0;
    bool fail = false;
    int read_int;
    int id;
    int length;
    int error;
    std::vector<int> params;
    int checksum;
        
    // Iteratively process the serial message
    while(mode != 7){
    
        // Read in the next byte
        try{
            port.ReadByte(read_byte, time_out);
            read_int = int(read_byte);
        }
        catch(const ReadTimeout&){
            std::cerr << "Read command has timed out" << std::endl;
        }
        
        // Retreive first header byte
        if (mode == 0){
            if (read_int == 255){
                mode = 1;
                std::cout << "First header retrieved" << std::endl;
            }
        }
            
        // Retreive second header byte
        else if (mode == 1){
            if (read_int == 255){
                mode = 2;
                std::cout << "Second header retrieved" << std::endl;
            }
            else{
                mode = 0;
            }
        }
        
        // Retreive ID number
        else if (mode == 2){
            if (read_int != 255){
                mode = 3;
                id = read_int;
                std::cout << "ID: " << id << std::endl;
            }
            else{
                mode = 0;
            }
        }
        
        // Retreive packet length
        else if (mode == 3){
            length = read_int;
            std::cout << "length: " << length << std::endl;
            mode = 4;
        }
    
        // Retreive error byte
        else if (mode == 4){
            error = read_int;
            std::cout << "error: " << error << std::endl;
            if (length == 2){
                mode = 6;
            }
            else{
                mode = 5;
            }
        }
        
        // Retreive a parameter
        else if (mode == 5){
            params.push_back(read_int);
            std::cout << "param: " << read_int << std::endl;
            if (params.size() + 2 == length){
                mode = 6;
            }
        }
        
        // Retrieve checksum
        else if (mode == 6){
            int param_sum = std::accumulate(params.begin(),params.end(),0);
            checksum = id + length + error + param_sum + read_int;
            std::cout << "checksum: " << checksum << std::endl;
            mode = 7;
            if (checksum % 256 != 255){
                fail = true;
            }
        }
        
        // Handle returning the message parameters
        if (fail == true){
            return {};
        }
    }
    
    return params;
}

// Send a message to the Arbotix and read the return message
std::vector<int> SerialInterface::execute(int id_in, int ins_in, std::vector<int> params_in){
    
    // Determine main parameters for write packet
    int num_params = params_inn.size();
    int sum_params = std::accumulate(params_in.begin(), params_in.end(), 0);
    int length_in = num_params + 2;
    int packet_length = length_in + 4;
    int checksum = 255 - ((id_in + length_in + ins_in + sum_params)%256);
    
    // Convert inputs into unsigned characters
    unsigned char id = id_in;
    unsigned char ins = ins_in;
    unsigned char length = length_in;
    unsigned char header = 255;
    
    // Create array for message chars
    unsigned char msg[packet_length]{header, header, id, length, ins};
    
    // Insert message params
    for (int i{0}; i < num_params; ++i){
        unsigned char param = params_in[i];
        msg[5 + i] = param;
    }
    
    // Insert checksum
    unsigned char checksum_char = checksum;
    msg[packet_length - 1] = checksum_char;
    
    // Write each component to the serial port
    for (int j{0}; j < packet_length; ++j){
        port.WriteByte(msg[j]);
        port.DrainWriteBuffer();
    }
    
    // Look for the response from the Arbotix
    std::vector<int> values = readPacket();
    
    return values;
}
