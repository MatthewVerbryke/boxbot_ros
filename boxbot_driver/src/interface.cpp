// Serial connection interface class file
//
// Copyright 2022 University of Cincinnati
// All rights reserved. See LICENSE file at:
// https://github.com/MatthewVerbryke/rse_dam
// Additional copyright may be held by others, as reflected in the commit history.

#include <iostream>
#include <numeric>
#include <vector>
#include <unistd.h>
#include <typeinfo>

#include "boxbot_driver/interface.h"
#include "boxbot_driver/control_table.h"

// Empty Constructor
SerialInterface::SerialInterface(){}

// Input parameters and setup the serial ports (compiles better than direclty in constructor)
void SerialInterface::setupPort(std::string port_name, int baud_rate, int ms_timeout){
    
    // Store input parameters
    name = port_name;
    timeout = static_cast<size_t>(ms_timeout);
    
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
            port.ReadByte(read_byte, timeout);
            read_int = int(read_byte);
        }
        catch(const ReadTimeout&){
            std::cerr << "Read command has timed out" << std::endl;
        }
        
        // Retreive first header byte
        if (mode == 0){
            if (read_int == 255){
                mode = 1;
                //std::cout << "First header retrieved" << std::endl;
            }
        }
            
        // Retreive second header byte
        else if (mode == 1){
            if (read_int == 255){
                mode = 2;
                //std::cout << "Second header retrieved" << std::endl;
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
                //std::cout << "ID: " << id << std::endl;
            }
            else{
                mode = 0;
            }
        }
        
        // Retreive packet length
        else if (mode == 3){
            length = read_int;
            //std::cout << "length: " << length << std::endl;
            mode = 4;
        }
    
        // Retreive error byte
        else if (mode == 4){
            error = read_int;
            //std::cout << "error: " << error << std::endl;
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
            //std::cout << "param: " << read_int << std::endl;
            if (params.size() + 2 == length){
                mode = 6;
            }
        }
        
        // Retrieve checksum
        else if (mode == 6){
            int param_sum = std::accumulate(params.begin(),params.end(),0);
            checksum = id + length + error + param_sum + read_int;
            //std::cout << "checksum: " << checksum << std::endl;
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
std::vector<int> SerialInterface::execute(int id, int ins, std::vector<int> params){
    
    // Determine main parameters for write packet
    int num_params = params.size();
    int sum_params = std::accumulate(params.begin(), params.end(), 0);
    int length = num_params + 2;
    int packet_length = length + 4;
    int checksum = 255 - ((id + length + ins + sum_params)%256);
    
    // Convert inputs into unsigned characters
    unsigned char id_c = id;
    unsigned char ins_c = ins;
    unsigned char length_c = length;
    unsigned char header_c = 255;
    
    // Create array for message chars
    unsigned char msg[packet_length]{header_c, header_c, id_c, length_c, ins_c};
    
    // Insert message params
    for (int i{0}; i < num_params; ++i){
        unsigned char param_c = params[i];
        msg[5 + i] = param_c;
    }
    
    // Insert checksum
    unsigned char checksum_c = checksum;
    msg[packet_length - 1] = checksum_c;
    
    // Write each component to the serial port
    for (int j{0}; j < packet_length; ++j){
        port.WriteByte(msg[j]);
        port.DrainWriteBuffer();
    }
    
    // Look for the response from the Arbotix
    std::vector<int> values = readPacket();
    
    return values;
}

// Read the values of a registered
std::vector<int> SerialInterface::read(std::vector<int> servos, int start, int length){
    
    // Prepare for read operation
    std::vector<int> params = {start, length};
    params.insert(params.end(), servos.begin(), servos.end());

    // Execute the sync read command
    std::vector<int> values = execute(254, AX_SYNC_READ, params);
    
    return values;
}

// Write the values to a register
int SerialInterface::write(std::vector<int> values, int start, int num_bytes){
    
    // Flush the input buffer
    port.FlushInputBuffer();

    // Convert inputs into unsigned characters
    unsigned char header_c = 255;
    unsigned char address_c = 254;
    int length = values.size() + 4;
    unsigned char length_c = length;    
    unsigned char ins_c = AX_SYNC_WRITE;
    
    // Create the message packet for chars
    int packet_length = length + 4;
    unsigned char msg[packet_length]{header_c, header_c, address_c, length_c, ins_c};
    
    // Insert start address and individual message size
    unsigned char start_c = start;
    unsigned char num_bytes_c = num_bytes;
    msg[5] = start_c;
    msg[6] = num_bytes_c;
    
    // Insert values to write to address
    for (int i{0}; i < values.size(); ++i){
        unsigned char value_c = values[i];
        msg[7 + i] = value_c;
    }
    
    // Insert message Checksum
    int val_sum = std::accumulate(values.begin(),values.end(),0);
    int checksum = 255 - ((254 + length + AX_SYNC_WRITE + start + num_bytes + val_sum)%256);
    unsigned char checksum_c = checksum;
    msg[packet_length-1] = checksum_c;  
    
    // Write each component to the serial port
    for (int j{0}; j < packet_length; ++j){
        port.WriteByte(msg[j]);
        port.DrainWriteBuffer();
    }
    
    // TODO: Replace with returning the error level
    return 0;
}
