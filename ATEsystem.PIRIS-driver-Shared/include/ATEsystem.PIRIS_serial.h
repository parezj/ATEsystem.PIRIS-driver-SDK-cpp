/******************************************
*
* Author:   Jakub Parez
* File:     ATEsystem.PIRIS_serial.h
* Project:  ATEsystem.PIRIS-driver
* Version:  1.0.1
* Company:  ATEsystem s.r.o
* Date:     2018/12/6 18:30
* License:  WTFPL
* Require:  C++17, Serial
*
*******************************************/

#pragma once

#include <string>
#include <vector>

#include "serial.h"


#include "ATEsystem.PIRIS.h"

#define PIRIS_SERIAL_BAUDRATE       (uint32_t)9600
#define PIRIS_SERIAL_BITS           serial::eightbits
#define PIRIS_SERIAL_PARITY         serial::parity_none
#define PIRIS_SERIAL_STOPBITS       serial::stopbits_one
#define PIRIS_SERIAL_FLOW           serial::flowcontrol_none

#define PIRIS_SERIAL_MAX_RX         100
#define PIRIS_SERIAL_EOL            "\r\n"


namespace ATEsystem_PIRIS
{
    /// <summary>  
    /// Main final child class providing RS232 (Serial) communication functionality.
    /// </summary>
    class CPirisSerial final : public CPirisMain
    {
    public:

        static int16_t scan(std::vector<serial::PortInfo>& devices, bool verbose = false);
        static IDevice* create();

        CPirisSerial();
        virtual ~CPirisSerial();    
        virtual void remove();

    protected:

        virtual Status open(const PirisDevice& dev, VerboseLevel verbose = VerboseLevel::NONE);
        virtual Status close(bool ignore_err = false);
        virtual Status write(const std::string data);
        virtual Status read(std::string& data);
        virtual Status flush(bool ignore_err = false);

    private:

        serial::Serial* serial;
        
        Status dev_set_up();
    };
}