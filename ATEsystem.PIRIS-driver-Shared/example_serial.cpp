/******************************************
*
* Author:   Jakub Parez
* File:     example_serial.cpp
* Project:  ATEsystem.PIRIS-driver
* Version:  1.0.1
* Company:  ATEsystem s.r.o
* Date:     2018/12/10 11:29
* License:  WTFPL
* Require:  C++17, Pylon 5, GenICam, Serial
*
*******************************************/

#include <iostream>
#include <cstdlib>
#include <ctime>

#include "serial.h"

#include "ATEsystem.PIRIS.h"

#define DEMO_IS_SEQUENCE(x)     (x & 0x1)


int main_serial(int d);

extern void check_stat(ATEsystem_PIRIS::Status stat);               // validate status from driver
extern void check_err(ATEsystem_PIRIS::StatusEx stat);              // validate error cluster from controller
extern void prompt(std::string msg = "", std::string msg2 = "");    // prompt user for key press
extern uint16_t input_num(int init, int lo, int hi, std::string msg);           // prompt user to input integer (device number)
extern ATEsystem_PIRIS::Status action_wait_handler(ATEsystem_PIRIS::IPiris& piris);  // prompt user or poll
extern bool input_values(int& f, int& z, int& p, int& ir, ATEsystem_PIRIS::DataParams& data_param); // prompt user to input values


int main_serial(int d)
{
    srand((unsigned int)time(NULL));                // init random number generator (for example purposes)

    std::string foo;                                // dummy var
    int16_t dev_total = -1;                         // number of available devices from scan
    uint16_t dev_chosen = 0;                        // chosen number of scanned device to open after
    int repeats = 0;                                // how many set cycles
    ATEsystem_PIRIS::DevID_t dev_id;                // PIRIS device unique ID to use with Factory static methods
    ATEsystem_PIRIS::IPiris piris;                  // main handle, PIRIS smart pointer
    std::vector<serial::PortInfo> devices_serial;   // scanned serial PIRIS device nodes

    std::cout << "Choose device number:\n";
    if (0 < (dev_total = ATEsystem_PIRIS::CPirisMain::ScanSerial(devices_serial, true)))              // scan for PIRIS devices
        dev_chosen = input_num(-1, 0, dev_total - 1, "> ");                                           // choose device number
    else
        return 0;                                                                                     // if no device present exit


    dev_id = ATEsystem_PIRIS::Factory::getInstance()->CreateDevice(ATEsystem_PIRIS::Mode::SERIAL);    // create PIRIS device
    piris = ATEsystem_PIRIS::Factory::getInstance()->GetDeviceInstance(dev_id);                       // get device instance

    check_stat(piris->Open((ATEsystem_PIRIS::PirisDevice)(devices_serial[dev_chosen]),                // open PIRIS device
                            ATEsystem_PIRIS::VerboseLevel::FULL));


    auto[status_ex1, data_id] = piris->ReadID();                // read data_id has 1.) device name 2.) firmware version X.X.X
    check_err(status_ex1);

    auto[status_ex2, data_pos1] = piris->ReadPosition();        // read data_pos has 1.) current positions of focus, zoom, iris
    check_err(status_ex2);                                      // 2.) bool indicating ir filter open/close state

    auto[status_ex3, data_param] = piris->ReadParams();         // read data_param has 1.) max values of focus, zoom, iris
    check_err(status_ex3);                                      // 2.) lens type 3.) bool ir present 4.) bool rear sensor present 

    auto[status_ex4, data_state] = piris->ReadState();          // read data_state has 1.) bool values indicating state of focus,
    check_err(status_ex4);                                      // zoom, iris 2.) bool indicating motors are busy

    if (piris->GetFwPollSupport() == ATEsystem_PIRIS::YesNoNA::NO)
        std::cout << "\nThis firmware version is deprecated, therefore it lacks some features!!\n";

    uint16_t sequence[11][4] =                                  // demo sequence
    { 
        {0, 0, 0, 0}, 
        {data_param.max_value.getFocus(), 0, 0, 0},
        {0, 0, 0, 0},
        {0, data_param.max_value.getZoom(), 0, 0},
        {0, 0, 0, 0},
        {0, 0, data_param.max_value.getIris(), 0},
        {0, 0, 0, 0},
        {0, 0, 0, 1},
        {0, 0, 0, 0},
        {data_param.max_value.getFocus(), data_param.max_value.getZoom(), data_param.max_value.getIris(), 1},
        {0, 0, 0, 0}
    };

    check_err(piris->DevHoming());                  // initiate device homing - each motor reinit to default position
    check_stat(action_wait_handler(piris));         // wait for action finish

    if (DEMO_IS_SEQUENCE(d))
    {
        repeats = 11;
        prompt("Press ENTER to continue...\n", "The Demo Sequence will start now. ");
    }
    else
    {
        repeats = 1000;
        prompt("Press ENTER to continue...\n", "(enter any string to quit)\nThe User Input Demo will start now. ");
    }

    for (int i = 1, j = 0; i <= repeats; i++, j++)  // set random values of focus, zoom, iris, ir filter and read back positions
    {
        int f = -1, z = -1, p = -1, ir = -1; 
        std::cout << "-----------------------------------------------------------";

        if (DEMO_IS_SEQUENCE(d))
            std::cout << "\n[" << i << "/" << repeats << "]\n";
        else if (input_values(f, z, p, ir, data_param) == false)
            break;

        // set absolute values of focus, zoom, iris and ir filter. either random values or user defined
        check_err(piris->SetAbsolute((DEMO_IS_SEQUENCE(d)) ? sequence[j][0] : (uint16_t)f,
                                     (DEMO_IS_SEQUENCE(d)) ? sequence[j][1] : (uint16_t)z,
                                     (DEMO_IS_SEQUENCE(d)) ? sequence[j][2] : (uint16_t)p,
                                     (DEMO_IS_SEQUENCE(d)) ? (bool)sequence[j][3] : (bool)ir));

        check_stat(action_wait_handler(piris));                 // wait for action finish

        auto[status_ex5, data_pos2] = piris->ReadPosition();    // read back position to validate
        check_err(status_ex5);

        if (DEMO_IS_SEQUENCE(d))
            _Sleep(500);
    }

    check_err(piris->DevReset());                               // initiate device reset

    prompt("Press ENTER to continue...\n", "Wait for action finish and then ");

    check_stat(piris->Close());                                 // finally close piris device

    //ATEsystem_PIRIS::Factory::getInstance()->RemoveDevice(dev_id);    // remove only if add more devices (piris is smart pointer)

    return 0;
}

