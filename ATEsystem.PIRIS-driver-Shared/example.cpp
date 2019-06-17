/******************************************
*
* Author:   Jakub Parez
* File:     example.cpp
* Project:  ATEsystem.PIRIS-driver
* Version:  1.0.1
* Company:  ATEsystem s.r.o
* Date:     2018/12/10 13:51
* License:  WTFPL
* Require:  C++17, Pylon 5, GenICam, Serial
*
*******************************************/


// printscreeny dema a popis + user inputu
// do anglictiny

#include <cstdlib>
#include <iostream>
#include <algorithm>
#include "ATEsystem.PIRIS.h"

#ifdef _MSC_VER
    #include <crtdbg.h>  
#endif

#define FLAG_NO_PYLON_GUI   "--no-gui" 


extern int main_ethernet(int d);                            // demo example code of Serial (UART) communication
extern int main_serial(int d);                              // demo example code of Ethernet (Basler ATE camera) communication

int8_t status = -20;                                        // catched error status

void check_stat(ATEsystem_PIRIS::Status stat);              // validate status from driver
void check_err(ATEsystem_PIRIS::StatusEx stat);             // validate error cluster from controller
void prompt(std::string msg = "", std::string msg2 = "");   // prompt user for key press
uint16_t input_num(int init, int lo, int hi, std::string msg);           // prompt user to input integer (device number)
bool arg_exists(char** begin, char** end, const std::string& option);    // check argument option exists
ATEsystem_PIRIS::Status action_wait_handler(ATEsystem_PIRIS::IPiris& piris);  // prompt user or poll
bool input_values(int& f, int& z, int& p, int& ir, ATEsystem_PIRIS::DataParams& data_param); // prompt user to input values


int main(int argc, char* argv[])
{
    #ifdef _MSC_VER
        _CrtSetReportMode(_CRT_ASSERT, 0);
    #endif
    std::cout << "This is ATEsystem P-IRIS Controller driver example.\n" <<
                 "www.atesystem.cz  produkty@atesystem.cz\n" <<
                 "===================================================\n";

    int ret = 0;
    int c = arg_exists(argv, argv + argc, FLAG_NO_PYLON_GUI) ? 0 : 0x2;
    int n = input_num(-1, 0, 1, "Choose device connection mode:\n  0 : Serial\n  1 : Ethernet\n> ");
    int d = input_num(-1, 0, 1, "Choose demo mode:\n  0 : User input\n  1 : Sequence\n> ");

    try
    {
        if (n == 0)
            ret = main_serial(d);       // call example with Serial (UART) communication
        else if (n == 1)
            ret = main_ethernet(d | c); // call example with Ethernet (Basler camera) communication
        else
            return ATEsystem_PIRIS::Status::UNDEFINED_ERROR;
    }
    catch (...) 
    { 
        ret = status; 
    }

    prompt("Press the Enter key to continue.");
    return ret;
}

// ******************** helper functions *************************

void check_stat(ATEsystem_PIRIS::Status stat)
{
    if (stat != ATEsystem_PIRIS::Status::SUCCESS)
    {
        std::cerr << "Program terminated due to: " << stat;
        status = (int8_t)status;
        throw stat;
    }
}

void check_err(ATEsystem_PIRIS::StatusEx stat)
{
    if (!stat.getOk() && stat.getErrNum() > 0 && stat.getErrNum() < 100)
    {
        std::cerr << "Error " << stat.getErrNum() << " : " << stat.getMsg() << std::endl;
    }
    check_stat(stat.status);
}

uint16_t input_num(int init, int lo, int hi, std::string msg)
{
    int ret = init;
    while (ret < lo || ret > hi)
    {
        std::cout << std::endl << msg;
        std::cin >> ret;
        std::cout << std::endl;
    }
    return (uint16_t)ret;
}

void prompt(std::string msg, std::string msg2)
{
    //std::cin.ignore(INT_MAX);
    //fgetc(stdin);
    std::cout << std::endl;
    if (!msg2.empty())
        std::cout << msg2;
#ifdef _WIN32
    system("pause");
#else //__linux__
    std::string foo;
    std::cout << msg;
    //fgetc(stdin);
    std::cin.ignore();
    //std::cin.get();
    std::getline(std::cin, foo);
#endif
    std::cout << std::endl;
}

ATEsystem_PIRIS::Status action_wait_handler(ATEsystem_PIRIS::IPiris& piris)
{
    if (piris != nullptr && piris->GetFwPollSupport() == ATEsystem_PIRIS::YesNoNA::YES)
    {
        ATEsystem_PIRIS::Status ret = ATEsystem_PIRIS::Status::RX_TIMEOUT;
        ATEsystem_PIRIS::VerboseLevel verbose = piris->GetVerboseLevel();
        piris->SetVerboseLevel(ATEsystem_PIRIS::VerboseLevel::NONE);
        std::cout << "Wait for action finish...";

        int timeout = 20000;
        int i = 0;
        while (timeout > 0)
        {
            auto[stat, dat] = piris->ReadState();
            if (!dat.motors_busy)
            {
                ret = ATEsystem_PIRIS::Status::SUCCESS;
                break;
            }  

            if (i >= 1)
            {
                i = 0;
                std::cout << ".";
            }
            else i++;

            _Sleep(200);
            timeout -= 200;
        }
        std::cout << "\n\n";
        piris->SetVerboseLevel(verbose);
        return ret;
    }
    else
    {
        //std::string foo;
        //prompt("Press ENTER to continue...\n", "Wait for action finish and then ");
        //std::cout << "\n\n";
        return ATEsystem_PIRIS::Status::SUCCESS;
    }
}

bool input_values(int& f, int& z, int& p, int& ir, ATEsystem_PIRIS::DataParams& data_param)
{
    int o = 0;
    while (f < 0 || f > data_param.max_value.getFocus() ||
        z < 0 || z > data_param.max_value.getZoom() ||
        p < 0 || p > data_param.max_value.getIris() ||
        ir < 0 || ir > 1)
    {
        try
        {
            if (o > 0)
                std::cout << "Out of range!\n";

            std::cout << std::endl << "Enter Focus <0;" << data_param.max_value.getFocus() <<
                "> Zoom <0;" << data_param.max_value.getZoom() <<
                "> Iris <0;" << data_param.max_value.getIris() <<
                "> and IR <0;1> in format: 0 0 0 0:\n> ";

            int h;
            std::vector<int> tokens;

            while (tokens.size() < 4 && std::cin >> h)
                tokens.push_back(h);

            if (std::cin.fail())
            {
                std::cout << "\n------------------------ DEMO QUIT ------------------------\n";
                return false;
            }

            if (tokens.size() < 4)
                f = -1;
            else
            {
                f = tokens[0];
                z = tokens[1];
                p = tokens[2];
                ir = tokens[3];
            }
            std::cout << std::endl;
        }
        catch (...)
        {
            f = -1;
        }

        o++;
    }
    return true;
}

bool arg_exists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}

//uint16_t rnd_i(uint16_t max, uint16_t min)
//{
//    return min + (rand() % (uint16_t)(max - min + 1));
//}
//bool rnd_b()
//{
//    return (rand() % 2) == 0;
//}