/******************************************
*
* Author:   Jakub Parez
* File:     ATEsystem.PIRIS_ethernet.h
* Project:  ATEsystem.PIRIS-driver
* Version:  1.0.1
* Company:  ATEsystem s.r.o
* Date:     2018/11/23 12:30
* License:  WTFPL
* Require:  C++17, Pylon 5, GenICam
*
*******************************************/

#pragma once

#include <string>
#include <vector>

#include "pylon/PylonIncludes.h"

#include "ATEsystem.PIRIS.h"

#define PIRIS_ETH_RX_ITER_DELAY       50
#define PIRIS_ETH_TX_BUF_FUL_WAIT     100


namespace ATEsystem_PIRIS
{
    /// <summary>  
    /// Main final child class providing Ethernet communication functionality.
    /// </summary>
    class CPirisEthernet final : public CPirisMain
    {
    public:

        static int16_t scan(std::vector<Pylon::CDeviceInfo>& devices, bool verbose = false);
        static IDevice* create();

        CPirisEthernet();
        virtual ~CPirisEthernet();    
        virtual void remove();

    protected:

        virtual Status open(const PirisDevice& dev, VerboseLevel verbose = VerboseLevel::NONE);
        virtual Status close(bool ignore_err = false);
        virtual Status write(const std::string data);
        virtual Status read(std::string& data);
        virtual Status flush(bool ignore_err = false);


    private:

        bool cam_overiden;

        Pylon::IPylonDevice* pDevice;
        Pylon::CInstantCamera* pCamera;

        GenApi::CIntegerPtr SerialIOControl;
        GenApi::CIntegerPtr SerialIOStatus;
        GenApi::CIntegerPtr SerialIODataOut;
        GenApi::CCommandPtr SerialIODataOutExecute;
        GenApi::CBooleanPtr SerialIOStatusDataOutBufferFull;
        GenApi::CBooleanPtr SerialIOStatusDataOutBufferEmpty;
        GenApi::CIntegerPtr SerialIODataIn;
        GenApi::CCommandPtr SerialIODataInExecute;
        GenApi::CBooleanPtr SerialIOStatusDataInBufferFull;
        GenApi::CBooleanPtr SerialIOStatusDataInBufferEmpty;
        
        Status dev_set_up();
    };
}