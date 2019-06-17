/******************************************
*
* Author:   Jakub Parez
* File:     ATEsystem.PIRIS_ethernet.cpp
* Project:  ATEsystem.PIRIS-driver
* Version:  1.0.1
* Company:  ATEsystem s.r.o
* Date:     2019/04/24 13:52
* License:  WTFPL
* Require:  C++17, Pylon 5, GenICam
*
*******************************************/

#include <iostream>

#include "pylon/PylonIncludes.h"

#include "ATEsystem.PIRIS.h"
#include "ATEsystem.PIRIS_ethernet.h"


namespace ATEsystem_PIRIS
{
    CPirisEthernet::CPirisEthernet()
    {
        Pylon::PylonInitialize();
    }

    CPirisEthernet::~CPirisEthernet()
    {
        //if (!this->cam_overiden)
        //{
            try
            {
                this->close(true);
            }
            catch (...) {}
        //}

        // Releases all pylon resources. 
        Pylon::PylonTerminate();

        //delete this;
    }
    
    int16_t CPirisEthernet::scan(std::vector<Pylon::CDeviceInfo>& devices, bool verbose)
    {
        try
        {
            Pylon::PylonInitialize();
            Pylon::CTlFactory& TlFactory = Pylon::CTlFactory::GetInstance();
            Pylon::DeviceInfoList_t lstDevices;
            TlFactory.EnumerateDevices(lstDevices);
            devices.clear();

            if (!lstDevices.empty())
            {
                int16_t ret = 0;
                Pylon::DeviceInfoList_t::const_iterator it = lstDevices.begin();
                for (uint16_t i = 0; it != lstDevices.end(); i++, it++)
                {
                    if (it->GetFullName().find("ATE") != std::string::npos)
                    {
                        devices.push_back(lstDevices[i]);
                        if (verbose)
                        {
                            std::cout << "  " << ret << ". " << it->GetFullName();
                            if (i != lstDevices.size() - 1) 
                                std::cout << std::endl;
                        }
                            
                        ret++;
                    }
                }

                return ret;
            }
            else
            {
                if (verbose) 
                    std::cout << "No devices found!" << std::endl;

                return 0;
            }
        }
        catch (...) 
        { 
            if (verbose)
                std::cerr << Status::SCAN_FAILED << std::endl;

            return Status::SCAN_FAILED; 
        }
    }

    IDevice* CPirisEthernet::create()        // ? CPirisMain
    {
        return new CPirisEthernet();
    }

    void CPirisEthernet::remove()
    {
        delete this;
    }

    Status CPirisEthernet::open(const PirisDevice& dev, VerboseLevel verbose)
    {
        uint8_t fail = 0;
        this->verbose_set(verbose);

        if (dev.get_mode() != Mode::ETHERNET)
        {
            if (this->verbose_is(VerboseLevel::FULL))
                std::cerr << Status::WRONG_MODE << std::endl;

            return Status::WRONG_MODE;
        }

        if (this->state_is(State::BUSY))
        {
            if (this->verbose_is(VerboseLevel::FULL))
                std::cerr << Status::DEVICE_IS_BUSY << std::endl;

            return Status::DEVICE_IS_BUSY;
        }

        if (!this->state_is(State::CLOSED))
        {
            if (this->verbose_is(VerboseLevel::FULL))
                std::cerr << Status::ALREADY_OPEN << std::endl;

            return Status::ALREADY_OPEN;
        }

        this->state_set(State::BUSY);
        try
        {
            // camera and pylon dev is created, opened and closed outside this driver
            if (dev.get_type() == PirisDeviceType::ETHERNET_CAMERA && 
                dev.get_camera() != nullptr)
            {
                this->cam_overiden = true;
                this->pDevice = nullptr;
                this->pCamera = (Pylon::CInstantCamera*)dev.get_camera();
            }
            // pylon dev and camera is created, opened and closed outside this driver
            else if (dev.get_type() == PirisDeviceType::ETHERNET_PYLON && 
                     dev.get_pylon_dev() != nullptr)
            {
                this->cam_overiden = true;
                this->pDevice = (Pylon::IPylonDevice*)dev.get_pylon_dev();
                this->pCamera = new Pylon::CInstantCamera(this->pDevice);
            }
            // pylon dev and camera lifetime is managed by this driver only
            else if (dev.get_type() == PirisDeviceType::ETHERNET_DEVICE)
            {    
                this->cam_overiden = false;
                Pylon::CTlFactory& TlFactory = Pylon::CTlFactory::GetInstance();
                this->pDevice = TlFactory.CreateDevice(dev.get_device_eth());
                this->pCamera = new Pylon::CInstantCamera(this->pDevice);
            }
            else
            {
                if (this->verbose_is(VerboseLevel::FULL))
                    std::cerr << Status::INVALID_DEVICE << std::endl;
                return Status::INVALID_DEVICE;
            }

            if (!this->pCamera->IsOpen())
                this->pCamera->Open();
            if (!this->pCamera->IsOpen())
                throw std::runtime_error("unknown error");
        }
        catch (const std::exception &ex)
        {
            if (this->verbose_is(VerboseLevel::FULL))
            {
                std::cerr << Status::OPEN_FAILED << " : " << ex.what() << std::endl;
                fail = 1;
            }
            else fail = 2;
        }
        catch (const GenICam::GenericException &ex)
        {
            if (this->verbose_is(VerboseLevel::FULL))
            {
                std::cerr << Status::OPEN_FAILED << " : " << ex.what() << std::endl;
                fail = 1;
            }
            else fail = 2;
        }
        catch (...)
        {
            fail = 2;
        }

        if (fail > 0)
        {
            if (fail == 2 && this->verbose_is(VerboseLevel::BASIC))
                std::cerr << Status::OPEN_FAILED << std::endl;

            this->state_set(State::CLOSED);
            return Status::OPEN_FAILED;
        }

        if (this->verbose_is(VerboseLevel::FULL))
            std::cout << "PIRIS open - success (" << dev.get_type() << ")" << std::endl;

        this->state_set(State::READY);
        return this->dev_set_up();
    }

    Status CPirisEthernet::dev_set_up()
    {
        uint8_t fail = 0;
        if (this->state_is(State::BUSY))
        {
            if (this->verbose_is(VerboseLevel::FULL))
                std::cerr << Status::DEVICE_IS_BUSY << std::endl;

            return Status::DEVICE_IS_BUSY;
        }

        this->state_set(State::BUSY);

        try
        {
            GenApi::INodeMap& nodemap = this->pCamera->GetNodeMap();

            // Get all the nodes who are responsible for uart communication.
            this->SerialIOControl = nodemap.GetNode("SerialIOControl");

            //Nodes for BaudRate Control (max 4800 Baud allowed due to low optocoppler speed)
            GenApi::CIntegerPtr SerialIOClockCntValue(nodemap.GetNode("SerialIOClockCntValue"));
            GenApi::CIntegerPtr SerialIOClockDividerValue(nodemap.GetNode("SerialIOClockDividerValue"));

            /*
                The baudrate is set via ClockCntValue and ClockDividerValue using the ControllerClk:
                ClockDividerValue =  ControllerClk/(Baudrate*ClockCntValue)
                Example:
                ClockCntValue=31
                ControllerClk=41.75MHz
                ClockDividerValue=41.75MHz/(4800*31)=271.81

                Add -1 to ClockCntValue and ClockDividerValue because counting in FPGA begins with 0:
                ClockCntValue=31
                ClockDividerValue=271
            */

            //Setup Baudrate (4800 default value)
            SerialIOClockCntValue->SetValue(31);
            SerialIOClockDividerValue->SetValue(271);

            /*
            Control Node Discription
            Bit0:    EnableModule            ‘1’: UART enabled
            Bit4:    ClearErrors             ‘1’: parity and stop bit errors cleared
            */

            // enable the UART
            this->SerialIOControl->SetValue(0x01);

            // init other nodes
            //this->SerialIOStatus = this->nodemap.GetNode("SerialIOStatus");
            this->SerialIODataOut = nodemap.GetNode("SerialIODataOut");
            this->SerialIODataOutExecute = nodemap.GetNode("SerialIODataOutExecute");
            this->SerialIOStatusDataOutBufferFull = nodemap.GetNode("SerialIOStatusDataOutBufferFull");
            //this->SerialIOStatusDataOutBufferEmpty = (this->nodemap.GetNode("SerialIOStatusDataOutBufferEmpty");
            this->SerialIODataIn = nodemap.GetNode("SerialIODataIn");
            this->SerialIODataInExecute = nodemap.GetNode("SerialIODataInExecute");
            //this->SerialIOStatusDataInBufferFull = this->nodemap.GetNode("SerialIOStatusDataInBufferFull");
            this->SerialIOStatusDataInBufferEmpty = nodemap.GetNode("SerialIOStatusDataInBufferEmpty");

            if (!this->verbose_is(VerboseLevel::NONE))
            {
                std::cout << "Camera Device Information" << std::endl;
                std::cout << "=========================" << std::endl;
                std::cout << "Vendor           : " << GenApi::CStringPtr(nodemap.GetNode("DeviceVendorName"))->GetValue() << std::endl;
                std::cout << "Model            : " << GenApi::CStringPtr(nodemap.GetNode("DeviceModelName"))->GetValue() << std::endl;
                std::cout << "Firmware version : " << GenApi::CStringPtr(nodemap.GetNode("DeviceFirmwareVersion"))->GetValue() << std::endl;
                std::cout << "=========================" << std::endl;
            }
            if (this->verbose_is(VerboseLevel::FULL))
                std::cout << "PIRIS set up - success" << std::endl << std::endl;
            else
                std::cout << std::endl;
        }

        catch (const std::exception &ex)
        {
            if (this->verbose_is(VerboseLevel::FULL))
            {
                std::cerr << Status::DEV_SET_FAILED << " : " << ex.what() << std::endl;
                fail = 1;
            }
            else fail = 2;
        }
        catch (const GenICam::GenericException &ex)
        {
            if (this->verbose_is(VerboseLevel::FULL))
            {
                std::cerr << Status::DEV_SET_FAILED << " : " << ex.what() << std::endl;
                fail = 1;
            }
            else fail = 2;
        }
        catch (...)
        {
            fail = 2;
        }

        if (fail > 0)
        {
            if (fail == 2 && this->verbose_is(VerboseLevel::BASIC))
                std::cerr << Status::DEV_SET_FAILED << std::endl;

            this->state_set(State::CLOSED);
            return Status::DEV_SET_FAILED;
        }

        this->state_set(State::READY);
        return Status::SUCCESS;
    } 

    Status CPirisEthernet::close(bool ignore_err)
    {
        uint8_t fail = 0;
        if (this->state_is(State::BUSY))
            return Status::DEVICE_IS_BUSY;

        if (!this->state_is(State::READY))
            return Status::DEVICE_IS_CLOSED;

        this->state_set(State::BUSY);
        try
        {
            this->SerialIOControl->SetValue(0x00);

            if (!this->cam_overiden)
            {
                this->pCamera->Close();
                this->pCamera->DestroyDevice();
                //Pylon::CTlFactory& TlFactory = Pylon::CTlFactory::GetInstance();
                //TlFactory.DestroyDevice(this->pDevice);
            }
            else if (this->pDevice != nullptr)
            {
                //this->pCamera->DestroyDevice(); // ?? todo test
                delete this->pCamera;
                this->pDevice = nullptr;
            }
        }
        catch (const std::exception &ex)
        {
            if (this->verbose_is(VerboseLevel::FULL))
            {
                std::cerr << Status::CLOSE_FAILED << " : " << ex.what() << std::endl;
                fail = 1;
            }
            else fail = 2;
        }
        catch (const GenICam::GenericException &ex)
        {
            if (this->verbose_is(VerboseLevel::FULL))
            {
                std::cerr << Status::CLOSE_FAILED << " : " << ex.what() << std::endl;
                fail = 1;
            }
            else fail = 2;
        }
        catch (...)
        {
            fail = 2;
        }

        if (fail > 0)
        {
            if (fail == 2 && this->verbose_is(VerboseLevel::BASIC))
                std::cerr << Status::CLOSE_FAILED << std::endl;

            this->state_set(State::UNKNOWN);
            return Status::OPEN_FAILED;
        }


        if (!ignore_err && this->verbose_is(VerboseLevel::FULL))
            std::cout << "PIRIS close - success" << std::endl;

        this->state_set(State::CLOSED);
        return Status::SUCCESS;
    }

    Status CPirisEthernet::write(const std::string data)
    {
        uint8_t fail = 0;
        if (this->state_is(State::BUSY))
        {
            if (this->verbose_is(VerboseLevel::FULL))
                std::cerr << Status::DEVICE_IS_BUSY << std::endl;

            return Status::DEVICE_IS_BUSY;
        }    

        if (!this->state_is(State::READY))
        {
            if (this->verbose_is(VerboseLevel::FULL))
                std::cerr << Status::DEVICE_IS_CLOSED << std::endl;

            return Status::DEVICE_IS_CLOSED;
        } 

        this->state_set(State::BUSY);
        try
        {
            for (std::string::size_type i = 0; i < data.size(); i++)
            {
                // wait if data out buffer is full
                while (this->SerialIOStatusDataOutBufferFull->GetValue() == true)
                {
                    _Sleep(PIRIS_ETH_TX_BUF_FUL_WAIT);
                }

                // set the output value
                this->SerialIODataOut->SetValue(data[i]);

                // write out the value
                this->SerialIODataOutExecute->Execute(false);
            }
        }
        catch (const std::exception &ex)
        {
            if (this->verbose_is(VerboseLevel::FULL))
            {
                std::cerr << Status::WRITE_ERROR << " : " << ex.what() << std::endl;
                fail = 1;
            }
            else fail = 2;
        }
        catch (const GenICam::GenericException &ex)
        {
            if (this->verbose_is(VerboseLevel::FULL))
            {
                std::cerr << Status::WRITE_ERROR << " : " << ex.what() << std::endl;
                fail = 1;
            }
            else fail = 2;
        }
        catch (...)
        {
            fail = 2;
        }

        if (fail > 0)
        {
            if (fail == 2 && this->verbose_is(VerboseLevel::BASIC))
                std::cerr << Status::WRITE_ERROR << std::endl;

            this->state_reset();
            return Status::WRITE_ERROR;
        }

        this->state_reset();
        return Status::SUCCESS;
    }

    Status CPirisEthernet::read(std::string& data)
    {
        uint8_t fail = 0;
        if (this->state_is(State::BUSY))
        {
            if (this->verbose_is(VerboseLevel::FULL))
                std::cerr << Status::DEVICE_IS_BUSY << std::endl;

            return Status::DEVICE_IS_BUSY;
        }

        if (!this->state_is(State::READY))
        {
            if (this->verbose_is(VerboseLevel::FULL))
                std::cerr << Status::DEVICE_IS_CLOSED << std::endl;

            return Status::DEVICE_IS_CLOSED;
        }

        this->state_set(State::BUSY);
        try
        {
            std::ostringstream ret;
            int16_t timeout = this->rx_timeout;

            while (timeout > 0)
            {
                while (this->SerialIOStatusDataInBufferEmpty->GetValue() == false)
                {
                    this->SerialIODataInExecute->Execute();
                    ret << static_cast<char>(this->SerialIODataIn->GetValue());
                }

                if (ret.tellp() > -1 && !ret.str().empty() && ret.str()[ret.str().length() - 1] == '\n')
                {
                    data = ret.str();
                    this->state_reset();
                    return Status::SUCCESS;
                }
                else
                {
                    timeout -= PIRIS_ETH_RX_ITER_DELAY;
                    _Sleep(PIRIS_ETH_RX_ITER_DELAY);
                }
            }
        }
        catch (const std::exception &ex)
        {
            if (this->verbose_is(VerboseLevel::FULL))
            {
                std::cerr << Status::READ_ERROR << " : " << ex.what() << std::endl;
                fail = 1;
            }
            else fail = 2;
        }
        catch (const GenICam::GenericException &ex)
        {
            if (this->verbose_is(VerboseLevel::FULL))
            {
                std::cerr << Status::READ_ERROR << " : " << ex.what() << std::endl;
                fail = 1;
            }
            else fail = 2;
        }
        catch (...)
        {
            fail = 2;
        }

        if (fail > 0)
        {
            if (fail == 2 && this->verbose_is(VerboseLevel::BASIC))
                std::cerr << Status::READ_ERROR << std::endl;

            this->state_reset();
            return Status::READ_ERROR;
        }

        this->state_reset();
        return Status::RX_TIMEOUT;
    }

    Status CPirisEthernet::flush(bool ignore_err)
    {
        try
        {
            while (this->SerialIOStatusDataInBufferEmpty->GetValue() == false)
            {
                this->SerialIODataInExecute->Execute(false);
            }

            //while (this->SerialIOStatusDataOutBufferEmpty->GetValue() == false)
            //{
            //    this->SerialIODataOutExecute->Execute(false);
            //}
        }
        catch (...)
        {
            if (!ignore_err && this->verbose_is(VerboseLevel::FULL))
            {
                std::cerr << "PIRIS flush buffer failed" << std::endl;
                return Status::GENERAL_ERROR;
            }
        }
        return Status::SUCCESS;
    }
}