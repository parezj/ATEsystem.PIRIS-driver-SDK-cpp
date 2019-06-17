/******************************************
*
* Author:   Jakub Parez
* File:     ATEsystem.PIRIS_serial.cpp
* Project:  ATEsystem.PIRIS-driver
* Version:  1.0.1
* Company:  ATEsystem s.r.o
* Date:     2018/12/6 18:31
* License:  WTFPL
* Require:  C++17, Serial
*
*******************************************/

#include <iostream>
#include <iomanip>

#include "serial.h"

#include "ATEsystem.PIRIS.h"
#include "ATEsystem.PIRIS_serial.h"


namespace ATEsystem_PIRIS
{
    CPirisSerial::CPirisSerial()
    {
    }

    CPirisSerial::~CPirisSerial()
    {
        //std::cout << "deleting serial object" << std::endl; // todo check

        this->close(true);
        delete this->serial;

        
        //delete this;    // todo test
    }
    
    int16_t CPirisSerial::scan(std::vector<serial::PortInfo>& devices, bool verbose)
    {
        try
        {
            devices.clear();
            devices = serial::list_ports();

            if (!devices.empty())
            {
                for (uint16_t i = 0; i < devices.size(); i++)
                {
                    if (verbose)
                    {
                        std::cout << "  " << i << ". " << devices[i].port 
                                  << Utils::PadStr(8, (int)devices[i].port.length())
                                  << devices[i].description;
                        if (i != devices.size() - 1)
                            std::cout << std::endl;
                    }   
                }

                return (int16_t)devices.size();
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

    IDevice* CPirisSerial::create()        // ? CPirisMain
    {
        return new CPirisSerial();
    }

    void CPirisSerial::remove()
    {
        delete this;
    }

    Status CPirisSerial::open(const PirisDevice& dev, VerboseLevel verbose)
    {
        uint8_t fail = 0;
        this->verbose_set(verbose);

        if (dev.get_mode() != Mode::SERIAL)
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
            if (dev.get_type() == PirisDeviceType::SERIAL_UART)
            {
                this->serial = new serial::Serial(dev.get_serial().port,
                                                  PIRIS_SERIAL_BAUDRATE,
                                                  serial::Timeout::simpleTimeout(this->rx_timeout),
                                                  PIRIS_SERIAL_BITS,
                                                  PIRIS_SERIAL_PARITY,
                                                  PIRIS_SERIAL_STOPBITS, 
                                                  PIRIS_SERIAL_FLOW);
                if (!this->serial->isOpen())
                    throw std::runtime_error("unknown error");
            }
            else
            {
                if (this->verbose_is(VerboseLevel::FULL))
                    std::cerr << Status::INVALID_DEVICE << std::endl;
                return Status::INVALID_DEVICE;
            }
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

    Status CPirisSerial::dev_set_up()
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
            this->serial->flush();
            this->serial->setDTR(true);
            this->serial->setRTS(true);

            if (this->verbose_is(VerboseLevel::FULL))
            {
                std::cout << "Serial Device Information" << std::endl;
                std::cout << "=========================" << std::endl;
                std::cout << "Port              : " << this->serial->getPort() << std::endl;
                std::cout << "Baudrate          : " << this->serial->getBaudrate() << std::endl;
                std::cout << "=========================" << std::endl << std::endl;
            }
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

    Status CPirisSerial::close(bool ignore_err)
    {
        uint8_t fail = 0;
        if (this->state_is(State::BUSY))
            return Status::DEVICE_IS_BUSY;

        if (!this->state_is(State::READY))
            return Status::DEVICE_IS_CLOSED;

        this->state_set(State::BUSY);
        try
        {
            this->serial->flush();
            this->serial->close();
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
        catch (...)
        {
            fail = 2;
        }

        if (fail > 0)
        {
            if (fail == 2 && this->verbose_is(VerboseLevel::BASIC))
                std::cerr << Status::CLOSE_FAILED << std::endl;

            this->state_set(State::UNKNOWN);
            return Status::CLOSE_FAILED;
        }


        if (!ignore_err && this->verbose_is(VerboseLevel::FULL))
            std::cout << "PIRIS close - success" << std::endl;

        this->state_set(State::CLOSED);
        return Status::SUCCESS;
    }

    Status CPirisSerial::write(const std::string data)
    {
        uint8_t fail = 0;
        if (this->state_is(State::BUSY))
        {
            if (this->verbose_is(VerboseLevel::FULL))
                std::cerr << Status::DEVICE_IS_BUSY << std::endl << std::endl;

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
            this->serial->write(data);
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

    Status CPirisSerial::read(std::string& data)
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
            if (this->serial->getTimeout().read_timeout_constant != this->rx_timeout)
            {
                serial::Timeout _timeout = serial::Timeout::simpleTimeout(this->rx_timeout);
                this->serial->setTimeout(_timeout);
            }    

            if (this->serial->readline(data, PIRIS_SERIAL_MAX_RX, PIRIS_SERIAL_EOL) > 0)
            {
                this->state_reset();
                return Status::SUCCESS;
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

    Status CPirisSerial::flush(bool ignore_err)
    {
        try
        {
            this->serial->flush();
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