/******************************************
*
* Author:   Jakub Parez
* File:     ATEsystem.PIRIS.cpp
* Project:  ATEsystem.PIRIS-driver
* Version:  1.0.1
* Company:  ATEsystem s.r.o
* Date:     2018/11/23 11:30
* License:  WTFPL
* Require:  C++17, Pylon 5, GenICam, Serial
*
*******************************************/

#include "pylon/PylonIncludes.h"

#include "ATEsystem.PIRIS.h"
#include "ATEsystem.PIRIS_ethernet.h"
#include "ATEsystem.PIRIS_serial.h"

namespace ATEsystem_PIRIS
{
    std::ostream& operator<<(std::ostream& os, const Status stat)
    {
        switch (stat)
        {
        default:
        case Status::GENERAL_ERROR:     return os << TXT_GENERAL_ERROR;
        case Status::SUCCESS:           return os << TXT_SUCCESS;
        case Status::OPEN_FAILED:       return os << TXT_OPEN_FAILED;
        case Status::CLOSE_FAILED:      return os << TXT_CLOSE_FAILED;
        case Status::ALREADY_OPEN:      return os << TXT_ALREADY_OPEN;
        case Status::DEVICE_IS_BUSY:    return os << TXT_DEVICE_IS_BUSY;
        case Status::DEVICE_IS_CLOSED:  return os << TXT_DEVICE_IS_CLOSED;
        case Status::INVALID_DEVICE:    return os << TXT_INVALID_DEVICE;
        case Status::WRITE_ERROR:       return os << TXT_WRITE_ERROR;
        case Status::READ_ERROR:        return os << TXT_READ_ERROR;
        case Status::SCAN_FAILED:       return os << TXT_SCAN_FAILED;
        case Status::INTERNAL_ERROR:    return os << TXT_INTERNAL_ERROR;
        case Status::RX_TIMEOUT:        return os << TXT_RX_TIMEOUT;
        case Status::DEV_SET_FAILED:    return os << TXT_DEV_SET_FAILED;
        case Status::WRONG_MODE:        return os << TXT_WRONG_MODE;
        case Status::DEVICE_NOT_EXIST:  return os << TXT_DEVICE_NOT_EXIST;
        case Status::PARSE_ANSWER_FAIL: return os << TXT_PARSE_ANSWER_FAIL;
        }
    }

    std::ostream& operator<<(std::ostream& os, const PirisDeviceType type)
    {
        switch (type)
        {
        default:
        case PirisDeviceType::ETHERNET_CAMERA: return os << TXT_ETHERNET_CAMERA;
        case PirisDeviceType::ETHERNET_DEVICE: return os << TXT_ETHERNET_DEVICE;
        case PirisDeviceType::ETHERNET_PYLON:  return os << TXT_ETHERNET_PYLON;
        case PirisDeviceType::SERIAL_UART:          return os << TXT_SERIAL;
        }
    }

    CPirisMain::CPirisMain()
    {
    }

    int16_t CPirisMain::ScanEthernet(std::vector<Pylon::CDeviceInfo>& devices, bool verbose)
    {
        return CPirisEthernet::scan(devices, verbose);
    }

    int16_t CPirisMain::ScanSerial(std::vector<serial::PortInfo>& devices, bool verbose)
    {
        return CPirisSerial::scan(devices, verbose);
    }

    Status CPirisMain::Open(void* dev, PirisDeviceType type, VerboseLevel verbose)
    {
        switch (type)
        {
        case PirisDeviceType::ETHERNET_DEVICE: 
            return Status::INVALID_DEVICE;
        case PirisDeviceType::SERIAL_UART: 
            return Status::INVALID_DEVICE;
        case PirisDeviceType::ETHERNET_PYLON: 
            return this->open((PirisDevice)(static_cast<Pylon::IPylonDevice*>(dev)), verbose);
        default:
        case PirisDeviceType::ETHERNET_CAMERA: 
            return this->open((PirisDevice)(static_cast<Pylon::CInstantCamera*>(dev)), verbose);
        } 
    }

    Status CPirisMain::Open(const PirisDevice& dev, VerboseLevel verbose)
    {
        return this->open(dev, verbose);
    }

    Status CPirisMain::Close()
    {
        return this->close();
    }

    std::tuple<StatusEx, DataID> CPirisMain::ReadID()
    {
        std::vector<std::string> tokens;

        StatusEx stat = send_msg(tokens, GET_VAR_NAME(CMD_READ_ID), MSG(CMD_READ_ID), LEN_CMD_READ_ID);
        if (stat.getOk())
        {
            try
            { 
                uint16_t maj = 0;
                uint16_t min = 0;
                uint16_t rev = 0;

                int a = _sscanf(tokens[2].c_str(), "version:%hu.%hu.%hu", &maj, &min, &rev);

                if (a == 2)
                    a = _sscanf(tokens[2].c_str(), "version:%hu.%1hu%1hu", &maj, &min, &rev);
                else if (a <= 0)
                    throw std::exception();
                if (a <= 0)
                    throw std::exception();

                DataID ret = DataID(tokens[1], Version(maj, min, rev));

                if (this->verbose_is(VerboseLevel::FULL))
                {
                    std::cout << GET_VAR_NAME(CMD_READ_ID) << "    " << TXT_SUCCESS2 << ":  " 
                              << TXT_NAME << "=" << Utils::PadStr(14, 4) << tokens[1] << std::endl
                              << Utils::PadStr(25) << TXT_VERSION << "=" << Utils::PadStr(14, 7) 
                              << ret.version << std::endl << std::endl;
                }

                this->fw_poll_support = (ret.version < ATEsystem_PIRIS::Version(FW_MIN_STAT_POLL)) ? YesNoNA::NO :
                                                                                                     YesNoNA::YES;
                this->rx_timeout = (this->fw_poll_support == YesNoNA::NO) ? RX_TIMEOUT_OLD_FW : RX_TIMEOUT_NEW_FW;

                return std::make_tuple(stat, ret);
            }
            catch (...)
            {
                if (!this->verbose_is(VerboseLevel::NONE))
                {
                    std::cerr << GET_VAR_NAME(CMD_READ_ID) << " " << TXT_FAILED << "." << std::endl;
                }

                return std::make_tuple(StatusEx(Status::PARSE_ANSWER_FAIL), DataID());
            }
        }
        else
        {
            return std::make_tuple(stat, DataID());
        }
    }

    std::tuple<StatusEx, DataPosition> CPirisMain::ReadPosition()
    {
        std::vector<std::string> tokens;

        StatusEx stat = send_msg(tokens, GET_VAR_NAME(CMD_READ_POS), MSG(CMD_READ_POS), LEN_CMD_READ_POS);
        if (stat.getOk())
        {
            try
            {
                uint16_t focus = 0;
                uint16_t zoom = 0;
                uint16_t iris = 0;

                int a = _sscanf(tokens[1].c_str(), RESP_FOCUS, &focus);
                int b = _sscanf(tokens[2].c_str(), RESP_ZOOM, &zoom);
                int c = _sscanf(tokens[3].c_str(), RESP_IRIS, &iris);

                if (a <= 0 || b <= 0 || c <= 0)
                    throw std::exception();

                bool ir = (tokens[4][1] == TRUE_VAL);

                DataPosition ret = DataPosition(FocusZoomIris(focus, zoom, iris), ir);

                if (this->verbose_is(VerboseLevel::FULL))
                {
                    std::cout << GET_VAR_NAME(CMD_READ_POS) << "   " << TXT_SUCCESS2 << ":  " << ret.position 
                              << std::endl << Utils::PadStr(25) << TXT_IR_FILTER << "=" << Utils::PadStr(14, 9) 
                              << ir << std::endl << std::endl;
                }

                return std::make_tuple(stat, ret);
            }
            catch (...)
            {
                if (!this->verbose_is(VerboseLevel::NONE))
                {
                    std::cerr << GET_VAR_NAME(CMD_READ_POS) << " " << TXT_FAILED << "." << std::endl;
                }

                return std::make_tuple(StatusEx(Status::PARSE_ANSWER_FAIL), DataPosition());
            }
        }
        else
        {
            return std::make_tuple(stat, DataPosition());
        }
    }

    std::tuple<StatusEx, DataParams> CPirisMain::ReadParams()
    {
        std::vector<std::string> tokens;

        StatusEx stat = send_msg(tokens, GET_VAR_NAME(CMD_READ_TYPE), MSG(CMD_READ_TYPE), LEN_CMD_READ_TYPE);
        if (stat.getOk())
        {
            try
            {
                uint16_t focus = 0;
                uint16_t zoom = 0;
                uint16_t iris = 0;

                int a = _sscanf(tokens[2].c_str(), RESP_FOCUS, &focus);
                int b = _sscanf(tokens[3].c_str(), RESP_ZOOM, &zoom);
                int c = _sscanf(tokens[4].c_str(), RESP_IRIS, &iris);

                if (a <= 0 || b <= 0 || c <= 0)
                    throw std::exception();

                bool ir = (tokens[5][1] == TRUE_VAL);
                bool sens = (tokens[5][1] == TRUE_VAL);

                DataParams ret = DataParams(FocusZoomIris(focus, zoom, iris), tokens[1], ir, sens);

                if (this->verbose_is(VerboseLevel::FULL))
                {
                    std::cout << GET_VAR_NAME(CMD_READ_TYPE) << "  " << TXT_SUCCESS2 << ":  " << TXT_TYPE << "="
                              << Utils::PadStr(14, 4) << tokens[1] << std::endl 
                              << Utils::PadStr(25) << ret.max_value << std::endl
                              << Utils::PadStr(25) << TXT_IR_PRESENT << "=" << Utils::PadStr(14, 10) << ir << std::endl
                              << Utils::PadStr(25) << TXT_SENS_PRESENT << "=" << Utils::PadStr(14, 12) << sens
                              << std::endl << std::endl;
                }

                return std::make_tuple(stat, ret);
            }
            catch (...)
            {
                if (!this->verbose_is(VerboseLevel::NONE))
                {
                    std::cerr << GET_VAR_NAME(CMD_READ_TYPE) << " "  << TXT_FAILED << "." << std::endl;
                }

                return std::make_tuple(StatusEx(Status::PARSE_ANSWER_FAIL), DataParams());
            }
        }
        else
        {
            return std::make_tuple(stat, DataParams());
        }
    }

    std::tuple<StatusEx, DataState> CPirisMain::ReadState()
    {
        std::vector<std::string> tokens;

        StatusEx stat = send_msg(tokens, GET_VAR_NAME(CMD_READ_STATE), MSG(CMD_READ_STATE), LEN_CMD_READ_STATE1);
        if (stat.getOk())
        {
            try
            {
                bool focus = (tokens[1][1] == TRUE_VAL);
                bool zoom = (tokens[1][2] == TRUE_VAL);
                bool iris = (tokens[1][3] == TRUE_VAL);
                bool busy = false;

                if (tokens.size() == LEN_CMD_READ_STATE2)
                    busy = (tokens[2][1] == TRUE_VAL);

                DataState ret = DataState(FocusZoomIris(focus, zoom, iris), busy);

                if (this->verbose_is(VerboseLevel::FULL))
                {
                    std::cout << GET_VAR_NAME(CMD_READ_STATE) << " " << TXT_SUCCESS2 << ":  " << ret.state 
                              << std::endl << Utils::PadStr(25) << TXT_BUSY << "=" << Utils::PadStr(14, 4) << busy
                              << std::endl << std::endl;
                }

                return std::make_tuple(stat, ret);
            }
            catch (...)
            {
                if (!this->verbose_is(VerboseLevel::NONE))
                {
                    std::cerr << GET_VAR_NAME(CMD_READ_STATE) << " " << TXT_FAILED << "." << std::endl;
                }

                return std::make_tuple(StatusEx(Status::PARSE_ANSWER_FAIL), DataState());
            }
        }
        else
        {
            return std::make_tuple(stat, DataState());
        }
    }

    StatusEx CPirisMain::DevReset()
    {
        std::vector<std::string> tokens;

        StatusEx stat = send_msg(tokens, GET_VAR_NAME(CMD_RESET), MSG(CMD_RESET), LEN_CMD_RESET);
        if (stat.getOk())
        {
            if (this->verbose_is(VerboseLevel::FULL))
            {
                std::cout << GET_VAR_NAME(CMD_RESET) << "      " << TXT_SUCCESS2 << std::endl << std::endl;
            } 
        }
        return stat;
    }

    StatusEx CPirisMain::DevHoming()
    {
        std::vector<std::string> tokens;

        StatusEx stat = send_msg(tokens, GET_VAR_NAME(CMD_HOMING), MSG(CMD_HOMING), LEN_CMD_HOMING);
        if (stat.getOk())
        {
            if (this->verbose_is(VerboseLevel::FULL))
            {
                std::cout << GET_VAR_NAME(CMD_HOMING) << "     " << TXT_SUCCESS2 << std::endl << std::endl;
            }
        }
        return stat;
    }

    StatusEx CPirisMain::SetRelative(int16_t focus, int16_t zoom, int16_t iris)
    {
        return SetRelative(FocusZoomIris<int16_t>(focus, zoom, iris));
    }

    StatusEx CPirisMain::SetRelative(const FocusZoomIris<int16_t>& values = FocusZoomIris<int16_t>(0, 0, 0))
    {
        std::vector<std::string> tokens;
        std::stringstream msg;
        msg << CMD_SET_REL << "F" << values.getFocus() << DELIM 
                           << "Z" << values.getZoom()  << DELIM
                           << "P" << values.getIris();

        StatusEx stat = send_msg(tokens, GET_VAR_NAME(CMD_SET_REL), msg.str() + CMD_SUFFIX, LEN_CMD_SET_REL);
        if (stat.getOk())
        {
            if (this->verbose_is(VerboseLevel::FULL))
            {
                std::cout << GET_VAR_NAME(CMD_SET_REL) << "    " << TXT_SUCCESS2 << ":  " << values
                          << std::endl << std::endl;
            }
        }
        return stat;
    }

    StatusEx CPirisMain::SetAbsolute(uint16_t focus, uint16_t zoom, uint16_t iris, bool ir_filter)
    {
        return SetAbsolute(FocusZoomIris<uint16_t>(focus, zoom, iris), ir_filter);
    }

    StatusEx CPirisMain::SetAbsolute(const FocusZoomIris<uint16_t>& values = FocusZoomIris<uint16_t>(0, 0, 0), 
                                     bool ir_filter)
    {
        std::vector<std::string> tokens;
        std::stringstream msg;
        msg << CMD_SET_ABS << "F" << values.getFocus() << DELIM 
                           << "Z" << values.getZoom()  << DELIM
                           << "P" << values.getIris()  << DELIM 
                           << "I" << ((ir_filter) ? "1" : "0");

        StatusEx stat = send_msg(tokens, GET_VAR_NAME(CMD_SET_ABS), msg.str() + CMD_SUFFIX, LEN_CMD_SET_ABS);
        if (stat.getOk())
        {
            if (this->verbose_is(VerboseLevel::FULL))
            {
                std::cout << GET_VAR_NAME(CMD_SET_ABS) << "    " << TXT_SUCCESS2 << ":  " << values << std::endl
                          << Utils::PadStr(25) << TXT_IR_FILTER << "=" << Utils::PadStr(14, 9) << ir_filter 
                          << std::endl << std::endl;
            }
        }
        return stat;
    }

    YesNoNA CPirisMain::GetFwPollSupport()
    {
        return this->fw_poll_support;
    }

    VerboseLevel CPirisMain::GetVerboseLevel()
    {
        return this->_verbose;
    }

    void CPirisMain::SetVerboseLevel(VerboseLevel level)
    {
        this->verbose_set(level);
    }

/* ******************************************************************************** */

    void CPirisMain::state_set(State __state)
    {
        this->_state = this->state;
        this->state = __state;
    }

    State CPirisMain::state_get()
    {
        return this->state;
    }

    bool CPirisMain::state_is(State __state)
    {
        return this->state == __state;
    }

    void CPirisMain::state_reset()
    {
        this->state = this->_state;
    }

    bool CPirisMain::verbose_is(VerboseLevel __verbose)
    {
        return this->_verbose == __verbose;
    }

    void CPirisMain::verbose_set(VerboseLevel __verbose)
    {
        this->_verbose = __verbose;
    }

    ErrorCluster CPirisMain::parse_answer(std::string answer_in, std::vector<std::string>& tokens_out, uint8_t n)
    {
        try
        {
            tokens_out.clear();
            size_t pos = 0;

            while ((pos = answer_in.find(DELIM)) != std::string::npos)
            {
                tokens_out.push_back(answer_in.substr(0, pos));
                answer_in.erase(0, pos + 1);
            }
            if (answer_in != "")
                tokens_out.push_back(answer_in);

            if (tokens_out.size() >= n)
            {
                return ErrorCluster(tokens_out[0]);
            }
            else if (tokens_out.size() >= 1)
            {
                if (tokens_out[0] == "ERR0")
                    return ErrorCluster(ERR0, true);
                else if (tokens_out[0] == "ERR1")
                    return ErrorCluster(ERR1, true);
                else if (tokens_out[0] == "ERR2")
                    return ErrorCluster(ERR2, true);
                else if (tokens_out[0] == "ERR3")
                    return ErrorCluster(ERR3, true);
                else if (tokens_out[0] == "ERR4")
                    return ErrorCluster(ERR4, true);
                else if (tokens_out[0] == "ERR5")
                    return ErrorCluster(ERR5, true);
                else if (tokens_out[0] == "ERR6")
                    return ErrorCluster(ERR6, true);
                else if (tokens_out[0] == "ERR7")
                    return ErrorCluster(ERR7, true);
                else
                    throw std::exception();
            }
            else throw std::exception();
        }
        catch (...)
        {
            tokens_out.clear();
            return ErrorCluster(ERR_PA, true);
        }
    }

    StatusEx CPirisMain::send_msg(std::vector<std::string>& tokens_out, std::string name, std::string cmd, uint8_t len)
    {
        Status stat = Status::GENERAL_ERROR;
        ErrorCluster err = ErrorCluster(ERR_PA, true);

        try
        {
            std::string data;
            std::ostringstream ret;

            this->flush(true);

            // 1. WRITE
            stat = this->write(cmd); // + CMD_SUFFIX);

            if (stat != Status::SUCCESS)
            {
                ret << stat;
                throw std::runtime_error(ret.str().c_str());
            }

            if (len == 0)
                return StatusEx(stat, ErrorCluster(OK));

            // 2. READ
            stat = this->read(data);

            if (stat != Status::SUCCESS)
            {
                ret << stat;
                throw std::runtime_error(ret.str().c_str());
            }

            data.pop_back();
            if (data[data.length() - 1] == '\r')
                data.pop_back();

            // 3. PARSE
            err = this->parse_answer(data, tokens_out, len);
            if (err.getOk())
            {
                return StatusEx(stat, err);
            }
            else
            {
                stat = Status::PARSE_ANSWER_FAIL;
                throw std::runtime_error(err.getMsg().c_str());
            }
        }
        catch (const std::exception &ex)
        {
            if (!this->verbose_is(VerboseLevel::NONE))
                std::cerr << name << " " << TXT_FAILED << ". " << " : " << ex.what() << std::endl;

            return StatusEx(stat, err);
        }
        catch (...)
        {
            if (!this->verbose_is(VerboseLevel::NONE))
                std::cerr << name << " " << TXT_FAILED << ". " << std::endl;

            return StatusEx(stat, err);
        }
    }

/* ******************************************************************************** */

    Version::Version(uint16_t major, uint16_t minor, uint16_t revision) :
        _major(major), _minor(minor), _revision(revision)
    {
    }

    const uint16_t Version::getMajor() const
    {
        return this->_major;
    }

    const uint16_t Version::getMinor() const
    {
        return this->_minor;
    }

    const uint16_t Version::getRevision() const
    {
        return this->_revision;
    }

    const std::tuple<uint16_t, uint16_t, uint16_t> Version::getVersion() const
    {
        return std::make_tuple(this->_major, this->_minor, this->_revision);
    }

    std::string Version::ToString() const
    {
        std::stringstream ret; 
        ret << this->_major << "." << this->_minor << "." << this->_revision;
        return ret.str();
    }

/* ******************************************************************************** */

    std::map<std::string, std::string> ErrorCluster::errors = { {GET_VAR_NAME(OK), OK },
                                                                {GET_VAR_NAME(ERR0),   ERR0   },
                                                                {GET_VAR_NAME(ERR1),   ERR1   },
                                                                {GET_VAR_NAME(ERR2),   ERR2   },
                                                                {GET_VAR_NAME(ERR3),   ERR3   },
                                                                {GET_VAR_NAME(ERR4),   ERR4   },
                                                                {GET_VAR_NAME(ERR5),   ERR5   },
                                                                {GET_VAR_NAME(ERR6),   ERR6   },
                                                                {GET_VAR_NAME(ERR7),   ERR7   },
                                                                {GET_VAR_NAME(ERR_UN), ERR_UN },
                                                                {GET_VAR_NAME(ERR_PA), ERR_PA }, };

    ErrorCluster::ErrorCluster(std::string err_raw, bool overide)
    {
        this->ok = false;
        this->err_num = -20;
        this->msg = Status::UNDEFINED_ERROR;

        if (overide)
            this->msg = err_raw;

        try
        {
            std::map<std::string, std::string>::iterator it = errors.find(err_raw);
            if (err_raw != "" && it != errors.end())
            {
                this->msg = it->second;
                if (it->second == OK)
                {
                    this->ok = true;
                    this->err_num = 0;
                }
                else
                {
                    std::size_t const n = it->second.find_first_of("0123456789");
                    if (n != std::string::npos)
                    {
                        std::size_t const m = it->second.find_first_not_of("0123456789", n);
                        this->err_num = (int8_t)atoi(it->second.substr(n, m != std::string::npos ? m - n : m).c_str());
                    }
                }
            }
        }
        catch (...)
        {
            this->ok = false;
            this->err_num = -1;
            this->msg = ERR_UN;
        }
    }

    const bool ErrorCluster::getOk() const
    {
        return this->ok;
    }

    const int8_t ErrorCluster::getErrNum() const
    {
        return this->err_num;
    }

    const std::string ErrorCluster::getMsg() const
    {
        return this->msg;
    }

    const std::tuple<bool, int8_t, std::string> ErrorCluster::getErrorCluster() const
    {
        return std::make_tuple(this->ok, this->err_num, this->msg);
    }

    std::string ErrorCluster::ToString() const
    {
        std::stringstream ret;
        if (this->ok)
        {
            ret << "Success";
        }
        else
        {
            ret << "Error " << this->err_num << " - " << this->msg;
        }
        return ret.str();
    }

/* ******************************************************************************** */

    StatusEx::StatusEx(Status status, ErrorCluster err) :
        status(status), ErrorCluster(err)
    {
    }

    DataID::DataID(std::string name, Version version) : 
        name(name), version(version)
    {
    }

    DataPosition::DataPosition(FocusZoomIris<uint16_t> pos, bool ir) : 
        position(pos), ir_filter(ir)
    {
    }

    DataParams::DataParams(FocusZoomIris<uint16_t> max, std::string lens, bool ir, bool sens) :
        max_value(max), lens(lens), ir_present(ir), sens_present(sens)
    {
    }

    DataState::DataState(FocusZoomIris<bool> state, bool busy) :
        state(state), motors_busy(busy)
    {
    }

    IComm::~IComm() { }
    IDevice::~IDevice() { }

/* ******************************************************************************** */

    Factory::Factory()
    {
        Register(Mode::SERIAL, &CPirisSerial::create);
        Register(Mode::ETHERNET, &CPirisEthernet::create);
    }

    void Factory::Register(Mode mode, CreateDeviceFn pfnCreate)
    {
        _factoryMap[mode] = pfnCreate;
    }

    DevID_t Factory::CreateDevice(Mode mode)
    {
        DevID_t ret;
        uint16_t id = 0;
        std::ostringstream dev;
        dev << "dev" << id;
        while (Status::ALREADY_OPEN == (ret = CreateDevice(dev.str(), mode)))
            dev.clear(), dev << "dev" << ++id;
        return ret;
    }

    DevID_t Factory::CreateDevice(const std::string& name, Mode mode)
    {
        bool found = false;
        for (DeviceMap::iterator it = _deviceMap.begin(); it != _deviceMap.end(); ++it)
        {
            if (it->second.first == name)
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            FactoryMap::iterator itFact = _factoryMap.find(mode);
            if (itFact != _factoryMap.end())
            {
                DevID_t _id = (DevID_t)_deviceMap.size();
                IDevice* dev = itFact->second();    // create final device instance
                _deviceMap.insert({ _id, { name, dev } });    // add to map
                return _id;
                //return dev;
            }
            else return Status::INTERNAL_ERROR;
        }
        else return Status::ALREADY_OPEN;
    }

    IDevice* Factory::GetDeviceInstance(DevID_t id)
    {
        DeviceMap::iterator itDev = _deviceMap.find(id);
        if (itDev != _deviceMap.end())
            return itDev->second.second;
        return nullptr;
    }

    std::string Factory::GetDeviceName(DevID_t id)
    {
        DeviceMap::iterator itDev = _deviceMap.find(id);
        if (itDev != _deviceMap.end())
            return itDev->second.first;
        return nullptr;
    }

    Status Factory::RemoveDevice(DevID_t id)
    {
        DeviceMap::iterator itDev = _deviceMap.find(id);
        if (itDev != _deviceMap.end())
        {
            IDevice* _dev = itDev->second.second;
            _deviceMap.erase(itDev);       // remove device from map
            delete _dev; // ->remove();                // delete this overriden on child
            return Status::SUCCESS;
        }
        return Status::DEVICE_NOT_EXIST;
    }

/* ******************************************************************************** */

    PirisDevice::PirisDevice(Pylon::CDeviceInfo& device) :
        serial(dummy_port), device_eth(device), mode(Mode::ETHERNET), type(PirisDeviceType::ETHERNET_DEVICE)
    {
    }

    PirisDevice::PirisDevice(Pylon::IPylonDevice* pylon_dev) : 
        serial(dummy_port), device_eth(dummy_dev), mode(Mode::ETHERNET), type(PirisDeviceType::ETHERNET_PYLON)
    {
        this->pylon_dev = pylon_dev;
    }

    PirisDevice::PirisDevice(Pylon::CInstantCamera* camera) : 
        serial(dummy_port), device_eth(dummy_dev), mode(Mode::ETHERNET), type(PirisDeviceType::ETHERNET_CAMERA)
    {
        this->camera = camera;
    }

    PirisDevice::PirisDevice(serial::PortInfo& serial) :
        serial(serial), device_eth(dummy_dev), mode(Mode::SERIAL), type(PirisDeviceType::SERIAL_UART)
    {
        this->serial = serial;
    }

    Pylon::CDeviceInfo& PirisDevice::get_device_eth() const
    {
        return this->device_eth;
    }

    Pylon::IPylonDevice* PirisDevice::get_pylon_dev() const
    {
        return this->pylon_dev;
    }

    Pylon::CInstantCamera* PirisDevice::get_camera() const
    {
        return this->camera;
    }

    serial::PortInfo& PirisDevice::get_serial() const
    {
        return this->serial;
    }

    const Mode PirisDevice::get_mode() const
    {
        return this->mode;
    }

    const PirisDeviceType PirisDevice::get_type() const
    {
        return this->type;
    }

    std::string PirisDevice::ToString() const
    {
        std::stringstream ret;
        ret << this->type;
        return ret.str();
    }

/* ******************************************************************************** */

    void Utils::CpyCStr2CStr(char *dest, const char *source, size_t max_len)
    {
        size_t i;
        for (i = 0; i < max_len - 1 && source[i]; i++)
            dest[i] = source[i];
        dest[i] = '\0';
    }

    void Utils::CopySTLStr2CStr(char *dest, std::string source)
    {
        CpyCStr2CStr(dest, source.c_str(), source.length() + 1);
    }

    std::string Utils::CopyCStr2STLStr(const char *src)
    {
        int i = 0;
        for (i = 0; i < STR_MAX_SIZE; i++)
            if (*src == '\0') break;
            else ++src;
        if (i > 0) return std::string(src - i, i);
        else return {};
    }
    std::string Utils::PadStr(const int total, const int minus, const char ch)
    {
        return std::string((size_t)(total - minus), ch);
    }
}
