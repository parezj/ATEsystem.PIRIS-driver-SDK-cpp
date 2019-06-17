/******************************************
*
* Author:   Jakub Parez
* File:     ATEsystem.PIRIS.h
* Project:  ATEsystem.PIRIS-driver
* Version:  1.0.1
* Company:  ATEsystem s.r.o
* Date:     2018/11/23 09:29
* License:  WTFPL
* Require:  C++17, Pylon 5, GenICam, Serial
*
*******************************************/

#pragma once

#ifdef _WIN32
    #include <windows.h>

    #define Interface   class //__interface
    #define _sscanf     sscanf_s
    #define _Sleep(ms)  Sleep(ms)

#elif __linux__ 
    #include <unistd.h>
    #include <stdlib.h>
    #include <cstdint>

    #define Interface   struct
    #define _sscanf     sscanf
    #define _Sleep(ms)  usleep(ms * 1000)

#else
    #error Unknown / unsupported platform
#endif

#include <string>
#include <map>
#include <vector>
#include <list>

#include "pylon/PylonIncludes.h"
#include "serial.h"


// ERROR TEXT
#define OK             "OK"
#define ERR0           "Command was not recognized."
#define ERR1           "Wrong input parameter."
#define ERR2           "Device is not initialized."
#define ERR3           "Internal driver circuit is in error state (overheat, undervoltage)."
#define ERR4           "Wrong lens type."
#define ERR5           "Damaged or missing rear sensor."
#define ERR6           "Step generator timer overflow."
#define ERR7           "Device is busy."
#define ERR_UN         "Unknown error."
#define ERR_PA         "Parse answer failed."

// STATUS TEXT
#define TXT_SUCCESS             "Success"
#define TXT_GENERAL_ERROR       "General Error"
#define TXT_OPEN_FAILED         "Open failed"
#define TXT_CLOSE_FAILED        "CLose failed"
#define TXT_ALREADY_OPEN        "Already open"
#define TXT_DEVICE_IS_BUSY      "Device is busy"
#define TXT_DEVICE_IS_CLOSED    "Device is closed"
#define TXT_INVALID_DEVICE      "Invalid device"
#define TXT_WRITE_ERROR         "Write error"
#define TXT_READ_ERROR          "Read error"
#define TXT_SCAN_FAILED         "Scan failed"
#define TXT_INTERNAL_ERROR      "Internal error"
#define TXT_RX_TIMEOUT          "Receive timeout"
#define TXT_DEV_SET_FAILED      "Device set failed"
#define TXT_WRONG_MODE          "Wrong mode"
#define TXT_DEVICE_NOT_EXIST    "Device does not exist"
#define TXT_PARSE_ANSWER_FAIL   "Parse answer failed"

// DEVICE MODE TEXT
#define TXT_ETHERNET_DEVICE     "Ethernet device"
#define TXT_ETHERNET_PYLON      "Ethernet pylon (camera overiden)"
#define TXT_ETHERNET_CAMERA     "Ethernet camera (camera overiden)"
#define TXT_SERIAL              "Serial"

// VERBOSE OUTPUT TEXT
#define TXT_SUCCESS2        "success"
#define TXT_FAILED          "failed"
#define TXT_NAME            "name"
#define TXT_VERSION         "version"
#define TXT_FOCUS           "focus"
#define TXT_ZOOM            "zoom"
#define TXT_IRIS            "iris"
#define TXT_LENS            "lens"
#define TXT_IR_FILTER       "ir_filter"
#define TXT_IR_PRESENT      "ir_present"
#define TXT_SENS_PRESENT    "sens_present"
#define TXT_BUSY            "busy"
#define TXT_TYPE            "type"

// ATE PIRIS PROTOCOL
#define CMD_READ_ID         "IDN"
#define CMD_READ_POS        "GP"
#define CMD_READ_TYPE       "GT"
#define CMD_READ_STATE      "GS"
#define CMD_RESET           "RST"
#define CMD_HOMING          "INI"
#define CMD_SET_ABS         "SETA:"
#define CMD_SET_REL         "SETR:"
#define CMD_SUFFIX          "\r\n"
#define RESP_FOCUS          "F%hu"
#define RESP_ZOOM           "Z%hu"
#define RESP_IRIS           "P%hu"
#define TRUE_VAL            '1'
#define DELIM               ";"

// RESPONSE CHAR LEN
#define LEN_CMD_READ_ID      3
#define LEN_CMD_READ_POS     5
#define LEN_CMD_READ_TYPE    7
#define LEN_CMD_READ_STATE1  2
#define LEN_CMD_READ_STATE2  3
#define LEN_CMD_RESET        0
#define LEN_CMD_HOMING       1
#define LEN_CMD_SET_ABS      1
#define LEN_CMD_SET_REL      1

#define FW_MIN_STAT_POLL     1, 7, 2    // minimum fw version which support cyclic status poll reading

#define RX_TIMEOUT_OLD_FW    20000
#define RX_TIMEOUT_NEW_FW    1000

#define MSG(x)               x CMD_SUFFIX

#define STR_MAX_SIZE   200 
#define GET_VAR_NAME(Variable) (#Variable)


namespace ATEsystem_PIRIS
{
    /// <summary>  
    /// Numerical representation of status. 0 is OK, everything else means error.
    /// </summary>  
    typedef int16_t Status_t; 

    /// <summary>  
    /// Unique device ID, that can be any device manipulated with.
    /// </summary>  
    typedef int16_t DevID_t;

    /// <summary>  
    /// Main operation mode, which depends on PIRIS device product version. Either RS232 (SERIAL) or ETHERNET.
    /// </summary>  
    enum Mode : uint8_t
    {
        SERIAL =   0,
        ETHERNET = 1
    };

    /// <summary>  
    /// Main status output, any action returns this status. 0 is OK, everything else means error.
    /// </summary>  
    enum Status : int16_t
    {
        SUCCESS =             0,
        GENERAL_ERROR =      -1,
        OPEN_FAILED =        -2, 
        CLOSE_FAILED =       -3, 
        ALREADY_OPEN =       -4,
        DEVICE_IS_BUSY =     -5,
        DEVICE_IS_CLOSED =   -6,
        INVALID_DEVICE =     -7,
        WRITE_ERROR =        -8,
        READ_ERROR =         -9,
        SCAN_FAILED =       -10,
        INTERNAL_ERROR =    -11,
        RX_TIMEOUT =        -12,
        DEV_SET_FAILED =    -13,
        WRONG_MODE =        -14,
        DEVICE_NOT_EXIST =  -15,
        PARSE_ANSWER_FAIL = -16,
        UNDEFINED_ERROR =   -20
    };
    std::ostream& operator<<(std::ostream& os, const Status stat);

    /// <summary>  
    /// State machine status. Internal enum used to handle driver states.
    /// </summary>  
    enum State : int8_t
    {
        UNKNOWN = 0,
        CLOSED =  1,
        BUSY =    2,
        READY =   3,
    };

    /// <summary>  
    /// More specific device type. ETHERNET_DEVICE is standalone example case, ETHERNET_PYLON
    /// and ETHERNET_CAMERA means pylon cam instance opened elsewhere and passed to driver.
    /// </summary>  
    enum PirisDeviceType : uint8_t
    {
        ETHERNET_DEVICE = 0,
        ETHERNET_PYLON =  1,
        ETHERNET_CAMERA = 2,
        SERIAL_UART =     3
    };
    std::ostream& operator<<(std::ostream& os, const PirisDeviceType type);
    
    /// <summary>  
    /// Level of verbosity. FULL is used to debug, NONE is for production.
    /// </summary> 
    enum VerboseLevel : uint8_t
    {
        NONE =  0,
        BASIC = 1,
        FULL =  2
    };

    /// <summary>  
    /// Simple 3 state logic handler.
    /// </summary> 
    enum YesNoNA : uint8_t
    {
        NOT_AVAILABLE = 0,
        YES =           1,
        NO =            2
    };

/* ******************************************************************************** */

    /// <summary>  
    /// Utility class mainly for C string and C++ string handle.
    /// </summary> 
    class Utils
    {
    public:

        static void CpyCStr2CStr(char* dest, const char* source, size_t max_len);
        static void CopySTLStr2CStr(char* dest, std::string source);
        static std::string CopyCStr2STLStr(const char* src);
        static std::string PadStr(const int total, const int minus = 0, const char ch = ' ');
    };

/* ******************************************************************************** */

    /// <summary>  
    /// This class holds ref or information about unique and specific PIRIS device.
    /// Its main purpose is to server as an input to open function to overcome multiple inputs problem.
    /// </summary> 
    class PirisDevice
    {
    private:

        const Mode mode = Mode::ETHERNET;
        const PirisDeviceType type = PirisDeviceType::ETHERNET_DEVICE;
        Pylon::CDeviceInfo dummy_dev;
        serial::PortInfo dummy_port;

        Pylon::CDeviceInfo& device_eth;
        Pylon::IPylonDevice* pylon_dev = nullptr;
        Pylon::CInstantCamera* camera = nullptr;
        serial::PortInfo& serial;

    public:

        PirisDevice(Pylon::CDeviceInfo& device);
        PirisDevice(Pylon::IPylonDevice* pylon_dev);
        PirisDevice(Pylon::CInstantCamera* camera);
        PirisDevice(serial::PortInfo& serial);

        Pylon::CDeviceInfo& get_device_eth() const;
        Pylon::IPylonDevice* get_pylon_dev() const;
        Pylon::CInstantCamera* get_camera() const;
        serial::PortInfo& get_serial() const;
        const Mode get_mode() const;
        const PirisDeviceType get_type() const;

        std::string ToString() const;
        friend std::ostream& operator<<(std::ostream& strm, const PirisDevice& dev)
        {
            strm << dev.ToString();
            return strm;
        }
    };

/* ******************************************************************************** */

    /// <summary>  
    /// Class with complete error information. That is ok (T = not error), err_num and msg.
    /// </summary> 
    class ErrorCluster
    {
    private:

        bool ok;
        int8_t err_num;
        std::string msg;

        static std::map<std::string, std::string> errors;

    public:

        ErrorCluster(std::string err_raw = "", bool overide = false);
        const bool getOk() const;
        const int8_t getErrNum() const;
        const std::string getMsg() const;
        const std::tuple<bool, int8_t, std::string> getErrorCluster() const;

        std::string ToString() const;
        friend std::ostream& operator<<(std::ostream& strm, const ErrorCluster& err) 
        { 
            strm << err.ToString();
            return strm;
        }
    };

/* ******************************************************************************** */

    /// <summary>  
    /// Simple version (Major, Minor, Rev) container class.
    /// </summary> 
    class Version
    {
    private:

        const uint16_t _major;
        const uint16_t _minor;
        const uint16_t _revision;

    public:

        Version(uint16_t major = 0, uint16_t minor = 0, uint16_t revision = 0);
        const uint16_t getMajor() const;
        const uint16_t getMinor() const;
        const uint16_t getRevision() const;
        const std::tuple<uint16_t, uint16_t, uint16_t> getVersion() const;

        std::string ToString() const;
        friend std::ostream& operator<<(std::ostream& strm, const Version& ver) 
        { 
            strm << ver.ToString();
            return strm;
        }
        friend bool operator==(const Version& ver1, const Version& ver2) 
        {
            return (ver1._major == ver2._major && ver1._minor == ver2._minor && ver1._revision == ver2._revision);
        }
        friend bool operator>(const Version& ver1, const Version& ver2)
        { 
            return (ver1._major > ver2._major || 
                   (ver1._major == ver2._major && ver1._minor > ver2._minor) || 
                   (ver1._major == ver2._major && ver1._minor == ver2._minor && ver1._revision > ver2._revision)); 
        }
        friend bool operator>=(const Version& ver1, const Version& ver2)
        {
            return (ver1._major >= ver2._major ||
                   (ver1._major == ver2._major && ver1._minor >= ver2._minor) ||
                   (ver1._major == ver2._major && ver1._minor == ver2._minor && ver1._revision >= ver2._revision));
        }
        friend bool operator<(const Version& ver1, const Version& ver2)
        {
            return (ver1._major < ver2._major ||
                (ver1._major == ver2._major && ver1._minor < ver2._minor) ||
                (ver1._major == ver2._major && ver1._minor == ver2._minor && ver1._revision < ver2._revision));
        }
        friend bool operator<=(const Version& ver1, const Version& ver2)
        {
            return (ver1._major <= ver2._major ||
                (ver1._major == ver2._major && ver1._minor <= ver2._minor) ||
                (ver1._major == ver2._major && ver1._minor == ver2._minor && ver1._revision <= ver2._revision));
        }
    };

/* ******************************************************************************** */

    /// <summary>  
    /// Simple Focus, Zoom, Iris container template class.
    /// </summary> 
    template <class T = uint16_t>
    class FocusZoomIris
    {
    private:

        int pad_len;

        const T _focus;
        const T _zoom;
        const T _iris;

    public:

        FocusZoomIris(T focus = NULL, T zoom = NULL, T iris = NULL, int str_pad_len = 25) :
            _focus(focus), _zoom(zoom), _iris(iris), pad_len(str_pad_len)
        {
        }
        const T getFocus() const
        {
            return this->_focus;
        }
        const T getZoom() const
        {
            return this->_zoom;
        }
        const T getIris() const
        {
            return this->_iris;
        }
        const std::tuple<T, T, T> getFocusZoomIris() const
        {
            return std::make_tuple(this->_focus, this->_zoom, this->_iris);
        }

        std::string ToString() const
        {
            std::stringstream ret;
            ret << TXT_FOCUS << "=" << Utils::PadStr(14, 5) << this->_focus << std::endl
                << Utils::PadStr(pad_len) << TXT_ZOOM << "=" << Utils::PadStr(14, 4) << this->_zoom << std::endl
                << Utils::PadStr(pad_len) << TXT_IRIS << "=" << Utils::PadStr(14, 4) << this->_iris;
            return ret.str();
        }

        friend std::ostream& operator<<(std::ostream& strm, const FocusZoomIris& vals)
        {
            strm << vals.ToString();
            return strm;
        }
    };

/* ******************************************************************************** */

    /// <summary>  
    /// Extended status class used to hold ErrorCluster and other information (version, name)
    /// </summary> 
    class StatusEx : public ErrorCluster
    {
    public:

        const Status status;
        StatusEx(Status status = Status::GENERAL_ERROR, ErrorCluster err = ErrorCluster());
    };

    /// <summary>  
    /// Data container class to provide PIRIS name and version.
    /// </summary> 
    class DataID
    {
    public:

        const std::string name;
        const Version version;

        DataID(std::string name = "", Version version = Version());
    };

    /// <summary>  
    /// Position container class to provide PIRIS motor actual positions and ir filter state.
    /// </summary> 
    class DataPosition
    {
    public:

        const FocusZoomIris<uint16_t> position;
        const bool ir_filter;

        DataPosition(FocusZoomIris<uint16_t> pos = FocusZoomIris<uint16_t>(), bool ir = false);
    };

    /// <summary>  
    /// Params container class to provide PIRIS motor max values, lens type, ir and sensor present flag.
    /// </summary> 
    class DataParams
    {
    public:

        const FocusZoomIris<uint16_t> max_value;
        const std::string lens;
        const bool ir_present;
        const bool sens_present;

        DataParams(FocusZoomIris<uint16_t> max = FocusZoomIris<uint16_t>(), std::string lens = "", 
                   bool ir = false, bool sens = false);
    };

    /// <summary>  
    /// State container class to provide PIRIS motor status (OK/NOK) and motor busy information
    /// </summary>
    class DataState
    {
    public:

        const FocusZoomIris<bool> state;
        const bool motors_busy;

        DataState(FocusZoomIris<bool> state = FocusZoomIris<bool>(), bool busy = false);
    };

/* ******************************************************************************** */

    /// <summary>  
    /// Main public interface presenting all available methods, that can be called on each PIRIS object by user.
    /// </summary>
    Interface IDevice
    {
    public:
        
        //********************************************** open/close ***************************************************//
        /// <summary>  
        /// Opens connection to PIRIS device (use in rare cases).
        /// </summary>
        /// <param name="dev">void pointer to PirisDevice class</param>  
        /// <param name="type">type of PIRIS device used to convert void dev to specific type</param>  
        /// <param name="verbose">level of verbosity</param>  
        /// <returns>Status class</returns>  
        virtual Status Open(void* dev, PirisDeviceType type, VerboseLevel verbose = VerboseLevel::NONE) = 0;

        /// <summary>  
        /// Open connection to PIRIS device.
        /// </summary>
        /// <param name="dev">PirisDevice class</param>  
        /// <param name="verbose">level of verbosity</param>  
        /// <returns>Status class</returns>  
        virtual Status Open(const PirisDevice& dev, VerboseLevel verbose = VerboseLevel::NONE) = 0;

        /// <summary>  
        /// Close connection with PIRIS device.
        /// </summary>
        /// <returns>Status class</returns>  
        virtual Status Close() = 0;

        //************************************************ read ******************************************************//
        /// <summary>  
        /// Read ID (name, FW version) from PIRIS device.
        /// </summary>
        /// For compatibility reason it is obligated to call this method on every start, to set corret timeouts.
        /// <returns>tuple of extended Satus class StatusEx and data container, yelding the payload</returns>  
        virtual std::tuple<StatusEx, DataID> ReadID() = 0;

        /// <summary>  
        /// Read actual position of motors from PIRIS device.
        /// </summary>
        /// <returns>tuple of extended Satus class StatusEx and data container, yelding the payload</returns>  
        virtual std::tuple<StatusEx, DataPosition> ReadPosition() = 0;

        /// <summary>  
        /// Read parameters (max values of motors, lens type) from PIRIS device.
        /// </summary>
        /// <returns>tuple of extended Satus class StatusEx and data container, yelding the payload</returns> 
        virtual std::tuple<StatusEx, DataParams> ReadParams() = 0;

        /// <summary>  
        /// Read status (state of motors, motors are busy) from PIRIS device.
        /// </summary>
        /// <returns>tuple of extended Satus class StatusEx and data container, yelding the payload</returns> 
        virtual std::tuple<StatusEx, DataState> ReadState() = 0;

        //******************************************** reset/homing **************************************************//
        /// <summary>  
        /// Perform a reset of whole PIRIS device. It takes about 5-15 seconds and time depends on actual motor positions.
        /// </summary>
        /// <returns>extended Satus class StatusEx</returns> 
        virtual StatusEx DevReset() = 0;

        /// <summary>  
        /// Perform a reset of lens of PIRIS device. It takes about 5-15 seconds and time depends on actual motor positions.
        /// </summary>
        /// <returns>extended Satus class StatusEx</returns> 
        virtual StatusEx DevHoming() = 0;

        //************************************************* set ******************************************************//
        /// <summary>  
        /// Set focus, zoom, iris and ir filter values in absolute mode (every value is ranged from 0 to its max)
        /// </summary>
        /// <param name="focus">focus value</param>  
        /// <param name="zoom">zoom value</param>  
        /// <param name="iris">iris value</param>  
        /// <param name="ir_filter">ir filter value</param>  
        /// <returns>extended Satus class StatusEx</returns>  
        virtual StatusEx SetAbsolute(uint16_t focus = 0, 
                                     uint16_t zoom = 0, 
                                     uint16_t iris = 0, 
                                     bool ir_filter = false) = 0;

        /// <summary>  
        /// Set focus, zoom, iris and ir by container class in absolute mode (every value is ranged from 0 to its max)
        /// </summary>
        /// <param name="focus">focus value</param>  
        /// <param name="zoom">zoom value</param>  
        /// <param name="iris">iris value</param>  
        /// <param name="ir_filter">ir filter value</param>  
        /// <returns>extended Satus class StatusEx</returns>  
        virtual StatusEx SetAbsolute(const FocusZoomIris<uint16_t>& values, 
                                     bool ir_filter = false) = 0;

        /// <summary>  
        /// Set focus, zoom, iris and ir filter values in relative mode (every next value has its base 0 from previous)
        /// </summary>
        /// <param name="focus">focus value</param>  
        /// <param name="zoom">zoom value</param>  
        /// <param name="iris">iris value</param>  
        /// <param name="ir_filter">ir filter value</param>  
        /// <returns>extended Satus class StatusEx</returns>  
        virtual StatusEx SetRelative(int16_t focus = 0, 
                                     int16_t zoom = 0, 
                                     int16_t iris = 0) = 0;

        /// <summary>  
        /// Set focus, zoom, iris and ir by container class in relative mode (every next value has its base from previous)
        /// </summary>
        /// <param name="focus">focus value</param>  
        /// <param name="zoom">zoom value</param>  
        /// <param name="iris">iris value</param>  
        /// <param name="ir_filter">ir filter value</param>  
        /// <returns>extended Satus class StatusEx</returns>  
        virtual StatusEx SetRelative(const FocusZoomIris<int16_t>& values) = 0;

        //************************************************ utils *****************************************************//

        /// <summary>  
        /// Return information about device FW, if it supports continuous poll (>=1.7.2) returns Yes. More in PDF manual
        /// </summary>
        /// <returns>3 state logic enum can be Yes, No or NA</returns> 
        virtual YesNoNA GetFwPollSupport() = 0;

        /// <summary>  
        /// Return level of verbosity of current PIRIS instance.
        /// </summary>
        /// <returns>Level of verbosity can be FULL, BASIC or NONE</returns> 
        virtual VerboseLevel GetVerboseLevel() = 0;

        /// <summary>  
        /// Set the level of verbosity of current PIRIS instance.
        /// </summary>
        /// <param name="level">Level of verbosity can be FULL, BASIC or NONE</param>  
        virtual void SetVerboseLevel(VerboseLevel level) = 0;

        virtual ~IDevice() = 0;
    };

    /// <summary>  
    /// Internal interface used to provide polymorphic communication functionality (Serial, Ethernet)
    /// </summary>
    Interface IComm
    {
    public:

        /// <summary>  
        /// Open connection to PIRIS device. Must be overridden in child.
        /// </summary>
        /// <param name="dev">PIRIS device container class</param>
        /// <param name="verbose">level of verbosity</param>
        /// <returns>Status class</returns> 
        virtual Status open(const PirisDevice& dev, VerboseLevel verbose = VerboseLevel::NONE) = 0;

        /// <summary>  
        /// Close connection to PIRIS device. Must be overridden in child.
        /// </summary>
        /// <param name="ignore_err">If true, force quit connection and discard any exception</param>
        /// <returns>Status class</returns> 
        virtual Status close(bool ignore_err = false) = 0;

        /// <summary>  
        /// Write data to PIRIS device. Must be overridden in child.
        /// </summary>
        /// <param name="data">ASCII data defined by ATEsystem PIRIS protocol (datasheet)</param>
        /// <returns>Status class</returns> 
        virtual Status write(const std::string data) = 0;

        /// <summary>  
        /// Read data to PIRIS device. Must be overridden in child.
        /// </summary>
        /// <param name="data">ASCII data defined by ATEsystem PIRIS protocol (datasheet)</param>
        /// <returns>Status class</returns> 
        virtual Status read(std::string& data) = 0;

        /// <summary>  
        /// Flush comm rx/tx buffer of PIRIS device. Must be overridden in child.
        /// </summary>
        /// <param name="ignore_err">If true, discard any exception</param>
        /// <returns>Status class</returns> 
        virtual Status flush(bool ignore_err = false) = 0;

        /// <summary>  
        /// Remove PIRIS device. Must be overridden in child.
        /// </summary>
        virtual void remove() = 0;

        virtual ~IComm() = 0;
    };

/* ******************************************************************************** */

    /// <summary>  
    /// Main abstract class with all key functionality, need to have overridden comm func in derived class.
    /// </summary>
    class CPirisMain : public IComm, public IDevice
    {
    public:

        static int16_t ScanEthernet(std::vector<Pylon::CDeviceInfo>& devices, bool verbose = false);
        static int16_t ScanSerial(std::vector<serial::PortInfo>& devices, bool verbose = false);

        Status Open(void* dev, PirisDeviceType type, VerboseLevel verbose = VerboseLevel::NONE);
        Status Open(const PirisDevice& dev, VerboseLevel verbose = VerboseLevel::NONE);
        Status Close();

        std::tuple<StatusEx, DataID> ReadID();              // IDN - !! need to call this on every start to set timeouts
        std::tuple<StatusEx, DataPosition> ReadPosition();  // GP
        std::tuple<StatusEx, DataParams> ReadParams();      // GT
        std::tuple<StatusEx, DataState> ReadState();        // GS

        StatusEx DevReset();     // RST
        StatusEx DevHoming();    // INI

        StatusEx SetAbsolute(uint16_t focus = 0, 
                             uint16_t zoom = 0, 
                             uint16_t iris = 0, 
                             bool ir_filter = false);
        StatusEx SetAbsolute(const FocusZoomIris<uint16_t>& values,   // SETA:FX;ZX;PX;IX
                             bool ir_filter = false);       
        StatusEx SetRelative(int16_t focus = 0, 
                             int16_t zoom = 0, 
                             int16_t iris = 0);
        StatusEx SetRelative(const FocusZoomIris<int16_t>& values);   // SETR:FX;ZX;PX

        YesNoNA GetFwPollSupport();
        VerboseLevel GetVerboseLevel();
        void SetVerboseLevel(VerboseLevel level);

        CPirisMain();
        virtual ~CPirisMain() {};

    protected:

        uint16_t rx_timeout = RX_TIMEOUT_NEW_FW;

        void state_set(State _state);
        State state_get();
        bool state_is(State _state);
        void state_reset();
        bool verbose_is(VerboseLevel _verbose);
        void verbose_set(VerboseLevel _verbose);

    private:

        VerboseLevel _verbose;
        State state = State::CLOSED;
        State _state = State::CLOSED;
        YesNoNA fw_poll_support = YesNoNA::NOT_AVAILABLE; // device firmware is 1.7.2 and higher (stat read poll supp)

        ErrorCluster parse_answer(std::string answer_in, std::vector<std::string>& tokens_out, uint8_t n);
        StatusEx send_msg(std::vector<std::string>& tokens_out, std::string name, std::string cmd, uint8_t len);
    };

/* ******************************************************************************** */

    /// <summary>  
    /// Generic Smart Pointer class, that can be used for mem mng of any type. Implementation by Basler AG.
    /// </summary>
    template <class T>
    class SmartPointer
    {
    private:

        T* m_pT;

    public:
        SmartPointer(void) noexcept: m_pT(NULL)
        {
        }

        ~SmartPointer(void)
        {
            delete m_pT;
        }

        void operator=(T *pB)
        {
            m_pT = dynamic_cast<T*>(pB);
        }

        operator T*(void) const
        {
            return m_pT;
        }

        T& operator*(void) const
        {
            return *m_pT;
        }

        T& operator()(void) const
        {
            return *m_pT;
        }

        T* operator->(void) const
        {
            return m_pT;
        }

        bool IsValid() const throw()
        {
            return m_pT != NULL;
        }

        operator bool(void) const throw()
        {
            return m_pT != NULL;
        }

        bool operator==(T* pT) const
        {
            return m_pT == pT;
        }
    };

    /// <summary>  
    /// IPiris Smart Pointer is main object, that represents single PIRIS device. No need to delete after.
    /// </summary>
    typedef SmartPointer<IDevice> IPiris;

/* ******************************************************************************** */

    /// <summary>  
    /// Factory helper function pointer.
    /// </summary>
    typedef IDevice* (*CreateDeviceFn)(void);

    /// <summary>  
    /// Factory class for manage PIRIS unique instances. Driver supports multiple opened PIRIS devices,
    /// that can be easily manipulated by unique DevID and IPiris Smart Pointer.
    /// </summary>
    class Factory
    {
    private:

        typedef std::map<Mode, CreateDeviceFn> FactoryMap;
        typedef std::map<DevID_t, std::pair<std::string, IDevice*>> DeviceMap;    // CPirisMain ?
        FactoryMap _factoryMap;
        DeviceMap _deviceMap;

        Factory();
        void Register(Mode mode, CreateDeviceFn pfnCreate);

    public:

        Factory(Factory const&) = delete;
        Factory &operator=(Factory const&) = delete;
        ~Factory() { _factoryMap.clear(); }

        /// <summary>  
        /// Return instance to singleton Factory class.
        /// </summary>
        /// <returns>Factory class pointer</returns>  
        static Factory* getInstance()
        {
            static Factory instance;
            return &instance;
        }

        /// <summary>  
        /// Create a PIRIS device with random string name. Usually first method to call.
        /// </summary>
        /// <param name="mode">Mode - PIRIS device product type (SERIAL/ETHERNET)</param>  
        /// <returns>Device unique id</returns>  
        DevID_t CreateDevice(Mode mode = Mode::ETHERNET);

        /// <summary>  
        /// Create a PIRIS device with specific string name. Usually first method to call.
        /// </summary>
        /// <param name="name">string name of created object</param>  
        /// <param name="mode">PIRIS device product type (SERIAL/ETHERNET)</param>  
        /// <returns>PIRIS Device unique id</returns>  
        DevID_t CreateDevice(const std::string& name, Mode mode);

        /// <summary>  
        /// Return Smart Pointer IPiris (IDevice). Usually call this when need to do any action with PIRIS.
        /// </summary>
        /// <param name="id">PIRIS Device unique id</param>  
        /// <returns>Smart Pointer IPiris (IDevice)</returns>  
        IDevice* GetDeviceInstance(DevID_t id);

        /// <summary>  
        /// Return PIRIS device name, either random or specified string.
        /// </summary>
        /// <param name="id">PIRIS Device unique id</param>  
        /// <returns>PIRIS device string name</returns>  
        std::string GetDeviceName(DevID_t id);

        /// <summary>  
        /// Remove PIRIS device. Usually call this, when driver is used to manipulate multiple PIRIS devices.
        /// </summary>
        /// <param name="id">PIRIS Device unique id</param>  
        /// <returns>Status class</returns>  
        Status RemoveDevice(DevID_t id);
    };
}