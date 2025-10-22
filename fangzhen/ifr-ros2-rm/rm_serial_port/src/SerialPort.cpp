#include "rm_serial_port/SerialPort.h"


#ifndef _MSC_VER
#include <cxxabi.h>
#endif
namespace rm_serial_port {

    Buffer::Buffer(std::size_t length) {
        data_.data.resize(length);
    }


    std::size_t Buffer::length() const { return data_.data.length(); }


    void Buffer::resize(size_t sz) { data_.data.resize(sz); }


    const std_msgs::msg::String &Buffer::raw() const { return data_; }


    std_msgs::msg::String &Buffer::raw() { return data_; }


    void Buffer::append(const Buffer &other, size_t offset, size_t sz) {
        if (UNLIKELY(offset > other.length())) throw std::out_of_range("Offset is out of range in the source buffer");
        if (sz == static_cast<size_t>(-1)) sz = other.length() - offset;
        if (UNLIKELY(offset + sz > other.length())) throw std::out_of_range("Size exceeds available data in the source buffer");
        size_t current_size = length();
        resize(current_size + sz);
        std::memcpy(data<uint8_t>(current_size), other.data<uint8_t>(offset), sz);
    }


    void Buffer::prepend(const Buffer &other, size_t offset, size_t sz) {
        if (UNLIKELY(offset > other.length())) throw std::out_of_range("Offset is out of range in the source buffer");
        if (sz == static_cast<size_t>(-1)) sz = other.length() - offset;
        if (UNLIKELY(offset + sz > other.length())) throw std::out_of_range("Size exceeds available data in the source buffer");
        size_t current_size = length();
        resize(current_size + sz);
        std::memmove(data<uint8_t>(sz), data<uint8_t>(), current_size);
        std::memcpy(data<uint8_t>(), other.data<uint8_t>(offset), sz);
    }


    size_t Buffer::read(uint8_t *other, size_t need) {
        size_t read = std::min(need, length());
        if (read > 0) std::memcpy(other, data<uint8_t>(), read);
        return read;
    }

    SerialPort::SerialPort(rclcpp::Node *node) : node(node) {

        node->declare_parameter("port", "/dev/ttyTHS0");
        this->declare_parameter_r("baudrate", 115200, 1, 115200);
        this->declare_parameter_r("timeout", 500, 0, 10 * 1000);
        this->declare_parameter_r("bytesize", 8, serial::fivebits, serial::eightbits);
        this->declare_parameter_r("parity", 0, serial::parity_none, serial::parity_space);
        this->declare_parameter_r("stopbits", 1, serial::stopbits_one, serial::stopbits_one_point_five);
        this->declare_parameter_r("flowcontrol", 0, serial::flowcontrol_none, serial::flowcontrol_hardware);
        node->declare_parameter<bool>("no_get_no_send", false);
        open();

        reopen_timer = node->create_wall_timer(std::chrono::seconds(1), [this]() {
            if (auto serial = serial_; serial == nullptr || !serial->isOpen()) {
                open();
            }
        });
    }


    SerialPort::~SerialPort() {
        close();
    }


    bool SerialPort::read(uint8_t *buffer, const size_t &sz) {
        return readByte(buffer, sz);
    }


    bool SerialPort::read(uint8_t *buffer, const size_t &sz, uint8_t header, bool has_data) {
        if (has_data && alignHeader(buffer, sz, header, true)) return true;// 对于buffer有可能的残余数据, 但当前完整数据包是无效时

        if (!readByte(buffer, sz)) return false;//没有残余数据
        return alignHeader(buffer, sz, header, false);
    }


    bool SerialPort::alignHeader(uint8_t *buffer, const size_t &sz, uint8_t header, bool skipFst) {
        using HEAD_T = decltype(header);
        if (!skipFst && *reinterpret_cast<HEAD_T *>(buffer) == header) return true;
        for (size_t i = 1; i <= sz - sizeof(HEAD_T); ++i) {
            if (*reinterpret_cast<HEAD_T *>(buffer + i) == header) {
                std::memmove(buffer, buffer + i, sz - i);
                if (!readByte(buffer + (sz - i), i)) return false;
                return true;
            }
        }
        return false;
    }


    bool SerialPort::writeByte(const uint8_t *const buffer, const size_t length) {
        auto serial = serial_;
        if (serial && serial->isOpen()) {
            if (lst_read_time && (std::chrono::steady_clock::now() - *lst_read_time) > wait_read_time) {
                RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2e3, "No serial port data is received, stop sending. Wait for the 控制组 to restart and send again");
                return false;
            }
            try {
                return serial->write(buffer, length) == length;
            } catch (std::exception &e) {
                RCLCPP_ERROR_STREAM(node->get_logger(), "Can not write; " << demangle(typeid(e).name()) << ": " << e.what());
                if (!serial->isOpen()) {
                    RCLCPP_WARN(node->get_logger(), "Serial port is not turned on, attempting to turn it on.");
                    open();
                }
            }
        } else if (serial) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1e4, "Serial port not opened! attempting to open.");
            open();
        }
        return false;
    }


    void SerialPort::open() {
        close();
        std::lock_guard lock(serial_mtx);
        auto now = std::chrono::steady_clock::now();
        if (now - lst_open < std::chrono::seconds(1)) return;
        lst_open = now;
        std::string port = node->get_parameter("port").as_string();
        int64_t baudrate = node->get_parameter("baudrate").as_int();
        int64_t timeout = node->get_parameter("timeout").as_int();
        int64_t bytesize = node->get_parameter("bytesize").as_int();
        int64_t parity = node->get_parameter("parity").as_int();
        int64_t stopbits = node->get_parameter("stopbits").as_int();
        int64_t flowcontrol = node->get_parameter("flowcontrol").as_int();

        if (node->get_parameter("no_get_no_send").as_bool()) lst_read_time = std::chrono::steady_clock::now();
        RCLCPP_INFO(node->get_logger(), "no receive no send(no_get_no_send): %c", lst_read_time ? 'T' : 'F');

        RCLCPP_INFO_STREAM(node->get_logger(), "port = " << port);
        try {
            serial_ = new serial::Serial(
                    port,
                    baudrate,
                    serial::Timeout::simpleTimeout(timeout),
                    static_cast<serial::bytesize_t>(bytesize),
                    static_cast<serial::parity_t>(parity),
                    static_cast<serial::stopbits_t>(stopbits),
                    static_cast<serial::flowcontrol_t>(flowcontrol));
            RCLCPP_DEBUG_STREAM(node->get_logger(), "port = " << serial_->getPort());
            RCLCPP_DEBUG_STREAM(node->get_logger(), "baudrate = " << serial_->getBaudrate());
            RCLCPP_DEBUG_STREAM(node->get_logger(), "bytesize = " << serial_->getBytesize());
            RCLCPP_DEBUG_STREAM(node->get_logger(), "parity = " << serial_->getParity());
            RCLCPP_DEBUG_STREAM(node->get_logger(), "stopbits = " << serial_->getStopbits());
            RCLCPP_DEBUG_STREAM(node->get_logger(), "flowcontrol = " << serial_->getFlowcontrol());
            if (serial_->isOpen())
                RCLCPP_INFO_STREAM(node->get_logger(), "Serial port turned on successfully: " << port);
            else {
                RCLCPP_FATAL_STREAM(node->get_logger(), "Can not open serial port: " << port);
                // exit(-1);
            }
        } catch (std::exception &e) {
            RCLCPP_ERROR_STREAM(node->get_logger(), "Can not open serial port: " << port << ", err = " << demangle(typeid(e).name()) << ": " << e.what());
            // exit(-1);
        }
    }


    void SerialPort::close() {
        std::lock_guard lock(serial_mtx);
        auto serial = serial_;
        serial_ = nullptr;
        if (serial != nullptr) {
            try {
                serial->close();
                RCLCPP_INFO_STREAM(node->get_logger(), "Serial port turned off successfully: " << serial->getPort());
            } catch (std::exception &e) {
                RCLCPP_FATAL_STREAM(node->get_logger(), "Failed to close serial; " << demangle(typeid(e).name()) << ": " << e.what());
            }
        }
    }


    bool SerialPort::readByte(uint8_t *buffer, const size_t need) {
        if (reader && std::this_thread::get_id() != reader->get_id()) {
            RCLCPP_FATAL(node->get_logger(), "在读取线程外调用读取! 请检查代码逻辑");
            throw std::runtime_error("[" + std::string{node->get_name()} + "] 在读取线程外调用读取! 请检查代码逻辑");
        }
        auto serial = serial_;
        if (!serial) {
            //等待timer自动打开
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1e4, "Serial port not opened! wait to open.");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            return false;
        }
        size_t readed = 0;
        while (rclcpp::ok()) {
            try {
                readed += buffer_.read(buffer, need - readed);
                if (readed < need) {
                    readed += serial->read(buffer, need - readed);
                    read_fail_time = read_reopen_time = 0;
                }
            } catch (std::exception &e) {
                RCLCPP_ERROR_STREAM(node->get_logger(), "Can not read, " << demangle(typeid(e).name()) << ": " << e.what());
                if (++read_fail_time > read_fail_threshold) {
                    read_fail_time = read_fail_threshold;
                    if (++read_reopen_time > read_reopen_threshold) {
                        read_reopen_time = read_reopen_threshold;
                        RCLCPP_WARN(node->get_logger(), "Fail to reopen serial port, exit.");
                        close();
                    }
                    RCLCPP_WARN(node->get_logger(), "Serial port has too many errors, attempting to restart");
                    read_fail_time = 0;
                    open();
                } else if (!serial->isOpen()) {
                    RCLCPP_WARN(node->get_logger(), "Serial port is not turned on, attempting to turn it on.");
                    open();
                }
                return false;
            }
            if (readed < need) {
                buffer += readed;
                RCLCPP_WARN_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 1e4, "Read " << readed << "/" << need << " bytes");
            } else {
                if (lst_read_time) lst_read_time = std::chrono::steady_clock::now();
                return true;
            }
        }
        return false;
    }


    std::string SerialPort::demangle(const char *mangled_name) {
#ifndef _MSC_VER
        std::size_t len = 0;
        int status = 0;
        std::unique_ptr<char, decltype(&std::free)> ptr(
                __cxxabiv1::__cxa_demangle(mangled_name, nullptr, &len, &status),
                &std::free);
        if (status == 0) {
            return std::string(ptr.get());
        }
        return "";
#else
        auto pos = strstr(mangled_name, " ");
        if (pos == nullptr)
            return std::string{mangled_name};
        else
            return std::string{pos + 1};
#endif
    }
}// namespace rm_serial_port