#ifndef IFR_ROS2_CV__PACKAGE_RM_SERIAL_PORT__SERIALPORT__H
#define IFR_ROS2_CV__PACKAGE_RM_SERIAL_PORT__SERIALPORT__H
#include "ifr_common/macros.hpp"
#include "serial/serial.h"
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <ifr_common/ext_funcs.h>
#include <iomanip>
#include <mutex>
#include <optional>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

namespace rm_serial_port {
    /// @brief 代表一个缓冲区, 不可复制
    /// @details 内部使用std_msgs::msg::String存储数据
    class Buffer {
        IFR_CLS_PTR(Buffer)
        IFR_CLS_NO_COPY_C(Buffer)

    public:
        Buffer() = default;
        explicit Buffer(std::size_t length);


        template<class T = uint8_t>
        const T *data(size_t off_byte = 0) const { return reinterpret_cast<const T *>(data_.data.data() + off_byte); }/// 获取指针
        template<class T = uint8_t>
        T *data(size_t off_byte = 0) { return reinterpret_cast<T *>(data_.data.data() + off_byte); }/// 获取指针
        template<class T>
        const T &ref(size_t off_byte = 0) const { return *reinterpret_cast<const T *>(data_.data.data() + off_byte); }/// 获取引用
        template<class T>
        T &ref(size_t off_byte = 0) { return *reinterpret_cast<T *>(data_.data.data() + off_byte); }/// 获取引用


        std::size_t length() const;/// buffer有效长度
        void resize(size_t sz);    /// 调整buffer大小


        const std_msgs::msg::String &raw() const;/// 获取原始数据
        std_msgs::msg::String &raw();            /// 获取原始数据


        /// @brief 向后插入数据
        void append(const Buffer &other, size_t offset = 0, size_t sz = -1);
        /// @brief 向前插入数据
        void prepend(const Buffer &other, size_t offset = 0, size_t sz = -1);

        /// @brief 从buffer读取数据
        size_t read(uint8_t *output, size_t need);

    private:
        std_msgs::msg::String data_;
    };


    namespace helpers {
        template<typename, typename = std::void_t<>>
        struct has_check_method : std::false_type {};
        template<typename T>
        struct has_check_method<T, std::void_t<decltype(std::declval<const T>().check())>>
            : std::is_same<decltype(std::declval<const T>().check()), bool> {};
        template<typename T>
        constexpr bool has_check_method_v = has_check_method<T>::value;//检测是否有 bool check() const;


        template<typename, typename = std::void_t<>>
        struct has_toRos_method : std::false_type {};
        template<typename T>
        struct has_toRos_method<T, std::void_t<decltype(std::declval<const T>().toRos())>>
            : std::true_type {
            using Type = decltype(std::declval<const T>().toRos());
        };
        template<typename T>
        constexpr bool has_toRos_method_v = has_toRos_method<T>::value;//检测是否有 Type toRos() const;

        template<typename, typename, typename = std::void_t<>>
        struct has_toRosT_method : std::false_type {};
        template<typename T, typename _Type>
        struct has_toRosT_method<T, _Type, std::void_t<decltype(std::declval<const T>(std::declval<_Type &>()).toRos())>>
            : std::is_same<std::decay_t<decltype(std::declval<const T>().toRos())>, _Type> {
            using Type = _Type;
        };
        template<typename T, typename Type>
        constexpr bool has_toRosT_method_v = has_toRos_method<T, Type>::value;//检测是否有 Type toRos(Type&) const;


        template<typename T, typename X, typename = std::void_t<>>
        struct has_HEAD : std::false_type {};
        template<typename T, typename X>
        struct has_HEAD<T, X, std::void_t<decltype(std::declval<T>().HEAD)>>
            : std::is_same<std::decay_t<decltype(T::HEAD)>, X> {};
        template<typename T, typename X>
        constexpr bool has_HEAD_v = has_HEAD<T, X>::value;//检测是否有静态成员X HEAD

    }// namespace helpers


    class SerialPort {
    private:
        rclcpp::Node *node;

        serial::Serial *serial_ = nullptr;
        Buffer buffer_;//内部缓冲, 仅在不得不读取但又需要归还由任一方读取时才可放入
        std::chrono::steady_clock::time_point lst_open{};
        rclcpp::TimerBase::SharedPtr reopen_timer;
        std::mutex serial_mtx;

        std::optional<std::thread> reader;
        std::optional<std::chrono::steady_clock::time_point> lst_read_time;//最后一次成功读取的时间, 如果存在则代表启用了no_read_no_send
        static constexpr std::chrono::steady_clock::duration wait_read_time = std::chrono::seconds(2);

    public:
        explicit SerialPort(rclcpp::Node *node);

        virtual ~SerialPort();

        /// @brief 将一个对象转换为 hex 字符串
        /// @param t 对象
        /// @param space 分隔符(设为\0为空)
        /// @return 字符串
        template<char space = ' ', class T = void>
        static std::string toHex(const T &t) {
            const uint8_t *data = reinterpret_cast<const uint8_t *>(&t);
            std::ostringstream oss;
            for (size_t i = 0; i < sizeof(T); i++) {
                if constexpr (space) {
                    if (i) oss << space;
                }
                oss << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(*(data + i));
            }
            return oss.str();
        }

        /// @brief 设置多头读取器
        /// @details 通过注册处理函数，激活自动读取线程, 不断从串口中读取数据, 校验通过后调用func进行后续处理
        /// @details 将会在数据流中查找包头, 首先匹配到的包类型会优先处理, 如果check失败则继续查找包头, 直到找到check成功的包类型, 调用对应的func
        /// @param func 处理函数, 应尽快处理, 会阻塞数据读取. 如果需要紧邻的继续读取, 应在func内读取
        /// @tparam Package 读取的数据包类型, 需要具有static const uint8_t HEAD=XXX; 和 bool check()const; 和无参公开构造函数
        template<class... Packages>
        void setMultiHeadReader(std::function<void(const Packages &pkg)>... funcs) {
            static_assert((... && (helpers::has_check_method_v<Packages>) ), "数据包需要实现: bool check()const;");
            static_assert((... && (helpers::has_HEAD_v<Packages, uint8_t>) ), "数据包需要具有: static const uint8_t HEAD=XXX;");
            static_assert((... && (std::is_default_constructible_v<Packages>) ), "数据包需要具有: 无参公开构造函数");
            if (reader) {
                RCLCPP_FATAL(node->get_logger(), "重复设置读取器! 请检查代码逻辑");
                throw std::runtime_error("[" + std::string{node->get_name()} + "] 重复设置读取器! 请检查代码逻辑");
            }
            using HEAD_T = uint8_t;

            std::vector<std::tuple<std::string, HEAD_T, size_t, std::function<bool(Buffer & buf)>>> infos;
            (..., infos.emplace_back(
                          demangle(typeid(Packages).name()),
                          Packages::HEAD,
                          sizeof(Packages),
                          [this, func = std::move(funcs), name = demangle(typeid(Packages).name())](Buffer &buf) {
                              auto &pkg = buf.ref<Packages>();
                              auto ret = pkg.check();
                              if (ret) func(pkg);
                              else
                                  RCLCPP_WARN_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 2e3, "Pkg frame check fail: " << name << ", hex: " << toHex(pkg));
                              return ret;
                          }));
            reader.emplace([this, infos = std::move(infos),
                            min_pkg = std::min({sizeof(Packages)...}),//最小读取
                            max_pkg = std::max({sizeof(Packages)...}) //最大读取
            ]() {
                Buffer buf(max_pkg);
                size_t now_sz = 0;
                while (rclcpp::ok()) {
                    if (!readByte(buf.data(), min_pkg)) continue;
                    now_sz = min_pkg;
                    for (size_t i = 0; (i <= now_sz - sizeof(HEAD_T)); ++i) {
                        auto &head = buf.ref<HEAD_T>(i);
                        for (const auto &[name, HEAD, sz, func]: infos) {
                            if (head != HEAD) continue;
                            
                            if (i > 0) std::memmove(buf.data(), buf.data(i), buf.length() - i);//有效位减少i
                            now_sz -= i;
                            if (auto need = sz - now_sz; need > 0) {//补充
                                if (readByte(buf.data(now_sz), need)) now_sz = sz;
                                else {
                                    now_sz = 0;
                                    goto read_new;
                                }
                            }
                            if (func(buf)) {                      //成功处理
                                if (auto r = now_sz - sz; r > 0) {//剩余数据归还
                                    buffer_.prepend(buf, sz, r);
                                }
                                now_sz = 0;
                                goto read_new;
                            } else if (UNLIKELY(!rclcpp::ok()))
                                goto read_new;
                            i = static_cast<size_t>(0);//当前头已经对齐buf头, 置0可使下一次读取为buf[1]
                            break;
                        }
                    }
                read_new:
                    continue;
                }
            });
        }
        /// @brief 设置读取器
        /// @details 通过注册处理函数，激活自动读取线程, 不断从串口中读取数据, 校验通过后调用func进行后续处理
        /// @param func 处理函数, 应尽快处理, 会阻塞数据读取. 如果需要紧邻的继续读取, 应在func内读取
        /// @tparam Package 读取的数据包类型, 需要具有static const uint8_t HEAD=XXX; 和 bool check()const; 和无参公开构造函数
        template<class Package>
        void setReader(std::function<void(const Package &pkg)> func) {
            static_assert(helpers::has_check_method_v<Package>, "数据包需要实现: bool check()const;");
            static_assert(helpers::has_HEAD_v<Package, uint8_t>, "数据包需要具有: static const uint8_t HEAD=XXX;");
            static_assert(std::is_default_constructible_v<Package>, "数据包需要具有: 无参公开构造函数");
            if (reader) {
                RCLCPP_FATAL(node->get_logger(), "重复设置读取器! 请检查代码逻辑");
                throw std::runtime_error("[" + std::string{node->get_name()} + "] 重复设置读取器! 请检查代码逻辑");
            }
            reader.emplace([this, func = std::move(func), name = demangle(typeid(Package).name())]() {
                static constexpr const auto HEAD = Package::HEAD;
                Package pkg;
                bool idata = true;//是否有残缺数据
                while (rclcpp::ok()) {
                    if (!(idata = read(reinterpret_cast<uint8_t *>(&pkg), sizeof(pkg), HEAD, idata))) continue;
                    if ((idata = !pkg.check())) {
                        RCLCPP_WARN_STREAM_THROTTLE(
                                node->get_logger(), *node->get_clock(), 2e3,
                                "Pkg frame check fail: " << name << ", hex: " << toHex(pkg)//
                        );
                        continue;
                    }
                    func(pkg);
                }
            });
        }
        /// @brief 读取串口
        /// @param buffer 数据缓冲区
        /// @param sz 读取总长度
        /// @return true: 成功读取有效数据, 可以正常处理; false: 读取失败, buffer任意部分不应看做一个有效的数据做处理
        bool read(uint8_t *buffer, const size_t &sz);

        inline bool read(Buffer &buffer) { return read(buffer.data(), buffer.length()); }                   /// 读取到buffer
        inline bool read(const Buffer::SharedPtr &buffer) { return read(buffer->data(), buffer->length()); }/// 读取到buffer
        inline bool read(const Buffer::UniquePtr &buffer) { return read(buffer->data(), buffer->length()); }/// 读取到buffer

        /// @brief 读取串口
        /// @param buffer 数据缓冲区
        /// @param sz 读取总长度
        /// @param find_header 代表要查找的头, 会确保将头放在buffer开始部分才可能返回true
        /// @param has_data buffer中是否有可能剩余的数据部分, 仅在header不为0的情况下有效. 例如: 对数据包check返回false, 但buffer中可能藏有真正的header
        /// @return true: 成功读取有效数据, 可以正常处理; false: 读取失败, buffer任意部分不应看做一个有效的数据做处理
        bool read(uint8_t *buffer, const size_t &sz, uint8_t header, bool has_data = false);


        ///@return 返回是否成功
        bool writeByte(const uint8_t *const buffer, const size_t length);

        /// @brief 输出数据
        /// @param data 待写出的数据
        ///@return 返回是否成功
        template<class SendT>
        inline bool writeData(const SendT &data) {
            return writeByte(reinterpret_cast<const uint8_t *>(&data), sizeof(SendT));
        }


        void open(); ///打开串口
        void close();///关闭串口


    private:
        uint16_t read_fail_time = 0, read_reopen_time = 0;
        static const uint16_t read_fail_threshold = 2, read_reopen_threshold = 5;


        /// @brief 底层读取方法, 处理错误
        ///@return 是否成功读取
        bool readByte(uint8_t *buffer, const size_t need);


        template<class T, class Min, class Max>
        void declare_parameter_r(std::string name, const T &t, const Min &min_t, const Max &max_t) {
            rcl_interfaces::msg::ParameterDescriptor desc;
            rcl_interfaces::msg::IntegerRange range;
            range.from_value = static_cast<int64_t>(min_t), range.to_value = static_cast<int64_t>(max_t);
            desc.integer_range.push_back(range);
            node->declare_parameter(name, t, desc);
        }


        /// @brief 从给定数据范围查找数据头并对齐
        /// @return 是否成功; 返回true代表成功找到数据头, 并已将头移动到buffer起始位置, 并读取填充了剩余数据. 返回false代表未找到头或者读取剩余数据时出错.
        ///         在返回false时, buffer数据不应再当做有效数据
        bool alignHeader(uint8_t *buffer, const size_t &sz, uint8_t header, bool skipFst = false);

        static std::string demangle(const char *mangled_name);
    };

}// namespace rm_serial_port
#endif// IFR_ROS2_CV__PACKAGE_RM_SERIAL_PORT__SERIALPORT__H
