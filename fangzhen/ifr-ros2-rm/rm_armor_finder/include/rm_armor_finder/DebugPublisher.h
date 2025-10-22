#ifndef IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__DEBUGPUBLISHER__H
#define IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__DEBUGPUBLISHER__H
#include "ifr_watcher/cpu_watcher.hpp"
#include <chrono>
#include <cstdint>
#include <foxglove_msgs/msg/image_marker_array.hpp>
#include <functional>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <string>
#include <visualization_msgs/msg/detail/image_marker__struct.hpp>
#include <visualization_msgs/msg/image_marker.hpp>

namespace rm_armor_finder {
    /// @brief 图像推送
    /// @details 以注册形式, 由timer定期在异步线程唤醒计算
    class ImgPublisher {
        ifr_watcher::cpu::RegisterClient cpu_watcher;

    public:
        using Img = sensor_msgs::msg::CompressedImage;

    private:
        struct Data {
            rclcpp::Publisher<Img>::SharedPtr pub;//发布者, 必须存在
            Img img;                              //暂存图像
            std::function<void(Img &)> caller;    //发布调用
            int err_cnt = 0;
        };
        using DataPtr = std::shared_ptr<Data>;

        rclcpp::Node *const node;
        std::vector<std::shared_ptr<Data>> datas;
        const float fidms;// frame interval duration milliseconds 帧间隔(毫秒, 1000/fps)

        std::thread timer_;
        void timer_callback();

    public:
        /// @brief 创建消息推送器
        /// @param node 定时器注册节点
        /// @param fps 推送帧率
        ImgPublisher(rclcpp::Node *node, float fps = 25);


        /// @brief 注册一个消息推送
        /// @param func
        DataPtr make_data(const std::string &name, const std::string &publish, const std::function<void(Img &)> &func);

        /// @brief 注册一个消息推送
        /// @param input 输入数据 cv::Mat / sensor_msgs::msg::CompressedImage
        template<class T>
        void register_data(const std::string &name, const std::string &publish, const T &input) {
            constexpr auto isSMat = std::is_same_v<T, std::shared_ptr<cv::Mat>>;
            constexpr auto isMat = std::is_same_v<T, cv::Mat>;
            constexpr auto isCI = std::is_same_v<T, sensor_msgs::msg::CompressedImage>;
            make_data(name, publish, [&input](Img &img) {
                static_assert(isSMat || isMat || isCI, "Can not parse T!");
                if constexpr (isSMat) {
                    if (input) {
                        img.format = "jpeg";
                        cv::imencode(".jpg", *input, img.data);
                    }
                } else if constexpr (isMat) {
                    img.format = "jpeg";
                    cv::imencode(".jpg", input, img.data);
                } else if constexpr (isCI) {
                    img.format = input.format;
                    img.data = input.data;
                }
            });
        }

        /// @brief 注册一个消息推送
        /// @param input 订阅节点
        /// @details 订阅数据类型需要为 sensor_msgs::msg::CompressedImage
        template<class T>
        void register_data(const std::string &name, const std::string &publish, const std::string &subscription) {
            auto sdata = make_data(name, publish, [](Img &img) {});
            auto last_time = std::make_shared<std::chrono::steady_clock::time_point>(std::chrono::steady_clock::now());
            node->create_subscription<T>(
                    subscription, rclcpp::SensorDataQoS(),
                    [last_time, sdata, this](const typename T::SharedPtr msg) {
                        auto now = std::chrono::steady_clock::now();
                        auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - *last_time);
                        if (time_diff.count() < fidms) return;
                        constexpr auto isCI = std::is_same_v<T, sensor_msgs::msg::CompressedImage>;
                        static_assert(isCI, "Can not parse T!");
                        if constexpr (isCI) {
                            sdata->img.format = msg->format;
                            sdata->img.data = msg->data;
                        }
                    });
        }
    };

    /// @brief 图像标注推送
    /// @details 以注册形式, 由timer定期在异步线程唤醒计算, 注册的函数调用draw接口绘制
    class ImgMarkerPublisher {
        ifr_watcher::cpu::RegisterClient cpu_watcher;
        struct Data;

    public:
        using token_t = const std::uint32_t &;
        using Color = const std_msgs::msg::ColorRGBA &;
        using CallFunc = std::function<void(ImgMarkerPublisher &, Data &, token_t)>;

    private:
        struct Data {
            using Markers = foxglove_msgs::msg::ImageMarkerArray;
            using Publisher = rclcpp::Publisher<Markers>::SharedPtr;
            const std::string name;
            Markers msg_marker_array;
            Publisher pub_marker_array;
            const CallFunc func;
            std::uint32_t id = 0;///< 渲染目标id, 每次渲染初会置零
            Data(const std::string &name, const Publisher &publisher, const CallFunc &func)
                : name(name), msg_marker_array(), pub_marker_array(publisher), func(func) {}
        };

    public:
        /// @brief 创建消息推送器
        /// @param node 定时器注册节点
        /// @param fps 推送帧率
        ImgMarkerPublisher(rclcpp::Node *node, const std::string &topic, float fps = 25);


        /// @brief 绘制一个多边形
        /// @param token 绘制token, 非当前token不会绘制
        /// @param name namespace
        /// @param pts 绘制多边形的点
        /// @param color 绘制颜色
        /// @param thickness 线宽
        /// @tparam m 像素缩放倍数
        /// @tparam T 点数据类型
        /// @tparam n 点个数
        template<size_t m = 1, class T, size_t n>
        void draw(token_t token, Data &data, const cv::Point_<T> (&pts)[n],
                  Color color, float thickness = 2) {
            if (check_token(token)) return;
            auto &marker = get(data, thickness);
            for (std::size_t i = 0; i < n; i++) {
                marker.points.emplace_back(cvt<m>(pts[i]));
                marker.points.emplace_back(cvt<m>(pts[(i + 1) % n]));
                marker.outline_colors.emplace_back(color);
            }
        }
        /// @brief 绘制一个多边形
        /// @param token 绘制token, 非当前token不会绘制
        /// @param name namespace
        /// @param pts 绘制多边形的点
        /// @param color 绘制颜色
        /// @param thickness 线宽
        /// @tparam m 像素缩放倍数
        /// @tparam T 点数据类型
        template<size_t m = 1, class T>
        void draw(token_t token, Data &data, const std::vector<cv::Point_<T>> &pts,
                  Color color, float thickness = 2) {
            if (check_token(token)) return;
            auto &marker = get(data, thickness);
            const auto n = pts.size();
            for (std::size_t i = 0; i < n; i++) {
                marker.points.emplace_back(cvt<m>(pts[i]));
                marker.points.emplace_back(cvt<m>(pts[(i + 1) % n]));
                marker.outline_colors.emplace_back(color);
            }
        }

        /// @brief 绘制一个旋转矩形
        /// @param token 绘制token, 非当前token不会绘制
        /// @param name namespace
        /// @param rr 旋转矩形
        /// @param color 绘制颜色
        /// @param thickness 线宽
        /// @tparam m 像素缩放倍数
        template<size_t m = 1>
        void draw(token_t token, Data &data, const cv::RotatedRect &rr,
                  Color color, float thickness = 2) {
            cv::Point2f pts[4];
            rr.points(pts);
            draw<m>(token, data, pts, color, thickness);
        }

        /// @brief 注册一个回调函数, 被注册的函数会被定期唤醒, 并传入当前渲染的publisher及token
        /// @param func 回调函数 void func(ImgMarkerPublisher &, token_t)
        void register_func(const std::string &name, const CallFunc &func);

        /// @brief 单行构造颜色函数 (r, g, b, a)
        inline static std_msgs::msg::ColorRGBA getColor(double r, double g, double b, double a = 1) {
            std_msgs::msg::ColorRGBA c(rosidl_runtime_cpp::MessageInitialization::SKIP);
            c.r = r;
            c.g = g;
            c.b = b;
            c.a = a;
            return c;
        }

    private:
        rclcpp::Node *const node;///< 节点
        const std::string topic; ///< 发布topic前缀
        std::uint32_t _token;    ///< 渲染token, 每次渲染会++, 保证渲染不会过期
        std::vector<Data> datas; ///< 所有注册的回调函数

        const float fidms;// frame interval duration milliseconds 帧间隔(毫秒, 1000/fps)
        std::thread timer_;
        /// @brief 回调函数
        void timer_callback();

        /// @brief 获取一个ImageMarker
        /// @details 会自动分配到msg_marker_array中, 并设置参数
        visualization_msgs::msg::ImageMarker &get(Data &data, float thickness = 2);

        /// @brief 检查token是否有效
        /// @return 是否应该跳过本次渲染
        inline bool check_token(token_t token) {
            const auto t = _token;
            if (token == t) return false;
            RCLCPP_WARN_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 1e3, "[ImgMarkerPublisher] Bad token, now = " << t << ", get = " << token);
            return true;
        }

        /// @brief 将OpenCV Point转换为geometry Point
        /// @tparam m 像素缩放倍数
        /// @tparam T 点数据类型
        template<size_t m = 1, class T>
        geometry_msgs::msg::Point cvt(const cv::Point_<T> &cv_point) {
            geometry_msgs::msg::Point geo_pt;
            geo_pt.x = cv_point.x * m;
            geo_pt.y = cv_point.y * m;
            return geo_pt;
        }
    };
}// namespace rm_armor_finder
#endif// IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__DEBUGPUBLISHER__H
