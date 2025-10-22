#include "rm_armor_finder/DebugPublisher.h"
#include "ifr_watcher/cpu_watcher.hpp"
#include <opencv2/core/fast_math.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/utilities.hpp>

namespace rm_armor_finder {
    void ImgPublisher::timer_callback() {
        cpu_watcher.update();
        const auto time = rclcpp::Clock().now();
        for (auto it = datas.begin(); it != datas.end() && rclcpp::ok();) {
            const auto &data = *it;
            if (data->pub->get_subscription_count() <= 0) {
                ++it;
                continue;
            }
            try {
                data->caller(data->img);
                data->img.header.stamp = time;
                data->pub->publish(data->img);
                data->err_cnt = 0;
            } catch (std::exception &e) {
                RCLCPP_WARN_STREAM(node->get_logger(),
                                   "[ImgPublisher] Can not handler " << data->img.header.frame_id << ", error: " << e.what());
            }
            if (data->err_cnt++ > 20)
                it = datas.erase(it);
            else
                ++it;
        }
        RCLCPP_INFO_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 5e3, "[ImgPublisher] calc time = " << (rclcpp::Clock().now() - time).to_chrono<std::chrono::milliseconds>().count() << " ms");
    }

    ImgPublisher::ImgPublisher(rclcpp::Node *node, float fps) : node(node), fidms(1000 / fps) {
        cpu_watcher.init("ImgPublisher", node);
        const auto period = std::chrono::milliseconds(static_cast<int>(fidms));
        timer_ = std::thread([this, period]() {
            while (rclcpp::ok()) {
                timer_callback();
                std::this_thread::sleep_for(period);
            }
        });
        RCLCPP_INFO_STREAM(node->get_logger(), "[ImgPublisher] register timer: " << period.count() << " ms");
    }

    ImgPublisher::DataPtr ImgPublisher::make_data(const std::string &name, const std::string &publish, const std::function<void(Img &)> &func) {
        std::shared_ptr<Data> sdata = std::make_shared<Data>();
        sdata->img.header.frame_id = name;
        sdata->pub = node->create_publisher<Img>(publish, rclcpp::SensorDataQoS());
        sdata->caller = func;
        sdata->err_cnt = 0;
        datas.push_back(sdata);
        RCLCPP_INFO_STREAM(node->get_logger(), "[ImgPublisher] register publisher: " << name << ", topic: " << publish);
        return sdata;
    }


    ImgMarkerPublisher::ImgMarkerPublisher(rclcpp::Node *node, const std::string &topic, float fps)
        : node(node), topic(topic), fidms(1000 / fps) {
        cpu_watcher.init("ImgMarkerPublisher", node);
        const auto period = std::chrono::milliseconds(static_cast<int>(fidms));
        timer_ = std::thread([this, period]() {
            while (rclcpp::ok()) {
                timer_callback();
                std::this_thread::sleep_for(period);
            }
        });
        RCLCPP_INFO_STREAM(node->get_logger(), "[ImgMarkerPublisher] register timer: " << period.count() << " ms, topic: " << topic);
    }

    void ImgMarkerPublisher::register_func(const std::string &name, const CallFunc &func) {
        const auto _topic = topic + "/" + name;
        datas.emplace_back(
                name,
                node->create_publisher<Data::Markers>(_topic, rclcpp::SensorDataQoS()),
                func);
        RCLCPP_INFO_STREAM(node->get_logger(), "[ImgMarkerPublisher] register publisher: " << _topic << ", name = " << name);
    }

    void ImgMarkerPublisher::timer_callback() {
        cpu_watcher.update();
        const auto token = ++_token;
        for (auto &data: datas) {
            data.id = 0;
            if (data.pub_marker_array->get_subscription_count() <= 0) continue;
            data.func(*this, data, token);
            const auto size = data.id;
            data.msg_marker_array.markers.resize(size);
            data.pub_marker_array->publish(data.msg_marker_array);
        }
        RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 5e3,//
                                     "[ImgMarkerPublisher] Push: " << topic << ([this]() {
                                         std::stringstream ss;
                                         for (auto &data: datas) ss << ", " << data.name << "=" << data.id;
                                         return ss.str();
                                     })());
    }

    visualization_msgs::msg::ImageMarker &ImgMarkerPublisher::get(Data &data, float thickness) {
        const auto id = data.id++;
        if (data.msg_marker_array.markers.size() <= id)
            data.msg_marker_array.markers.resize(id + 1);
        auto &marker = data.msg_marker_array.markers[id];
        marker.ns = data.name;
        marker.id = id;
        marker.type = visualization_msgs::msg::ImageMarker::LINE_LIST;
        marker.action = visualization_msgs::msg::ImageMarker::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(fidms * 500);// fidms / 1000 * 2
        marker.scale = thickness;
        marker.header.stamp = rclcpp::Clock().now();
        marker.outline_colors.clear();
        marker.points.clear();
        return marker;
    }


}// namespace rm_armor_finder