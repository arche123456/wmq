#include "rm_serial_port/TargetCalc.h"
#include "rm_common/defs.h"
#include <cmath>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/create_timer_ros.h>


namespace rm_serial_port::utils {
    TargetCalc::TargetCalc(rclcpp::Node *node) : node(node) {

        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
                node->get_node_base_interface(), node->get_node_timers_interface());
        tf2_buffer_->setCreateTimerInterface(timer_interface);
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

        frame_gun = node->declare_parameter("frame.odom", "gun_link");

        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            rcl_interfaces::msg::FloatingPointRange r;
            r.set__from_value(0 + std::numeric_limits<float>::epsilon()).set__to_value(100);
            desc.description = "子弹射速";
            desc.floating_point_range.push_back(r);
            node->declare_parameter("tc.v0", 30.0, desc);
        }
        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            rcl_interfaces::msg::FloatingPointRange r;
            r.set__from_value(0 + std::numeric_limits<float>::epsilon()).set__to_value(90 * d2r);
            desc.description = "攻击最小角度差值(弧度)";
            desc.floating_point_range.push_back(r);
            node->declare_parameter("tc.amy", 30.0 * d2r, desc);
        }
        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            rcl_interfaces::msg::FloatingPointRange r;
            r.set__from_value(0 + std::numeric_limits<float>::epsilon()).set__to_value(100);
            desc.description = "反小陀螺模式旋转速度阈值";
            desc.floating_point_range.push_back(r);
            node->declare_parameter("tc.gyro_thresold", M_PI_2, desc);
        }
        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            rcl_interfaces::msg::FloatingPointRange r;
            r.set__from_value(-5).set__to_value(5);
            desc.description = "预测时间补偿(s)";
            desc.floating_point_range.push_back(r);
            node->declare_parameter("tc.time_delta", 0.0, desc);
        }


        params_callback_handle = node->add_on_set_parameters_callback(
                std::bind(&TargetCalc::param_callback, this, std::placeholders::_1));
        auto r = param_callback(node->get_parameters({"tc.v0", "tc.amy", "tc.gyro_thresold"}));
        if (!r.successful) {
            RCLCPP_FATAL(node->get_logger(), "Can not init param: " + r.reason);
        }
    }

    rcl_interfaces::msg::SetParametersResult TargetCalc::param_callback(const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &param: params) {
            bool success;
            const auto &name = param.get_name();
            if (name == "tc.v0") {
                double v0 = param.as_double();
                if ((success = (v0 > 0)))
                    setV0(v0);
                else
                    result.reason = "Non positive numbers are not allowed \"tc.v0\"";
            } else if (name == "tc.amy") {
                double amy = param.as_double();
                if ((success = (amy >= 0)))
                    setAMY(amy);
                else
                    result.reason = "Negative numbers are not allowed \"tc.amy\"";
            } else if (name == "tc.gyro_thresold") {
                double gyro = param.as_double();
                if ((success = (gyro >= 0)))
                    setGyroThreshold(gyro);
                else
                    result.reason = "Non positive numbers are not allowed \"tc.gyro_thresold\"";
            } else if (name == "tc.time_delta") {
                double t = param.as_double();
                if ((success = (-5 <= t && t <= 5)))
                    setTimeDelta(t);
                else
                    result.reason = "\"tc.time_delta\" need in [-5, 5]";
            } else
                success = true;
            result.successful &= success;
        }
        return result;
    }

    void TargetCalc::setV0(double v0) { this->v0 = v0, this->g_v02 = g / (v0 * v0); }
    void TargetCalc::setAMY(double attack_max_yaw) { this->attack_max_yaw = attack_max_yaw; }
    void TargetCalc::setGyroThreshold(double gyro_threshold) { this->gyro_threshold = gyro_threshold; }
    void TargetCalc::setTimeDelta(double time_delta) { this->time_delta = time_delta; }


    /// 获取飞行时间 t
    /// @param data 目标数据
    /// @param dt 数据采集到最新的时间差
    /// @return 从数据采集到子弹命中的时间差
    double TargetCalc::getT(const rm_interface::msg::RmTarget::SharedPtr &data, double dt) const {
        const auto X2 = std::pow(data->position.x + data->velocity.x * dt, 2) +
                        std::pow(data->position.y + data->velocity.y * dt, 2);
        const auto Y = -data->position.z + data->velocity.z * dt;
        const auto l = std::sqrt(X2 + std::pow(Y, 2));
        const auto sin0 = Y / l;


        const auto a = (std::asin(sin0) - std::asin((g_v02 * l * (1 - std::pow(sin0, 2))) - sin0)) / 2.0;

        const auto X = std::sqrt(X2);
        const auto t = X / (std::cos(a) * v0);
        return dt + t;
    }

    /// 计算弹道补偿值
    /// @param p 攻击位置
    /// @return 枪口上抬角度
    double TargetCalc::getAlpha(const geometry_msgs::msg::Point &p) const {
        const auto X2 = std::pow(p.x, 2) + std::pow(p.y, 2);
        const auto Y = -p.z;
        const auto l = std::sqrt(X2 + std::pow(Y, 2));
        const auto sin0 = Y / l;

        const auto a = (std::asin(sin0) - std::asin((g_v02 * l * (1 - std::pow(sin0, 2))) - sin0)) / 2.0;
        return a;
    }

    /// 转换结果
    /// @param attack 是否要攻击
    /// @param position 变换前攻击位置
    /// @details 会叠加弹道补偿数据
    TargetCalc::ArmorTarget TargetCalc::getResult(bool attack, geometry_msgs::msg::PointStamped &position) const {

        transformToGun(position);

        auto pitch = getAlpha(position.point);
        RCLCPP_INFO_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 1e3, "TC: " << position.point.z << "; " << pitch);
        auto yaw = std::atan2(position.point.y, position.point.x);
        return {attack, yaw, pitch};
    }

    bool TargetCalc::canAttack(const cv::Point2d &car_center, const cv::Point2d &armor_center, double yaw, const bool &is_lg) const {
        yaw += M_PI_2;
        const auto w = (is_lg ? ifr_interface::defs::rm::const_values::arm_lg_w : ifr_interface::defs::rm::const_values::arm_sm_w) / 2;
        cv::Point2d v(std::cos(yaw) * w, std::sin(yaw) * w);

        /// 判断圆心指向车中心的向量 与 装甲板左侧点指向右侧点的向量 是否有交点
        return cross_test(car_center, armor_center - v, armor_center + v);
    }

    bool TargetCalc::cross_test(const cv::Point2d &B, const cv::Point2d &C, const cv::Point2d &D) {
        if ((std::min(0.0, B.x) <= std::max(C.x, D.x) &&
             std::min(C.x, D.x) <= std::max(0.0, B.x) &&
             std::min(0.0, B.y) <= std::max(C.y, D.y) &&
             std::min(C.y, D.y) <= std::max(0.0, B.y))) {// 快速排斥实验

            // 跨立实验
            double ACxAB = (C.x - 0.0) * (B.y - 0.0) - (C.y - 0.0) * (B.x - 0.0);
            double ADxAB = (D.x - 0.0) * (B.y - 0.0) - (D.y - 0.0) * (B.x - 0.0);
            double CAxCD = (0.0 - C.x) * (D.y - C.y) - (0.0 - C.y) * (D.x - C.x);
            double CBxCD = (B.x - C.x) * (D.y - C.y) - (B.y - C.y) * (D.x - C.x);

            // 判断两个数是否符号相反
            return ((ACxAB < 0 && ADxAB > 0) || (ACxAB > 0 && ADxAB < 0)) &&
                   ((CAxCD < 0 && CBxCD > 0) || (CAxCD > 0 && CBxCD < 0));
        } else
            return false;
    }

    /// 获取最佳装甲板击打位置
    /// @param data 目标数据
    /// @return 攻击信息
    TargetCalc::ArmorTarget TargetCalc::getArmor(const rm_interface::msg::RmTarget::SharedPtr &data) const {
        ///装甲板数量
        const auto amt = ifr_interface::defs::rm::armor_id::getNum(data->type);

        ///从数据采集到子弹命中的时间差
        auto t = std::max((node->now() - data->header.stamp).seconds(), 0.0);
        t = getT(data, t + time_delta);

        ///子弹命中时, 中心点坐标
        cv::Point2d center{
                data->position.x + data->velocity.x * t,
                data->position.y + data->velocity.y * t,
                //   data->position.z + data->velocity.z * t,
        };
        const auto cz = data->position.z + data->velocity.z * t;
        ///子弹命中时, yaw轴旋转角度
        const auto yaw = data->yaw + data->v_yaw * t;
        ///变量引出
        const auto &r1 = data->radius_1, &r2 = data->radius_2;


        cv::Point2d a2c;//装甲板指向车中心的向量
        cv::Point2d o2a;//原点指向装甲板的向量
        double r, tmp_yaw, min_angle = 10;
        int min_i = -1, t_i;
        ///选择最正对自己的装甲板
        for (uint i = 0; i < amt; i++) {
            tmp_yaw = yaw + (t_i = i) * (2 * M_PI / amt);
            r = ((amt == 4) && (i & 1)) ? r2 : r1;
            a2c.x = r * std::cos(tmp_yaw);
            a2c.y = r * std::sin(tmp_yaw);
            o2a = center - a2c;
            auto angle = std::acos(o2a.dot(a2c) / std::sqrt(a2c.dot(a2c) * o2a.dot(o2a)));
            // if (angle > attack_max_yaw) continue;// 超出可攻击的角度
            if (angle < min_angle) min_angle = angle, min_i = i;
            if (attack_max_yaw < M_PI / amt) break;// 如果最大攻击角度小于平均角度，提前结束循环
        }

        // 如果找到更合适的装甲板
        if (min_i != t_i) {
            tmp_yaw = yaw + min_i * (2 * M_PI / amt);
            r = ((amt == 4) && (min_i & 1)) ? r2 : r1;
            a2c.x = r * std::cos(tmp_yaw);
            a2c.y = r * std::sin(tmp_yaw);
            o2a = center - a2c;
        }

        geometry_msgs::msg::PointStamped target;
        target.header = data->header;

        if (std::abs(data->v_yaw) > gyro_threshold) {// 如果目标的旋转速度超过阈值
            /// 反小陀螺模式
            bool cross = canAttack(center, o2a, tmp_yaw, ifr_interface::defs::rm::armor_id::arm_is_lg.count(data->type));

            target.point.x = center.x;
            target.point.y = center.y;
            target.point.z = cz + (((amt == 4) && (min_i & 1)) ? data->dz : 0);
            return getResult(cross, target);
        } else {

            ///当前没有可以攻击的装甲板(不在可攻击范围内)
            if (min_angle > attack_max_yaw) {
                target.point.x = center.x;
                target.point.y = center.y;
                target.point.z = cz + (((amt == 4) && (min_i & 1)) ? data->dz : 0);
                return getResult(false, target);
            }

            target.point.x = o2a.x;
            target.point.y = o2a.y;
            target.point.z = cz + (((amt == 4) && (min_i & 1)) ? data->dz : 0);
            return getResult(true, target);
        }
    }


    void TargetCalc::transformToGun(geometry_msgs::msg::PointStamped &position) const {
        try {
            auto t = tf2_buffer_->lookupTransform(position.header.frame_id, frame_gun, tf2::TimePointZero);
            geometry_msgs::msg::PointStamped zero, target;
            zero.header = position.header;
            tf2::doTransform(zero, target, t);
            position.point.x -= target.point.x;
            position.point.y -= target.point.y;
            position.point.z -= target.point.z;
        } catch (tf2::LookupException &e) {
            RCLCPP_ERROR_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 100, "Can not get gun(" << frame_gun << ") -> gimbal(" << position.header.frame_id << "), tf2::LookupException: " << e.what());
        } catch (tf2::ConnectivityException &e) {
            RCLCPP_ERROR_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 100, "Can not get gun(" << frame_gun << ") -> gimbal(" << position.header.frame_id << "), tf2::ConnectivityException: " << e.what());
        } catch (tf2::ExtrapolationException &e) {
            RCLCPP_ERROR_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 100, "Can not get gun(" << frame_gun << ") -> gimbal(" << position.header.frame_id << "), tf2::ExtrapolationException: " << e.what());
        } catch (tf2::InvalidArgumentException &e) {
            RCLCPP_ERROR_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 100, "Can not get gun(" << frame_gun << ") -> gimbal(" << position.header.frame_id << "), tf2::InvalidArgumentException: " << e.what());
        }
    }
}// namespace rm_serial_port::utils