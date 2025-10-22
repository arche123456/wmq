#ifndef IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__ARMOR__H
#define IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__ARMOR__H
#include "Values.h"
#include <cstdint>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/node.hpp>
#include <sstream>
#include <string>
namespace rm_armor_finder {
    ///@brief 灯条信息
    struct LightBar {
        const cv::RotatedRect rr;///< 灯条包围矩形
        const float angle;       ///< 灯条实际角度 [0,180)
        const float width;       ///< 灯条宽度
        const float height;      ///< 灯条高度
        const float aspectRatio; ///< 长宽比
        const float area;        ///< 灯条面积

        ///@brief 输入原始最小包围矩形, 获取灯条数据
        ///@details 会扩大一倍像素
        static LightBar getLightBar(cv::RotatedRect rr);
    };
    /**
    * @brief 代表装甲板的所有特征点
    * @details 点序: top , bottom , bottom , top
    */
    struct ArmorPt {
        cv::Point2f light_pt[4]; ///< 灯条顶点
        cv::Point2f inner_pt[4]; ///< 内部装甲板顶点
        cv::Point2f number_pt[4];///< 数字区域顶点
        cv::Point2f center;      ///< 装甲板中心点

        void set(const Values &val, const bool &is_lg, const cv::RotatedRect &box1, const cv::RotatedRect &box2);


        /// 计算2个灯条中心点到给定点距离的平方
        float dot(const cv::Point2f &p) const;

        //计算面积
        float area() const;
    };

    /**
     * @brief 轮廓对, 代表2个灯条的组合
     * @details 代表一个可能的装甲板, 仅供内部使用
     */
    struct ContourPair {
        size_t i1{}, i2{};///< 轮廓下标
        float w{}, h{};   ///< 装甲板尺寸
        float angle{};    ///< 装甲板旋转角度(0~180)
        bool is_large{};  ///< 是否是大装甲板
        float bad{};      ///< 坏分, 越大代表越不可能是装甲板

        bool skip = true;///< 是否要跳过(内部包含其它轮廓对)

        ArmorPt pts;///< 灯条

        bool operator==(const ContourPair &o) const noexcept { return (i1 == o.i1) && (i2 == o.i2); }
    };


    /// 代表一个装甲板信息
    struct Armor {
        ArmorPt apt;     ///< 装甲板特征点
        bool is_lg;      ///< 是否是大装甲板
        float confidence;///< 置信度
        uint8_t type;    ///< 所属类别
        Armor(const ArmorPt &apt = {}, bool is_lg = false, float confidence = -2, uint8_t type = 0)
            : apt(apt), is_lg(is_lg), confidence(confidence), type(type) {}

        /// 转换为完整字符串
        std::string toString() const {
            std::stringstream ss;
            for (const auto &p: apt.light_pt) ss << '[' << p.x << ", " << p.y << "], ";
            ss << "lg = " << is_lg << ", c = " << confidence << ", t = " << static_cast<int>(type);
            return ss.str();
        }
        /// 转换为分类字符串
        std::string toTypeString() const {
            std::stringstream ss;
            ss << static_cast<int>(type) << ": "
               << std::fixed << std::setprecision(1) << (confidence * 100.0) << "%";
            return ss.str();
        }
        /// 获取中心点
        cv::Point2f center() const { return apt.center; }
    };

}// namespace rm_armor_finder
#endif// IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__ARMOR__H
