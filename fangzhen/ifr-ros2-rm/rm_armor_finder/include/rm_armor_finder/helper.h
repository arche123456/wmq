#ifndef IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__HELPER__H
#define IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__HELPER__H
#include "ifr_common/common.h"
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <rcpputils/endian.hpp>
#include <sensor_msgs/msg/image.hpp>
namespace rm_armor_finder::af_helper {

    ///@brief 标准化角度值
    ///@param angle 角度
    ///@return [0,180)
    template<class T>
    FORCE_INLINE constexpr T normalizeAngle(T angle) {
        angle = fmod(angle, 180.0);
        if (angle < 0) angle += 180.0;
        return angle;
    }

    ///@return rr的实际角度 [0,180)
    FORCE_INLINE auto getRealAngle(const cv::RotatedRect &rr) {
        auto angle = rr.angle;
        if (rr.size.height > rr.size.width) angle += 90;
        return normalizeAngle(angle);
    }

    ///@brief 计算角度差
    ///@param f1 实际角度 [0,180)
    ///@param f2 实际角度 [0,180)
    ///@return 角度差 [0,90]
    template<bool normalize = false, class T>
    FORCE_INLINE constexpr T getAngleDistance(T f1, T f2) {
        // // 将角度归一化
        if constexpr (normalize) {
            f1 = normalizeAngle(f1);
            f2 = normalizeAngle(f2);
        }
        // 计算角度差值
        T diff = std::abs(f1 - f2);

        // 取最小的角度差值
        diff = std::min(diff, static_cast<T>(180.0) - diff);
        return diff;
    }

    ///@brief 获取两个角度的夹角
    ///@param a 实际角度 [0,180)
    ///@param b 实际角度 [0,180)
    ///@return 平均角度 [0,180)
    template<bool normalize = false, class T>
    FORCE_INLINE T getAngleMiddle(T a, T b) {
        // // 将角度归一化
        if constexpr (normalize) {
            a = normalizeAngle(a);
            b = normalizeAngle(b);
        }
        if (b < a) std::swap(a, b);
        if (a + 180 - b < b - a) a += 180;
        return (a + b) / 2;
    }

    ///@brief 获取两个值的相似度
    ///@details 用较大的除以较小的
    ///@returns 相似度[0,1]
    template<class T>
    FORCE_INLINE T getSimilarity(T l1, T l2) {
        if (l1 < 1e-8 && l2 < 1e-8) return 0;
        return (l1 < l2) ? (l1 / l2) : (l2 / l1);
    }

    /// 将OpenCV图像转换为ROS2图像
    FORCE_INLINE bool toRosImageMsg(const cv::Mat &src, sensor_msgs::msg::Image &dst, const std_msgs::msg::Header &header) {
        dst.header = header;
        if (src.empty()) return false;
        dst.height = src.rows;
        dst.width = src.cols;
        dst.encoding = src.channels() == 1 ? "mono8" : "bgr8";
        dst.is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
        dst.step = src.cols * src.elemSize();
        size_t size = dst.step * src.rows;
        dst.data.resize(size);

        CV_Assert(src.isContinuous());

        memcpy(reinterpret_cast<char *>(&dst.data[0]), src.data, size);
        return true;
    }

    FORCE_INLINE void drawRotatedRect(const cv::Mat &mask, const cv::RotatedRect &rr, const cv::Scalar &color,
                                      int thickness = 1, int lineType = cv::LineTypes::LINE_8) {
        cv::Point2f ps[4];
        rr.points(ps);
        std::vector<std::vector<cv::Point>> tmpContours;
        std::vector<cv::Point> contours;
        for (const auto &p: ps) { contours.emplace_back(cv::Point2i(p)); }
        tmpContours.insert(tmpContours.end(), contours);
        cv::drawContours(mask, tmpContours, 0, color, thickness, lineType);
    }

    template<class T, size_t n>
    FORCE_INLINE void drawPts(const cv::Mat &mask, const cv::Point_<T> (&pts)[n], const cv::Scalar &color,
                              int thickness = 1, int lineType = cv::LineTypes::LINE_8) {
        std::vector<std::vector<cv::Point>> tmpContours;
        std::vector<cv::Point> contours;
        for (const auto &p: pts) { contours.emplace_back(cv::Point2i(p)); }
        tmpContours.insert(tmpContours.end(), contours);
        drawContours(mask, tmpContours, 0, color, 1, cv::LINE_AA);
    }


}// namespace rm_armor_finder::af_helper
namespace rm_armor_finder::armor_helper {


    /// 返回两点距离的平方
    template<class T>
    FORCE_INLINE T SquareDistance(const cv::Point_<T> &p1, const cv::Point2f &p2) {
        return std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2);
    }

    // 返回距离p[0] 最近点的下标
    template<class T>
    FORCE_INLINE int getMinPt(const cv::Point_<T> (&p)[4]) {
        T min_dis = SquareDistance(p[0], p[1]);
        int min_i = 1;

        for (int i = 2; i < 4; i++) {
            T dis = SquareDistance(p[0], p[i]);
            if (dis < min_dis) {
                min_dis = dis;
                min_i = i;
            }
        }
        return min_i;
    }

    ///选择点
    /// @param flip 是否选择远离pt[0]的那两个点
    /// @param left 是否选择偏左的点
    /// @param min_i 最靠近p[0]的那个点
    /// @param pt 四个点
    /// @return 被选中的点
    FORCE_INLINE const cv::Point2f &selectPt(bool flip, bool left, int min_i, const cv::Point2f (&pt)[4]) {
        auto &p1 = pt[flip ? (min_i == 1 ? 2 : 1) : 0];
        auto &p2 = pt[flip ? (min_i == 3 ? 2 : 3) : min_i];
        return (p1.x < p2.x) == left ? p1 : p2;
    }

    ///计算三角形面积
    FORCE_INLINE float triangleArea(const cv::Point2f &a, const cv::Point2f &b, const cv::Point2f &c) {
        return 0.5 * std::abs((a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y)));
    }
}// namespace rm_armor_finder::armor_helper
#endif// IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__HELPER__H
