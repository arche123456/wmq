#ifndef IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__PNP__H
#define IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__PNP__H

#include "Armor.h"
#include "Values.h"
#include <array>
#include <geometry_msgs/msg/pose.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>

namespace rm_armor_finder {

    /// @brief 法向量方向枚举
    enum class NormalDirection {
        NONE = 0,
        LEFT,
        RIGHT,
        FRONT,
        BACK
    };

    class PnPSolver {
    public:
        PnPSolver(const Values *values,
                  const std::array<double, 9> &camera_matrix,
                  const std::vector<double> &distortion_coefficients);

        /// @brief 求解 PnP，给出两个解并选择最优解
        /// @param armor 输入的装甲板信息
        /// @param pose  输出最优解
        /// @param err1  第一组解的重投影误差
        /// @param other 输出次优解
        /// @param err2  第二组解的重投影误差
        bool solvePnP(const Armor &armor, geometry_msgs::msg::Pose &pose,
                      float &err1, geometry_msgs::msg::Pose &other, float &err2);

        /// @brief 计算图像坐标到图像中心的距离
        float calculateDistanceToCenter(const cv::Point2f &image_point);

        /// @brief 将旋转矩阵+平移向量转换为 geometry_msgs::msg::Pose
        static void trans(const cv::Matx33d &rmat, const cv::Mat &tvec, geometry_msgs::msg::Pose &pose);

        // 新增的 applyCompensation 函数声明
        void applyCompensation(cv::Mat &tvec);

    private:
        // 相机内参、畸变
        cv::Matx33d camera_matrix;         ///< 相机内参矩阵
        cv::Matx<double, 1, 5> dist_coeffs;///< 相机畸变矩阵
        const Values *const values;        ///< 外部参数

        /// @brief 3d 中装甲板的四个顶点 (小/大)
        /// @details 从左上开始逆时针: (左上, 左下, 右下, 右上)
        /// @details 模型坐标：x 向前，y 向左，z 向上
        std::vector<cv::Point3f> armor_sm, armor_lg;

        //----------------- 以下是新增的法向量辅助pnp解算跳变的解决办法 -----------------//

        double yaw_threshold = 0.34; ///< 判定“接近正对相机”的yaw阈值(弧度)
        double error_threshold = 0.3;///< 判定err1与err2“非常接近”的阈值，可调参

        /// @brief 计算法向量(相机坐标系下)
        /// @param R 旋转矩阵(3x3)
        /// @return (nx, ny, nz) 法向量在相机坐标系下
        cv::Vec3d computeNormalCam(const cv::Mat &R);

        /// @brief 计算法向量在相机坐标系下的 yaw = atan2(ny, nx)
        double computeYaw(const cv::Vec3d &normalCam);

        /// @brief 判断灯条是否顺时针倾斜
        bool isClockwiseRotated(const Armor &armor);

        /// @brief 计算灯条的长度(面积简化)
        double calculateLightBarLength(const cv::Point2f &pt1, const cv::Point2f &pt2);

        /// @brief 对两组解(含法向量、误差)进行挑选
        ///        返回 true => 选了第一个解, false => 选了第二个解
        bool decideBestSolution(const Armor &armor,
                                const cv::Vec3d &nCam1, float err1,
                                const cv::Vec3d &nCam2, float err2,
                                const cv::Matx33d &R1, const cv::Mat &tvec1,
                                const cv::Matx33d &R2, const cv::Mat &tvec2);

        /// @brief 若需要对两个旋转矩阵取“平均”，在误差很接近而无法判定时可用
        void averageRotation(const cv::Matx33d &R1, const cv::Matx33d &R2, cv::Matx33d &Rout);

        /// @brief 计算候选解的连续评分，较低评分代表更优解
        float calcCandidateScore(const cv::Vec3d &nCam, float err);

        /// @brief 球面线性插值（Slerp），在两个四元数间进行插值
        tf2::Quaternion slerp(const tf2::Quaternion &q1, const tf2::Quaternion &q2, float t);

        /// @brief 利用 softmax 融合两个候选解
        /// @param nCam1 法向量1, err1 重投影误差1, R1 平移旋转解1, tvec1 平移向量1
        /// @param nCam2 法向量2, err2 重投影误差2, R2 平移旋转解2, tvec2 平移向量2
        /// @param fusedPose 融合后的解, otherPose 为备用解
        void fuseCandidateSolution(
                const cv::Vec3d &nCam1, float err1, const cv::Matx33d &R1, const cv::Mat &tvec1,
                const cv::Vec3d &nCam2, float err2, const cv::Matx33d &R2, const cv::Mat &tvec2,
                geometry_msgs::msg::Pose &fusedPose, geometry_msgs::msg::Pose &otherPose);
    };

}// namespace rm_armor_finder

#endif
