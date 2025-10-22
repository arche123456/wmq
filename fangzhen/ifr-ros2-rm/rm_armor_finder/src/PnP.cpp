#include "rm_armor_finder/PnP.h"
#include "rm_armor_finder/ippe.hpp"
#include <cmath>
#include <ifr_common/macros.hpp>
#include <limits>
#include <rclcpp/logging.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace rm_armor_finder {
    PnPSolver::PnPSolver(const Values *values,
                         const std::array<double, 9> &cam,
                         const std::vector<double> &dist)
        : values(values) {
        // 相机内参
        this->camera_matrix = cv::Matx33d(
                cam[0], cam[1], cam[2],
                cam[3], cam[4], cam[5],
                cam[6], cam[7], cam[8]);

        // 畸变
        IFR_ASSERT(dist.size() >= 5, "distortion_coefficients size = " + std::to_string(dist.size()) + " < 5");
        this->dist_coeffs = cv::Matx<double, 1, 5>(dist[0], dist[1], dist[2], dist[3], dist[4]);

        // 初始化装甲板大小(米)
        for (int is_lg = 0; is_lg <= 1; is_lg++) {
            double half_y = (is_lg ? values->arm_lg_w : values->arm_sm_w) / 2.0 / 1000;// mm->m
            double half_z = values->arm_l_h / 2.0 / 1000;
            auto &armor = (is_lg ? armor_lg : armor_sm);

            // 从左上开始逆时针 (左上, 左下, 右下, 右上)
            // x 前, y 左, z 上
            armor.emplace_back(0.0f, +static_cast<float>(half_y), +static_cast<float>(half_z));
            armor.emplace_back(0.0f, +static_cast<float>(half_y), -static_cast<float>(half_z));
            armor.emplace_back(0.0f, -static_cast<float>(half_y), -static_cast<float>(half_z));
            armor.emplace_back(0.0f, -static_cast<float>(half_y), +static_cast<float>(half_z));
        }
    }

    // 计算法向量(相机坐标系下): 假定世界法向量 = (0,0,-1)
    cv::Vec3d PnPSolver::computeNormalCam(const cv::Mat &R) {
        // 这里假设在模型坐标中，法向量是(0,0,-1)，
        // 即板子朝负Z方向(因为你说Z负则XY指向中心)。
        // normalCam = R * nWorld (不加 tvec，因为方向不需平移)
        cv::Matx33d rmat;
        R.copyTo(rmat);// 转成 Matx33d

        cv::Vec3d nWorld(0.0, 0.0, -1.0);
        cv::Vec3d normalCam = rmat * nWorld;
        return normalCam;
    }

    // 计算 yaw = atan2(ny, nx)
    double PnPSolver::computeYaw(const cv::Vec3d &normalCam) {
        double nx = normalCam[0];
        double ny = normalCam[1];
        return std::atan2(ny, nx);// 范围(-pi, pi)
    }

    // 判断灯条是否顺时针倾斜
    bool PnPSolver::isClockwiseRotated(const Armor &armor) {
        // left bar: apt.light_pt[0], apt.light_pt[1]
        // right bar: apt.light_pt[2], apt.light_pt[3]
        cv::Point2f left_vec = armor.apt.light_pt[0] - armor.apt.light_pt[1]; // 向量
        cv::Point2f right_vec = armor.apt.light_pt[2] - armor.apt.light_pt[3];// 向量

        double left_angle = std::atan2(left_vec.y, left_vec.x);
        double right_angle = std::atan2(right_vec.y, right_vec.x);
        double avg_angle = (left_angle + right_angle) * 0.5;

        // 顺时针旋转一般认为角度>0
        return (avg_angle > 0.0);
    }

    // 计算灯条长度(代替面积)
    double PnPSolver::calculateLightBarLength(const cv::Point2f &pt1, const cv::Point2f &pt2) {
        return cv::norm(pt1 - pt2);
    }

    // 若需要将两个旋转矩阵取平均
    void PnPSolver::averageRotation(const cv::Matx33d &R1, const cv::Matx33d &R2, cv::Matx33d &Rout) {
        // 1. 分别转 quaternion
        tf2::Matrix3x3 m1(
                R1(0, 0), R1(0, 1), R1(0, 2),
                R1(1, 0), R1(1, 1), R1(1, 2),
                R1(2, 0), R1(2, 1), R1(2, 2));
        tf2::Matrix3x3 m2(
                R2(0, 0), R2(0, 1), R2(0, 2),
                R2(1, 0), R2(1, 1), R2(1, 2),
                R2(2, 0), R2(2, 1), R2(2, 2));

        tf2::Quaternion q1, q2;
        m1.getRotation(q1);
        m2.getRotation(q2);

        // 2. 取平均(不考虑反向重叠等复杂情况)
        q1.normalize();
        q2.normalize();
        tf2::Quaternion qavg = tf2::Quaternion(
                (q1.x() + q2.x()) * 0.5,
                (q1.y() + q2.y()) * 0.5,
                (q1.z() + q2.z()) * 0.5,
                (q1.w() + q2.w()) * 0.5);
        qavg.normalize();

        // 3. 转回矩阵
        tf2::Matrix3x3 mavg(qavg);
        Rout = cv::Matx33d(
                mavg[0][0], mavg[0][1], mavg[0][2],
                mavg[1][0], mavg[1][1], mavg[1][2],
                mavg[2][0], mavg[2][1], mavg[2][2]);
    }

    bool PnPSolver::decideBestSolution(const Armor &armor,
                                       const cv::Vec3d &nCam1, float err1,
                                       const cv::Vec3d &nCam2, float err2,
                                       const cv::Matx33d &R1, const cv::Mat &tvec1,
                                       const cv::Matx33d &R2, const cv::Mat &tvec2) {
        // 1. 先判断 z 分量(因为我们定义 nWorld=(0,0,-1), z<0 => 面向相机)
        bool faceCam1 = (nCam1[2] < 0.0);
        bool faceCam2 = (nCam2[2] < 0.0);

        // 如果只有一个面向相机 => 选它
        if (faceCam1 && !faceCam2) return true;
        if (!faceCam1 && faceCam2) return false;
        // 如果都不面向 or 都面向 => 继续往下

        // 2. 计算 yaw
        double yaw1 = computeYaw(nCam1);// (-pi, pi)
        double yaw2 = computeYaw(nCam2);

        double abs1 = std::fabs(yaw1);
        double abs2 = std::fabs(yaw2);

        // 3. 如果 |yaw| < yaw_threshold => “接近正对相机”
        auto isNearlyFront = [&](double a) { return std::fabs(a) < yaw_threshold; };
        bool nf1 = isNearlyFront(yaw1);
        bool nf2 = isNearlyFront(yaw2);

        // 正对相机区间
        if (nf1 && nf2) {
            // a1) X>0 => 前方
            bool xPos1 = (nCam1[0] > 0.0);
            bool xPos2 = (nCam2[0] > 0.0);

            // 如果只有一个 X>0 => 选它
            if (xPos1 && !xPos2) return true;
            if (!xPos1 && xPos2) return false;

            // 如果均相同 => 看误差
            double diffErr = std::fabs(err1 - err2);
            if (diffErr > error_threshold) {
                // 不接近 => 选误差小
                return (err1 < err2);
            } else {
                // 误差接近 => “左右灯条抖动” => 看灯条面积
                // 先分别计算左灯条长度
                // left bar => apt.light_pt[0], apt.light_pt[1]
                double lenL1 = calculateLightBarLength(armor.apt.light_pt[0], armor.apt.light_pt[1]);
                double lenL2 = calculateLightBarLength(armor.apt.light_pt[0], armor.apt.light_pt[1]);// 其实对同一 Armor 不变...
                // 这里实际上需要把(解1,解2)对灯条图像投影? 你可以更复杂
                // 简化演示: 用 isClockwiseRotated

                // 根据灯条长度来决定法向量方向
                if (lenL1 < lenL2) {
                    // 右侧灯条较长 => 法向量朝右，取 Y 为负
                    bool yNeg1 = (nCam1[1] < 0.0);
                    bool yNeg2 = (nCam2[1] < 0.0);
                    if (yNeg1 && !yNeg2) return true;
                    if (!yNeg1 && yNeg2) return false;
                    return (err1 < err2);
                } else {
                    // 左侧灯条较长 => 法向量朝左，取 Y 为正
                    bool yNeg1 = (nCam1[1] > 0.0);
                    bool yNeg2 = (nCam2[1] > 0.0);
                    if (yNeg1 && !yNeg2) return true;
                    if (!yNeg1 && yNeg2) return false;
                    return (err1 < err2);
                }

                // bool cw = isClockwiseRotated(armor);
                // // 如果是顺时针 => 我们倾向于 Y 为负
                // // nCam1 => Y<0 => 说明是 右?
                // bool yNeg1 = (nCam1[1] < 0.0);
                // bool yNeg2 = (nCam2[1] < 0.0);

                // if (cw) {
                //     // 选 Y<0
                //     if (yNeg1 && !yNeg2) return true;
                //     if (!yNeg1 && yNeg2) return false;
                //     // 都相同 => 干脆看误差: 选小
                //     return (err1 < err2);
                // } else {
                //     // 选 Y>0
                //     if (!yNeg1 && yNeg2) return true;
                //     if (yNeg1 && !yNeg2) return false;
                //     return (err1 < err2);
                // }
            }
        }
        // 一个接近正对,一个不接近
        else if (nf1 && !nf2) {
            return true;
        } else if (!nf1 && nf2) {
            return false;
        }
        // 都不在“正对相机”区间 => yaw>=t1 => 左前 or 右前
        else {
            // 用 ny >0 => LEFT, ny<0 => RIGHT
            bool left1 = (nCam1[1] > 0.0);
            bool left2 = (nCam2[1] > 0.0);

            // 如果只有一个左 => 另一个右 => 选相应情况
            if (left1 && !left2) return true;
            if (!left1 && left2) return false;

            // 如果都一样 => 看误差
            double diffErr = std::fabs(err1 - err2);
            if (diffErr > error_threshold) {
                // 不接近 => 选小
                return (err1 < err2);
            } else {
                // 误差接近，则通过顺时针或逆时针判断
                bool cw = isClockwiseRotated(armor);// 判断是否顺时针旋转

                if (cw) {
                    // 顺时针旋转，表示装甲板朝右前
                    return (left1 ? true : false);// 如果nCam1是左前，则选nCam1解，否则选nCam2解
                } else {
                    // 逆时针旋转，表示装甲板朝左前
                    return (left1 ? false : true);// 如果nCam1是右前，则选nCam1解，否则选nCam2解
                }
            }
        }
    }

    // 新增：基于连续评分的候选解评分函数
    float PnPSolver::calcCandidateScore(const cv::Vec3d &nCam, float err) {
        const float w_err = 1.0f;
        const float w_yaw = 0.5f;
        const float w_face = 10.0f;// 若解不面向相机，施加大惩罚
        double yaw = std::fabs(computeYaw(nCam));
        float facePenalty = (nCam[2] >= 0.0 ? 1.0f : 0.0f);
        float huber_err = (err < 1.0f ? err : 1.0f + 0.5f * (err - 1.0f));
        return w_err * huber_err + w_yaw * static_cast<float>(yaw) + w_face * facePenalty;
    }

    // 新增：球面线性插值(Slerp)
    tf2::Quaternion PnPSolver::slerp(const tf2::Quaternion &q1, const tf2::Quaternion &q2, float t) {
        float dot = q1.x() * q2.x() + q1.y() * q2.y() + q1.z() * q2.z() + q1.w() * q2.w();
        tf2::Quaternion q2_copy = q2;
        if (dot < 0.0f) {
            dot = -dot;
            q2_copy = tf2::Quaternion(-q2.x(), -q2.y(), -q2.z(), -q2.w());
        }
        if (dot > 0.9995f) {
            tf2::Quaternion result(
                    q1.x() + t * (q2_copy.x() - q1.x()),
                    q1.y() + t * (q2_copy.y() - q1.y()),
                    q1.z() + t * (q2_copy.z() - q1.z()),
                    q1.w() + t * (q2_copy.w() - q1.w()));
            result.normalize();
            return result;
        }
        float theta_0 = std::acos(dot);
        float theta = theta_0 * t;
        float sin_theta = std::sin(theta);
        float sin_theta_0 = std::sin(theta_0);
        float s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
        float s1 = sin_theta / sin_theta_0;
        tf2::Quaternion result(
                q1.x() * s0 + q2_copy.x() * s1,
                q1.y() * s0 + q2_copy.y() * s1,
                q1.z() * s0 + q2_copy.z() * s1,
                q1.w() * s0 + q2_copy.w() * s1);
        result.normalize();
        return result;
    }

    // 新增：利用 softmax 融合两个候选解
    void PnPSolver::fuseCandidateSolution(
            const cv::Vec3d &nCam1, float err1, const cv::Matx33d &R1, const cv::Mat &tvec1,
            const cv::Vec3d &nCam2, float err2, const cv::Matx33d &R2, const cv::Mat &tvec2,
            geometry_msgs::msg::Pose &fusedPose, geometry_msgs::msg::Pose &otherPose) {
        float score1 = calcCandidateScore(nCam1, err1);
        float score2 = calcCandidateScore(nCam2, err2);
        const float sigma = 0.5f;// 控制 softmax 的“软性”
        float w1 = std::exp(-score1 / sigma);
        float w2 = std::exp(-score2 / sigma);
        float sum = w1 + w2;
        w1 /= sum;
        w2 /= sum;
        cv::Mat tvec_fused = w1 * tvec1 + w2 * tvec2;
        tf2::Quaternion q1, q2;
        {
            tf2::Matrix3x3 m1(
                    R1(0, 0), R1(0, 1), R1(0, 2),
                    R1(1, 0), R1(1, 1), R1(1, 2),
                    R1(2, 0), R1(2, 1), R1(2, 2));
            m1.getRotation(q1);
        }
        {
            tf2::Matrix3x3 m2(
                    R2(0, 0), R2(0, 1), R2(0, 2),
                    R2(1, 0), R2(1, 1), R2(1, 2),
                    R2(2, 0), R2(2, 1), R2(2, 2));
            m2.getRotation(q2);
        }
        tf2::Quaternion q_fused = slerp(q1, q2, w2);// 以 w2 为插值因子
        fusedPose.position.x = tvec_fused.at<double>(0);
        fusedPose.position.y = tvec_fused.at<double>(1);
        fusedPose.position.z = tvec_fused.at<double>(2);
        fusedPose.orientation.x = q_fused.x();
        fusedPose.orientation.y = q_fused.y();
        fusedPose.orientation.z = q_fused.z();
        fusedPose.orientation.w = q_fused.w();
        // 备用解选取得分较低的候选解
        if (score1 < score2)
            trans(R1, tvec1, otherPose);
        else
            trans(R2, tvec2, otherPose);
    }

    // 在PnPSolver类中添加补偿函数实现
    void PnPSolver::applyCompensation(cv::Mat &tvec) {
        if (!values->enable_distance_compensation ||
            values->compensate_model != 3) {// 仅分段模式生效
            return;
        }

        double x = tvec.at<double>(0);
        double y = tvec.at<double>(1);
        double z = tvec.at<double>(2);
        double d_estimated = cv::norm(tvec);

        // 分段补偿逻辑
        double d_compensated = d_estimated;
        if (d_estimated <= values->compensate_segment1_max) {
            // 分段1：0~3米
            d_compensated = values->compensate_segment1_k * d_estimated + values->compensate_segment1_b;
        } else if (d_estimated <= values->compensate_segment2_max) {
            // 分段2：3~5米
            d_compensated = values->compensate_segment2_k * d_estimated + values->compensate_segment2_b;
        } else {
            // 分段3：5米以上
            d_compensated = values->compensate_segment3_k * d_estimated + values->compensate_segment3_b;
        }

        // 应用缩放
        if (d_estimated > 1e-6) {
            double scale = d_compensated / d_estimated;
            tvec.at<double>(0) = x * scale;
            tvec.at<double>(1) = y * scale;
            tvec.at<double>(2) = z * scale;
        }
    }

    bool PnPSolver::solvePnP(const Armor &armor,
                             geometry_msgs::msg::Pose &pose, float &err1,
                             geometry_msgs::msg::Pose &other, float &err2) {
        try {
            std::vector<cv::Point2f> imagePoints(armor.apt.light_pt, armor.apt.light_pt + 4);
            const std::vector<cv::Point3f> &objPts = (armor.is_lg ? armor_lg : armor_sm);
            cv::Mat cam(3, 3, CV_64F), distC(1, 5, CV_64F);
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    cam.at<double>(i, j) = camera_matrix(i, j);
                }
            }
            for (int i = 0; i < 5; i++) {
                distC.at<double>(0, i) = dist_coeffs(0, i);
            }
            cv::Mat rvec1, tvec1, rvec2, tvec2;
            IPPE::PoseSolver::solveGeneric(
                    objPts, imagePoints,
                    cam, distC,
                    rvec1, tvec1, err1,
                    rvec2, tvec2, err2);
            // 补偿解
            applyCompensation(tvec1);
            applyCompensation(tvec2);
            cv::Mat Rmat1, Rmat2;
            cv::Rodrigues(rvec1, Rmat1);
            cv::Rodrigues(rvec2, Rmat2);
            cv::Matx33d R1, R2;
            Rmat1.copyTo(R1);
            Rmat2.copyTo(R2);
            cv::Vec3d nCam1 = computeNormalCam(Rmat1);
            cv::Vec3d nCam2 = computeNormalCam(Rmat2);
            // 使用新的软决策逻辑融合候选解
            fuseCandidateSolution(nCam1, err1, R1, tvec1,
                                  nCam2, err2, R2, tvec2,
                                  pose, other);
            return true;
        } catch (const cv::Exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("rm_pnp"), "PnP解算错误: %s", e.what());
            return false;
        }
    }

    // bool PnPSolver::solvePnP(const Armor &armor, geometry_msgs::msg::Pose &pose, float &err1, geometry_msgs::msg::Pose &other, float &err2) {
    //     cv::Mat rvec1, tvec1, rvec2, tvec2;
    //     IPPE::PoseSolver::solveGeneric(
    //             armor.is_lg ? armor_lg : armor_sm,
    //             std::vector<cv::Point2f>(armor.apt.light_pt, armor.apt.light_pt + 4),
    //             camera_matrix, dist_coeffs,
    //             rvec1, tvec1, err1, rvec2, tvec2, err2);

    //     cv::Matx33d R1, R2;
    //     cv::Rodrigues(rvec1, R1);
    //     cv::Rodrigues(rvec2, R2);

    //     trans(R1, tvec1, pose);
    //     trans(R2, tvec2, other);

    //     return true;

    // const auto angle1 = atan2(R1(1, 0), R1(0, 0));
    // const auto angle2 = atan2(R2(1, 0), R2(0, 0));

    // const auto diff = armor.apt.light_pt[1] - armor.apt.light_pt[0] + armor.apt.light_pt[2] - armor.apt.light_pt[3];
    // RCLCPP_INFO_STREAM_THROTTLE(values->get_node()->get_logger(), *values->get_node()->get_clock(), 1e3, "[pnp] diff = " << diff);
    // float angle = abs(diff.y) <= std::numeric_limits<decltype(diff.y)>().epsilon() ? M_PI_2 : atan(diff.x / diff.y);

    // __tmp_debug.clear();
    // __tmp_debug.emplace_back(angle * Values::r2d);
    // {
    //     auto pitch = atan2(-R1(2, 0), sqrt(R1(0, 0) * R1(0, 0) + R1(1, 0) * R1(1, 0)));
    //     auto yaw = atan2(R1(1, 0) / cos(pitch), R1(0, 0) / cos(pitch));
    //     auto roll = atan2(R1(2, 1) / cos(pitch), R1(2, 2) / cos(pitch));
    //     __tmp_debug.emplace_back(pitch * Values::r2d);
    //     __tmp_debug.emplace_back(yaw * Values::r2d);
    //     __tmp_debug.emplace_back(roll * Values::r2d);
    // }
    // {
    //     auto pitch = atan2(-R2(2, 0), sqrt(R2(0, 0) * R2(0, 0) + R2(1, 0) * R2(1, 0)));
    //     auto yaw = atan2(R2(1, 0) / cos(pitch), R2(0, 0) / cos(pitch));
    //     auto roll = atan2(R2(2, 1) / cos(pitch), R2(2, 2) / cos(pitch));
    //     __tmp_debug.emplace_back(pitch * Values::r2d);
    //     __tmp_debug.emplace_back(yaw * Values::r2d);
    //     __tmp_debug.emplace_back(roll * Values::r2d);
    // }


    // if (std::abs(angle - angle1) < std::abs(angle - angle2)) {
    //     // rvec1, tvec1 是正确的解
    // trans(R1, tvec1, pose);
    // trans(R2, tvec2, other);
    // } else {
    //     // rvec2, tvec2 是正确的解
    //     trans(R1, tvec1, other);
    //     trans(R2, tvec2, pose);
    //     std::swap(err1, err2);
    // }

    // return true;
    // }

    //--------------------------------------------
    float PnPSolver::calculateDistanceToCenter(const cv::Point2f &image_point) {
        float cx = camera_matrix(0, 2);
        float cy = camera_matrix(1, 2);
        return cv::norm(image_point - cv::Point2f(cx, cy));
    }

    //--------------------------------------------
    void PnPSolver::trans(const cv::Matx33d &rmat, const cv::Mat &tvec,
                          geometry_msgs::msg::Pose &pose) {
        pose.position.x = tvec.at<double>(0);
        pose.position.y = tvec.at<double>(1);
        pose.position.z = tvec.at<double>(2);

        tf2::Matrix3x3 tf2_rot(
                rmat(0, 0), rmat(0, 1), rmat(0, 2),
                rmat(1, 0), rmat(1, 1), rmat(1, 2),
                rmat(2, 0), rmat(2, 1), rmat(2, 2));
        tf2::Quaternion q;
        tf2_rot.getRotation(q);

        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
    }

}// namespace rm_armor_finder
