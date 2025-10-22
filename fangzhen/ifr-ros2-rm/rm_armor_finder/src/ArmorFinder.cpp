#include "rm_armor_finder/ArmorFinder.h"
#include "ifr_common/defs.h"
#include "rm_armor_finder/Armor.h"
#include "rm_armor_finder/Cudas.h"
#include "rm_armor_finder/PnP.h"
#include "rm_armor_finder/helper.h"
#include <cstddef>
#include <cstdint>
#include <cv_bridge/cv_bridge.h>
#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <ifr_common/common.h>
#include <memory>
#include <opencv2/core/base.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rcpputils/endian.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <string>
#include <vector>

namespace rm_armor_finder {


    void ArmorFinder::image_input(const ifr_interface::msg::BayerImage::SharedPtr msg) {
        if (!can_run) return;
        src = cv::Mat(msg->height, msg->width, CV_8UC1, reinterpret_cast<void *>(msg->data));
        // IF_NEED_PUBLISH(pub_debug_target) {
        //     cpu_debug_target = src.clone();
        // }
        frame_header = msg->header;

        step0_prepare();
        msg_back_ptr.ptr = msg->data;
        pub_back_ptr->publish(msg_back_ptr);
        step1_findContours();
        step2_getMinRect();
        step3_goodIndex();
        step4_goodPair();
        step5_calcPairRR();
        step6_testConflict();
        step7_sortPair();
        step8_toTarget();
        step9_pnp();
        stepA_publish();
    }

    // 修改后的 computeStableRotatedRect：保证旋转矩阵与角度一致，同时增加日志输出便于调试
    cv::RotatedRect ArmorFinder::computeStableRotatedRect(const std::vector<cv::Point> &contour) {

        // RCLCPP_INFO_STREAM_THROTTLE(
        //         this->get_logger(), *this->get_clock(), 5000,
        //         "computeStableRotatedRect: contour has " << contour.size() << " points");

        if (contour.size() < 2) {
            RCLCPP_WARN_STREAM_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Contour has less than 2 points, fallback to minAreaRect");
            return cv::minAreaRect(contour);
        }

        // 使用 float 类型计算 moments（满足 OpenCV 要求）
        std::vector<cv::Point2f> contour2f;
        contour2f.reserve(contour.size());
        for (const auto &pt: contour) {
            contour2f.emplace_back(static_cast<float>(pt.x), static_cast<float>(pt.y));
        }
        // RCLCPP_INFO_STREAM_THROTTLE(
        //         this->get_logger(), *this->get_clock(), 5000,
        //         "Converted contour to float with " << contour2f.size() << " points");

        cv::Moments m = cv::moments(contour2f);
        // RCLCPP_INFO_STREAM_THROTTLE(
        //         this->get_logger(), *this->get_clock(), 5000,
        //         "Moments: m00=" << m.m00 << ", m10=" << m.m10 << ", m01=" << m.m01);
        if (m.m00 <= 1e-6) {
            // RCLCPP_WARN_STREAM_THROTTLE(
            //         this->get_logger(), *this->get_clock(), 5000,
            //         "m00 is too small, fallback to minAreaRect");
            return cv::minAreaRect(contour);
        }
        // 转换为 double 精度的中心
        cv::Point2d center(static_cast<double>(m.m10 / m.m00), static_cast<double>(m.m01 / m.m00));
        // RCLCPP_INFO_STREAM_THROTTLE(
        //         this->get_logger(), *this->get_clock(), 5000,
        //         "Computed contour center (double): (" << center.x << ", " << center.y << ")");

        // 构造双精度点集用于后续计算
        std::vector<cv::Point2d> contour2d;
        contour2d.reserve(contour2f.size());
        for (const auto &pt: contour2f) {
            contour2d.emplace_back(static_cast<double>(pt.x), static_cast<double>(pt.y));
        }

        // PCA 仍使用 float 类型进行
        cv::PCA pca;
        try {
            cv::Mat data_pts(static_cast<int>(contour2f.size()), 2, CV_32F);
            for (size_t i = 0; i < contour2f.size(); ++i) {
                data_pts.at<float>(static_cast<int>(i), 0) = contour2f[i].x - static_cast<float>(center.x);
                data_pts.at<float>(static_cast<int>(i), 1) = contour2f[i].y - static_cast<float>(center.y);
            }
            pca = cv::PCA(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);
            // RCLCPP_INFO_STREAM_THROTTLE(
            //         this->get_logger(), *this->get_clock(), 5000,
            //         "PCA computed: eigenvector[0] = ("
            //                 << pca.eigenvectors.at<float>(0, 0) << ", "
            //                 << pca.eigenvectors.at<float>(0, 1) << ")");
        } catch (...) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "PCA failed, fallback to minAreaRect");
            return cv::minAreaRect(contour);
        }

        const cv::Vec2f eigen_vec = pca.eigenvectors.row(0);
        const double radian_angle = std::atan2(eigen_vec[1], eigen_vec[0]);
        double initial_angle = radian_angle * 180.0 / CV_PI;
        // RCLCPP_INFO_STREAM_THROTTLE(
        //         this->get_logger(), *this->get_clock(), 5000,
        //         "Initial computed angle (deg): " << initial_angle);

        double final_angle = initial_angle;
        cv::Mat rot_mat_initial = cv::getRotationMatrix2D(center, final_angle, 1.0);// CV_64F
        std::vector<cv::Point2d> rotated_pts_initial;
        cv::transform(contour2d, rotated_pts_initial, rot_mat_initial);
        // --- 转换 rotated_pts_initial 为 float 类型，用于 boundingRect ---
        std::vector<cv::Point2f> rotated_pts_initial_f;
        rotated_pts_initial_f.reserve(rotated_pts_initial.size());
        for (const auto &pt: rotated_pts_initial) {
            rotated_pts_initial_f.push_back(cv::Point2f(static_cast<float>(pt.x), static_cast<float>(pt.y)));
        }
        cv::Rect2d aabb_initial = cv::boundingRect(rotated_pts_initial_f);
        // RCLCPP_INFO_STREAM_THROTTLE(
        //         this->get_logger(), *this->get_clock(), 5000,
        //         "Initial AABB (double): x=" << aabb_initial.x << ", y=" << aabb_initial.y
        //                                     << ", width=" << aabb_initial.width << ", height=" << aabb_initial.height);

        if (aabb_initial.width < aabb_initial.height) {
            final_angle += 90.0;
            // RCLCPP_INFO_STREAM_THROTTLE(
            //         this->get_logger(), *this->get_clock(), 5000,
            //         "Swapping dimensions, adjusted angle: " << final_angle);
        }
        final_angle = std::fmod(final_angle + 360.0, 180.0);
        if (final_angle >= 90.0)
            final_angle -= 180.0;
        // RCLCPP_INFO_STREAM_THROTTLE(
        //         this->get_logger(), *this->get_clock(), 5000,
        //         "Normalized final angle (deg): " << final_angle);

        cv::Mat rot_mat_final = cv::getRotationMatrix2D(center, final_angle, 1.0);
        // RCLCPP_INFO_STREAM_THROTTLE(
        //         this->get_logger(), *this->get_clock(), 5000,
        //         "Final rotation matrix (rot_mat_final, CV_64F): ["
        //                 << rot_mat_final.at<double>(0, 0) << ", " << rot_mat_final.at<double>(0, 1) << ", " << rot_mat_final.at<double>(0, 2)
        //                 << "]; [" << rot_mat_final.at<double>(1, 0) << ", " << rot_mat_final.at<double>(1, 1) << ", " << rot_mat_final.at<double>(1, 2) << "]");
        std::vector<cv::Point2d> rotated_pts_final;
        cv::transform(contour2d, rotated_pts_final, rot_mat_final);
        // --- 转换 rotated_pts_final 为 float 类型，用于 boundingRect ---
        std::vector<cv::Point2f> rotated_pts_final_f;
        rotated_pts_final_f.reserve(rotated_pts_final.size());
        for (const auto &pt: rotated_pts_final) {
            rotated_pts_final_f.push_back(cv::Point2f(static_cast<float>(pt.x), static_cast<float>(pt.y)));
        }
        cv::Rect2d aabb_final = cv::boundingRect(rotated_pts_final_f);
        // RCLCPP_INFO_STREAM_THROTTLE(
        //         this->get_logger(), *this->get_clock(), 5000,
        //         "Final AABB (double): x=" << aabb_final.x << ", y=" << aabb_final.y
        //                                   << ", width=" << aabb_final.width << ", height=" << aabb_final.height);

        cv::Point2d aabb_center_final(aabb_final.x + aabb_final.width * 0.5,
                                      aabb_final.y + aabb_final.height * 0.5);
        // RCLCPP_INFO_STREAM_THROTTLE(
        //         this->get_logger(), *this->get_clock(), 5000,
        //         "Final AABB center in rotated frame (double): (" << aabb_center_final.x << ", " << aabb_center_final.y << ")");

        cv::Mat inv_rot_mat_final;
        cv::invertAffineTransform(rot_mat_final, inv_rot_mat_final);
        // RCLCPP_INFO_STREAM_THROTTLE(
        //         this->get_logger(), *this->get_clock(), 5000,
        //         "Inverse rotation matrix (inv_rot_mat_final, CV_64F): ["
        //                 << inv_rot_mat_final.at<double>(0, 0) << ", " << inv_rot_mat_final.at<double>(0, 1) << ", " << inv_rot_mat_final.at<double>(0, 2)
        //                 << "]; [" << inv_rot_mat_final.at<double>(1, 0) << ", " << inv_rot_mat_final.at<double>(1, 1) << ", " << inv_rot_mat_final.at<double>(1, 2) << "]");

        cv::Mat aabb_center_mat = (cv::Mat_<double>(3, 1) << aabb_center_final.x, aabb_center_final.y, 1.0);
        cv::Mat real_center_mat = inv_rot_mat_final * aabb_center_mat;
        cv::Point2d real_center_final(real_center_mat.at<double>(0, 0), real_center_mat.at<double>(1, 0));
        // RCLCPP_INFO_STREAM_THROTTLE(
        //         this->get_logger(), *this->get_clock(), 5000,
        //         "Final computed real center (double): (" << real_center_final.x << ", " << real_center_final.y << ")");

        cv::Size2f final_size(static_cast<float>(aabb_final.width), static_cast<float>(aabb_final.height));
        // RCLCPP_INFO_STREAM_THROTTLE(
        //         this->get_logger(), *this->get_clock(), 5000,
        //         "Final computed size: (" << final_size.width << ", " << final_size.height << ")");

        if (final_size.width < 1.0f || final_size.height < 1.0f) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Final size too small, fallback to minAreaRect");
            return cv::minAreaRect(contour);
        }

        // RCLCPP_INFO_STREAM_THROTTLE(
        //         this->get_logger(), *this->get_clock(), 5000,
        //         "Final computed RotatedRect: center=(" << real_center_final.x << ", " << real_center_final.y
        //                                                << "), size=(" << final_size.width << ", " << final_size.height
        //                                                << "), angle=" << final_angle);

        return cv::RotatedRect(cv::Point2f(static_cast<float>(real_center_final.x), static_cast<float>(real_center_final.y)),
                               final_size,
                               static_cast<float>(final_angle));
    }

    // 获取四元数的RPY角
    void ArmorFinder::getEulerFromQuaternion(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw) {
        tf2::Quaternion tf2_q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
    }

    // 设置四元数的RPY角
    void ArmorFinder::setQuaternionFromEuler(geometry_msgs::msg::Quaternion &q, double roll, double pitch, double yaw) {
        tf2::Quaternion tf2_q;
        tf2_q.setRPY(roll, pitch, yaw);
        q.x = tf2_q.x();
        q.y = tf2_q.y();
        q.z = tf2_q.z();
        q.w = tf2_q.w();
    }

    // 归一化角度到[-π, π]范围
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle <= -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // 计算投影误差（代价函数）
    // 修改calculateProjectionError函数
    double ArmorFinder::calculateProjectionError(const Armor &armor, const geometry_msgs::msg::Pose &pose, double yaw) {
        // 1. 提取位姿信息
        cv::Mat tvec = (cv::Mat_<double>(3, 1) << pose.position.x, pose.position.y, pose.position.z);

        // 2. 获取当前的roll和pitch，但使用新的yaw
        double roll, pitch, currentYaw;
        getEulerFromQuaternion(pose.orientation, roll, pitch, currentYaw);
        // double tmp_r, tmp_p, currentYaw;
        // getEulerFromQuaternion(pose.orientation, tmp_r, tmp_p, currentYaw);
        // double roll = FIXED_ROLL_RAD;
        // double pitch = FIXED_PITCH_RAD;

        // 3. 创建旋转矩阵
        tf2::Quaternion newQ;
        // newQ.setRPY(ARMOR_FIXED_ROLL, ARMOR_FIXED_PITCH, yaw);
        newQ.setRPY(roll, pitch, yaw);
        tf2::Matrix3x3 rotMat(newQ);

        cv::Matx33d rmat(
                rotMat[0][0], rotMat[0][1], rotMat[0][2],
                rotMat[1][0], rotMat[1][1], rotMat[1][2],
                rotMat[2][0], rotMat[2][1], rotMat[2][2]);

        cv::Mat rvec;
        cv::Rodrigues(cv::Mat(rmat), rvec);

        // 4. 获取装甲板的3D模型点
        std::vector<cv::Point3f> objectPoints;
        if (armor.is_lg) {
            double half_y = values->arm_lg_w / 2.0 / 1000;// mm->m
            double half_z = values->arm_l_h / 2.0 / 1000;

            // 考虑15度倾角
            const double tilt_angle_lg = ARMOR_FIXED_PITCH;// 15度倾角转弧度
            // 调整z坐标以反映15度倾角
            double z_offset_lg = half_y * std::tan(tilt_angle_lg);

            // 从左上开始逆时针 (左上, 左下, 右下, 右上)
            objectPoints.emplace_back(0.0f, +static_cast<float>(half_y), +static_cast<float>(half_z - z_offset_lg));
            objectPoints.emplace_back(0.0f, +static_cast<float>(half_y), -static_cast<float>(half_z - z_offset_lg));
            objectPoints.emplace_back(0.0f, -static_cast<float>(half_y), -static_cast<float>(half_z - z_offset_lg));
            objectPoints.emplace_back(0.0f, -static_cast<float>(half_y), +static_cast<float>(half_z - z_offset_lg));
        } else {
            double half_y = values->arm_sm_w / 2.0 / 1000;// mm->m
            double half_z = values->arm_l_h / 2.0 / 1000;

            // 考虑15度倾角
            const double tilt_angle_sm = ARMOR_FIXED_PITCH;// 15度倾角转弧度
            // 调整z坐标以反映15度倾角
            double z_offset_sm = half_y * std::tan(tilt_angle_sm);

            // 从左上开始逆时针 (左上, 左下, 右下, 右上)
            objectPoints.emplace_back(0.0f, +static_cast<float>(half_y), +static_cast<float>(half_z - z_offset_sm));
            objectPoints.emplace_back(0.0f, +static_cast<float>(half_y), -static_cast<float>(half_z - z_offset_sm));
            objectPoints.emplace_back(0.0f, -static_cast<float>(half_y), -static_cast<float>(half_z - z_offset_sm));
            objectPoints.emplace_back(0.0f, -static_cast<float>(half_y), +static_cast<float>(half_z - z_offset_sm));
        }

        // 5. 使用保存的相机参数
        std::vector<cv::Point2f> projectedPoints;
        std::vector<cv::Point2f> imagePoints(armor.apt.light_pt, armor.apt.light_pt + 4);
        cv::projectPoints(objectPoints, rvec, tvec, camera_matrix_for_trisection, dist_coeffs_for_trisection, projectedPoints);

        // 6. 参考top_model.cpp中的get_pts_cost函数计算误差
        // 定义误差计算参数
        const double inclined = M_PI / 4.0;              // 倾斜度，这个值可根据实际情况调整
        const double DETECTOR_ERROR_PIXEL_BY_SLOPE = 2.0;// 与top_model.cpp保持一致

        double cost = 0.0;
        for (int i = 0; i < 4; i++) {
            int p = (i + 1) % 4;// 下一个点

            // 参考点和预测点之间的向量
            cv::Point2f ref_d = imagePoints[p] - imagePoints[i];       // 参考线段
            cv::Point2f pt_d = projectedPoints[p] - projectedPoints[i];// 预测线段

            // 计算像素距离误差 - 位置误差
            double pixel_dis = (0.5 * (cv::norm(imagePoints[i] - projectedPoints[i]) +
                                       cv::norm(imagePoints[p] - projectedPoints[p])) +
                                std::fabs(cv::norm(ref_d) - cv::norm(pt_d))) /
                               cv::norm(ref_d);

            // 计算角度误差
            double ref_angle = std::atan2(ref_d.y, ref_d.x);
            double pt_angle = std::atan2(pt_d.y, pt_d.x);
            double angular_dis = cv::norm(ref_d) * std::fabs(normalizeAngle(ref_angle - pt_angle));

            // 组合误差，使用sin和cos进行加权
            double cost_i = std::pow(pixel_dis * std::sin(inclined), 2) +
                            std::pow(angular_dis * std::cos(inclined), 2) * DETECTOR_ERROR_PIXEL_BY_SLOPE;

            cost += std::sqrt(cost_i);
        }

        return cost;
    }

    // 三分法优化yaw角
    // 在optimizeYawWithTrisection函数中添加完整日志
    void ArmorFinder::optimizeYawWithTrisection(const Armor &armor, geometry_msgs::msg::Pose &pose) {
        // 获取当前yaw
        double roll, pitch, currentYaw;
        getEulerFromQuaternion(pose.orientation, roll, pitch, currentYaw);
        // double tmp_r, tmp_p, currentYaw;
        // getEulerFromQuaternion(pose.orientation, tmp_r, tmp_p, currentYaw);
        // double roll = FIXED_ROLL_RAD;
        // double pitch = FIXED_PITCH_RAD;

        // 设置搜索范围
        double leftBound = currentYaw - YAW_SEARCH_RANGE;
        double rightBound = currentYaw + YAW_SEARCH_RANGE;

        // 记录每次迭代的结果
        std::vector<double> costs;

        // 三分法迭代
        for (int i = 0; i < FIND_ANGLE_ITERATIONS; ++i) {
            double range = rightBound - leftBound;
            double mid1 = leftBound + range / 3;
            double mid2 = rightBound - range / 3;

            double cost1 = calculateProjectionError(armor, pose, mid1);
            double cost2 = calculateProjectionError(armor, pose, mid2);

            costs.push_back(std::min(cost1, cost2));

            if (cost1 < cost2) {
                rightBound = mid2;
            } else {
                leftBound = mid1;
            }
        }

        // 计算最优yaw并归一化
        double optimizedYaw = normalizeAngle((leftBound + rightBound) / 2.0);
        double finalCost = calculateProjectionError(armor, pose, optimizedYaw);

        // 记录最终结果，同时输出弧度和角度
        RCLCPP_INFO_STREAM_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Armor Final yaw: " << optimizedYaw << " rad ("
                                    << (optimizedYaw * 180.0 / M_PI) << " deg), "
                                    << "Change: " << (optimizedYaw - currentYaw) << " rad ("
                                    << ((optimizedYaw - currentYaw) * 180.0 / M_PI) << " deg), "
                                    << "Final cost: " << finalCost);

        // 更新优化后的yaw到姿态中
        // setQuaternionFromEuler(pose.orientation, ARMOR_FIXED_ROLL, ARMOR_FIXED_PITCH, optimizedYaw);
        setQuaternionFromEuler(pose.orientation, roll, pitch, optimizedYaw);
        // setQuaternionFromEuler(pose.orientation, FIXED_ROLL_RAD, FIXED_PITCH_RAD, optimizedYaw);
    }

    void ArmorFinder::step0_prepare() {
        src_size = static_cast<float>(src.size().area());
        // --- 将内存映射到显存 ---
        void *cuda_src_ptr = memMapPool->getDevicePointer(src.data, src.dataend - src.datastart);
        gpu_src = cv::cuda::GpuMat(src.rows, src.cols, src.type(), cuda_src_ptr);

        // --- 分支1 : 均值滤波, 用于数字检测 ---
        if (values->use_predict) {
            gpu_gray.create(src.size() / 2, CV_32FC1);
            // cv::cuda::cvtColor(gpu_src, gpu_gray, cv::COLOR_BayerRG2GRAY, 0, stream1);
            cudas::toGray(gpu_src, gpu_gray, values->submat_threshold_low, values->submat_threshold_high, stream1);
        }

        // --- 分支2 : 颜色卷积, 用于灯条检测 ---
        memMapPool->malloc(cpu_enemy, gpu_enemy, src.size() / 2, src.type());
        if (self_team == ifr_interface::defs::Team::BLUE) {
            cudas::getColorFromBayerRG(gpu_src, gpu_enemy, true,
                                       values->threshold_cuda_red_hlow, values->threshold_cuda_red_hhigh,
                                       values->threshold_cuda_red_slow, values->threshold_cuda_red_shigh,
                                       values->threshold_cuda_red_vlow, values->threshold_cuda_red_vhigh,
                                       stream2);
        } else {
            cudas::getColorFromBayerRG(gpu_src, gpu_enemy, false,
                                       values->threshold_cuda_blue_hlow, values->threshold_cuda_blue_hhigh,
                                       values->threshold_cuda_blue_slow, values->threshold_cuda_blue_shigh,
                                       values->threshold_cuda_blue_vlow, values->threshold_cuda_blue_vhigh,
                                       stream2);
        }
        stream2.waitForCompletion();
    }

    void ArmorFinder::step1_findContours() {
        cv::findContours(cpu_enemy, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    }

    // 在 step2_getMinRect 中加入日志输出，观察每个轮廓计算得到的 RotatedRect 参数
    void ArmorFinder::step2_getMinRect() {
        lbs.clear();
        for (size_t i = 0; i < contours.size(); i++) {
            cv::RotatedRect rr = computeStableRotatedRect(contours[i]);
            lbs.push_back(LightBar::getLightBar(rr));
        }
    }

    void ArmorFinder::step3_goodIndex() {
        goodIndex.clear();
        for (size_t i = 0; i < lbs.size(); i++) {//初步筛选
            const auto &lb = lbs[i];
            if (abs(lb.angle - 90) >= values->rrMaxAngle) continue;//倾斜角度

            const auto sr = src_size / lb.area;//面积比(画面大小除以轮廓框大小)
            if (sr < values->minSizeRatio || values->maxSizeRatio < sr) continue;
            const auto ar = lb.aspectRatio;
            if (ar < values->minAspectRatio || values->maxAspectRatio < ar) continue;
            goodIndex.push_back(i);
        }
    }

    void ArmorFinder::step4_goodPair() {
        static const constexpr float r2d = (180.0 / CV_PI);//弧度转角度
        goodPair.clear();
        for (size_t i = 0; i < goodIndex.size(); i++) {//将所有较好的轮廓尝试配对
            const auto &ilb = lbs[goodIndex[i]];       //i的旋转矩形
            for (size_t j = i + 1; j < goodIndex.size(); j++) {
                const auto &jlb = lbs[goodIndex[j]];//j的旋转矩形


                const auto sr = af_helper::getSimilarity(ilb.area, jlb.area);    //面积相似度
                const auto wr = af_helper::getSimilarity(ilb.width, jlb.width);  //宽相似度
                const auto hr = af_helper::getSimilarity(ilb.height, jlb.height);//高相似度
                if (sr < values->minAreaSimilarity || wr < values->minWidthSimilarity || hr < values->minHeightSimilarity) continue;


                const auto dis_a = af_helper::getAngleDistance(ilb.angle, jlb.angle);//灯条角度差
                if (dis_a > values->maxAngleDistance) continue;

                const auto link_vec = ilb.rr.center - jlb.rr.center;                                   //两个轮廓中心点连线
                const auto armor_h = ((ilb.height + jlb.height) / 2) * values->arm_h / values->arm_l_h;//装甲板高度
                const auto armor_w = sqrt(link_vec.dot(link_vec));                                     //装甲板宽度
                const auto armor_r = armor_w / armor_h;                                                //装甲板长宽比
                if (armor_r < values->arm_min_r || values->arm_max_r < armor_r) continue;


                const auto link_a = af_helper::normalizeAngle((atan2(link_vec.y, link_vec.x) * r2d) - 90.F);//连线角度
                const auto midd_a = af_helper::getAngleMiddle(ilb.angle, jlb.angle);                        //灯条角度
                const auto miss_a = af_helper::getAngleDistance(link_a, midd_a);                            //角度差


                //TODO 通过miss_a过滤    这里有问题  无法计算两个灯条的平均角度（0度和180度会被平均为90度），暂时只拿其中一个代替。

                if (miss_a > values->maxAngleMiss) continue;

                goodPair.push_back({goodIndex[i], goodIndex[j], armor_w, armor_h, midd_a, armor_r > values->arm_middle_r,
                                    (1 - sr)                                  //面积比
                                            + (1 - wr)                        //宽比
                                            + (1 - hr)                        //高比
                                            + dis_a / values->maxAngleDistance//灯条角度差
                                            + miss_a / values->maxAngleMiss,  //连线角度与真实连线角度的差值
                                    false});
            }
        }
    }

    void ArmorFinder::step5_calcPairRR() {
        for (auto &p: goodPair) {//
            p.pts.set(*values, p.is_large, lbs[p.i1].rr, lbs[p.i2].rr);
        }
    }

    void ArmorFinder::step6_testConflict() {
        for (auto &p: goodPair) {//灯条对的交集测试, 当一个灯条对的区域内包含另一个灯条的中心时, 则忽略此灯条对
            if (p.skip) continue;
            std::vector<cv::Point2f> points(p.pts.inner_pt, p.pts.inner_pt + 4);
            for (const auto &sub: goodPair) {
                if (sub == p) continue;
                if (cv::pointPolygonTest(points, lbs[sub.i1].rr.center, false) >= 0 ||
                    cv::pointPolygonTest(points, lbs[sub.i2].rr.center, false) >= 0) {
                    p.skip = true;
                    break;
                }
            }
        }
    }

    void ArmorFinder::step7_sortPair() {
        cv::Point2f center = cv::Point2f(src.size()) / 2;
        std::sort(goodPair.begin(), goodPair.end(), [&center](const ContourPair &l, const ContourPair &r) {
            if (l.skip != r.skip) return r.skip;
            const auto dl = l.pts.dot(center);
            const auto dr = r.pts.dot(center);
            return (l.pts.area() - dl - l.bad) > (r.pts.area() - dr - r.bad);
        });
    }

    void ArmorFinder::step8_toTarget() {
        good_targets.clear();
        good_targets.reserve(goodPair.size());

        bad_targets.clear();
        bad_targets.reserve(goodPair.size());

        if (values->use_predict) {
            stream1.waitForCompletion();
            RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1e3, "[Gray] " << gpu_gray.size() << ", " << values->use_predict);
        }
        identify->identifyAll(gpu_gray, goodPair, good_targets, bad_targets);
    }

    void ArmorFinder::step9_pnp() {
        if (pnp != nullptr) {
            msg_armors.header = msg_armor_marker.header = msg_badpnp_marker.header = msg_text_marker.header = frame_header;
            msg_armor_marker.id = 0;
            msg_badpnp_marker.id = 0;
            msg_text_marker.id = 0;
            msg_armors.armors.clear();
            msg_marker_arr.markers.clear();

            float err1, err2;
            for (const auto &armor: good_targets) {
                if (!pnp->solvePnP(armor, msg_armor.pose, err1, msg_badpnp_marker.pose, err2)) {
                    // if (!pnp->solvePnP(armor, msg_badpnp_marker.pose, err2, msg_armor.pose, err1)) {
                    RCLCPP_WARN(this->get_logger(), "Pnp Failed: [ %s ]", armor.toString().c_str());
                    continue;
                }

                // 使用三分法优化yaw角度
                // optimizeYawWithTrisection(armor, msg_armor.pose);

                msg_armor.type = armor.type;
                msg_armor.distance_to_image_center = pnp->calculateDistanceToCenter(armor.center());
                msg_armors.armors.emplace_back(msg_armor);

                // msg_debug_angle.set__data(pnp->__tmp_debug);

                IF_NEED_PUBLISH(pub_marker_arr) {
                    msg_text_marker.id++;
                    msg_text_marker.pose.position = msg_armor.pose.position;
                    msg_text_marker.pose.position.y -= 0.1;
                    msg_text_marker.text = armor.toTypeString();
                    msg_marker_arr.markers.emplace_back(msg_text_marker);

                    auto info = msg_text_marker.text + "\npnp_err: ";
                    msg_armor_marker.id++;
                    msg_armor_marker.pose = msg_armor.pose;
                    msg_armor_marker.text = info + std::to_string(err1);
                    msg_marker_arr.markers.emplace_back(msg_armor_marker);

                    msg_badpnp_marker.id++;
                    msg_badpnp_marker.text = info + std::to_string(err2);
                    msg_marker_arr.markers.emplace_back(msg_badpnp_marker);
                }
            }
            msg_debug_armor = msg_armor.pose;
        }
    }

    void ArmorFinder::stepA_publish() {
        {
            pub_debug_armor->publish(msg_debug_armor);
            pub_armors->publish(msg_armors);
            pub_marker_arr->publish(msg_marker_arr);
            // pub_debug_angle->publish(msg_debug_angle);
        }
        return;
        // {
        //     using namespace af_helper;
        //     IF_NEED_PUBLISH(pub_debug_light) {
        //         if (toRosImageMsg(cpu_enemy, msg_debug_light, frame_header))
        //             pub_debug_light->publish(msg_debug_light);
        //     }
        //     IF_NEED_PUBLISH(pub_debug_gray) {
        //         if (!gpu_gray.empty()) {
        //             gpu_gray.download(cpu_gray);
        //             if (toRosImageMsg(cpu_gray * 255, msg_debug_gray, frame_header))
        //                 pub_debug_gray->publish(msg_debug_gray);
        //         }
        //     }
        //     IF_NEED_PUBLISH(pub_debug_target) {
        //         if (!cpu_debug_target.empty() && cpu_debug_target.channels() == 1) {
        //             drawDebug();
        //             if (toRosImageMsg(cpu_debug_target, msg_debug_target, frame_header))
        //                 pub_debug_target->publish(msg_debug_target);
        //         }
        //     }
        // }
    }

    void ArmorFinder::drawDebug() {
        if (cpu_debug_target.empty()) return;
        cv::cvtColor(cpu_debug_target, cpu_debug_target, cv::COLOR_BayerRG2RGB);
        const cv::Scalar color_light_pt{255, 0, 0};
        const cv::Scalar color_bad_light_pt{0, 0, 255};
        const cv::Scalar color_number_pt{255, 255, 0};
        const cv::Scalar color_bad_number_pt{0, 255, 255};
        const cv::Scalar color_inner_pt{255, 100, 0};
        const cv::Scalar color_bad_inner_pt{0, 100, 255};
        const cv::Scalar color_contours{100, 200, 0};
        const cv::Scalar color_contours_gi{100, 200, 255};

        for (auto &c: contours)
            for (auto &p: c) p *= 2;
        for (const auto &i: goodIndex)
            af_helper::drawRotatedRect(cpu_debug_target, lbs[i].rr, color_contours_gi, 1, cv::LINE_AA);
        cv::drawContours(cpu_debug_target, contours, -1, color_contours, 1, cv::LINE_AA);
        for (const auto &t: good_targets) af_helper::drawPts(cpu_debug_target, t.apt.light_pt, color_light_pt, 1, cv::LINE_AA);
        for (const auto &t: good_targets) af_helper::drawPts(cpu_debug_target, t.apt.number_pt, color_number_pt, 1, cv::LINE_AA);
        for (const auto &t: good_targets) af_helper::drawPts(cpu_debug_target, t.apt.inner_pt, color_inner_pt, 1, cv::LINE_AA);
        for (const auto &t: bad_targets) af_helper::drawPts(cpu_debug_target, t.apt.light_pt, color_bad_light_pt, 1, cv::LINE_AA);
        for (const auto &t: bad_targets) af_helper::drawPts(cpu_debug_target, t.apt.number_pt, color_bad_number_pt, 1, cv::LINE_AA);
        for (const auto &t: bad_targets) af_helper::drawPts(cpu_debug_target, t.apt.inner_pt, color_bad_inner_pt, 1, cv::LINE_AA);
    }


    ArmorFinder::ArmorFinder(const rclcpp::NodeOptions &options)
        : rclcpp::Node("ArmorFinder", options),
          img_publisher(this),
          img_marker_publisher(this, "/rm/finder/debug/img_markers") {

        RCLCPP_INFO(this->get_logger(), "ArmorFinder is starting...");


        RCLCPP_INFO(this->get_logger(), "Init: MemMapPool...");
        memMapPool = std::make_unique<MemMapPool>(this);

        RCLCPP_INFO(this->get_logger(), "Init: Values...");
        values = std::make_unique<Values>(this);

        RCLCPP_INFO(this->get_logger(), "Init: ArmorIdentify...");
        identify = std::make_unique<ArmorIdentify>(
                values.get(),
                this->declare_parameter("model_name", "model-test"),
                this->declare_parameter("armor_ignores", std::vector<uint8_t>()));

        RCLCPP_INFO(this->get_logger(), "Init: ROS interface...");
        // 在收到相机参数时同时存储一份
        sub_cam_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/image_bayer/camera_info", rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::CameraInfo::SharedPtr cam) {
                    RCLCPP_INFO(this->get_logger(), "Received camera infos");

                    // 保存相机参数，用于三分法优化
                    camera_matrix_for_trisection = cv::Mat(3, 3, CV_64F);
                    for (int i = 0; i < 9; i++) {
                        camera_matrix_for_trisection.at<double>(i / 3, i % 3) = cam->k[i];
                    }

                    dist_coeffs_for_trisection = cv::Mat(1, 5, CV_64F);
                    for (int i = 0; i < 5; i++) {
                        dist_coeffs_for_trisection.at<double>(0, i) = cam->d[i];
                    }

                    // 初始化 PnP 求解器，使用相机内参
                    this->pnp = std::make_unique<PnPSolver>(values.get(), cam->k, cam->d);

                    sub_cam_info.reset();
                });

        sub_image = this->create_subscription<ifr_interface::msg::BayerImage>(
                "/image_bayer/image", rclcpp::SensorDataQoS(),
                std::bind(&ArmorFinder::image_input, this, std::placeholders::_1));

        sub_team = this->create_subscription<ifr_interface::msg::Team>(
                "/rm/self_team", rclcpp::SensorDataQoS(),
                [this](ifr_interface::msg::Team::SharedPtr team) {
                    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1e5, "Received team: " << int(team->team));
                    this->self_team = ifr_interface::defs::toTeam(team);
                });

        sub_aim_type = this->create_subscription<ifr_interface::msg::AimType>(
                "/rm/aim_type", rclcpp::SensorDataQoS(),
                [this](ifr_interface::msg::AimType::SharedPtr type) {
                    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1e5, "Received type: " << int(type->type));
                    this->can_run = (type->type == 0) ? true : false;
                });

        pub_debug_armor = this->create_publisher<decltype(msg_debug_armor)>("/rm/finder/debug/armor", 10);
        pub_armors = this->create_publisher<decltype(msg_armors)>("/rm/finder/armors", 10);
        pub_marker_arr = this->create_publisher<decltype(msg_marker_arr)>("/rm/finder/markers", 10);
        // pub_debug_angle = this->create_publisher<decltype(msg_debug_angle)>("/rm/finder/debug/angle", 10);
        pub_back_ptr = this->create_publisher<decltype(msg_back_ptr)>("image_bayer/return_ptr", 10);
        // pub_debug_light = this->create_publisher<decltype(msg_debug_light)>("/rm/finder/debug/light", 1);
        // pub_debug_gray = this->create_publisher<decltype(msg_debug_gray)>("/rm/finder/debug/gray", 1);
        // pub_debug_target = this->create_publisher<decltype(msg_debug_target)>("/rm/finder/debug/target", 1);


        // Visualization Marker Publisher
        // See http://wiki.ros.org/rviz/DisplayTypes/Marker
        msg_armor_marker.ns = "armors";
        msg_armor_marker.action = visualization_msgs::msg::Marker::ADD;
        msg_armor_marker.type = visualization_msgs::msg::Marker::CUBE;
        msg_armor_marker.scale.x = 0.03;
        msg_armor_marker.scale.y = 0.15;
        msg_armor_marker.scale.z = 0.12;
        msg_armor_marker.color.a = 1.0;
        msg_armor_marker.color.g = 0.5;
        msg_armor_marker.color.r = 0.5;
        msg_armor_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

        msg_badpnp_marker.ns = "bad_pnp";
        msg_badpnp_marker.action = visualization_msgs::msg::Marker::ADD;
        msg_badpnp_marker.type = visualization_msgs::msg::Marker::CUBE;
        msg_badpnp_marker.scale.x = 0.03;
        msg_badpnp_marker.scale.y = 0.15;
        msg_badpnp_marker.scale.z = 0.12;
        msg_badpnp_marker.color.a = 0.5;
        msg_badpnp_marker.color.g = 0.5;
        msg_badpnp_marker.color.r = 0.5;
        msg_badpnp_marker.color.b = 0.5;
        msg_badpnp_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

        msg_text_marker.ns = "classification";
        msg_text_marker.action = visualization_msgs::msg::Marker::ADD;
        msg_text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        msg_text_marker.scale.z = 0.1;
        msg_text_marker.color.a = 1.0;
        msg_text_marker.color.r = 1.0;
        msg_text_marker.color.g = 1.0;
        msg_text_marker.color.b = 1.0;
        msg_text_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

        img_publisher.register_data("raw", "/rm/finder/debug/raw", src);
        img_publisher.make_data("raw", "/rm/finder/debug/raw_gpu", [this](ImgPublisher::Img &img) {
            cv::Mat cpu;
            gpu_src.download(cpu);
            cv::imencode(".jpg", cpu, img.data);
        });
        auto rgb = std::make_shared<cv::cuda::GpuMat>();
        img_publisher.make_data("src", "/rm/finder/debug/src", [rgb, this](ImgPublisher::Img &img) {
            cv::Mat cpu_rgb;
            cv::cuda::cvtColor(gpu_src, *rgb, cv::COLOR_BayerRG2RGB);
            rgb->download(cpu_rgb);
            cv::imencode(".jpg", cpu_rgb, img.data);
        });
        img_publisher.register_data("light", "/rm/finder/debug/light", cpu_enemy);
        img_publisher.make_data("light", "/rm/finder/debug/light_gpu", [this](ImgPublisher::Img &img) {
            cv::Mat cpu;
            gpu_enemy.download(cpu);
            cv::imencode(".jpg", cpu, img.data);
        });
        auto gray = std::make_shared<cv::cuda::GpuMat>();
        img_publisher.make_data("gray", "/rm/finder/debug/gray_gpu", [gray, this](ImgPublisher::Img &img) {
            if (gpu_gray.empty()) return;
            cv::Mat cpu, cpu2;
            gpu_gray.convertTo(*gray, CV_8UC1, 255.0);
            gray->download(cpu);
            cv::resize(cpu, cpu2, src.size(), cv::INTER_NEAREST);
            cv::imencode(".jpg", cpu2, img.data);
        });
#if USE_MLP
        if (values->use_predict) {
            img_publisher.make_data("number", "/rm/finder/debug/number_sm", [this](ImgPublisher::Img &img) {
                cv::Mat cpu, cpu2;
                identify->debug_number_mat(false).download(cpu);
                cpu.convertTo(cpu2, CV_8UC1, 255.0);
                cv::imencode(".jpg", cpu2, img.data);
            });
            img_publisher.make_data("number", "/rm/finder/debug/number_lg", [this](ImgPublisher::Img &img) {
                cv::Mat cpu, cpu2;
                identify->debug_number_mat(true).download(cpu);
                cpu.convertTo(cpu2, CV_8UC1, 255.0);
                cv::imencode(".jpg", cpu2, img.data);
            });
        }
#endif

        img_marker_publisher.register_func("rr", [this](ImgMarkerPublisher &imp, auto &data, const auto &token) {
            const auto color_contours_gi = imp.getColor(100, 200, 255);
            for (const auto &i: goodIndex) imp.draw(token, data, lbs[i].rr, color_contours_gi);
        });
        img_marker_publisher.register_func("contour", [this](ImgMarkerPublisher &imp, auto &data, const auto &token) {
            const auto color_contours = imp.getColor(100, 200, 0);
            for (const auto &c: contours) imp.draw<2>(token, data, c, color_contours);
        });
        img_marker_publisher.register_func("good_light", [this](ImgMarkerPublisher &imp, auto &data, const auto &token) {
            const auto color_light_pt = imp.getColor(255, 0, 0);
            for (const auto &t: good_targets) imp.draw(token, data, t.apt.light_pt, color_light_pt);
        });
        img_marker_publisher.register_func("good_number", [this](ImgMarkerPublisher &imp, auto &data, const auto &token) {
            const auto color_number_pt = imp.getColor(255, 255, 0);
            for (const auto &t: good_targets) imp.draw(token, data, t.apt.number_pt, color_number_pt);
        });
        img_marker_publisher.register_func("good_inner", [this](ImgMarkerPublisher &imp, auto &data, const auto &token) {
            const auto color_inner_pt = imp.getColor(255, 100, 0);
            for (const auto &t: good_targets) imp.draw(token, data, t.apt.inner_pt, color_inner_pt);
        });
        img_marker_publisher.register_func("bad_light", [this](ImgMarkerPublisher &imp, auto &data, const auto &token) {
            const auto color_bad_light_pt = imp.getColor(0, 0, 255);
            for (const auto &t: bad_targets) imp.draw(token, data, t.apt.light_pt, color_bad_light_pt);
        });
        img_marker_publisher.register_func("bad_number", [this](ImgMarkerPublisher &imp, auto &data, const auto &token) {
            const auto color_bad_number_pt = imp.getColor(0, 255, 255);
            for (const auto &t: bad_targets) imp.draw(token, data, t.apt.number_pt, color_bad_number_pt);
        });
        img_marker_publisher.register_func("bad_inner", [this](ImgMarkerPublisher &imp, auto &data, const auto &token) {
            const auto color_bad_inner_pt = imp.getColor(0, 100, 255);
            for (const auto &t: bad_targets) imp.draw(token, data, t.apt.inner_pt, color_bad_inner_pt);
        });


        RCLCPP_INFO(this->get_logger(), "ArmorFinder successfully started!");
    }

}// namespace rm_armor_finder


// Register node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_armor_finder::ArmorFinder)