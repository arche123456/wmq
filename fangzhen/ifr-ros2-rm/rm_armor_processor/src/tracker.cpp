#include "rm_armor_processor/tracker.hpp"
#include "rm_common/defs.h"
#include <angles/angles.h>
#include <cfloat>
#include <ostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>


namespace rm_armor_processor {
    Tracker::Tracker(rclcpp::Node *node, double max_match_distance, double max_match_yaw_diff)
        : tracker_state(LOST),
          tracked_id(ifr_interface::defs::rm::armor_id::arm_undefined),
          measurement(Eigen::VectorXd::Zero(4)),
          target_state(Eigen::VectorXd::Zero(9)),
          max_match_distance_(max_match_distance),
          max_match_yaw_diff_(max_match_yaw_diff),
          node(node) {
    }

    void Tracker::init(const Armors::SharedPtr &armors_msg) {
        if (armors_msg->armors.empty()) {
            return;
        }

        // Simply choose the armor that is closest to image center
        double min_distance = DBL_MAX;
        tracked_armor = armors_msg->armors[0];
        for (const auto &armor: armors_msg->armors) {
            if (armor.distance_to_image_center < min_distance) {
                min_distance = armor.distance_to_image_center;
                tracked_armor = armor;
            }
        }

        initEKF(tracked_armor);
        RCLCPP_DEBUG(node->get_logger(), "Init EKF!");

        tracked_id = tracked_armor.type;
        tracker_state = DETECTING;
    }

    void Tracker::update(const Armors::SharedPtr &armors_msg) {
        // KF predict
        Eigen::VectorXd ekf_prediction = ekf.predict();
        RCLCPP_DEBUG(node->get_logger(), "EKF predict");

        // 输出预测的三维坐标和yaw值
        // RCLCPP_INFO(node->get_logger(), "Predicted State: xc = %.3f, yc = %.3f, za = %.3f, yaw = %.3f",
        //             ekf_prediction(0), ekf_prediction(2), ekf_prediction(4), ekf_prediction(6));

        bool matched = false;
        // Use KF prediction as default target state if no matched armor is found
        target_state = ekf_prediction;

        if (!armors_msg->armors.empty()) {
            // Find the closest armor with the same id
            Armor same_id_armor;
            int same_id_armors_count = 0;
            auto predicted_position = getArmorPositionFromState(ekf_prediction);
            double min_position_diff = DBL_MAX;
            double yaw_diff = DBL_MAX;
            for (const auto &armor: armors_msg->armors) {
                // Only consider armors with the same id
                if (armor.type == tracked_id) {
                    same_id_armor = armor;
                    same_id_armors_count++;
                    // Calculate the difference between the predicted position and the current armor position
                    auto p = armor.pose.position;
                    Eigen::Vector3d position_vec(p.x, p.y, p.z);
                    double position_diff = (predicted_position - position_vec).norm();
                    if (position_diff < min_position_diff) {
                        // Find the closest armor
                        min_position_diff = position_diff;
                        yaw_diff = abs(orientationToYaw(armor.pose.orientation) - ekf_prediction(6));
                        tracked_armor = armor;
                    }
                }
            }

            // Store tracker info
            info_position_diff = min_position_diff;
            info_yaw_diff = yaw_diff;


            // Check if the distance and yaw difference of closest armor are within the threshold
            if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
                // Matched armor found
                matched = true;
                auto p = tracked_armor.pose.position;
                // Update EKF
                double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
                measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
                // RCLCPP_INFO(
                //     node->get_logger(),
                //     "[rm_serial]: Send: [ %d, %.4f, %.4f, %.4f (in->out=%.3fms) ]",
                //     tracked_id,
                //     measurement(0),  // x
                //     measurement(1),  // y
                //     measurement(2),  // z
                //     measured_yaw     // yaw
                // );
                target_state = ekf.update(measurement);
                // RCLCPP_INFO(
                //     node->get_logger(),
                //     "[rm_serial]: receive: [%.4f %.4f %.4f], t=%.1f p=%.1f",
                //     target_state(0),  // x
                //     target_state(2),  // y
                //     target_state(4),  // z
                //     target_state(6),  // yaw
                //     target_state(8)   // 半径 r
                // );
                // 输出更新后的三维坐标和yaw值
                // RCLCPP_INFO(node->get_logger(), "Updated State: xc = %.3f, yc = %.3f, za = %.3f, yaw = %.3f",
                //             target_state(0), target_state(2), target_state(4), target_state(6));
                RCLCPP_DEBUG(node->get_logger(), "EKF update");
            } else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
                // Matched armor not found, but there is only one armor with the same id
                // and yaw has jumped, take this case as the target is spinning and armor jumped
                handleArmorJump(same_id_armor);
            } else {
                // No matched armor found
                RCLCPP_WARN_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 1e2, "No matched armor found!  pos_diff = " << min_position_diff << ", yaw_diff = " << yaw_diff);
            }
        }

        // Prevent radius from spreading
        if (target_state(8) < 0.12) {
            target_state(8) = 0.12;
            ekf.setState(target_state);
        } else if (target_state(8) > 0.4) {
            target_state(8) = 0.4;
            ekf.setState(target_state);
        }

        // Tracking state machine
        if (tracker_state == DETECTING) {
            if (matched) {
                detect_count_++;
                if (detect_count_ > tracking_thres) {
                    detect_count_ = 0;
                    tracker_state = TRACKING;
                }
            } else {
                detect_count_ = 0;
                tracker_state = LOST;
            }
        } else if (tracker_state == TRACKING) {
            if (!matched) {
                tracker_state = TEMP_LOST;
                lost_count_++;
            }
        } else if (tracker_state == TEMP_LOST) {
            if (!matched) {
                lost_count_++;
                if (lost_count_ > lost_thres) {
                    lost_count_ = 0;
                    tracker_state = LOST;
                }
            } else {
                tracker_state = TRACKING;
                lost_count_ = 0;
            }
        }
    }

    void Tracker::initEKF(const Armor &a) {
        double xa = a.pose.position.x;
        double ya = a.pose.position.y;
        double za = a.pose.position.z;
        last_yaw_ = 0;
        double yaw = orientationToYaw(a.pose.orientation);

        // Set initial position at 0.2m behind the target
        target_state = Eigen::VectorXd::Zero(9);
        double r = 0.26;
        double xc = xa + r * cos(yaw);
        double yc = ya + r * sin(yaw);
        dz = 0, another_r = r;
        target_state << xc, 0, yc, 0, za, 0, yaw, 0, r;

        ekf.setState(target_state);
    }

    void Tracker::handleArmorJump(const Armor &current_armor) {
        double yaw = orientationToYaw(current_armor.pose.orientation);
        target_state(6) = yaw;
        // Only 4 armors has 2 radius and height
        if (ifr_interface::defs::rm::armor_id::arm_num_4.count(current_armor.type)) {
            dz = target_state(4) - current_armor.pose.position.z;
            target_state(4) = current_armor.pose.position.z;
            std::swap(target_state(8), another_r);
        }
        RCLCPP_WARN_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 1e2, "Armor jump!");

        // If position difference is larger than max_match_distance_,
        // take this case as the ekf diverged, reset the state
        auto p = current_armor.pose.position;
        Eigen::Vector3d current_p(p.x, p.y, p.z);
        Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);
        if ((current_p - infer_p).norm() > max_match_distance_) {
            double r = target_state(8);
            target_state(0) = p.x + r * cos(yaw);// xc
            target_state(1) = 0;                 // vxc
            target_state(2) = p.y + r * sin(yaw);// yc
            target_state(3) = 0;                 // vyc
            target_state(4) = p.z;               // za
            target_state(5) = 0;                 // vza
            RCLCPP_ERROR_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 1e2, "Reset State!");
        }

        ekf.setState(target_state);
    }

    double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion &q) {
        // Get armor yaw
        tf2::Quaternion tf_q;
        tf_q.setW(q.w);
        tf_q.setX(q.x);
        tf_q.setY(q.y);
        tf_q.setZ(q.z);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        // Make yaw change continuous (-pi~pi to -inf~inf)
        yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
        last_yaw_ = yaw;
        // 输出yaw值
        // RCLCPP_INFO(node->get_logger(), "Extracted Yaw: %.3f", yaw);
        return yaw;
    }

    Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd &x) {
        // Calculate predicted position of the current armor
        double xc = x(0), yc = x(2), za = x(4);
        double yaw = x(6), r = x(8);
        double xa = xc - r * cos(yaw);
        double ya = yc - r * sin(yaw);
        // 输出预测的装甲板位置
        // RCLCPP_INFO(node->get_logger(), "Predicted Armor Position: xa = %.3f, ya = %.3f, za = %.3f", xa, ya, za);
        return Eigen::Vector3d(xa, ya, za);
    }

}// namespace rm_armor_processor