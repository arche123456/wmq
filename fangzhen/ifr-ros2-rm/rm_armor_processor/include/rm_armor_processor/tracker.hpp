#ifndef IFR_ROS2_CV__PACKAGE_RM_ARMOR_PROCESSOR__TRACKER__HPP
#define IFR_ROS2_CV__PACKAGE_RM_ARMOR_PROCESSOR__TRACKER__HPP
#include "rm_armor_processor/extended_kalman_filter.hpp"
#include "rm_interface/msg/rm_armors.hpp"
#include "rm_interface/msg/rm_target.hpp"
#include <cstdint>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <string>


namespace rm_armor_processor {

    class Tracker {
    public:
        Tracker(rclcpp::Node *node, double max_match_distance, double max_match_yaw_diff);

        using Armors = rm_interface::msg::RmArmors;
        using Armor = rm_interface::msg::RmArmor;

        void init(const Armors::SharedPtr &armors_msg);

        void update(const Armors::SharedPtr &armors_msg);

        ExtendedKalmanFilter ekf;

        int tracking_thres;
        int lost_thres;

        enum State {
            LOST,
            DETECTING,
            TRACKING,
            TEMP_LOST,
        } tracker_state;

        uint8_t tracked_id;
        Armor tracked_armor;

        double info_position_diff;
        double info_yaw_diff;

        Eigen::VectorXd measurement;

        Eigen::VectorXd target_state;

        // To store another pair of armors message
        double dz, another_r;

    private:
        void initEKF(const Armor &a);


        void handleArmorJump(const Armor &a);

        double orientationToYaw(const geometry_msgs::msg::Quaternion &q);

        Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd &x);

        double max_match_distance_;
        double max_match_yaw_diff_;

        int detect_count_ = 0;
        int lost_count_ = 0;

        double last_yaw_;

        rclcpp::Node *node;
    };

}// namespace rm_armor_processor

#endif// IFR_ROS2_CV__PACKAGE_RM_ARMOR_PROCESSOR__TRACKER__HPP
