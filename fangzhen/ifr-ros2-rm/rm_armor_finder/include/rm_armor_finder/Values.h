#ifndef IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__VALUES__H
#define IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__VALUES__H
#include "ifr_common/defs.h"
#include <cstdint>
#include <functional>
#include <opencv2/opencv.hpp>
#include <rcl_interfaces/msg/detail/floating_point_range__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rm_common/defs.h>
#include <string>
#include <type_traits>
#include <unordered_map>
namespace rm_armor_finder {
    class Values {
        static constexpr const char prefix[] = "val_";
        rclcpp::Node *node = nullptr;

        std::unordered_map<std::string, std::function<void(const rclcpp::Parameter &)>> all_params;///< 参数地址保存
        ///参数更改
        rcl_interfaces::msg::SetParametersResult callback(const std::vector<rclcpp::Parameter> &params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &param: params) {
                const auto &name = param.get_name();
                if (all_params.count(name)) {
                    all_params[name](param);
                } else if (!name.compare(0, sizeof(prefix) - 1, prefix)) {
                    result.successful = false;
                    result.reason = "Not Found: " + name;
                }
            }
            return result;
        }
        /**
        声明一个变量
        @param type 类型名称
        @param name 变量名称
        @param val 默认值
        @param min_v 最小值
        @param max_v 最大值
        @param desc 描述
        @param ptr 修改地址 (小于0则不修改)
        */
        template<class T>
        T declare(std::string type, std::string name, T val, T min_v, T max_v, std::string desc, int64_t ptr = -1) {
            name = prefix + name;
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.name = name;
            descriptor.description = desc;
            if constexpr (std::is_same_v<std::remove_cv_t<T>, bool>) {
            } else if constexpr (std::is_integral_v<T>) {
                rcl_interfaces::msg::IntegerRange range;
                range.from_value = min_v, range.to_value = max_v;// range.step = 1;
                descriptor.integer_range.push_back(range);
            } else if constexpr (std::is_floating_point_v<T>) {
                rcl_interfaces::msg::FloatingPointRange range;
                range.from_value = min_v, range.to_value = max_v;//, range.step = std::numeric_limits<T>::epsilon();
                descriptor.floating_point_range.push_back(range);
            }
            if (ptr >= 0) {
                all_params[name] = [this, ptr](const rclcpp::Parameter &p) {
                    T *target = reinterpret_cast<T *>((reinterpret_cast<uint8_t *>(this) + ptr));
                    if constexpr (std::is_same_v<std::remove_cv_t<T>, bool>)
                        *target = p.as_bool();
                    else if constexpr (std::is_integral_v<T>)
                        *target = static_cast<T>(p.as_int());
                    else if constexpr (std::is_floating_point_v<T>)
                        *target = static_cast<T>(p.as_double());
                };
            } else
                descriptor.read_only = true;

            auto new_val = node->declare_parameter(name, val, descriptor);
            RCLCPP_DEBUG_STREAM(
                    node->get_logger(),
                    "declare var" << (ptr >= 0 ? "" : "(ro)") << ": "
                                  << type << " " << name << " = " << new_val << "(" << val << ") // " << desc);
            return new_val;
        }

    public:
        explicit Values(rclcpp::Node *node) : node(node) {
            params_callback_handle_ = node->add_on_set_parameters_callback(
                    std::bind(&Values::callback, this, std::placeholders::_1));
        }
        rclcpp::Node *get_node() const { return node; }
#define VALUES_PREFIX static constexpr const
#define DECLARE_VALUES_R(type, name, val, min, max, desc) \
public:                                                   \
    const type name = declare<std::remove_cv_t<type>>(#type, #name, val, min, max, desc)
#define DECLARE_VALUES_M(type, name, val, min, max, desc) \
public:                                                   \
    type name = declare<std::remove_cv_t<type>>(#type, #name, val, min, max, desc, reinterpret_cast<intptr_t>(&reinterpret_cast<Values *>(0)->name))

        DECLARE_VALUES_R(bool, use_predict, false, false, true, "是否使用预测");
        DECLARE_VALUES_R(bool, predict_img_maybe_save, false, false, true, "预测图像是否可能保存");
        DECLARE_VALUES_M(bool, predict_img_saving, false, false, true, "预测图像是否正在保存");
        DECLARE_VALUES_M(int, submat_threshold_low, 3, 0, 255, "子图的阈值");
        DECLARE_VALUES_M(int, submat_threshold_high, 35, 0, 255, "子图的阈值");


        DECLARE_VALUES_M(float, rrMaxAngle, 40, 0, 90, "灯条轮廓最大倾斜角度(绝对值), 垂直认为是0度");
        DECLARE_VALUES_M(float, maxSizeRatio, 60000, 1, 100000, "最大面积比(画面大小除以轮廓框大小), 超过此值则认为是噪声");
        DECLARE_VALUES_M(float, minSizeRatio, 4, 1, 100000, "最小面积比(画面大小除以轮廓框大小), 低于此值则认为非法框");
        DECLARE_VALUES_M(float, maxAspectRatio, 50, 1, 100, "最大长宽比(灯条)");
        DECLARE_VALUES_M(float, minAspectRatio, 2, 1, 100, "最小长宽比(灯条)");
        // DECLARE_VALUES_R(float, maxBetweenSizeRatio, 6, 0, 100, "(灯条)轮廓间最大面积比(相除-1取绝对值)");
        // DECLARE_VALUES_R(float, maxBetweenWHRatio, 4, 0, 100, "(灯条)轮廓间最大长或宽比(相除-1取绝对值)");
        DECLARE_VALUES_M(float, minAreaSimilarity, 0.8, 0, 1, "(灯条)轮廓间面积相似度阈值");
        DECLARE_VALUES_M(float, minWidthSimilarity, 0.8, 0, 1, "(灯条)轮廓间长/宽相似度阈值");
        DECLARE_VALUES_M(float, minHeightSimilarity, 0.8, 0, 1, "(灯条)轮廓间长/宽相似度阈值");
        DECLARE_VALUES_M(float, maxAngleDistance, 15, 0, 360, "最大角度差(超过此值则认为两个轮廓不平行)");
        DECLARE_VALUES_M(float, maxAngleMiss, 10, 0, 360, "最大角度差值(灯条中心点连线的角度与灯条角度)");

        static const constexpr auto r2d = (180.0 / CV_PI);//弧度转角度

        VALUES_PREFIX float arm_l_h = ifr_interface::defs::rm::const_values::arm_l_h;  //< 装甲板 灯条高度
        VALUES_PREFIX float arm_h = ifr_interface::defs::rm::const_values::arm_h;      //< 装甲板高度
        VALUES_PREFIX float arm_sm_w = ifr_interface::defs::rm::const_values::arm_sm_w;//< 小装甲板宽度
        VALUES_PREFIX float arm_lg_w = ifr_interface::defs::rm::const_values::arm_lg_w;//< 大装甲板宽度
        VALUES_PREFIX float arm_min_r = arm_sm_w / (arm_h * 4);                        //装甲板长宽比最低值
        VALUES_PREFIX float arm_sm_r = arm_sm_w / arm_h;                               //小装甲板长宽比
        VALUES_PREFIX float arm_lg_r = arm_lg_w / arm_h;                               //大装甲板长宽比
        VALUES_PREFIX float arm_max_r = (arm_lg_w * 1.5F) / arm_h;                     //装甲板长宽比最高值
        VALUES_PREFIX float arm_middle_r = (arm_sm_r + arm_lg_r) / 2;                  //大小装甲板长宽比的中间值

        DECLARE_VALUES_R(float, arm_num_w, 84.7F, 0, 1000, "装甲板中心图案宽度");
        DECLARE_VALUES_R(float, arm_num_sm_h, 125.0F, 0, 1000, "小装甲板中心图案高度");
        DECLARE_VALUES_R(float, arm_num_lg_h, 110.7F, 0, 1000, "大装甲板中心图案高度");


        VALUES_PREFIX int arm_to_w = ifr_interface::defs::rm::const_values::arm_number_pixel_w;//装甲板透射变换后的宽度
        VALUES_PREFIX int arm_to_h = ifr_interface::defs::rm::const_values::arm_number_pixel_h;//装甲板透射变换后的高度

        DECLARE_VALUES_M(float, model_threshold, 0.8F, 0, 1, "模型结果阈值");

        DECLARE_VALUES_M(bool, debug_show_extra, 1, false, true, "调试: 是否展示额外数据");

        // 新增距离补偿参数
        DECLARE_VALUES_M(bool, enable_distance_compensation, false, false, true, "启用距离补偿");
        DECLARE_VALUES_M(int, compensate_model, 0, 0, 3, "补偿模型: 0-无 1-线性 2-二次 3-分段");
        DECLARE_VALUES_M(float, compensate_segment1_max, 3.0, 0, 10, "分段1最大距离(米)");
        DECLARE_VALUES_M(float, compensate_segment1_k, 0.95, 0, 2, "分段1线性k");
        DECLARE_VALUES_M(float, compensate_segment1_b, 0.05, -1, 1, "分段1线性b");
        DECLARE_VALUES_M(float, compensate_segment2_max, 5.0, 0, 10, "分段2最大距离(米)");
        DECLARE_VALUES_M(float, compensate_segment2_k, 0.92, 0, 2, "分段2线性k");
        DECLARE_VALUES_M(float, compensate_segment2_b, 0.1, -1, 1, "分段2线性b");
        DECLARE_VALUES_M(float, compensate_segment3_k, 0.88, 0, 2, "分段3线性k");
        DECLARE_VALUES_M(float, compensate_segment3_b, 0.15, -1, 1, "分段3线性b");

        DECLARE_VALUES_M(int, threshold_cuda_red_hlow, -255, -255, 255, "颜色过滤阈值: 红装甲H通道下限");
        DECLARE_VALUES_M(int, threshold_cuda_red_hhigh, 255, -255, 255, "颜色过滤阈值: 红装甲H通道上限");
        DECLARE_VALUES_M(int, threshold_cuda_red_slow, 150, 0, 255, "颜色过滤阈值: 红装甲S通道");
        DECLARE_VALUES_M(int, threshold_cuda_red_shigh, 255, 0, 255, "颜色过滤阈值: 红装甲S通道");
        DECLARE_VALUES_M(int, threshold_cuda_red_vlow, 50, 0, 255, "颜色过滤阈值: 红装甲V通道下限");
        DECLARE_VALUES_M(int, threshold_cuda_red_vhigh, 255, 0, 255, "颜色过滤阈值: 红装甲V通道上限");

        DECLARE_VALUES_M(int, threshold_cuda_blue_hlow, -255, -255, 255, "颜色过滤阈值: 蓝装甲H通道下限");
        DECLARE_VALUES_M(int, threshold_cuda_blue_hhigh, 255, -255, 255, "颜色过滤阈值: 蓝装甲H通道上限");
        DECLARE_VALUES_M(int, threshold_cuda_blue_slow, 150, 0, 255, "颜色过滤阈值: 蓝装甲S通道");
        DECLARE_VALUES_M(int, threshold_cuda_blue_shigh, 255, 0, 255, "颜色过滤阈值: 蓝装甲S通道");
        DECLARE_VALUES_M(int, threshold_cuda_blue_vlow, 50, 0, 255, "颜色过滤阈值: 蓝装甲V通道下限");
        DECLARE_VALUES_M(int, threshold_cuda_blue_vhigh, 255, 0, 255, "颜色过滤阈值: 蓝装甲V通道上限");

#undef VALUES_PREFIX
#undef DECLARE_VALUES_R
#undef DECLARE_VALUES_M
    private:
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;///< 监听回调句柄
    };

}// namespace rm_armor_finder
#endif// IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__VALUES__H
