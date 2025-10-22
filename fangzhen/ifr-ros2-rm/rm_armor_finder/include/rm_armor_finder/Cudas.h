#ifndef IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__CUDAS__H
#define IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__CUDAS__H
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>
namespace rm_armor_finder::cudas {
    /**
     * @brief 通过BayerRG图片获取指定颜色
     * @details 此方法同时执行 卷积、阈值、缩小 三个操作
     * @param src BayerRG原始图像
     * @param dst 指定颜色的单通道图像, 大小是原图的一半
     * @param getRed true = 获取Red, false = 获取Blue
     * @param thresh 二值化的阈值
     * @param tr 红色通道权重
     * @param tg1 绿色通道权重
     * @param tg2 绿色通道权重
     * @param tb 蓝色通道权重
     */
    void getColorFromBayerRG(const cv::cuda::GpuMat &src, const cv::cuda::GpuMat &dst, bool getRed,
                             int h_low, int h_high, int s_low, int s_high, int v_low, int v_high,
                             cv::cuda::Stream &stream = cv::cuda::Stream::Null());

    /**
     * @brief 将BayerRG转为灰度图
     * @details 此方法进行范围阈值操作
     * @param src BayerRG原始图像
     * @param dst 同样大小的单通道二值图
     * @param low
     * @param high
     * @details low<= v && v<=high
     */
    void toGray(const cv::cuda::GpuMat &src, const cv::cuda::GpuMat &dst, uint8_t low, uint8_t high, cv::cuda::Stream &stream = cv::cuda::Stream::Null());

    // /**
    //  * @brief 将BayerRG转为灰度图
    //  * @details 此方法进行范围阈值操作
    //  * @param src BayerRG原始图像
    //  * @param dst 同样大小的单通道二值图
    //  * @param low
    //  * @param high
    //  * @details low<= v && v<=high
    //  */
    // void toGray_0(const cv::cuda::GpuMat &src, const cv::cuda::GpuMat &dst, uint8_t low, uint8_t high, cv::cuda::Stream &stream = cv::cuda::Stream::Null());


    // void toDoublePtr(const cv::cuda::GpuMat &src, double *&dst);

}// namespace rm_armor_finder::cudas
#endif// IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__CUDAS__H
