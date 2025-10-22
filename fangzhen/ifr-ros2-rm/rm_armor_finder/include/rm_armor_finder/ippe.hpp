/// src: https://github.com/tobycollins/IPPE
#ifndef IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__IPPE__HPP
#define IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__IPPE__HPP

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <limits>

#define IPPE_SMALL 1e-7// 用于测试接近零的“小”值的一个小常数。

namespace IPPE {

    class PoseSolver {
    public:
        /// @brief 寻找给定一组对应点及其各自的重投影误差的平面物体的两种可能姿态。按照重投影误差从低到高的顺序对姿态进行排序，第一个姿态具有最低的重投影误差。
        /// @param _objectPoints  在对象坐标系中定义的包含4个或更多共面对象点的数组。1xN/Nx1 3通道（float或double），其中N是点的数量
        /// @param _imagePoints   对应的图像点的数组，1xN/Nx1 2通道。可以是像素坐标或归一化像素坐标。
        /// @param _cameraMatrix  内部相机矩阵（与OpenCV的定义相同）。如果_imagePoints是归一化像素坐标，则必须设置_cameraMatrix = cv::noArray()
        /// @param _distCoeffs    内部相机畸变向量（与OpenCV的定义相同）。如果_imagePoints是归一化像素坐标，则必须设置_cameraMatrix = cv::noArray()
        /// @param _rvec1         第一个旋转解（3x1旋转向量）
        /// @param _tvec1         第一个平移解（3x1向量）
        /// @param reprojErr1     第一个解的重投影误差
        /// @param _rvec2         第二个旋转解（3x1旋转向量）
        /// @param _tvec2         第二个平移解（3x1向量）
        /// @param reprojErr2     第二个解的重投影误差
        static void solveGeneric(cv::InputArray _objectPoints, cv::InputArray _imagePoints, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                                 cv::OutputArray _rvec1, cv::OutputArray _tvec1, float &reprojErr1, cv::OutputArray _rvec2, cv::OutputArray _tvec2, float &reprojErr2);


        /// @brief 使用IPPE找到正方形平面物体的两种可能姿态及其各自的重投影误差。这些姿态被排序，使得第一个姿态具有最低的重投影误差。
        /// @param _squareLength      正方形的边长（也是它的宽度），以对象坐标单位表示（例如毫米、米等）
        /// @param _imagePoints       对应的图像点的数组，1xN/Nx1 2通道。可以是像素坐标或归一化像素坐标。
        /// @param _cameraMatrix      内部相机矩阵（与OpenCV的定义相同）。如果_imagePoints是归一化像素坐标，则必须设置_cameraMatrix = cv::noArray()
        /// @param _distCoeffs        内部相机畸变向量（与OpenCV的定义相同）。如果_imagePoints是归一化像素坐标，则必须设置_cameraMatrix = cv::noArray()
        /// @param _rvec1             第一个旋转解（3x1旋转向量）
        /// @param _tvec1             第一个平移解（3x1向量）
        /// @param reprojErr1         第一个解的重投影误差
        /// @param _rvec2             第二个旋转解（3x1旋转向量）
        /// @param _tvec2             第二个平移解（3x1向量）
        /// @param reprojErr2         第二个解的重投影误差
        static void solveSquare(float squareLength, cv::InputArray _imagePoints, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                                cv::OutputArray _rvec1, cv::OutputArray _tvec1, float &reprojErr1, cv::OutputArray _rvec2, cv::OutputArray _tvec2, float &reprojErr2);


        /// @brief 生成正方形平面物体的4个对象点
        /// @param squareLength      正方形的边长（也是它的宽度），以对象坐标单位表示（例如毫米、米等）
        /// @param _objectPoints     4个对象点的集合（1x4 3通道双精度）
        static void generateSquareObjectCorners3D(double squareLength, cv::OutputArray _objectPoints);


        /// @brief 生成正方形平面物体的4个对象点，不包括z分量（所有点的z值均为0）。
        /// @param squareLength      正方形的边长（也是它的宽度），以对象坐标单位表示（例如毫米、米等）
        /// @param _objectPoints     4个对象点的集合（1x4 2通道双精度）
        static void generateSquareObjectCorners2D(double squareLength, cv::OutputArray _objectPoints);


        /// @brief 计算物体在相机坐标系中姿态下的平均深度
        /// @param objectPoints    在3D物体空间中定义的对象点
        /// @param rvec            姿态的旋转分量
        /// @param tvec            姿态的平移分量
        /// @return                物体的平均深度
        static double meanSceneDepth(cv::InputArray objectPoints, cv::InputArray rvec, cv::InputArray tvec);

    private:
        /// @brief 给定一组在归一化像素坐标中的对应点，找到平面物体的两种可能姿态。这些姿态**不**按重投影误差排序。请注意，返回的姿态是物体到相机的变换，而不是相机到物体的变换。
        /// @param _objectPoints         在对象坐标系中定义的包含4个或更多共面对象点的数组。1xN/Nx1 3通道（float或double）。
        /// @param _normalizedImagePoints   对应的图像点在归一化像素坐标中的数组，1xN/Nx1 2通道（float或double）。
        /// @param _Ma                   第一个姿态解（未排序）
        /// @param _Mb                   第二个姿态解（未排序）
        static void solveGeneric(cv::InputArray _objectPoints, cv::InputArray _normalizedImagePoints, cv::OutputArray _Ma, cv::OutputArray _Mb);

        /// @brief 给定一组在归一化像素坐标中的对应点，找到平面物体在其规范位置上的两种可能姿态。这些姿态**不**按重投影误差排序。请注意，返回的姿态是物体到相机的变换，而不是相机到物体的变换。
        /// @param _canonicalObjPoints      在对象坐标系中定义的包含4个或更多共面对象点的数组。1xN/Nx1 3通道（double），其中N是点的数量
        /// @param _normalizedInputPoints   对应的图像点在归一化像素坐标中的数组，1xN/Nx1 2通道（double），其中N是点的数量
        /// @param _H                   将canonicalObjPoints映射到normalizedInputPoints的单应矩阵。
        /// @param _Ma
        /// @param _Mb
        static void solveCanonicalForm(cv::InputArray _canonicalObjPoints, cv::InputArray _normalizedInputPoints, cv::InputArray _H,
                                       cv::OutputArray _Ma, cv::OutputArray _Mb);

        /// @brief 给定旋转解，计算平移解
        /// @param _objectPoints              对应的对象点的数组，1xN/Nx1 3通道，其中N是点的数量
        /// @param _normalizedImagePoints     对应的图像点（无畸变），1xN/Nx1 2通道，其中N是点的数量
        /// @param _R                         旋转解（3x1旋转向量）
        /// @param _t  Translation solution   平移解（3x1旋转向量）
        static void computeTranslation(cv::InputArray _objectPoints, cv::InputArray _normalizedImgPoints, cv::InputArray _R, cv::OutputArray _t);

        /// @brief 根据物体平面上点（ux,uy）处的单应性矩阵H的雅可比矩阵计算两个旋转解。为了获得最高的准确性，雅可比矩阵应在点对应关系的质心处计算（请参阅IPPE论文以了解解释）。对于物体平面上的点（ux,uy），假设单应性矩阵H将（ux,uy）映射到图像中的点（在无畸变和归一化像素坐标中）。雅可比矩阵[J00，J01；J10，J11]是在（ux,uy）处评估的映射的雅可比矩阵。
        /// @param j00                        在（ux,uy）处的单应性雅可比系数
        /// @param j01                        在（ux,uy）处的单应性雅可比系数
        /// @param j10                        在（ux,uy）处的单应性雅可比系数
        /// @param j11                        在（ux,uy）处的单应性雅可比系数
        /// @param p                          将点（ux,uy）映射到图像中的x坐标（无畸变和归一化位置）
        /// @param q                          将点（ux,uy）映射到图像中的y坐标（无畸变和归一化位置）
        static void computeRotations(double j00, double j01, double j10, double j11, double p, double q, cv::OutputArray _R1, cv::OutputArray _R2);

        /// @brief 使用方形的四个角点的四个对应关系的闭合形式解决方案（它将源点映射到目标点）。源点是由以下方式定义的以零为中心的正方形的四个角：
        /// 点0: [-squareLength / 2.0，squareLength / 2.0]
        /// 点1: [squareLength / 2.0，squareLength / 2.0]
        /// 点2: [squareLength / 2.0，-squareLength / 2.0]
        /// 点3: [-squareLength / 2.0，-squareLength / 2.0]
        /// @param _targetPoints              四个对应目标点的数组，1x4/4x1 2通道。注意，点应按照与点0、1、2和3对应的顺序排列。
        /// @param halfLength                 方形的半边长（即squareLength/2.0）
        /// @param _H                         将源点映射到目标点的单应矩阵，3x3单通道
        static void homographyFromSquarePoints(cv::InputArray _targetPoints, double halfLength, cv::OutputArray _H);

        /// @brief 使用Rodrigues'公式将旋转矩阵快速转换为旋转向量
        /// @param _R               输入旋转矩阵，3x3 1通道（double）
        /// @param _r               输出旋转向量，3x1/1x3 1通道（double）
        static void rot2vec(cv::InputArray _R, cv::OutputArray _r);

        /// @brief 将一组平面物体点转换为“规范”对象坐标，即它们具有零均值并且位于z=0平面上
        /// @param _objectPoints            在对象坐标系中定义的包含4个或更多共面对象点的数组。1xN/Nx1 3通道（float或double），其中N是点的数量
        /// @param _canonicalObjectPoints   规范坐标中的对象点 1xN/Nx1 2通道（double）
        /// @param _MobjectPoints2Canonical 将_objectPoints映射到_canonicalObjectPoints的转换矩阵：4x4 1通道（double）
        static void makeCanonicalObjectPoints(cv::InputArray _objectPoints, cv::OutputArray _canonicalObjectPoints, cv::OutputArray _MobjectPoints2Canonical);

        /// @brief 计算姿态解的均方根（RMS）重投影误差。
        /// @param _objectPoints             在对象坐标系中定义的包含4个或更多共面对象点的数组。1xN/Nx1 3通道（float或double），其中N是点的数量
        /// @param _imagePoints              对应的图像点的数组，1xN/Nx1 2通道。这可以是像素坐标或归一化像素坐标。
        /// @param _cameraMatrix             内部相机矩阵（与OpenCV的定义相同）。如果_imagePoints是归一化像素坐标，则必须设置_cameraMatrix = cv::noArray()。
        /// @param _distCoeffs               内部相机畸变向量（与OpenCV的定义相同）。如果_imagePoints是归一化像素坐标，则必须设置_cameraMatrix = cv::noArray()。
        /// @param _M                        从3D对象到相机坐标的姿态矩阵：4x4 1通道（double）
        /// @param err                       RMS重投影误差
        static void evalReprojError(cv::InputArray _objectPoints, cv::InputArray _imagePoints, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs, cv::InputArray _M, float &err);

        /// @brief 根据它们的RMS重投影误差对两个姿态解进行排序（最低的排在前面）。
        /// @param _objectPoints             在对象坐标系中定义的包含4个或更多共面对象点的数组。1xN/Nx1 3通道（float或double），其中N是点的数量
        /// @param _imagePoints              对应的图像点的数组，1xN/Nx1 2通道。这可以是像素坐标或归一化像素坐标。
        /// @param _cameraMatrix             内部相机矩阵（与OpenCV的定义相同）。如果_imagePoints是归一化像素坐标，则必须设置_cameraMatrix = cv::noArray()。
        /// @param _distCoeffs               内部相机畸变向量（与OpenCV的定义相同）。如果_imagePoints是归一化像素坐标，则必须设置_cameraMatrix = cv::noArray()。
        /// @param _Ma                       姿态矩阵1：4x4 1通道
        /// @param _Mb                       姿态矩阵2：4x4 1通道
        /// @param _M1                       具有最低RMS重投影误差的(Ma,Mb}中的成员。执行深度复制。
        /// @param _M2                       具有最高RMS重投影误差的(Ma,Mb}中的成员。执行深度复制。
        /// @param err1                      _M1的RMS重投影误差
        /// @param err2                      _M2的RMS重投影误差
        static void sortPosesByReprojError(cv::InputArray _objectPoints, cv::InputArray _imagePoints, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs, cv::InputArray _Ma, cv::InputArray _Mb, cv::OutputArray _M1, cv::OutputArray _M2, float &err1, float &err2);

        /// @brief 找到将向量_a旋转到z轴（0,0,1）的旋转_Ra
        /// @param _a                        向量：3x1矩阵（double）
        /// @param _Ra                       旋转：3x3矩阵（double）
        static void rotateVec2ZAxis(cv::InputArray _a, cv::OutputArray _Ra);

        /// @brief 计算将对象点旋转到平面z=0的旋转_R。这使用了第一个三个对象点的叉乘法。
        /// @param _objectPoints             在对象坐标系中定义的N>=3共面对象点的数组。1xN/Nx1 3通道（float或double），其中N是点的数量
        /// @param _R                        旋转矩阵：3x3（double）
        /// @return                          成功（true）或失败（false）
        static bool computeObjextSpaceR3Pts(cv::InputArray _objectPoints, cv::OutputArray _R);

        /// @brief 计算将对象点旋转到平面z=0的旋转_R。这使用了第一个三个对象点的叉乘法。
        /// @param _objectPointsZeroMean     零均值的共面对象点：3xN矩阵（double），其中N>=3
        /// @param _R                        旋转矩阵：3x3（double）
        /// @return                          成功（true）或失败（false）
        static bool computeObjextSpaceRSvD(cv::InputArray _objectPointsZeroMean, cv::OutputArray _R);
    };
}// namespace IPPE

namespace HomographyHO {

    /// 使用Harker和O'Leary的方法计算从源点到目标点的最佳拟合单应性矩阵：
    /// Harker, M., O'Leary, P., Computation of Homographies, Proceedings of the British Machine Vision Conference 2005, Oxford, England.
    /// 这不是作者的实现。
    /// @param srcPoints         源点的数组：1xN/Nx1 2通道（float或double），其中N是点的数量
    /// @param targPoints        目标点的数组：1xN/Nx1 2通道（float或double）
    /// @param H                 从源到目标的单应性矩阵：3x3 1通道（double）
    static void homographyHO(cv::InputArray srcPoints, cv::InputArray targPoints, cv::OutputArray H);

    /// 在单应性估计之前执行数据归一化。详情请参见Hartley, R., Zisserman, A., Multiple View Geometry in Computer Vision,
    /// Cambridge University Press, Cambridge, 2001
    /// @param Data             源数据点的数组：1xN/Nx1 2通道（float或double），其中N是点的数量
    /// @param DataN            归一化数据点：1xN/Nx1 2通道（float或double），其中N是点的数量
    /// @param T                从源到归一化的齐次变换：3x3 1通道（double）
    /// @param Ti               从归一化到源的齐次变换：3x3 1通道（double）
    static void normalizeDataIsotropic(cv::InputArray Data, cv::OutputArray DataN, cv::OutputArray T, cv::OutputArray Ti);

}// namespace HomographyHO

#endif// IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__IPPE__HPP
