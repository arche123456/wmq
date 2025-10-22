#include "rm_armor_finder/Cudas.h"
#include <cstdint>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <utility>

namespace rm_armor_finder {
    namespace cudas {

        __global__ void getRedFromBayerRG_call(cv::cuda::PtrStepSz<uint8_t> src, cv::cuda::PtrStepSz<uint8_t> dst, int thresh,
                                               float t_r, float t_g1, float t_g2, float t_b) {
            int _i = (int(threadIdx.x) + blockIdx.x * blockDim.x);
            int _j = (int(threadIdx.y) + blockIdx.y * blockDim.y);
            int i = _i * 2;
            int j = _j * 2;
            if (i >= src.cols || j >= src.rows) return;
            uint8_t r = src(j, i), g1 = src(j, i + 1),
                    g2 = src(j + 1, i), b = src(j + 1, i + 1);
            uint8_t max = r;
            if (g1 > max) max = g1;
            if (g2 > max) max = g2;
            if (b > max) max = b;
            float v = t_r * r + t_g1 * g1 + t_g2 * g2 + t_b * b;
            dst(_j, _i) = (v + max) > thresh ? 255 : 0;
        }

        __global__ void getBlueFromBayerRG_call(cv::cuda::PtrStepSz<uint8_t> src, cv::cuda::PtrStepSz<uint8_t> dst, int thresh,
                                                float t_r, float t_g1, float t_g2, float t_b) {
            int _i = (int(threadIdx.x) + blockIdx.x * blockDim.x);
            int _j = (int(threadIdx.y) + blockIdx.y * blockDim.y);
            int i = _i * 2 + 1;
            int j = _j * 2 + 1;
            if (i >= src.cols || j >= src.rows) return;
            uint8_t b = src(j, i), g2 = src(j, i - 1),
                    g1 = src(j - 1, i), r = src(j - 1, i - 1);
            uint8_t max = r;
            if (g1 > max) max = g1;
            if (g2 > max) max = g2;
            if (b > max) max = b;
            float v = t_r * r + t_g1 * g1 + t_g2 * g2 + t_b * b;
            dst(_j, _i) = (v + max) > thresh ? 255 : 0;
        }

        __global__ void toGray_call(cv::cuda::PtrStepSz<uint8_t> src, cv::cuda::PtrStepSz<float> dst, int low, int high) {
            int _i = (int(threadIdx.x) + blockIdx.x * blockDim.x);
            int _j = (int(threadIdx.y) + blockIdx.y * blockDim.y);
            int i = _i * 2;
            int j = _j * 2;
            if (_i >= dst.cols || _j >= dst.rows) return;
            auto v = src(j, i) + src(j + 1, i) + src(j, i + 1) + src(j + 1, i + 1);
            dst(_j, _i) = low <= v && v <= high ? 1.F : 0.F;
        }

        __global__ void getBlueFromBayerRG_2_call(cv::cuda::PtrStepSz<uint8_t> src, cv::cuda::PtrStepSz<uint8_t> dst,
                                                  int h_low, int h_high, int s_low, int s_high, int v_low, int v_high) {
            const int _i = (int(threadIdx.x) + blockIdx.x * blockDim.x);
            const int _j = (int(threadIdx.y) + blockIdx.y * blockDim.y);
            const int i = _i * 2;
            const int j = _j * 2;
            if (_i >= dst.cols || _j >= dst.rows) return;
            const uint8_t r = src(j, i);
            const uint8_t g1 = src(j, i + 1);
            const uint8_t g2 = src(j + 1, i);
            const uint8_t b = src(j + 1, i + 1);
            const uint8_t g = (g1 + g2) / 2;

            uint8_t res = 0;

            const uint8_t m = (g < r ? g : r);         // m = min(r,g,b)
            if (b > m) {                               // V = B
                const int C = b - m;                   // C = V - m
                const int H = (r - g) * 255 / C;       // H = 60° * ((R-G)/C+4) if V = B
                const int S = b == 0 ? 0 : C * 255 / b;// S = C/V
                // H: [-255,255] S: [0,255] V: [0,255]
                if (h_low <= H && H <= h_high && s_low <= S && S <= s_high && v_low <= b && b <= v_high) {
                    res = 255;
                }
            }
            dst(_j, _i) = res;
        }

        __global__ void getRedFromBayerRG_2_call(cv::cuda::PtrStepSz<uint8_t> src, cv::cuda::PtrStepSz<uint8_t> dst,
                                                 int h_low, int h_high, int s_low, int s_high, int v_low, int v_high) {
            const int _i = (int(threadIdx.x) + blockIdx.x * blockDim.x);
            const int _j = (int(threadIdx.y) + blockIdx.y * blockDim.y);
            const int i = _i * 2;
            const int j = _j * 2;
            if (_i >= dst.cols || _j >= dst.rows) return;
            // int b = src(j, i);// swap r and b define: 交换红蓝定义, 避免Red的H计算被切分为两段
            // int g = (src(j, i + 1) + src(j + 1, i)) / 2;
            // int r = src(j + 1, i + 1);
            // uint8_t cnt = 0;

            int r = 0, g = 0, b = 0;
            uint8_t cnt = 0;
            constexpr const int neibor_size = 1;
            for (int x = -2 * neibor_size; x <= 2 * neibor_size; x += 2) {
                if (x + i < 0 || x + i >= src.cols) continue;
                for (int y = -2 * neibor_size; y <= 2 * neibor_size; y += 2) {
                    if (y + j < 0 || y + j >= src.rows) continue;
                    b += src(j + y, i + x);
                    g += (src(j + y, i + x + 1) + src(j + y + 1, i + x)) / 2;
                    r += src(j + y + 1, i + x + 1);
                    cnt += 1;
                }
            }
            r /= cnt;
            g /= cnt;
            b /= cnt;

            uint8_t res = 0;

            const int m = (g < r ? g : r);             // m = min(r,g,b)
            if (b > m) {                               // V = B
                const int C = b - m;                   // C = V - m
                const int H = (r - g) * 255 / C;       // H = 60° * ((R-G)/C+4) if V = B
                const int S = b == 0 ? 0 : C * 255 / b;// S = C/V
                // H: [-255,255] S: [0,255] V: [0,255]
                if (h_low <= H && H <= h_high && s_low <= S && S <= s_high && v_low <= b && b <= v_high) {
                    res = 255;
                }
            }
            dst(_j, _i) = res;
        }

        // __global__ void toDoublePtr_call(cv::cuda::PtrStepSz<uint8_t> src, double *dst) {
        //     int x = blockIdx.x * blockDim.x + threadIdx.x;
        //     int y = blockIdx.y * blockDim.y + threadIdx.y;

        //     if (x < src.cols && y < src.rows) {
        //         dst[y * src.cols + x] = static_cast<double>(src(y, x));
        //     }
        // }

        void toGray(const cv::cuda::GpuMat &src, const cv::cuda::GpuMat &dst, uint8_t low, uint8_t high, cv::cuda::Stream &stream) {
            CV_DbgAssert(src.type() == CV_8UC1);
            CV_DbgAssert(dst.type() == CV_32FC1);
            CV_DbgAssert(!src.empty() && !dst.empty());

            dim3 blockDim(32, 32);
            dim3 gridDim((dst.cols + blockDim.x - 1) / blockDim.x, (dst.rows + blockDim.y - 1) / blockDim.y);
            auto s = cv::cuda::StreamAccessor::getStream(stream);
            toGray_call<<<gridDim, blockDim, 0, s>>>(src, dst, low * 4, high * 4);
        }


        void getColorFromBayerRG(const cv::cuda::GpuMat &src, const cv::cuda::GpuMat &dst, bool getRed,
                                 int h_low, int h_high, int s_low, int s_high, int v_low, int v_high,
                                 cv::cuda::Stream &stream) {
            CV_DbgAssert(src.type() == CV_8UC1);
            CV_DbgAssert(!src.empty() && !dst.empty());

            dim3 blockDim(32, 32);
            dim3 gridDim((dst.cols + blockDim.x - 1) / blockDim.x, (dst.rows + blockDim.y - 1) / blockDim.y);
            auto s = cv::cuda::StreamAccessor::getStream(stream);
            // if (getRed)
            //     getRedFromBayerRG_call<<<gridDim, blockDim, 0, s>>>(src, dst, thresh,
            //                                                         t_r, t_g1, t_g2, t_b);
            // else
            //     getBlueFromBayerRG_call<<<gridDim, blockDim, 0, s>>>(src, dst, thresh,
            //                                                          t_r, t_g1, t_g2, t_b);
            if (getRed)
                getRedFromBayerRG_2_call<<<gridDim, blockDim, 0, s>>>(src, dst, -255, 255, 150, 255, 50, 255);
            else
                getBlueFromBayerRG_2_call<<<gridDim, blockDim, 0, s>>>(src, dst, -255, 255, 150, 255, 50, 255);
        }

        // void toDoublePtr(const cv::cuda::GpuMat &src, double *&dst) {
        //     dim3 blockDim(std::min(src.cols, 32), std::min(src.rows, 32));
        //     dim3 gridDim((src.cols + blockDim.x - 1) / blockDim.x, (src.rows + blockDim.y - 1) / blockDim.y);
        //     toDoublePtr_call<<<gridDim, blockDim>>>(src, dst);
        // }
    }// namespace cudas
}// namespace rm_armor_finder