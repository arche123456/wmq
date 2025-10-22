#ifndef IFR_ROS2_CV__PACKAGE_RM_SERIAL_PORT__PACKET__H
#define IFR_ROS2_CV__PACKAGE_RM_SERIAL_PORT__PACKET__H
#include <array>
#include <cstdint>
#include <ifr_common/crc.h>
#include <ifr_interface/msg/detail/serial_control__struct.hpp>
#include <ifr_interface/msg/serial_control.hpp>
#include <ifr_interface/msg/serial_imu_data.hpp>

namespace rm_serial_port {
    namespace packet {
#pragma pack(1)
        struct IMU_Test {
            static const constexpr uint8_t HEAD = 0x5C;
            uint8_t head; //帧头0x5C
            int16_t yaw;  //Yaw轴角度数据（绕z轴转动）*100,-180~180
            int16_t pitch;//Pitch轴角度数据（绕x轴转动）*100
            int16_t roll; //Roll轴角度数据（绕y轴转动）*100
            int16_t qua_w;//四元数*100
            int16_t qua_x;//四元数*100
            int16_t qua_y;//四元数*100
            int16_t qua_z;//四元数*100
            uint8_t crc;  //校验

            ///@return [-180,180]
            float getYaw() const { return yaw / 100.0F; }

            float getPitch() const { return pitch / 100.0F; }

            float getRoll() const { return roll / 100.0F; }

            std::array<float, 4> getQua() const { return {
                    qua_w / 100.0F,
                    qua_x / 100.0F,
                    qua_y / 100.0F,
                    qua_z / 100.0F,
            }; }

            ///进行crc校验
            bool check() const { return head == HEAD && crc::CRC8::check_crcT(this); }
        };
        struct IMU_Data {
            static const constexpr uint8_t HEAD = 0x5C;
            uint8_t head;          //帧头0x5C
            int16_t yaw;           //Yaw轴角度数据（绕z轴转动）*100,-180~180
            int16_t pitch;         //Pitch轴角度数据（绕x轴转动）*100
            int16_t roll;          //Roll轴角度数据（绕y轴转动）*100
            bool is_red : 1;       //是否是红方
            bool use_forecast : 1; //是否开启预测
            uint8_t imu_type : 2;  ///陀螺仪坐标系类型
            uint8_t aim_type : 1;  ///自瞄类型
            uint8_t need_flash : 1;///是否需要刷新
            uint8_t reserve : 2;   //保留
            uint8_t crc;           //校验

            enum class Type {
                Abt = 0,///艾博特 & 潘佳俊 & 王禹波 坐标系
                Lxs = 1,///廖旭昇 坐标系
                Reserve1 = 2,
                Reserve2 = 3
            };

            enum class AimType {
                Armor = 0,///装甲板
                Power = 1,///能量机关
            };

            [[nodiscard]] inline ifr_interface::msg::SerialImuData toRos() const {
                ifr_interface::msg::SerialImuData msg;
                toRos(msg);
                return msg;
            }
            inline ifr_interface::msg::SerialImuData &toRos(ifr_interface::msg::SerialImuData &msg) const {
                msg.head = head;
                msg.yaw = getYaw();
                msg.pitch = getPitch();
                msg.roll = getRoll();
                msg.is_red = is_red;
                msg.use_forecast = use_forecast;
                msg.imu_type = imu_type;
                msg.aim_type = aim_type;
                msg.need_flash = need_flash;
                msg.crc = crc;

                return msg;
            }

            ///@return [-180,180]
            float getYaw() const { return yaw / 100.0F; }

            float getPitch() const { return pitch / 100.0F; }

            float getRoll() const { return roll / 100.0F; }

            ///进行crc校验
            bool check() const { return head == HEAD && crc::CRC8::check_crcT(this); }
        };
        ///云台控制
        struct MotorControl {
            static const constexpr uint8_t HEAD = 0x5b;
            static const constexpr float factor = 1e4;
            static const constexpr double send2angle = 180 / M_PI / factor;
            uint8_t head = HEAD;//固定包头
            bool has_target : 1;//是否有目标
            bool can_attack : 1;//是否可以攻击
            uint8_t delay : 6;  //延时(ms)
            int16_t move_x = 0; //云台水平方向误差(弧度制, 扩大10^4传输)
            int16_t move_y = 0; //云台垂直方向误差(弧度制, 扩大10^4传输)
            uint8_t crc = 0;    //CRC8校验

            [[nodiscard]] inline ifr_interface::msg::SerialControl toRos() const {
                ifr_interface::msg::SerialControl msg;
                toRos(msg);
                return msg;
            }
            inline ifr_interface::msg::SerialControl &toRos(ifr_interface::msg::SerialControl &msg) const {
                msg.crc = crc;
                msg.head = head;
                msg.move_fb = 0;
                msg.move_lr = 0;
                msg.gun_yaw = move_x * send2angle;
                msg.gun_pitch = move_y * send2angle;
                msg.has_target = has_target;
                msg.can_attack = can_attack;
                msg.scan = false;
                msg.gyro = false;
                msg.delay = delay;
                msg.crc = crc;
                return msg;
            }

            MotorControl()
                : has_target(false),
                  can_attack(false),
                  delay(0) {}
            inline void set(bool can_attack, float x, float y) {
                this->can_attack = can_attack;
                this->move_x = int16_t(x * factor);
                this->move_y = int16_t(y * factor);
            }

            inline void finish(bool has_target, int64_t delay) {
                this->has_target = has_target;
                this->delay = delay > ((1 << 6) - 1) ? (1 << 6) - 1 : delay;
                crc = sumCrc();
            }

            ///通过数据计算crc8的值
            inline uint8_t sumCrc() const { return crc::CRC8::summon_crcT(this); }

            ///进行crc校验
            bool check() const { return head == HEAD && crc == sumCrc(); }
        };

        ///完整控制
        struct FullControl {
            static const constexpr uint8_t HEAD = 0x5a;
            static const constexpr float factor = 1e4;
            static const constexpr double send2angle = 180 / M_PI / factor;
            uint8_t head = HEAD;   //固定包头
            uint16_t move_y = 1024;//前后 y（364-1684），1024 代表速度 0 ; 正/负 660 控制前/后的速度
            uint16_t move_x = 1024;//左右 x（364-1684），1024 代表速度 0 ; 正/负 660 控制右/左的速度
            int16_t gun_yaw = 0;   //云台垂直方向误差(弧度制, 扩大10^4传输)
            int16_t gun_pitch = 0; //云台水平方向误差(弧度制, 扩大10^4传输)
            bool is_scan : 1;      //是否进入扫描模式, 如果进入扫描模式则忽略gun_*值
            bool gyro : 1;         //是否开启陀螺
            bool shoot : 1;        //是否开启发射, 只负责告知发射结构可以允许被使用，最终发射子弹的机制（频率，射速，是否发射弹丸）由控制组决定
            uint8_t reserve : 5;   //保留
            uint8_t crc = 0;       //CRC8校验

            [[nodiscard]] inline ifr_interface::msg::SerialControl toRos() const {
                ifr_interface::msg::SerialControl msg;
                toRos(msg);
                return msg;
            }
            inline ifr_interface::msg::SerialControl &toRos(ifr_interface::msg::SerialControl &msg) const {
                msg.head = head;
                msg.move_fb = move_y - 1024;
                msg.move_lr = move_x - 1024;
                msg.gun_yaw = gun_yaw * send2angle;
                msg.gun_pitch = gun_pitch * send2angle;
                msg.has_target = true;
                msg.can_attack = shoot;
                msg.scan = is_scan;
                msg.gyro = gyro;
                msg.delay = -1;
                msg.crc = crc;
                return msg;
            }

            ///格式化移动范围
            static inline uint16_t f_move(const float &in) {
                static constexpr const uint16_t offset = 1024;
                static constexpr const uint16_t min = offset - 660;
                static constexpr const uint16_t max = offset + 660;
                const auto v = static_cast<int16_t>(in) + static_cast<int16_t>(offset);
                if (v < 0 || v < min)
                    return min;
                else if (v > max)
                    return max;
                else
                    return v;
            }

            ///格式化角度
            static inline int16_t f_angle(const float &angle) noexcept {
                return int16_t(angle * factor);
            }

            FullControl(float _move_x = 0, float _move_y = 0, float _gun_yaw = 0, float _gun_pitch = 0,
                        bool _is_scan = false, bool _gyro = false, bool _shoot = false)
                : move_y(f_move(_move_y)), move_x(f_move(_move_x)),
                  gun_yaw(f_angle(_gun_yaw)), gun_pitch(f_angle(_gun_pitch)),
                  is_scan(_is_scan), gyro(_gyro), shoot(_shoot) {
            }

            inline void setMoveX(float x) { move_x = f_move(x); }
            inline void setMoveY(float y) { move_y = f_move(y); }
            inline void setMove(float x, float y) {
                move_x = f_move(x);
                move_y = f_move(y);
            }
            inline void setGunYaw(float yaw) { gun_yaw = f_angle(yaw); }
            inline void setGunPitch(float pitch) { gun_pitch = f_angle(pitch); }
            inline void setGun(float yaw, float pitch) {
                gun_yaw = f_angle(yaw);
                gun_pitch = f_angle(pitch);
            }
            inline void setScan(bool enable) { is_scan = enable; }
            inline void setGyro(bool enable) { gyro = enable; }
            inline void setShoot(bool enable) { shoot = enable; }
            int getMoveX() const { return move_x - 1024; }
            int getMoveY() const { return move_y - 1024; }
            int getGunYaw() const { return gun_yaw / factor; }
            int getGunPitch() const { return gun_pitch / factor; }
            bool isGyro() const { return gyro; }
            bool isShoot() const { return shoot; }

            ///通过数据计算crc8的值
            inline uint8_t sumCrc() const { return crc::CRC8::summon_crcT(this); }

            ///完成数据包 (填充crc)
            inline void finish() { crc = sumCrc(); }

            ///进行crc校验
            bool check() const { return head == HEAD && crc == sumCrc(); }
        };

        ///空数据包
        struct EmptyPackage {
            static const constexpr uint8_t HEAD = 0x00;
            uint8_t head = HEAD;
            uint8_t crc;
            constexpr bool check() const { return false; }
        };
#pragma pack()
    }// namespace packet
}// namespace rm_serial_port
#endif// IFR_ROS2_CV__PACKAGE_RM_SERIAL_PORT__PACKET__H
