#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <vector>

using namespace std::chrono_literals;

class PointCloudTransformer : public rclcpp::Node {
public:
  PointCloudTransformer() : Node("pointcloud_transformer") {
    // 初始化保存目录
    save_dir_ = "/home/ifr/ifr-ros2/ws-sentry-lib/src/laser_to_foot/pcd_save";
    std::filesystem::create_directories(save_dir_);

    // 初始化TF2组件
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 配置QoS
    // auto qos = rclcpp::SensorDataQoS()
    //                .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    //                .history(rclcpp::HistoryPolicy::KeepLast)
    //                .durability(rclcpp::DurabilityPolicy::Volatile);

    // 创建订阅器
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/Laser_map", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          process_pointcloud(msg);
        });

    RCLCPP_INFO(get_logger(), "节点已启动，等待数据...");
  }

private:
  bool is_little_endian() {
    uint16_t test = 0x0001;
    return reinterpret_cast<char *>(&test)[0] == 0x01;
  }

  void process_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    try {
      // 坐标变换
      auto transform = tf_buffer_->lookupTransform("map", msg->header.frame_id,
                                                   tf2::TimePoint(), 1s);

      // 执行点云变换
      sensor_msgs::msg::PointCloud2 transformed_cloud;
      tf2::doTransform(*msg, transformed_cloud, transform);
      transformed_cloud.header.stamp = now();
      transformed_cloud.header.frame_id = "map";

      // 保存点云
      save_as_pcd(transformed_cloud);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "变换失败: %s", ex.what());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "处理错误: %s", e.what());
    }
  }

  void save_as_pcd(const sensor_msgs::msg::PointCloud2 &cloud) {
    // 验证必要字段
    struct FieldInfo {
      bool exists;
      uint32_t offset;
      uint8_t type;
    };
    std::map<std::string, FieldInfo> fields{{"x", {false, 0, 0}},
                                            {"y", {false, 0, 0}},
                                            {"z", {false, 0, 0}},
                                            {"intensity", {false, 0, 0}}};

    for (const auto &field : cloud.fields) {
      if (fields.count(field.name)) {
        fields[field.name] = {true, field.offset, field.datatype};
      }
    }

    // 检查字段完整性
    for (const auto &[name, info] : fields) {
      if (!info.exists || info.type != sensor_msgs::msg::PointField::FLOAT32) {
        RCLCPP_ERROR(get_logger(), "无效或缺失字段: %s", name.c_str());
        return;
      }
    }

    // 准备数据存储
    const bool sys_le = is_little_endian();
    const size_t point_count = cloud.width * cloud.height;
    std::vector<std::array<float, 4>> points;
    points.reserve(point_count);

    // 解析点数据
    const uint8_t *data_ptr = cloud.data.data();
    for (size_t i = 0; i < point_count; ++i) {
      const uint8_t *point_ptr = data_ptr + i * cloud.point_step;
      std::array<float, 4> point;

      // 通用字段读取函数
      auto read_field = [&](const std::string &name) {
        uint32_t bytes;
        memcpy(&bytes, point_ptr + fields[name].offset, 4);
        if (cloud.is_bigendian && sys_le)
          bytes = __builtin_bswap32(bytes);
        return *reinterpret_cast<float *>(&bytes);
      };

      point[0] = read_field("x");
      point[1] = read_field("y");
      point[2] = read_field("z");
      point[3] = read_field("intensity");
      points.emplace_back(point);
    }

    // 生成文件名
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
    std::string filename =
        save_dir_ + "/map_" + std::to_string(timestamp) + ".pcd";

    // 写入文件
    std::ofstream file(filename);
    if (!file) {
      RCLCPP_ERROR(get_logger(), "文件创建失败: %s", filename.c_str());
      return;
    }

    // 写入PCD头
    file << "VERSION .7\n"
         << "FIELDS x y z intensity\n"
         << "SIZE 4 4 4 4\n"
         << "TYPE F F F F\n"
         << "COUNT 1 1 1 1\n"
         << "WIDTH " << points.size() << "\n"
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << points.size() << "\n"
         << "DATA ascii\n";

    // 写入点数据
    for (const auto &p : points) {
      file << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << "\n";
    }

    RCLCPP_INFO(get_logger(), "成功保存地图点云: %s", filename.c_str());
  }

  std::string save_dir_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}