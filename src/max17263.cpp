#include "i2c/i2c.h"
#include "max17263_cfg.hpp"
#include "max1726x.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include <cmath>

class MAX17263Node : public rclcpp::Node {
public:
  MAX17263Node() : Node("max17263_node") {
    configure_driver();

    this->declare_parameter<std::string>("dev_path", "/dev/i2c-1");
    this->declare_parameter<std::string>("frame_id", "imu");
    this->declare_parameter<std::string>("topic", "/imu");

    std::string I2C_DEVICE_PATH;
    std::string ROS_FRAME_ID;
    std::string ROS_TOPIC_NAME;

    this->get_parameter("dev_path", I2C_DEVICE_PATH);
    this->get_parameter("frame_id", ros_frame_id);
    this->get_parameter("topic", ROS_TOPIC_NAME);

    // display set parameters
    RCLCPP_INFO(this->get_logger(), "dev_path: %s", I2C_DEVICE_PATH.c_str());
    RCLCPP_INFO(this->get_logger(), "frame_id: %s", ROS_FRAME_ID.c_str());
    RCLCPP_INFO(this->get_logger(), "topic: %s", ROS_TOPIC_NAME.c_str());

    int i2c_fd = i2c_open(I2C_DEVICE_PATH.c_str());
    if (i2c_fd < 0) {
      RCLCPP_FATAL(this->get_logger(), "Could not open i2c device");
      exit(1);
    }

    i2c_dev.bus = i2c_fd;
    i2c_dev.addr = MAX1726X_I2C_ADDR;
    i2c_dev.iaddr_bytes = 1;
    i2c_dev.page_bytes = 256;

    ros_publisher = this->create_publisher<sensor_msgs::msg::BatteryState>(
        ROS_TOPIC_NAME, rclcpp::SensorDataQoS());
    RCLCPP_INFO(this->get_logger(), "Started MAX17263 Fueld Gauge Node");

    ros_timer = this->create_wall_timer(PUBLISH_PERIOD, [this] {
      if (auto x = make_message(); x != nullptr) {
        ros_publisher->publish(std::move(x));
      }
    });
  }

private:
  I2CDevice i2c_dev;
  max1726x_t driver_handle;
  rclcpp::TimerBase::SharedPtr ros_timer;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr ros_publisher;
  std::string ros_frame_id;

  void configure_driver() {
    max17263_init(&driver_handle, &driver_i2c_write, driver_i2c_read,
                  driver_delay_ms, this, R_SENSE);

    max1726x_ez_config_t config = {.charge_voltage_mv = 4200,
                                   .design_cap_mah = DESIGN_CAPACITY_mAh,
                                   .i_chg_term_ma = 100,
                                   .v_empty =
                                       max1726x_v_empty_from_mv(3300, 2880)};

    max1726x_ez_config(&driver_handle, &config);

    if (LED_COUNT != UINT8_MAX) {
      max1726x_led_cfg1_t led_cfg = {.n_bars = LED_COUNT,
                                     .gr_en = true,
                                     .l_chg = true,
                                     .led_md = 1,
                                     .ani_md = 3,
                                     .ani_step = 2,
                                     .led_timer = 5};

      max17263_write_led_cfg1(&driver_handle, &led_cfg);
    }
  }

  std::unique_ptr<sensor_msgs::msg::BatteryState> make_message() {
    auto ret = std::make_unique<sensor_msgs::msg::BatteryState>();
    ret->header.stamp = this->get_clock()->now();
    ret->header.frame_id = ros_frame_id;

    double voltage_V;
    max1726x_read_voltage(&driver_handle, &voltage_V);
    ret->voltage = voltage_V;

    ret->temperature = NAN;

    double current_A;
    max1726x_read_current(&driver_handle, &current_A);
    ret->current = current_A;

    double capacity_ah;
    max1726x_read_remaining_capacity(&driver_handle, &capacity_ah);

    ret->design_capacity = DESIGN_CAPACITY_mAh / 1000.f;

    double percentage;
    max1726x_read_soc(&driver_handle, &percentage);
    ret->percentage = percentage;

    ret->present = true;

    return ret;
  }

  static int driver_i2c_read([[maybe_unused]] uint8_t dev_addr,
                             const uint8_t *write_data,
                             [[maybe_unused]] size_t write_len,
                             uint8_t *read_data, size_t read_len,
                             void *user_data) {
    auto *this_ = reinterpret_cast<const MAX17263Node *>(user_data);

    const uint8_t reg = write_data[0];
    return i2c_ioctl_read(&this_->i2c_dev, reg, read_data, read_len) > 0;
  }

  static int driver_i2c_write([[maybe_unused]] uint8_t addr, const uint8_t *buf,
                              size_t len, void *user_data) {
    auto *this_ = reinterpret_cast<const MAX17263Node *>(user_data);

    const uint8_t reg = buf[0];

    if (i2c_ioctl_write(&this_->i2c_dev, reg, &buf[1], len - 1) < 0) {
      return 1;
    }

    return 0;
  }

  static void driver_delay_ms(uint32_t ms, [[maybe_unused]] void *user_data) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MAX17263Node>());
  rclcpp::shutdown();
}