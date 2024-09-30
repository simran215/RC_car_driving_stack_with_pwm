#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <servo_msgs/msg/state.hpp>  // Include your custom message header
#include <servo_msgs/msg/steer.hpp> 
#include "i2c_driver/i2c_driver.h"              // Update to your ROS 2 compatible I2C driver
#include "pca9685/pca9685.h"                   // Update to your ROS 2 compatible PCA9685 driver

using namespace std::chrono_literals;
using servo_msgs::msg::State;
using servo_msgs::msg::Steer;
using std_msgs::msg::Float64;


class TemplateI2CNode : public rclcpp::Node
{
public:
  TemplateI2CNode()
    : Node("template_i2c_internal")
  {
    // Initialize the I2C driver and PCA9685 driver here
    m_i2c_device_name = "/dev/i2c-1";
    m_pca9685_address = 0x42;
    motor_channel=9;
    servo_channel=1;
    m_i2c_driver = std::make_unique<I2C_Driver>(m_i2c_device_name.c_str());
    m_pca9685_servo_driver = std::make_unique<PCA9685>(m_i2c_driver.get(), m_pca9685_address);

    // Initialize publisher
    m_current_publisher = this->create_publisher<State>("/sensors/core", 10);
    m_steer_publisher = this->create_publisher<Float64>("/sensors/wheel_angle", 10);

    // Initialize subscriber
    m_servo_subscriber = this->create_subscription<Float64>(
      "/commands/servo/position", 1,
      [this](const Float64::SharedPtr msg) {
        this->templateServoSubscriberCallback(msg);
      });
    m_motor_subscriber = this->create_subscription<Float64>(
      "/commands/motor/speed", 1,
      [this](const Float64::SharedPtr msg) {
        this->templateMotorSubscriberCallback(msg);
      });
    

    // Initialize timer
    m_timer_for_publishing = this->create_wall_timer(
      20ms,
      [this]() { this->timerCallbackForPublishing(); }
    );

    // Open the I2C device
    bool open_success = m_i2c_driver->open_i2c_device();
    if (!open_success)
    {
      RCLCPP_ERROR(this->get_logger(), "FAILED to open I2C device named %s", m_i2c_driver->get_device_name());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Successfully opened named %s, with file descriptor = %d",
                  m_i2c_driver->get_device_name(), m_i2c_driver->get_file_descriptor());
    }

    // Set the configuration of the servo driver
    float new_frequency_in_hz = 50.0;
    bool verbose_display_for_servo_driver_init = false;
    bool result_servo_init = m_pca9685_servo_driver->initialise_with_frequency_in_hz(new_frequency_in_hz, verbose_display_for_servo_driver_init);
    if (!result_servo_init)
    {
      RCLCPP_ERROR(this->get_logger(), "FAILED - while initializing servo driver with I2C address %d", m_pca9685_servo_driver->get_i2c_address());
    }
  }

  ~TemplateI2CNode()
  { 
    m_pca9685_servo_driver->set_pwm_pulse_in_microseconds(motor_channel, 1500);
    m_pca9685_servo_driver->set_pwm_pulse_in_microseconds(servo_channel, 1650);
    // Close the I2C device
    bool close_success = m_i2c_driver->close_i2c_device();
    if (!close_success)
    {
      RCLCPP_ERROR(this->get_logger(), "FAILED to close I2C device named %s", m_i2c_driver->get_device_name());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Successfully closed device named %s", m_i2c_driver->get_device_name());
    }
  }

private:
  // Callback for subscriber
  void templateServoSubscriberCallback(const Float64::SharedPtr msg)
  {
    // Extract the channel and pulse width from the message
    uint8_t channel = servo_channel;//msg->channel;
    uint16_t pulse_width_in_us=-1851.852*msg->data+1648.15;//-1612.9*msg->data+1693.54;// 1000*msg->data+1500;//msg->pulse_width_in_microseconds;

    // Display the message received
    RCLCPP_INFO(this->get_logger(), "Message received for servo with channel = %d, and pulse width [us] = %d",
                channel, pulse_width_in_us);

    // Limit the pulse width to be either:
    // > zero
    // > in the range [1000,2000]
    if (pulse_width_in_us > 0)
    {
      if (pulse_width_in_us < 1000)
        pulse_width_in_us = 1000;
      if (pulse_width_in_us > 2000)
        pulse_width_in_us = 2000;
    }

    // Call the function to set the desired pulse width
    bool result = m_pca9685_servo_driver->set_pwm_pulse_in_microseconds(channel, pulse_width_in_us);

    // Display if an error occurred
    if (!result)
    {
      RCLCPP_ERROR(this->get_logger(), "FAILED to set pulse width for servo at channel %d", channel);
    }

    /// publish
    float steer=0;
    steer=msg->data;
    //if (msg->data< -0.187)
    //   steer= -0.187;
    // if (msg->data > 0.29)
    //   steer=0.29;       
    Float64 state_msg;
    // state_msg.header.stamp = now();
    state_msg.data = steer;
    m_steer_publisher->publish(state_msg);
  }

  // Callback for subscriber
  void templateMotorSubscriberCallback(const Float64::SharedPtr msg)
  {
    // Extract the channel and pulse width from the message
    uint8_t channel = motor_channel;//msg->channel;
    float speed=0;
    if (msg->data>=0)
      speed=msg->data;
    else
      speed=0;
    uint16_t pulse_width_in_us=70*speed+1715;//77//5*speed+1900;//msg->pulse_width_in_microseconds;

    // Display the message received
    // RCLCPP_INFO(this->get_logger(), "Message received for motor with channel = %d, and pulse width [us] = %d",
    //             channel, pulse_width_in_us);
    m_pca9685_servo_driver->set_pwm_pulse_in_microseconds(channel, 2000);
    m_pca9685_servo_driver->set_pwm_pulse_in_microseconds(channel, 1500);
    // Limit the pulse width to be either:
    // > zero
    // > in the range [1000,2000]
    if (pulse_width_in_us > 0)
    {
      if (pulse_width_in_us < 1500)
        pulse_width_in_us = 1500;
      if (pulse_width_in_us > 2000)
        pulse_width_in_us = 2000;
    }

    // Call the function to set the desired pulse width
    bool result = m_pca9685_servo_driver->set_pwm_pulse_in_microseconds(channel, pulse_width_in_us);

    // Display if an error occurred
    if (!result)
    {
      RCLCPP_ERROR(this->get_logger(), "FAILED to set pulse width for motor at channel %d", channel);
    }

    /// publish
    auto state_msg = State();
    state_msg.header.stamp = now();
    state_msg.speed = speed;
    m_current_publisher->publish(state_msg);


  }

  // Callback for timer
  void timerCallbackForPublishing()
  {
    // Read the current measurement here (this would be specific to your use case)

    // Publish a message
    // auto msg = State();
    // msg.header.stamp = now();
    // msg.speed = ;
    // m_current_publisher->publish(msg);
  }

  // Member variables
  std::string m_i2c_device_name;
  uint8_t m_pca9685_address;
  uint8_t motor_channel;
  uint8_t servo_channel;
  std::unique_ptr<I2C_Driver> m_i2c_driver;
  std::unique_ptr<PCA9685> m_pca9685_servo_driver;

  rclcpp::Publisher<State>::SharedPtr m_current_publisher;
  rclcpp::Publisher<Float64>::SharedPtr m_steer_publisher;
  rclcpp::Subscription<Float64>::SharedPtr m_servo_subscriber;
  rclcpp::Subscription<Float64>::SharedPtr m_motor_subscriber;
  rclcpp::TimerBase::SharedPtr m_timer_for_publishing;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TemplateI2CNode>());
  rclcpp::shutdown();
  return 0;
}
