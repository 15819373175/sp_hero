#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  // 发送控制指令
  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

private:
  // ==========================================
  // BCP 方案 B 协议常量
  // ==========================================
  static constexpr uint8_t HEAD_BYTE = 0xFF;
  static constexpr uint8_t DESTINATION_ADDRESS = 0x02;
  static constexpr uint8_t ID_GIMBAL = 0x20;
  static constexpr uint8_t GIMBAL_PAYLOAD_LENGTH = 25;

  serial::Serial serial_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  std::atomic<bool> serial_connected_{false};
  int consecutive_tx_failures_ = 0;
  mutable std::mutex mutex_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  
  // 用于插值的四元数队列
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  // ==========================================
  // 接收状态机相关
  // ==========================================
  enum RxState {
    WAIT_HEAD,
    WAIT_D_ADDR,
    WAIT_ID,
    WAIT_LEN,
    WAIT_DATA,
    WAIT_SUM_CHECK,
    WAIT_ADD_CHECK
  };

  void read_thread();    // 接收线程函数
  void reconnect();      // 重连逻辑
  void process_packet(uint8_t id, const std::vector<uint8_t>& data); // 解包处理
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
