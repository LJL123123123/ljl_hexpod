#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#include <iostream>
#include <stdexcept>
#include <unordered_map>
#include <array>
#include <linux/can.h>
#include <memory>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>

namespace damiao
{

typedef int id_t;

typedef struct 
{
  float Q_MIN = -12.5;
  float Q_MAX = 12.5;
  float DQ_MAX = 30;
  float TAU_MAX = 10;

  struct {
    float kp;
    float kd;
    float q;
    float dq;
    float tau;
  } cmd;

  struct {
    float q;     //quadrant
    float dq;    //vel rad/s
    float tau;   //torque
  } state;

} MotorParam; //电机参数

class Motor_can
{
public:
  Motor_can() {}

  ~Motor_can()
  {
    for (auto& socket : can_sockets_) {
      close(socket.second);
    }
  }

void configureCANInterface(const std::string& interface, int bitrate) {
    // 加载 CAN 模块
    if (system("sudo modprobe can") != 0 ||
        system("sudo modprobe can_raw") != 0 ||
        system("sudo modprobe can_dev") != 0) {
        throw std::runtime_error("Failed to load CAN modules");
    }

    // 设置 CAN 接口的波特率
    std::string command = "sudo ip link set " + interface + " type can bitrate " + std::to_string(bitrate);
    if (std::system(command.c_str()) != 0) {
        std::cerr << "Warning: Failed to set CAN bitrate. Interface might already be up." << std::endl;
    }

    // 启用 CAN 接口
    command = "sudo ip link set " + interface + " up";
    if (std::system(command.c_str()) != 0) {
        throw std::runtime_error("Failed to set CAN interface up");
    }
}

void setSocketTimeout(int can_socket, int timeout_ms) {
    struct timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    if (setsockopt(can_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        throw std::runtime_error("Error setting socket receive timeout");
    }
}

void addMotor(id_t MOTOR_ID, id_t MASTER_ID, const std::string& can_interface = "can0", int bitrate = 1000000)
{
    if (can_sockets_.find(can_interface) == can_sockets_.end()) {
      configureCANInterface(can_interface, bitrate);

      int can_socket;
      // 初始化 CAN 套接字
      if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        throw std::runtime_error("Error while opening socket");
      }

      // 设置接收超时时间
      setSocketTimeout(can_socket, 5); // 设置超时时间为 5 毫秒

      struct ifreq ifr;
      std::strcpy(ifr.ifr_name, can_interface.c_str());
      if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
        throw std::runtime_error("Error while getting interface index");
      }
      
      struct sockaddr_can addr;
      std::memset(&addr, 0, sizeof(addr));
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;

      if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        throw std::runtime_error("Error in socket bind");
      }

      can_sockets_[can_interface] = can_socket;
    }

    motors[MOTOR_ID] = std::make_shared<MotorParam>();
    motors[MASTER_ID] = motors[MOTOR_ID];
    motor_interfaces_[MOTOR_ID] = can_interface;
}

  void enable(id_t id) { control_cmd(id, 0xFC); }
  void reset(id_t id) { control_cmd(id, 0xFD); }
  void zero_position(id_t id) { control_cmd(id, 0xFE); }

void control(id_t id, float kp, float kd, float q, float dq, float tau)
{
    // 位置、速度和扭矩采用线性映射的关系将浮点型数据转换成有符号的定点数据
    static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
        float span = xmax - xmin;
        float data_norm = (x - xmin) / span;
        return static_cast<uint16_t>(data_norm * ((1 << bits) - 1));
    };

    if (motors.find(id) == motors.end()) {
        throw std::runtime_error("Motor id not found");
    }

    auto& m = motors[id];

    m->cmd = {kp, kd, q, dq, tau}; // 保存控制命令

    uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
    uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
    uint16_t q_uint = float_to_uint(q, m->Q_MIN, m->Q_MAX, 16);
    uint16_t dq_uint = float_to_uint(dq, -m->DQ_MAX, m->DQ_MAX, 12);
    uint16_t tau_uint = float_to_uint(tau, -m->TAU_MAX, m->TAU_MAX, 12);

    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = (q_uint >> 8) & 0xff;
    frame.data[1] = q_uint & 0xff;
    frame.data[2] = dq_uint >> 4;
    frame.data[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
    frame.data[4] = kp_uint & 0xff;
    frame.data[5] = kd_uint >> 4;
    frame.data[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
    frame.data[7] = tau_uint & 0xff;

    int can_socket = can_sockets_[motor_interfaces_[id]];

    // std::cout << "Sending CAN frame to motor " << id << " on interface " << motor_interfaces_[id] << "..." << std::endl;
    if (write(can_socket, &frame, sizeof(frame)) != sizeof(frame)) {
        std::cerr << "Error sending CAN frame: " << strerror(errno) << std::endl;
        throw std::runtime_error("Error sending CAN frame");
    }
    // std::cout << "CAN frame sent to motor " << id << "." << std::endl;

    recv(can_socket);
}

void recv(int can_socket)
{
    struct can_frame frame;
    int nbytes = read(can_socket, &frame, sizeof(frame));
    if (nbytes < 0) {
        throw std::runtime_error("Error receiving CAN frame");
    }

    if (motors.find(frame.can_id) == motors.end()) {
        std::cout << "Unknown motor id: " << std::hex << frame.can_id << std::endl;
        return;
    }

    auto &m = motors[frame.can_id];
    static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
        float span = xmax - xmin;
        float data_norm = float(x) / ((1 << bits) - 1);
        float data = data_norm * span + xmin;
        return data;
    };

    uint16_t q_uint = (uint16_t(frame.data[1]) << 8) | frame.data[2];
    uint16_t dq_uint = (uint16_t(frame.data[3]) << 4) | (frame.data[4] >> 4);
    uint16_t tau_uint = (uint16_t(frame.data[4] & 0xf) << 8) | frame.data[5];

    m->state.q = uint_to_float(q_uint, m->Q_MIN, m->Q_MAX, 16);
    m->state.dq = uint_to_float(dq_uint, -m->DQ_MAX, m->DQ_MAX, 12);
    m->state.tau = uint_to_float(tau_uint, -m->TAU_MAX, m->TAU_MAX, 12);

    // std::cout << "Received CAN frame from motor " << frame.can_id << " with data: ";
    // for (int i = 0; i < frame.can_dlc; ++i) {
    //     std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
    // }
    // std::cout << std::endl;
}
  std::unordered_map<id_t, std::shared_ptr<MotorParam>> motors;

private:
  void control_cmd(id_t id , uint8_t cmd)
  {
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8; // 数据长度为 8 字节
    std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
    std::copy(data_buf.begin(), data_buf.end(), frame.data);

    int can_socket = can_sockets_[motor_interfaces_[id]];
    // std::cout << "Sending CAN frame to motor " << id << " on interface " << motor_interfaces_[id] << "..." << std::endl;
    if (write(can_socket, &frame, sizeof(frame)) != sizeof(frame)) {
      std::cerr << "Error sending CAN frame: " << strerror(errno) << std::endl;
      throw std::runtime_error("Error sending CAN frame");
    }
    // std::cout << "CAN frame sent to motor " << id << "." << std::endl;

    usleep(1000);
    recv(can_socket);
  }

  std::unordered_map<std::string, int> can_sockets_;
  std::unordered_map<id_t, std::string> motor_interfaces_;
};

}; // namespace damiao

#endif // MOTOR_CAN_H