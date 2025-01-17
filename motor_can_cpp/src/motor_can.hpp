#ifndef MOTOR_CAN_HPP
#define MOTOR_CAN_HPP

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
// #include <termios.h>
#include <asm/termbits.h>
#endif


// #include "Dji_M3508.hpp"
#include "can_tools.h"
#include "PeriodicTask.hpp"
#include "CircularQueue.hpp"
#include "Msg.hpp"
#include <shared_mutex>
#include <mutex>
#include <iomanip>

int s;
bool stopFlag=0;
std::string canSeries, passwd;

class copy_solver
{
    public:
        uint8_t* target_adrr = nullptr;
        uint8_t copy_num;
        //收包检测助手
        uint32_t m_update_rateS = 0;  
        uint32_t m_update_rate = 0;

        copy_solver(uint8_t* _target_adrr, uint8_t _copy_num)
        {
            target_adrr = _target_adrr;
            copy_num = _copy_num;
        }
};

class MotorCan: public PeriodicTask{
public:
    MotorCan(std::string _name, uint8_t _prefer_cpu, float _update_rate, int _baudrate, const char *_pathname, uint32_t _send_buffer_size, uint32_t _recieve_buffer_size, uint8_t _every_time_send_max):
    PeriodicTask(_update_rate,_name,_prefer_cpu,sched_get_priority_max(SCHED_FIFO),SCHED_FIFO)
        ,m_recieve_buffer(_recieve_buffer_size,"CAN_recieve")
        ,m_send_buffer(_send_buffer_size,"CAN_send")
        ,baudrate(_baudrate)
        ,canname(_pathname)
        ,every_time_send_max(_every_time_send_max){}

        //收包助手
        uint32_t data_num_pers = 0;
        uint32_t data_num = 0;

        uint32_t m_packet_send_allS = 0;  
        uint32_t m_packet_send_all = 0;
    //     ~MotorCan() {
    //     close(s_);
    // }

    void send_can_frame(const struct can_frame& frame) {
        if (write(s_, &frame, sizeof(struct can_frame)) == -1) {
            std::cerr << "send_can_error" << std::endl;
        }
    }

    void receive_can_frame(struct can_frame& frame) {
        if (read(s_, &frame, sizeof(struct can_frame)) < 0) {
            perror("receive_can_Error");
        }
    }

    void Others_enquene_send(Msg _msg)
    {
        //加写锁
        std::unique_lock<std::shared_mutex> send_lock(send_quene_mutex);
        m_send_buffer.enqueue(_msg);
    }

    //继承自PeriodTask的函数
    void init() override;
    void run() override;
    void cleanup() override;

    //通用接口
    // void com_init();
    int can_write(const char *w_buf,size_t len);
    // ssize_t safe_write(const char *vptr, size_t n);

    void read_line(int fd);

    // int uart_set(int fd, int speed);
    void can_open(const char *padthname);

    // bool Change_uart_priority();

    //ssize_t转化为uint32_t
    uint32_t ssize_t_to_uint32_t(ssize_t ssize_value) 
    {
        if (ssize_value < 0) {
            // 如果 ssize_t 值为负数，这里可以选择返回一个默认值或者抛出异常
            std::cerr << "Error: negative ssize_t value cannot be converted to uint32_t." << std::endl;
            return 0; // 返回一个默认值，表示出错或无效情况
        }

        // 确保 ssize_t 值在 uint32_t 能表示的范围内
        if (ssize_value > std::numeric_limits<uint32_t>::max()) {
            // 处理超出范围的情况，这里可以选择返回一个默认值或者抛出异常
            std::cerr << "Error: ssize_t value exceeds the maximum value of uint32_t." << std::endl;
            return std::numeric_limits<uint32_t>::max(); // 返回 uint32_t 的最大值
        }

        return static_cast<uint32_t>(ssize_value);
    }

protected:
    CircularQueue<uint8_t> m_recieve_buffer;
    CircularQueue<Msg> m_send_buffer;

    std::shared_mutex recieve_quene_mutex;
    std::shared_mutex send_quene_mutex;

private:
    int socketCANInit(const std::string& can_interface) {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return -1;
    }

    strcpy(ifr.ifr_name, can_interface.c_str());
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -2;
    }

    return s;
    }
    int fd;
    const char* canname;
    int baudrate;
    uint8_t every_time_send_max;

    std::string can_interface_;
    int s_;
    bool stopFlag = false;
    int is_cmd_active = 0;
    struct can_frame txFrame, rxFrame;
    // Motor_M3508 motor[4];
};

#endif // MOTOR_CAN_HPP