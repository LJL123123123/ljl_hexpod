#include "motor_can.hpp"
#include "Timer.h"

#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>
#include <cstring>
#include <string.h>

template <>
void CircularQueue<uint8_t>::printcontent() const {
    if (isEmpty()) {
        fprintf(stderr,"uint8_t quene Empty\n");
        return;
    }
    size_t index = front;
    while (index != rear) {
        std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(arr[index]) << " ";
        index = (index + 1) % capacity;
    }
    std::cout << std::endl;
}

int64_t time_sample_Uartcommunication = 0; 

void MotorCan::run()
{
    if (Gloabl_Timer::hasElapsedMs(1000, time_sample_Uartcommunication))
    {
        data_num_pers = data_num;
        data_num = 0;

        m_packet_send_allS = m_packet_send_all;
        m_packet_send_all = 0;
        // std::cout << std::dec << "send_rate " << m_packet_send_allS<< std::endl;
    }

    if (!m_send_buffer.isEmpty())
    {
        //加写锁
        std::unique_lock<std::shared_mutex> send_lock(send_quene_mutex);
        // 处理队列中的元素
        int framesToSend = std::min((int)every_time_send_max, m_send_buffer.size());
        for (int i = 0; i < framesToSend; i++)
        {
            Msg Send_msg = m_send_buffer.dequeue();  // 从队列中取出消息
            //Send_msg.printHex();
            const char* char_data = reinterpret_cast<const char*>(Send_msg.msg_content.data());  // 获取数据
            if(can_write(char_data, Send_msg.msg_content.size()) > 0)
            {
                ++m_packet_send_all;
            }  // 写入数据
        }
    }
    read_line(s_);
}

void MotorCan::init()
{
    can_open(canname);
}

void MotorCan::cleanup()
{
    close(s_);
}

// bool MotorCan::Change_uart_priority()
// {
//     std::string command = "sudo chmod 777 " + std::string(uartname);

//     // 执行系统命令
//     int result = std::system(command.c_str());

//     if (result != 0) {
//         std::cerr << "Failed to execute command: " << command << std::endl;
//         return true;
//     }

//     std::cout << "Command executed successfully: " << command << std::endl;
//     return false;
// }

// void MotorCan::com_init()
// {
//     fd = uart_open(fd,uartname);
//     if(fd == -1)
//     {
//         fprintf(stderr,"uart_open error\n");
//         exit(EXIT_FAILURE);
//     }
//     if(uart_set(fd,baudrate) == -1)
//     {
//         fprintf(stderr,"uart set failed!\n");
//         exit(EXIT_FAILURE);
//     }
// }

void MotorCan::read_line(int s_)
{
    // uint8_t *buffer = nullptr;
    size_t buf_size = 0;
    ssize_t total_read = 0;
    struct can_frame frame;
    ssize_t n;

    // 从 CAN 接口读取数据
    n = read(s_, &frame, sizeof(struct can_frame));
    if (n < 0) {
        perror("Read Error");
        return;
    }

    // 将 CAN 帧的数据部分复制到缓冲区
    uint8_t *buffer = frame.data;
    size_t total_read_num = frame.can_dlc;

    // 把数据压入循环队列
    if (total_read_num > 0)
    {
        m_recieve_buffer.enqueue(buffer, total_read_num);
        data_num += total_read_num;
    }

    // 使用完毕后释放内存
    free(buffer);
    buffer = nullptr;
}
 
// ssize_t MotorCan::safe_write(const char *vptr, size_t n)
// {
//     send_can_frame(const struct can_frame& frame)
//     return(n);
// }

int convert_to_can_frame(const char *w_buf, size_t len, struct can_frame &frame) {
     if (len < 13) {
        std::cerr << "Invalid data length" << std::endl;
        return -1;
    }

    // 提取 CAN ID
    canid_t can_id;
    std::memcpy(frame.data, &w_buf[5], 8);
    // std::memcpy(&can_id, w_buf, sizeof(canid_t));
    // frame.can_id = can_id;
    frame.can_id = (w_buf[0] << 24) | (w_buf[1] << 16) | (w_buf[2] << 8) | w_buf[3];
    frame.can_dlc = w_buf[4];

    // 提取数据长度
    // size_t data_len = len - sizeof(canid_t);
    // if (data_len > CAN_MAX_DLEN) {
    //     std::cerr << "Data length: " << data_len << std::endl;
    //     std::cerr << "Buffer data: ";
    //     for (size_t i = 0; i < len; ++i) {
    //         std::cerr << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(w_buf[i]) << " ";
    //     }
    //     std::cerr << std::endl;
    //     std::cerr << "Data length exceeds maximum CAN frame length" << std::endl;
    //     return -1;
    // }

    // frame.can_dlc = data_len;
    // std::memcpy(frame.data, w_buf + sizeof(canid_t), data_len);

    return 0;
}

int MotorCan::can_write(const char *w_buf,size_t len)
{
    can_frame send_frame;
    convert_to_can_frame(w_buf, len, send_frame);
    send_can_frame(send_frame);
    return len;
}

void MotorCan::can_open(const char *pathname){
    canSetup(pathname, "1");
}