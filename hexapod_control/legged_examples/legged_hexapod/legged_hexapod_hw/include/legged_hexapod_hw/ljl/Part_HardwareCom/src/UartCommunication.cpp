#include "UartCommunication.hpp"
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
#include <string.h>

#include <thread>
#include <atomic>



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
    // std::cout << std::endl;
}

int64_t time_sample_Uartcommunication = 0; 

void UartCom::run()
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
            // 提取canid, can数据和校验
            uint8_t canid = Send_msg.msg_content[13];
            std::vector<uint8_t> can_data(Send_msg.msg_content.begin() + 15+6, Send_msg.msg_content.begin() + 22+7);
            uint8_t checksum = Send_msg.msg_content[22+7];
            
            // 打印提取出来的数据
            // std::cout << "CAN ID: 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(canid) << std::endl;
            // std::cout << "CAN Data: ";
            // for (const auto& byte : can_data) {
            //     std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            // }
            // std::cout << std::endl;
            // std::cout << "Checksum: 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(checksum) << std::endl;

            // 将canid，can_data，checksum发送到can
            struct can_frame frame;
            frame.can_id = canid;
            frame.can_dlc = can_data.size();
            std::copy(can_data.begin(), can_data.end(), frame.data);
            // frame.data[can_data.size()] = checksum;
            if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
                if(errno == ENOBUFS) {
                    // 缓冲区满,等待一段时间
                    usleep(1000);
                    continue;
                }
                perror("Write");
            }
            // else
            // {
            //     fprintf(stderr,"can send success\n");
            // }
            struct can_frame read_frame;
            // int n;
            // n  = read(s, &read_frame, sizeof(read_frame));
            
            // if (ret < 0)
            // {
            //     if (errno == EAGAIN || errno == EWOULDBLOCK)
            //     {
            //         // 没有可读数据，继续发送
            //     }
            //     else if (errno == ENOBUFS)
            //     {
            //         usleep(1000);
            //         continue;
            //     }
            //     else
            //     {
            //         perror("Read");
            //     }
            // }
            // else if (ret == sizeof(read_frame))
            // {
            //     // 正常读到一帧
            //     // 可在此处添加读取后处理逻辑
            // }

            ++m_packet_send_all;

        }
    }
    // read_line(s);
}

void UartCom::init()
{
    Change_uart_priority();
    if(!com_init()) {
        fprintf(stderr, "com_init failed.\n");
        return; 
    }

    // 设置非阻塞
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);

    stopCanRead.store(false);
    canReadThread = std::thread(&UartCom::readCANLoop, this);
}

void UartCom::cleanup()
{
    stopCanRead.store(true);
    if (canReadThread.joinable()) {
        canReadThread.join();
    }

    close(s);
}

bool UartCom::Change_uart_priority()
{
    // std::string command = "sudo chmod 777 " + std::string(uartname);
    std::string command = "ip link set " + std::string(uartname) + " up type can bitrate 1000000";

    // 执行系统命令
    int result = std::system(command.c_str());

    // if (result != 0) {
    //     std::cerr << "Failed to execute command: " << command << std::endl;
    //     return true;
    // }

    // std::cout << "Command executed successfully: " << command << std::endl;
    return false;
}

bool UartCom::com_init()
{
    if (s < 0) {
        perror("Socket");
        return false;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, uartname);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("IOCTL");
        close(s);
        return false;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        close(s);
        return false;
    }
    return true;
    
}

void UartCom::read_line(int fd)
{
    uint8_t *buffer = nullptr;
    size_t buf_size = 0;
    ssize_t total_read = 0;
    ssize_t n;

    do {
        // 扩展缓冲区大小
        if (total_read >= buf_size) {
            buf_size += 512; // 每次增加4096字节
            buffer = (uint8_t *)realloc(buffer, buf_size);
            if (!buffer) {
                fprintf(stderr,"Memory reallocation failed.\n");
            }
        }

        // 从串口读取数据
        struct can_frame read_frame;
        n = read(s, &read_frame, sizeof(read_frame));
        total_read += n;
    } while (n > 0);
    
    if(total_read < 0)
    {
        return;
    }
    
    uint32_t total_read_num = ssize_t_to_uint32_t(total_read);
    // 把数据压入循环队列
    if(total_read_num > 0)
    {
        m_recieve_buffer.enqueue(buffer,total_read_num);
        data_num+=total_read_num;
    }

    // 使用完毕后释放内存
    free(buffer);
    buffer = nullptr;
}
 
ssize_t UartCom::safe_write(int fd, const char *vptr, size_t n)
{
    size_t  nleft;
    ssize_t nwritten;
    const char *ptr;
 
    ptr = vptr;
    nleft = n;
 
    while(nleft > 0)
    {
    if((nwritten = write(fd, ptr, nleft)) <= 0)
        {
            if(nwritten < 0&&errno == EINTR)
                nwritten = 0;
            else
                return -1;
        }
        nleft -= nwritten;
        ptr   += nwritten;
    }
    return(n);
}

int UartCom::uart_write(int fd,const char *w_buf,size_t len)
{
    ssize_t cnt = 0;
    cnt = safe_write(fd,w_buf,len);
    if(cnt == -1)
    {
        // fprintf(stderr,"write error!\n");
        return -1;
    }
    return cnt;
}

int UartCom::uart_open(int fd,const char *pathname){
    assert(pathname);
    /*打开串口*/
    // fd = open(pathname,O_RDWR|O_NOCTTY|O_NDELAY);
    // if(fd == -1)
    // {
    //     perror("Open UART failed!");
    //     return -1;
    // }
    return fd;
}

//设置自定义波特率接口
int UartCom::uart_set(int fd, int speed) {

  return 0;
}

// 新增：循环读取线程
void UartCom::readCANLoop()
{
    while (!stopCanRead.load())
    {
        struct can_frame rframe;
        int ret = read(s, &rframe, sizeof(rframe));        

        if (ret > 0)
        {
            // 处理读到的一帧
            
            Msg recv_msg;
            recv_msg.msg_content.resize(16);
            recv_msg.msg_content[0] = 0xAA;
            recv_msg.msg_content[1] = 0x11;
            recv_msg.msg_content[2] = 0x08;
            recv_msg.msg_content[3] = rframe.can_id;
            recv_msg.msg_content[4] = 0x00;
            recv_msg.msg_content[5] = 0x00;
            recv_msg.msg_content[6] = 0x00;
            for (int i = 0; i < rframe.can_dlc; ++i) {
                recv_msg.msg_content[7 + i] = rframe.data[i];
            }
            recv_msg.msg_content[15] = 0x55;
            
            // std::cout << std::endl;
            m_recieve_buffer.enqueue(recv_msg.msg_content.data(), recv_msg.msg_content.size());
            data_num+=1;
        }
        else if (ret < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // 暂无可读数据
            }
            else
            {
                perror("read");
            }
        }
        // 避免空转
        usleep(1000);
    }
}

