#ifndef CAN_SERIAL_H
#define CAN_SERIAL_H
#include<opencv4/opencv2/opencv.hpp>
#include<chrono>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include "./const.h"
using namespace cv;
#define SEARIAL_DELAY 0.002//can通信的时间延迟需要改；
#define CAN_NAME "can0"
#define STATE_SEND_ID 0x106
#define DATA_SEND_ID 0x108
#define RECEIVE_TIME 0x100
#define IMU_RECEIVE_ID_0 0x102
#define IMU_RECEIVE_ID_1 0x103
#define IMU_RECEIVE_ID_2 0x0FF
#define MODE_RECEIVE_ID 0x110
#define BS_RECEIVE_ID 0x125
#define ROBOT_RECEIVE_ID 0x129

//发送的数据包括：0x104(1.状态 2.id) 0x106(3.pitch 4.yaw 5.time);
/*
接受的数据的结构体：0x100:接受前八位  0x101:接受后四位；
* 0：  w的高位
 * 1:   w的低位
 * 2：  x的高位
 * 3：  x的低位
 * 4：  y的高位
 * 5：  y的低位
 * 6：  z的高位
 * 7：  z的低位
 * 8：  时间的7-0位
 * 9：  时间的15-8位
 * 10： 时间的23-16
 * 11:  时间的31-24位
*/
struct SendData{
    ArmorState state;
    int id;
    float pitch;
    float yaw;
    int32_t time;
    SendData(){
        state=NO_FIND_NO_SHOOT;
        id=0;
        pitch=0.0;
        yaw=0.0;
        time=0;
    }
}__attribute__((packed));

struct ReceiveData{
    float q[4];
    int32_t time;
    ReceiveData(){
        for (int i=0;i<4;i++){
            q[i]=0.0;
        }
        time=0;
    }
}__attribute__((packed));

enum CanError{
    SUCCESS,
    DLC_ERROR,
    WRITE_ERROR,
    READ_ERROR,
    TIME_ERROR    
};
class CanSerial
{

private:
    int socket_fd;

    struct sockaddr_can addr;
    struct ifreq interface_request;

    SendData send_data;                 // 发送的数据
    ReceiveData receive_data;           // 接受的数据

    u_char PY_data[4];  // pitch 和 yaw
public:

    //double startTime;

    std::chrono::steady_clock::time_point last_send1=std::chrono::steady_clock::now();
    
    std::chrono::steady_clock::time_point last_send2=std::chrono::steady_clock::now();

/**
 *  @brief  自瞄状态与目标发送接口API
 *  @param  state   识别状态
 *  @param  target_id   目标ID
 */
    int send_state(const ArmorState state, const int target_id);

/**
 *  @brief  自瞄数据发送接口API
 *  @param  pitch_yaw   pitch: 0     yaw: 1
 *  @param  time    所使用的IMU数据的时间？？？？
 *  发送当前的相对角度和所使用的IMU数据的时间，这里使用的IMU数据的时间是预测时计算姿态使用的角度数据对应的时间
 */
    int send_Data(float pitch,float yaw, u_int32_t time);

/**
 *  @brief  IMU数据接收API
 *  @param  imu_data    
 *              0: pitch
 *              1: pitch角速度
 *              2: yaw
 *              3: yaw角速度
 *              4: roll
 *              5: roll角速度
 *  @param  time    IMU时间，单位ms
 * 将接受的协议改成下面的协议：四元数[w,x,y,z]
 * 0：  w的高位
 * 1:   w的低位
 * 2：  x的高位
 * 3：  x的低位
 * 4：  y的高位
 * 5：  y的低位
 * 6：  z的高位
 * 7：  z的低位
 * 8：  时间的7-0位
 * 9：  时间的15-8位
 * 10： 时间的23-16
 * 11:  时间的31-24位
 */
    int  receive_IMU(float* imu_data, u_int32_t &time);

/**
 *  @brief  弹速上限接收API
 *  @param  bullet_speed    弹速上限
 */
    int receive_BS(float &bullet_speed);

/**
 *  @brief  机器人信息接收API
 *  @param  color   己方颜色
 */
    int receive_Robo(int &color);

/**
 *  @brief  自瞄模式切换命令接收API
 *  @param  mode    自瞄模式
 */
    int receive_mode(int &mode);

/**
 *  @brief  接收数据，根据id区分数据包，需要大于1000Hz频率接收
 *  @return error_code
 */
    int can_receive(uint &id, u_char *buf, u_char &dlc);
/**
 *  @brief  转化接收的IMU数据
 */
    void transformIMU(u_char *IMU_data);

    CanSerial();
    ~CanSerial() = default;

private:
/**
 *  @brief  将角度数据转换为uint8以发送
 */
    void transformData(float pitch_,float yaw_);

/**
 * @brief   发送数据
 * @param   id  数据对应的ID
 * @param   buf 数据，长度小于等于8
 * @param   dlc 数据长度
 * @return  error_code
*/
    int can_send(uint id, u_char *buf, u_char dlc);

};

#endif