#include "../../include/Infantry-can/CanSerial.h"
//接受数据的线程中接受一次需要休眠几毫秒 ，需要设置接受的频率  加入反小陀螺模块； 
int CanSerial::send_state(const ArmorState state, const int target_id)
{
//前后两次发射的时间间隔需大于等于4ms;
    double time1 =std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-last_send1).count();
    if (time1<3){
        //std::cout<<"状态位发送超时"<<std::endl;
        return TIME_ERROR;
    }
    last_send1=std::chrono::steady_clock::now();
    u_char buf[2];
    switch (state)
    {
        case ArmorState::NO_FIND_NO_SHOOT:
            buf[0] = 0;
            break;
        
        default:
            buf[0] = 1;
            break;
    }

    // 暂时忽略 ID
    buf[1] = target_id;

    return can_send(STATE_SEND_ID, buf, 2);
}

int CanSerial::send_Data(float pitch,float yaw, u_int32_t time)
{
//前后两次接受的时间间隔需大于等于700us;
    double time2=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-last_send2).count();
    if(time2<200){
        //std::cout<<"角度发送超时"<<std::endl;
        return TIME_ERROR;
    }
    last_send2=std::chrono::steady_clock::now();
    transformData(pitch,yaw);
    u_char buf[8];
    for (int i = 0; i < 4; i++)
        buf[i] = PY_data[i];
    // for (int i = 4; i < 8; i++)
    // {
    //     buf[i] |= time;
    //     time >>= 8;
    // }
    memcpy(&buf[4], &time, 4);
    return can_send(DATA_SEND_ID, buf, 8);
}

void CanSerial::transformData(float pitch_,float yaw_)
{
    int16_t pitch = (int16_t)(pitch_ * 32768.0 / 180.0);
    int16_t yaw = (int16_t)(yaw_ * 32768.0 / 180.0);

    for (int i = 0; i < 4; i++) PY_data[i] = 0;

    PY_data[0] |= pitch;
    PY_data[1] |= (pitch >> 8);
    PY_data[2] |= yaw;
    PY_data[3] |= (yaw >> 8);
}

void CanSerial::transformIMU(u_char *IMU_data)
{
    for (int i=0;i<4;i++){
        float temp=static_cast<int16_t>((IMU_data[2*i]<<8)|IMU_data[2*i+1])/100.0;
        receive_data.q[i]=temp;
    }
    int32_t t=0;
    for (int i = 0; i < 4; i++)
    {
        t |=  ((u_int32_t)IMU_data[8 + i] << (i * 8));
    }
    receive_data.time=t;   
}

int CanSerial::can_send(uint id, u_char *buf, u_char dlc)
{
    if (dlc > 8)    return DLC_ERROR;

    struct can_frame send_frame;

    send_frame.can_id = id;
    send_frame.can_dlc = dlc;

    for (int i = 0; i < (int)dlc; i++)
        send_frame.data[i] = buf[i];
    
    int t = write(socket_fd, &send_frame, sizeof(send_frame));
    if (t > 0)  return SUCCESS;
    return WRITE_ERROR;
}

int CanSerial::can_receive(uint &id, u_char *buf, u_char &dlc)//这儿只有一个函数，如何接受多个id发送的数据；
{
    struct can_frame frame;
    int t = read(socket_fd, &frame, sizeof(frame));
    if (t <= 0) {
        //std::cout<<"receive failed"<<std::endl;
        return READ_ERROR;
    }
    id = frame.can_id;
    dlc = frame.can_dlc;
    //buf =(u_char*)malloc(dlc*sizeof(u_char));//为什么不能这样把buf定义成动态数组然后分配空间；
    memcpy(buf, frame.data, dlc);
    //transformIMU(buf);
    //std::cout<<"receive successfully"<<std::endl;
    return SUCCESS;
}

CanSerial::CanSerial()
{
    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    // 配置 Socket CAN 为非阻塞IO
    int flags=fcntl(socket_fd, F_GETFL, 0); 
    flags |= O_NONBLOCK;
    fcntl(socket_fd, F_SETFL, flags);

    // 指定can设备
    strcpy(interface_request.ifr_name, CAN_NAME);
    ioctl(socket_fd, SIOCGIFINDEX, &interface_request);
    addr.can_family = AF_CAN;
    addr.can_ifindex = interface_request.ifr_ifindex;
    
    // 将套接字与can设备绑定
    bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr));
}