#ifndef IMAGEPROCESS_H_
#define IMAGEPROCESS_H_
#include"../autoaim/autoaim.hpp"
#include"../mercure/mercure_driver.h"
#include<mutex>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<queue>
#include<chrono>
using namespace std;
using namespace cv;
using namespace Eigen;
#define LIMIT_TIME (10)//和同步时间戳相关的参数；
//#define ANGLE_CONFIG
//#define SEARIAL_DEBUG 0
//#define WRITE_VIDEO
//#define SEND_DEBUG 0
//#define COMPUTE_PROCESS_TIME
/*
问题：std::chrono::_V2::steady_clock::time_point的使用
考虑到目标坐标与姿态信息时间上并不同步，需根据其时间戳确定两者时间偏移关系
时间延迟:图像采集，目标识别和结算；
需根据时间偏移关系从历史姿态信息中找出与目标坐标时间对应的四元数
*/
class ImageProcesspredict{
public:
    mutex gyro_mutex;

    cv::Point2f send_pitch_yaw;
private:
    //使用队列来存储需要处理的图像的结构体，是多个线程共享的资源，需要使用线程锁
    queue<Mat_t> image_queue;

    //队列中最大元素的个数的限制；
    const int BUFFER_LENGTH;
    short int ANGLE_CONFIG;
    short int SERIAL_DEBUG;
    short int WRITE_VIDEO;
    short int SEND_DEBUG;
    short int COMPUTE_PROCESS_TIME;
    short int predict_DEBUG;
    mutex image_lock;
    mutex frame_mutex;
    mutex send_mutex;
    //使用volatile修饰表示该变量为多个进程所共享，
    volatile int accepted_number;//表示当前接受的帧数
    volatile int processed_number;//表示当前已经处理的帧数；

    //自瞄器
    Autoaim aimer;

    //电控和视觉共同的时间戳；
    double start_time=getTickCount();

    //云台的四元数的历史数据；
    vector<GimblaPose> history_quaternion;

    //是否需要重新对齐时间戳
    int need_reset_timestamp=0;
    
    uint32_t pose_timestamp;

    GimblaPose newest_pose;
    
    uint32_t send_time;

    //vector<GimblaPose> gyro_buffer;

    Mat frame;

    cv::VideoWriter writer;

public:

    /**
     * @brief 使用相机驱动获取图像；
     * 获取的图像存储到image_queue中；
     */
    void get_image();

    void get_eneny_color();
    /**
     * @brief 获取队首的图像->做装甲板的识别->pnp结算->预测；
     * 
     */
    void process_image();

    /**
     * @brief 获取录制视频的名称；
     * 
     * @return String 
     */
    String getVideoName();

    void sendData();

    void receiveData();

    void receiveBS();

    void write_video();
    
    //void receiveIMU();
    
    /**
     * @brief 这里必须要使用插值函数，使用一个容器来存储电控发送的四元数
     * 因为目标坐标和位姿信息时间上不同步，需要根据时间戳来确定两者时间偏移关系，根据时间偏移关系从历史姿态信息中找出和目标坐标时间对应的四元数；
     * 
     * @param src 
     * @return true 
     * @return false 
     */
    bool interpolate(Mat_t &src);

    ImageProcesspredict(int accepted_num,int processed_num,int color,CanSerial &serial);
    
};

#endif 