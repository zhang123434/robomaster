#ifndef AUTOAIM_H_
#define AUTOAIM_H_
#include<opencv2/opencv.hpp>
#include"../detector/detector.hpp"
#include"../pnp/pnp.hpp"
//#include"../serial/serial.h"
#include"../gravity_compensate/compensate.hpp"
#include"../Infantry-can/CanSerial.h"
#include"../Infantry-can/const.h"
#include"../Antitop/SpinDetector.h"
#include<Eigen/Dense>
//#define CONFIG
//#define DEBUG_ANTITOP
//#define DETECT_CONFIG
//#define SHOW_ROI
//将图像和识别到的数据封装成一个结构体：
struct GimblaPose{
    double q[4]={0};//按照w,x,y,z的顺序来存储；
    double timestamp=0; 
};
struct Mat_t{
    cv::Mat image;
    GimblaPose pose;//里面的时间是电控发送来的，以ms为单位
    u_int32_t mat_timestamp=0;//单位是ms
};

constexpr double shoot_delay=0.00450;//表示子弹发射延迟；

class Autoaim{
    friend class ImageProcess;
public:
    short int DEBUG_ANTITOP;
    short int DETECT_CONFIG;
    short int SHOW_ROI;
    short int SHOW_PREDICT_INFO;
    short int SHOW_PREDICT_PHOTO;
    mutex send_mutex;
    //初始时设置为NOT_END_SOLVER，找到了目标装甲板设置为FIND_NOT_SHOOT,没有找到目标装甲板：NOT_FIND_NOT_SHOOT;
    ArmorState shoot_code=NO_FIND_NO_SHOOT;
 
    Pnp pnpsolver;

    typedef enum{
        SEARCHING,
        TRACKING,
        STANDY
    }State;

    //敌方颜色,0为红色，1为蓝色
    int enemy_color;

    int16_t bullet_speed=15;

    BulletModel bullet;

    SpinDetector spin_detector;

    State state;

    std::shared_ptr<cv::KalmanFilter> KF;

    Detector detector;

    CanSerial &serial;

    bool current_is_find_in_big_roi;

    double last_t;

    float threshold;

    //防止胡乱切换目标的计数器；
    int anti_switch_cnt=0;

    //记录跟踪状态的帧数，用于定时退出跟踪模式；
    int tracking_cnt=0;

    float min_distance;

    bool is_target_change;

    armor_t target_armor,last_armor;

    bool current_is_find_target;

    bool last_is_find_target;
    //Mat roi;

    //用于计时，调试使用；
    double allcosttime=0;
    int nums=0;

    /**
     * @brief 搜索状态，在整张图像中寻找；
     * 找到优先级最高的装甲板，并避免胡乱切换对象
     * 
     */
    bool stateSearchingTarget(cv::Mat &src);

    /**
     * @brief 跟踪状态，在ROI区域中进行搜索；
     * 
     */
    bool stateTrackingTarget(cv::Mat &src);

    /**
     * @brief 矫正状态，作为缓冲状态；
     * 将状态设置为SEACHING;
     */
    bool stateStandBy();

    //获得目标的装甲板：
    bool findtargetarmor(cv::Mat src,armor_t &target);

public:

    void run(cv::Mat &src,uint32_t mat_timestamp);
    /**
     * @brief Get the predict angle object
     * 
     * @param time 预测的时间
     * @param q 四元数；
     * 
     */
    Point2d getpredict_angle(double time,double q[],Mat img);

    Autoaim(const int &color,CanSerial& u);

    ~Autoaim()=default;

    bool iftargetchange();
};



#endif