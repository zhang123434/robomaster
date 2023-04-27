#ifndef SPINDETECTOR_H
#define SPINDETECTOR_H

#include"../Infantry-can/const.h"
#include"../detector/detector.hpp"
#include"../pnp/pnp.hpp"
#include"rm_types.h"
#include<opencv2/opencv.hpp>
#include <vector>
using namespace cv;
using namespace std;
#define ARMOR_WINDOW_LENGTH 150
#define ARMOR_TO_JUDGE_LENGTH  2

class SpinDetector{
private:
    vector<armor_t> armor_info_window;   // 滑动窗口
    vector<cv::Point3f> coord_info_window; 
    vector<uint32_t> spin_timestamp_list;       // 跳变帧序列
    vector<float> x_realworld;
    vector<float> y_realworld;
    vector<float> z_realworld;
    int num_frame=0;

public:
    cv::Point3f mean_point;
    RM::spin_param param;                       // 参数
    bool is_spin;

private:
    /**
     *  @brief  判断近是符否合陀螺状态规定
     */
    bool ifLegal();

public:
    /**
     *  @brief  清空状态
     */
    void reset();

    /**
     *  @brief  判断目标是否处于陀螺状态
     *  @param  coord   坐标
     *  @param  time    陀螺仪时间戳
     *  @return 是否进入反小陀螺策略
     */
    bool judgeSpin(armor_t armor,  uint32_t timestamp,bool if_spin);

    /**
     *  @brief  添加滤波结果的三维坐标
     */
    void pushCoord(cv::Point3f coord);

    /**
     *  @brief  获取坐标均值
     *  @param  coord   结果参数，坐标均值
     *  @return 是否成功获取坐标均值
     */
    bool solverSpin(cv::Point3f &coord);
    
    void Recode_xyz(double x, double y,double z);
    SpinDetector();
    ~SpinDetector() = default;
};


#endif