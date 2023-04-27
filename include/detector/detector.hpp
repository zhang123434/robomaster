#ifndef DETECTOR_H_
#define DETECTOR_H_
#include<opencv2/opencv.hpp>
#include<bits/stdc++.h>
#include"../digital_classifier/classfier.h"
#include<cmath>
#define IMAGE_HEIGHT        (720)//图片高度//   480   
#define IMAGE_WIDTH         (1280)//图片宽度//  640
using namespace std;
using namespace cv;

//由于后面需要比较装甲板的优先级；需要在结构体中重载运算符；
/*重载运算符的格式如下：
bool operator 运算符 (const 结构体名称 b) const
{
    return(什么时候这个运算符对结构体成立);//注意对此运算符使用this->元素名；
} */

/*问题：
???需要将下面的旋转矩阵转换为正矩形存储到armor_t的容器中
旋转矩形对应的正矩形rect_的设置？？？；
//？？？这里可以考虑优化，每次调用不用都导入参数创建一个对象；classifier类的使用；
*/
struct armor_t{
    cv::RotatedRect rect;
    cv::Rect2f rect_;
    int armor_type;//0-小装甲板，1-大装甲板
    int id;
    armor_t(){
        rect=cv::RotatedRect();
        armor_type=-1;
        id=-1;
        rect_=cv::Rect2f();
    }
    armor_t(RotatedRect rect1,int type,int robotid){
        rect=rect1;
        armor_type=type;
        id=robotid;
        rect_=rect1.boundingRect2f();

    }
    void clear(){
        rect=cv::RotatedRect();
        id=-1;
        armor_type=-1;
        rect_=cv::Rect2f();
        
    }
//比较原则：得分高的优先级高，得分相同时，选择距离图像的中心更近的；
    bool operator <(const armor_t box) const{
//id={"base", "hero", "engineer", "infantry3", "infantry4", "infantry5", "sentry", "outpost", "error101", "error010", "error111"};
        int score[11]={5, 4, 1, 2, 2, 2, 3, 3, 0, 0, 0};
        if (score[id]!=score[box.id]&&id!=-1&&box.id!=-1){
            return score[id]<score[box.id];
        }
        auto d1=sqrt(pow(rect.center.x-IMAGE_WIDTH/2.0,2)+pow(rect.center.x-IMAGE_HEIGHT/2.0,2));
        auto d2=sqrt(pow(box.rect.center.x-IMAGE_WIDTH/2.0,2)+pow(box.rect.center.x-IMAGE_HEIGHT/2.0,2));
        return d1<d2;
    }
};
//储存装甲板的信息的结构体：
struct Armor_Data {//检测到的装甲板数据
  float width        = 0;       // 装甲板宽度
  float height       = 0;       // 装甲板高度
  float aspect_ratio = 0;       // 装甲板宽高比
  float tan_angle    = 0;       // 装甲板tan角度

  cv::RotatedRect armor_rect;   // 装甲板旋转矩形
  cv::RotatedRect left_light;   // 左灯条旋转矩形
  cv::RotatedRect right_light;  // 右灯条旋转矩形

  int distance_center = 0;      // 装甲板距离中心点距离
  int distinguish     = 0;      // 装甲板类型（ 0 小装甲板 1 大装甲板）

  float left_light_width   = 0;
  float right_light_width  = 0;
  float left_light_height  = 0;
  float right_light_height = 0;

  float light_height_aspect = 0;
  float light_width_aspect  = 0;
};
//筛选装甲板所需的数据的结构体
struct Armor_Config {//能够被识别成装甲板的条件
  // 装甲板绘制和调试开关
  int armor_edit;
  int armor_draw;
  int armor_forecast;
  // 左右灯条高度范围
  int light_height_ratio_min;
  int light_height_ratio_max;
  // 左右灯条宽度范围
  int light_width_ratio_min;
  int light_width_ratio_max;
  // 左右灯条 y 的差值不超过灯条平均高度的倍数
  int light_y_different;
  // 左右灯条高度差值不超过灯条平均高度的倍数
  int light_height_different;
  // 左右灯条角度差
  int armor_angle_different;
  // 装甲板比例范围
  int small_armor_aspect_min;
  int armor_type_th;//为大小装甲板的临界值.
  int big_armor_aspect_max;
};

struct Light_Config {//被检测为灯条的条件
  // 绘图和调试开关
  int light_draw;
  int light_edit;
  // 灯条高宽比范围
  int ratio_w_h_min;
  int ratio_w_h_max;
  // 灯条角度范围
  int angle_min;
  int angle_max;
  // 灯条周长范围
  int perimeter_max;
  int perimeter_min;
};

struct Image_Config {
  // 红蓝 BGR 及灰度图参数
  int red_armor_gray_th;
  int red_armor_color_th;
  int blue_armor_gray_th;
  int blue_armor_color_th;
  // 调试开关
  int gray_edit  = 0;
  int color_edit = 0;//设置为1，可以显示预处理的结果；
};
class Detector{
    public:
        vector<armor_t> armor_finded;
        vector<Armor_Data> armor_set;
        vector<RotatedRect> light_set;
        //筛选用的参数：通过构造函数从xml文件中读取出来
        Armor_Config armor_config_;
        Image_Config image_config_;
        Light_Config light_config_;
        Armor_Data   armor_data_;
        Mat src_image;
        Mat final_gray;
        Mat thresh_gray_img;
        Mat bgr_thresh_img;
        Mat blue_red_img,red_blue_img;
        Mat draw_img_;
        Classifier classifier;
        const cv::Mat light_trackbar_  = cv::Mat::zeros(1, 300, CV_8UC1);
        const cv::Mat armor_trackbar_  = cv::Mat::zeros(1, 300, CV_8UC1);
        const cv::Mat sentry_trackbar_ = cv::Mat::zeros(1, 300, CV_8UC1);
        const cv::Mat gray_trackbar_   = cv::Mat::zeros(1, 300, CV_8UC1);
        const cv::Mat bgr_trackbar_    = cv::Mat::zeros(1, 300, CV_8UC1);
        const cv::Mat hsv_trackbar_    = cv::Mat::zeros(1, 300, CV_8UC1);
        const cv::Mat ele_ = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        Detector(string armor_config);
        ~Detector()=default;
    //1.调节相机的曝光，帧率和gamma值，预处理
        Mat preprocess(Mat src,int color);//直接返回预处理最终的结果
    //2.查找轮廓，获得灯条；
        bool findlight(Mat f_gray);
    //3.做装甲板识别 ：（1）做灯条匹配   （2）计算装甲板的中心的平均颜色强度
        bool findarmor();
        bool lightmatch(int i,int j);
        int getid();//在获取id前先做中心区域的亮度筛选；
        float getDistance(const cv::Point a, const cv::Point b);
        void Gamma(Mat src, Mat &dst, float fGamma);
};

#endif

