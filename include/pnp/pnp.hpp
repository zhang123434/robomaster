#ifndef PREDICTOREKF_H_
#define PREDICTOREKF_H_
#include <eigen3/Eigen/Dense>
#include<Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include<opencv2/opencv.hpp>
#include<vector>
#include"../detector/detector.hpp"
#include<opencv2/core.hpp>
#include<mutex>
using namespace cv;
using namespace std;
using namespace Eigen;
//#define PREDICT_DEBUG
#define IMAGE_X_DIS ((1920-IMAGE_WIDTH)/2-20)
#define IMAGE_Y_DIS ((1200-IMAGE_HEIGHT)/2+70)
/*相机参数文件中需要存储内参矩阵、畸变矩阵、从相机到云台的坐标转换矩阵，这三个矩阵都是通过标定获得；
坐标转换问题：pnp结算出来的是相机坐标系中的坐标；
1.相机坐标系->云台坐标系
2.云台坐标系->陀螺仪绝对坐标系；
四元数三维空间内所有的旋转都可以用四个数来表示，更便于插值：H=w+xi+yj+zk;
Eigen中四元数可以用Eigen::Quaterniond(double类型)或者Eigen::Quaternionf(float类型)表示
       构造方法：Eigen::Quaterniond/f 对象名(const Scalar &w,const Scalar *x,const Scalar &y,const Scalar &z);
               w为四元数的实部，x,y,z都是四元数的虚部，但在内存中四个元素的顺序是x,y,z,w，可以通过访问对象名的数据成员.coeffs;
       使用函数matrix（）函数可以将四元数对象转化为矩阵；
*/
/**
 * @brief 获取任意四边形的中心点
 * 
 * @param pts 
 * @return cv::Point2f 
 */
inline cv::Point2f points_center(cv::Point2f pts[4]) {
        cv::Point2f center;
        center.x = (pts[0].x + pts[1].x + pts[2].x + pts[3].x) / 4;
        center.y = (pts[0].y + pts[1].y + pts[2].y + pts[3].y) / 4;
        return center;
}

struct Pnp_config{//单位是毫米
    float offset_armor_pitch=-0.6;
    float offset_armor_yaw=2.65;
};
class Pnp{
public:
//定义坐标转换的旋转矩阵和相机的内参矩阵和畸变矩阵；
    Eigen::Matrix3f gy_c_R; //从陀螺仪坐标系到相机坐标系的旋转矩阵
    Eigen::Vector3f gy_c_t;//从陀螺仪坐标系到相机坐标系的平移矩阵,相机坐标系中陀螺仪的坐标
    Eigen::Matrix3f c_gy_R;//从相机坐标系到陀螺仪坐标的旋转矩阵；
    cv::Mat cameraMatrix_Mat;
    cv::Mat distcoeffs_Mat;
    Pnp_config pnp_config;
    mutex pnp_mutex;

//定义pnp结算中用到的一些特征点容器：单位是mm
    float    small_armor_height  =63.0,  small_armor_width = 128;
    float   big_armor_width     = 225, big_armor_height  = 65;
    std::vector<cv::Point3f> big_featurepoint;
    std::vector<cv::Point3f> small_featurepoint;

//解算出来的数据：
    float yaw;//左右偏转的角度；
    float pitch;//上下旋转的角度,这两个角度以度为单位
    float distance;//相机坐标中装甲板中心和该坐标系原点之间的距离，这个距离以m为单位
    Point3f pcamera_;//世界坐标系中装甲板的中心在相机坐标系中的坐标
    Mat tvec;//相机坐标系到世界坐标系的平移矩阵
    Mat rvec;//相机坐标系到世界坐标系的旋转矩阵
//定义和pnp解算相关的函数：
    /**
     * @brief 相机坐标系内的坐标转换到"陀螺仪"的绝对"世界坐标系"中，陀螺仪中的坐标是可变的；
     * @param P_world 表示从世界坐标系到陀螺仪坐标系；
     */
    /*inline Eigen::Vector3f camera_to_world(const Eigen::Vector3f pcamera,Eigen::Matrix3f P_world){
        return P_world.transpose()*(c_gy_R*pcamera+gy_c_t);
    }*/
    inline Point3f camera_to_world1(Point3f pcamera,double q[],cv::Point3f offset){
        pcamera.y+=offset.y;
        pcamera.z+=offset.z;
        Eigen::Quaterniond q1(q[0],q[1],q[2],q[3]);
        Eigen::Quaterniond p(0,pcamera.z,-pcamera.x,-pcamera.y);
        Eigen::Quaterniond result=q1*p*q1.inverse();
        return Point3f(float(result.x()),float(result.y()),float(result.z()));
    }
    /**
     * @brief 将陀螺仪的世界坐标系中的点的坐标转换到相机坐标系中
     * 
     */
    /*inline Eigen::Vector3f world_to_camera(const Eigen::Vector3f pworld,Eigen::Matrix3f P_world){
        Eigen::Vector3f temp=P_world*pworld;
        return gy_c_R*temp-gy_c_t;
    }*/
    
    inline Point3f world_to_camera1(Point3f pworld,double q[]){
        Eigen::Quaterniond p(0,pworld.x,pworld.y,pworld.z);
        Eigen::Quaterniond q1(q[0],q[1],q[2],q[3]);
        Eigen::Quaterniond result=q1.inverse()*p*q1;
        return Point3f(float(-result.y()+24.86),float(-result.z()-103.54),float(result.x()));
    }

    /**
     * @brief 将相机坐标系内的坐标转化到图像的坐标系中的坐标
     * 相机到陀螺仪的偏移量：102.3，50，20
     */
    inline Point3f camera_to_image(Mat pcamera){
        Mat_<float> temp= cameraMatrix_Mat*pcamera/pcamera.at<float>(2,0);
        return Point3f(temp.at<float>(0,0),temp.at<float>(1,0),temp.at<float>(2,0));
    }
    /**
     * @brief 将陀螺仪的世界坐标系中点的坐标转化到图像的坐标系中，然后用圆点绘制
     * 
     */
    Point3f project_point_draw(cv::Mat &image,const Point3f pworld,Scalar color,double q[]){//Eigen::Matrix3d P_world){//
        Point3f pimage_;
        pimage_=world_to_camera1(pworld,q);//pimage_为相机坐标系中物体的坐标；
        //pimage_=world_to_camera(pworld,P_world);
        //Mat p=(cv::Mat_<float>(3,1)<<pimage_.x,pimage_.y,pimage_.z);
        //Point3f pimage=camera_to_image(p);
        //将目标在相机坐标系中的坐标转化到图像坐标系中可以使用projectPoints函数：
        cv::Point2f p_2d;
        vector<cv::Point3f> p1;
        p1.push_back(cv::Point3f(0,0,0));
        vector<cv::Point2f> p2;
        cv::Mat t(pimage_);//可以直接将Point3f类型的数据转化为mat类型的数据；
        cv::projectPoints(p1,rvec,t,cameraMatrix_Mat,distcoeffs_Mat,p2);
        p_2d=p2[0];
        // vector<cv::Point3f> p1;
        // p1.push_back(pimage_);
        // vector<cv::Point2f> p2;
        // cv::projectPoints(p1,rvec,tvec,cameraMatrix_Mat,distcoeffs_Mat,p2);
        // p_2d=p2[0];
        cv::circle(image,p_2d,3,cv::Scalar(0,255,0),3); 
        //cout<<p_2d<<endl; 
        //putText(image,"",p_2d,FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,0),2);
        return pimage_;
    }
    /**
     * @brief 对四个特征点按照左上->右上->右下->左下的顺序排列在容器中
     * //double flytime=pworld.norm()/bullet_speed;
     * @param rect 
     * @return vector<cv::Point2f> 
     */
    vector<cv::Point2f> get_points_inorder(const cv::RotatedRect &rect){
        vector<Point2f> pimage(4);
        Point2f t[4];
        rect.points(t);
        Point2f lu,ld,ru,rd;
        //先按照x升序排列
        sort(t,t+4,[](const Point2f p1,Point2f p2){return p1.x<p2.x;});
        //相邻项比较y坐标
        if (t[0].y<t[1].y){
            lu=t[0];
            ld=t[1];
        }
        else{
            lu=t[1];
            ld=t[0];
        }
        if (t[2].y<t[3].y){
            ru=t[2];
            rd=t[3];
        }
        else{
            ru=t[3];
            rd=t[2];
        }
        pimage[0]=lu;//+cv::Point2f(IMAGE_X_DIS, IMAGE_Y_DIS);
        pimage[1]=ru;//+cv::Point2f(IMAGE_X_DIS, IMAGE_Y_DIS);
        pimage[2]=rd;//+cv::Point2f(IMAGE_X_DIS, IMAGE_Y_DIS);
        pimage[3]=ld;//+cv::Point2f(IMAGE_X_DIS, IMAGE_Y_DIS);
        return pimage;
    }
    /**
     * @brief 根据装甲板的旋转矩形和装甲板的类型来做pnp解算；
     * 
     * @param p 
     * @param armor_type 
     * @return Eigen::Vector3f
     */
    Point3f pnp_get_pc(const cv::RotatedRect &rect,int armor_type){
        vector<cv::Point2f> pimage=get_points_inorder(rect);

        if (armor_type==0){
            cv::solvePnP(small_featurepoint,pimage,cameraMatrix_Mat,distcoeffs_Mat,rvec,tvec,false,cv::SOLVEPNP_ITERATIVE);
        }
        else if (armor_type==1){
            cv::solvePnP(big_featurepoint,pimage,cameraMatrix_Mat,distcoeffs_Mat,rvec,tvec,false,cv::SOLVEPNP_ITERATIVE);
        }
        distance=pow(tvec.at<double>(0,0)*tvec.at<double>(0,0)+tvec.at<double>(1,0)*tvec.at<double>(1,0)+tvec.at<double>(2,0)*tvec.at<double>(2,0),0.5)/1000.0;
        pnp_mutex.lock();
        yaw=atan2(tvec.at<double>(0,0),(tvec.at<double>(2,0)))*180.0/CV_PI;//加10
        pitch=atan2(-(tvec.at<double>(1,0)),(tvec.at<double>(2,0)))*180.0/CV_PI+2.5;//加80.48,60,10
        pcamera_.x=tvec.at<double>(0,0);
        pcamera_.y=tvec.at<double>(1,0);
        pcamera_.z=tvec.at<double>(2,0);
        pnp_mutex.unlock();
        //cout<<"camera postion:"<<pcamera_<<endl;
        return pcamera_;
    }

public:

    Pnp(){
        big_featurepoint.emplace_back(cv::Point3f(-big_armor_width  * 0.5,
            -big_armor_height * 0.5, 0));
        big_featurepoint.emplace_back(cv::Point3f(big_armor_width  * 0.5,
                -big_armor_height * 0.5, 0));
        big_featurepoint.emplace_back(cv::Point3f(big_armor_width  * 0.5,
                big_armor_height * 0.5, 0));
        big_featurepoint.emplace_back(cv::Point3f(-big_armor_width  * 0.5,
                big_armor_height * 0.5, 0));

        small_featurepoint.emplace_back(Point3f(-small_armor_width   * 0.5,
                -small_armor_height * 0.5, 0));
        small_featurepoint.emplace_back(Point3f(small_armor_width   * 0.5,
                -small_armor_height * 0.5, 0));
        small_featurepoint.emplace_back(Point3f(small_armor_width   * 0.5,
                small_armor_height * 0.5, 0));
        small_featurepoint.emplace_back(Point3f(-small_armor_width   * 0.5,
                small_armor_height * 0.5, 0));
        //注意yaml，yml文件中不能存储Matrix类型的元素
        //注意这里的相对目录一定不能用"../../parameter/camera_param.xml"
        cv::FileStorage fs("../parameter/camera_param.xml",cv::FileStorage::READ);
        if (!fs.isOpened()){
            cout<<"load camera parameter failed"<<endl;
        }
        else{
            cout<<"load camera parameter successfully"<<endl;
        }
        fs["cameraMatrix"]>>cameraMatrix_Mat;
        fs["distcoeffs"]>>distcoeffs_Mat;
        
        //cv::cv2eigen(cameraMatrix_Mat,cameraMatrix);
        //cv::cv2eigen(distcoeffs_Mat,distcoeffs);
        gy_c_R<<0,1,0,0,0,1,1,0,0;
        c_gy_R<<0,0,1,1,0,0,0,1,0;
        gy_c_t<<24.86,0,-103.54;
    }
};

#endif