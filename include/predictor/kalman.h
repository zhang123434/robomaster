#ifndef KALNAN_H_
#define KALMAN_H_
/*
可以使用Matrix类中的类模板和using关键字来定义特定的矩阵的数据类型；
由于预测时状态量和测量量的维数可能发生变化，可以使用模板类来定义class kalman
使用匀速模型；
*/
#include <eigen3/Eigen/Dense>
#include<Eigen/Core>
#include<opencv2/core/eigen.hpp>
#include<opencv2/opencv.hpp>
#include"../pnp/pnp.hpp"
//#include"../serial/serial.h"
#include"../Infantry-can/CanSerial.h"
#include<opencv2/core.hpp>
using namespace Eigen;
using namespace cv;
//注意关键字constexpr的使用

template<int measure_dim,int state_dim>
class  Kalman{
    public:
//先定义需要使用到的特定尺寸的矩阵,x表示状态量的维度，z表示测量量的维度；
        using Matrix_x1d=Matrix<double,state_dim,1>;
        using Matrix_z1d=Matrix<double,measure_dim,1>;
        using Matrix_xxd=Matrix<double,state_dim,state_dim>;
        using Matrix_zxd=Matrix<double,measure_dim,state_dim>;
        using Matrix_zzd=Matrix<double,measure_dim,measure_dim>;
        using Matrix_xzd=Matrix<double,state_dim,measure_dim>;
//使用前面定义的矩阵类型去创建卡尔曼滤波所需的参数矩阵；
        Matrix_x1d x;//表示前一个时刻的状态量
        Matrix_xxd A;//转移矩阵；
        Matrix_zzd Q;//预测过程总的噪声偏差的方差；
        Matrix_xzd T;//和预测噪声相关；
        Matrix_zxd H;//观测矩阵，在测量量和预测量之间做转换
        Matrix_zzd R;//测量的噪声偏差，需要通过测量实验获得；
        Matrix_xzd K;//kalman增益矩阵
        Matrix_xxd P;//估计的协方差矩阵
        double threshold;//卡方检验中阈值
        double last_t=0;
        float processNoise,measureNoise;
        //使用一个标识符表示使用文件中的矩阵来做初始化是否成功；
        bool flag;
    public:
/**
 * @brief Construct a new Kalman object可以在构造函数中修改测量、预测过程中的噪声矩阵；
 * 
 * @param p 
 * @param m 
 */
        Kalman(float p=5,float m=2):processNoise(p),measureNoise(m){};
/**
 * @brief 参数初始化；
 *
 */
        void init(){
            cv::Mat m_A;
			cv::Mat m_P;
			cv::Mat m_H;
			cv::Mat m_R;
			cv::Mat m_Q;
            FileStorage fs("../parameter/Kalman.xml",cv::FileStorage::READ);
            if (!fs.isOpened()){
                //给所有矩阵设置默认值；
                A<< 1, 1, 0, 0,  0, 0,
                    0, 1,  0, 0, 0, 0,
                    0, 0,  1, 1, 0, 0,
                    0, 0,  0, 1, 0, 0,
                    0, 0,  0, 0, 1, 1,
                    0, 0,  0, 0, 0, 1;
                H<<1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0;
                //注意将Eigen矩阵设置为单位矩阵和将Mat类型的矩阵设置为单位矩阵的方法是不一样的，不能混淆；
                //P的初始值表示对初始状态x0的不确定程度，常设置为"单位矩阵"；
                P=Eigen::Matrix<double,6,6>::Identity(6,6);
                /*噪声Q,R都是以0为均值的白噪声；
                Q代表建模误差，代表对预测值的置信度
                R和传感器的特性有关，代表了对测量值的置信度，越大表示越不相信测量值
                如果P0、Q、R无法精确获得，只知道可能的取值范围，则采用可能的较大值（保守）
                如果不确切知道Q、R、P0的准确先验信息，应适当增大Q的取值，以增大对实时量测值的利用权重，俗称调谐。
                但是调谐存在盲目性，无法知道Q要调到多大才行。
                调参方法：就是固定其中一个调另一个
                */
                Q=processNoise*Eigen::Matrix<double,3,3>::Identity(3,3);
                R=measureNoise*Eigen::Matrix<double,3,3>::Identity(3,3);
                return ;
            }
            fs["A"]>>m_A;//注意这里从文件中读取的变量需要用双引号放在方括号中，用单引号会报错
            fs["P"]>>m_P;
            fs["H"]>>m_H;
            fs["R"]>>m_R;
            fs["Q"]>>m_Q;
            fs["threshold"]>>threshold;
            flag=true;
            cv::cv2eigen(m_R, R);
            cv::cv2eigen(m_A, A);
            cv::cv2eigen(m_H, H);
            cv::cv2eigen(m_Q, Q);
            cv::cv2eigen(m_P, P);
        }
/**
 * @brief 做卡尔曼滤波中的预测；
 * 
 * @param x 
 * @param dt 
 * @return Matrix_x1d 
 */
        Matrix_x1d predict(double timestamp,double &dt){
            //cout<<"success"<<endl;
            /*dt=(timestamp-last_t)/1000;
            last_t=timestamp;
            cout<<dt<<endl;*/
            dt=0.1;
        //根据时间参数来给传递矩阵A做初始化；
            Eigen::Matrix<double,6,6> A_;
            A_<<1, dt, 0, 0, 0, 0,
                0, 1,  0, 0, 0, 0,
                0, 0,  1, dt,0, 0,
                0, 0,  0, 1, 0, 0,
                0, 0,  0, 0, 1, dt,
                0, 0,  0, 0, 0, 1;
            A=A_;
            T<<0.5*dt*dt,0,         0,
               dt,       0,         0,
               0,        0.5*dt*dt, 0,
               0,        dt,        0,
               0,        0,         0.5*dt*dt,
               0,        0,         dt;
            Matrix<double,3,1> w;
            cv::RNG rng;
            for (int i=0;i<3;i++){
                w(i,0)=rng.gaussian(Q(i,i));
            }
            x=A*x+T*w;
            P=A*P*A.transpose()+T*Q*T.transpose();
            return x;
        }
/*
并非每个卡尔曼滤波器更新周期中 miniPC 都能解算出的坐标
    量测有效（在卡尔曼周期中解算出来了坐标）：在 miniPC 解算出坐标的周期进行量测更新
    否则只进行过程更新，即异步量测
*/
/*
@brief 直接在该函数中实现卡尔曼滤波中的更新的过程；
此时量测是有效的，需要做卡方检测
卡方检测：
当识别的目标发生了切换，卡尔曼滤波的位置量测会和当前的位置估计有显著的差异，直接进行量测更新会得到一个极大地额速度估计值，
    因此使用卡方检验来判断跟踪目标是否发生变化；
 */     
        Matrix_x1d update(Eigen::Vector3d measure,double timestamp){
            //卡方检验:
            Matrix<double,3,1> e=measure-H*x;
            double r=e.transpose()*((H*P*H.transpose()+R).inverse())*e;
            if (r>=threshold){//发生了目标切换，对状态及其协方差矩阵进行复位以迅速重新收敛
                cout<<"target change"<<endl;
                cout<<"r"<<r<<endl;
                Matrix<double,6,3> G;
                P=Eigen::Matrix<double,6,6>::Identity(6,6);
                x<<measure(0),0,measure(1),0,measure(2),0;//将状态向量中的位置坐标转换成当前测量坐标，三个轴的速度都设置为0；
                last_t=timestamp;//注意发生目标切换后需要更新状态向量和时间戳；
                return x;
            }
            else{
                cout<<"r"<<r<<endl;
                K=P*H.transpose()*(H*P*H.transpose()+R).inverse();
                x=x+K*(measure-H*x);
                P=(Matrix_xxd::Identity(6,6)-K*H)*P;
                return x;
            }
        }
};
/*
class Predictor{
    public:

        Kalman<6,3> kalman_;

        Predictor();

        ~Predictor()=default;
        
         * @brief 使用kalman滤波实现预测；
         * 
         * @param yaw 表示yaw轴的绝对角度，不是相机坐标系中的相对角度
         * @param pitch 表示pitch轴的绝对角度，不是相对角度
         * @param time 表示从接受到图像到处理完图像所需的时间
         * @param bullet_speed 
         * @return cv::Point2f 将预测的yaw，pitch值用坐标的形式返回；
         
        cv::Point2f predict(float yaw,float pitch,float time,float distance,float bullet_speed,Eigen::Vector3d pworld){
        //测量矩阵：[yaw,pitch];
            Eigen::Vector2d measure(2);
        // 矫正目标在相机坐标系中的距离：由于测距误差在0.1*distance 之内 人为进行矫正
            double m_distance=1.10*distance;
        // yaw轴的预测的时间：弹道补偿时间+通信延迟+发弹延迟
            double yaw_time=1.25*m_distance/bullet_speed+2.0*SETIAL_DELAY+kalman_.delay;
        // pitch轴的预测时间：Pitch 轴上不希望云台太抖,给pitch轴上的预测时间加上0.0050;
            double pitch_time=0.3*distance/bullet_speed;
            double dt = 1*time;//化为整数；

        //获得绝对坐标：
        //Eigen::Vector3d pworld=
        //设置测量量和状态量：


        //调用卡尔曼滤波类的预测更新的方法,得到预测的绝对角度
            kalman_.update(measure,time);
        
        需要考虑时间延迟,假设延迟时间内，目标做“均匀直线运动"，需要将状态量量的角速度转换为线速度；
        组内代码是将车在延迟时间的运动视为匀角速度运动
            predict_yaw=yaw_corrected+yaw_delay_time*yaw_speed_corrected;
            predict_pitch=yaw_corrected+pitch_delay_time*pitch_speed_corrected;
    
            double yaw_linear_speed=kalman_.x(2)*pworld.norm();
            double pitch_linear_speed=kalman_.x(3)*pworld.norm();
            double yaw_predict=kalman_.x(0)+atan2(yaw_time*yaw_linear_speed,pworld.norm());
            //可能有问题？？？
            double pitch_predict=kalman_.x(1)+atan2(pitch_time*pitch_linear_speed,pworld.norm());

        //做重力补偿：
        
        }

};*/
#endif