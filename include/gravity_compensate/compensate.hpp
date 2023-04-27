/*
已知落点，求解出射的角度这个问题可以推算，公式复杂，而且存在很多解、无解的情况
考虑使用""基于模型前向迭代的数值解法""
    利用重力导致的下落高度进行迭代补偿
    考虑空气阻力
算法过程如下：
    设置最终目标点targetPoint
    设临时目标点tempPoint=tempPoint
    循环迭代10次：
        计算仰角angle=枪管指向tempPoint的角度
        利用抛物线模型，计算实际命中点realPoint.得到误差，即下落高度deltaH=targetPoint-realPoint
        更新tempPoint=tempPoint+deltaH
    输出仰角angle，与误差deltaH
单方向的空气阻力模型：只考虑水平方向；
当弹道的角度较小(<10)时可以只考虑水平方向的空气阻力，不考虑竖直方向的空气阻力
    该模型需要角度的迭代次数，适合远距离击打，且在抛物线下降沿打中目标，否则会造成反效果；
空气阻力系数的求解：
    1.可以通过测试一次获得参数，然后基于模型前向迭代求解；
    2.可以通过参数：水平位移x，竖直位移y，仰角a，速度v，基于公式使用牛顿迭代近似求解；

完全空气阻力模型：考虑水平和竖直方向的空气阻力；
假设y方向上升过程，存在空气阻力，下降过程无空气阻力
*/
/*需要实现的功能：
1.使用牛顿迭代法计算子弹飞行的时间：需要考虑空气阻力和时间延迟；
2.基于单方面的空气阻力模型利用前向迭代的数值求解方法实现重力补偿，获得仰角；
3.可以通过测试获得空气阻力模型中的系数参数；
*/
#include<iostream>
#include<opencv2/opencv.hpp>
#include<Eigen/Dense>
using namespace Eigen;
using namespace cv;
using namespace std;
//define BULLET_MODEL_DEBUG
class BulletModel{
public:
    double gravity=9.7913;
    double k=0.0452;//空气阻力的系数，该参数可能需要调节???
    double iter_num=20;
    double bs_coeff;
    double bs0;//0.5-1.5
    double bs1;//1.5-2.5
    double bs2; //2.5-4.5;
    double bs3;//4.5--
    BulletModel(){
        cv::FileStorage fp("../include/gravity_compensate/bs.xml",cv::FileStorage::READ);
        fp["BS0"]>>bs0;
        fp["BS1"]>>bs1;
        fp["BS2"]>>bs2;
        fp["BS3"]>>bs3;
        fp.release();
    }
    ~BulletModel()=default;
   
    double getflytime(double bullet_speed,Point3f pworld){//,double predictflytime,double speed_x,double speed_y){//调用该函数之前需要获取k值；
        double level_distance=sqrt(pow(pworld.x,2)+pow(pworld.y,2))/1000;
        /*double a,b;
        double T=sqrt(pow(pworld.x,2)+pow(pworld.y,2))/(1000*bullet_speed);//子弹飞行的时间；
        double t;
        do{
            t=T-predictflytime;//需要加绝对值吗,预测后计算子弹飞行时间应该比预测前的子弹飞行时间更长
            double distance=(sqrt(pow(pworld.x+speed_x*t,2)+pow(pworld.y+speed_y*t,2)));
            //考虑车子在预测时间内的移动：target*(T+0.00450+0.0035*2)可以用预测中前后两个点之间的距离表示；
            a=(1.0/k)*log(k*bullet_speed*T+1)-distance/1000;
            b=bullet_speed/(k*bullet_speed*T+1)-(1.0/distance)*(speed_x*(pworld.x+t*speed_x)+speed_y*(pworld.y+t*speed_y))*0.001;
            T=T-a/b;
        }while(a>1e-5||a<-(1e-5));//需要控制精度，逼近处理；
        cout<<"t"<<t<<endl;//如果这个时间差值太小可以忽略；
        cout<<"T"<<T<<endl;*/
        //直接进行求解;
        double T=(exp(level_distance*k)-1)/(k*bs_coeff*bullet_speed);
        return T;
    }
   
    double getpredictflytime(double bullet_speed,float speed_x,float speed_y,Point3f pworld,double predict_time){//调用该函数之前需要获取k值；
        double a,b;
        //double T=sqrt(pow(pworld.x-pbarrel(0),2)+pow(pworld.y-pbarrel(1),2))/(1000*bullet_speed);//子弹飞行的时间；
        double T=sqrt(pow(pworld.x,2)+pow(pworld.y,2))/(1000*bullet_speed);
        do{
            //考虑车子在预测时间内的移动：target*(T+0.00450+0.0035*2)可以用预测中前后两个点之间的距离表示；
            //double temp=sqrt(pow(pbarrel(0)-predict_world.x,2)+pow(pbarrel(1)-predict_world.y,2));
            double dt=T+predict_time;//一定要放在循环里面，否则dt的值不会变化；
            double temp=sqrt(pow(pworld.x+dt*speed_x,2)+pow(pworld.y+dt*speed_y,2));
            a=(1.0/k)*log(k*bullet_speed*T+1)-temp/1000;
            b=bullet_speed/(k*bs_coeff*bullet_speed*T+1)-(1.0/temp)*(speed_x*(pworld.x+dt*speed_x)+speed_y*(pworld.y+dt*speed_y))*0.001;
            T=T-a/b;
        }while(a>1e-5||a<-(1e-5));//需要控制精度，逼近处理；  
        return T;
    }

    double compensate(double bullet_speed,Point3f predict_world,double t,double speed_x,double speed_y){
        //double time1=getTickCount();
        double y_temp=0.0;
        double y_actual=0.0;
        double dy=0.;
        double angle=0.;//击打的应该是预测点的位置；
        double x=sqrt(pow(predict_world.x,2)+pow(predict_world.y,2))/1000;//水平距离
        double y=predict_world.z/1000;//竖直距离
        y_temp=y;
        for (int i=0;i<iter_num;i++){
            angle=atan2(y_temp,x);
            //double t_actual=double(exp(k*x-1)/(k*bullet_speed*cos(angle)));
            double t_actual=getflytime(bullet_speed*cos(angle),predict_world);//,t,speed_x,speed_y);
            y_actual=double(bullet_speed*bs_coeff*sin(angle)*t_actual-gravity*t_actual*t_actual/2.0);
            dy=y-y_actual;
            y_temp+=dy;
            if (fabsf(dy)<0.001)
                break;
        }
        //cout<<"after predict,flytime: "<<y_actual<<endl;
        //double time2=(getTickCount()-time1)/cv::getTickFrequency()*1000.0;
        //cout<<"计算弹道的时间:"<<time2<<" ms"<<endl;
        return angle/M_PI*180.0;//返回的是角度；
    }
    void setbs_coeff(double level_distance,double speed){
        if (speed<16){
            if (level_distance<=1.5){
                bs_coeff=bs0;
            }
            else if (level_distance>1.5&&level_distance<=2.5){
                bs_coeff=bs1;
            }
            else if (level_distance<=4.5&&level_distance>2.5){
                bs_coeff=bs2;
            }
            else if (level_distance>4.5){
                bs_coeff=bs3;
            }
        }
        //cout<<"子弹的速度的系数："<<bs_coeff<<endl;
    }
};