#ifndef KALNAN_H_
#define KALMAN_H_
/*
卡尔曼滤波处理的是线性模型；
实际比赛中，相机结算出来的是yaw，pitch轴的"相对角度”，定义一个 yaw 和 pitch 匀角速运动的车辆显然不符合实际的物理模型（意味着车在一个球面上运动）
    车辆实际上是在世界坐标系中较长的时间内都近似做"匀速直线运动"
    从观测量[yaw,pitch]到预测量[yaw,pitch](实质上是绝对坐标系中的坐标)是一个"非线性系统"，
使用EKF的优势：
可以避免观测量之间的噪声相互叠加损害预测器的工作，消除变量之间的相关性；
可以运用于非线性系统，预测会更加准确；

EKF核心思想：在每一次迭代中使用一阶Taylor去线性逼近Fk,Hk;
用观测值来更新预测值，类似于形成一个闭环的反馈
通过设置预测量和观测量的协方差矩阵（类似于噪声），让预测器不会超前和滞后
    线性逼近只针对了Fk,Hk，在进一步的预测和更新误差的时候都是用了真实的f,h两个函数;
    该算法的误差主要出现在矩阵运算中；
具体实现：需要实现kalmanfilter滤波
*/
/*
可以使用Matrix类中的类模板和using关键字来定义特定的矩阵的数据类型；
由于预测时状态量和测量量的维数可能发生变化，可以使用模板类来定义class EKF
*/
#include<ceres/jet.h>
#include<Eigen/Dense>
using namespace Eigen;
template<int state_dim,int measure_dim >
class  EKF{
    public:
//先定义需要使用到的特定尺寸的矩阵,x表示状态量的维度，z表示测量量的维度；
        using Matrix_x1d=Matrix<double,state_dim,1>;
        using Matrix_y1d=Matrix<double,measure_dim,1>;
        using Matrix_xxd=Matrix<double,state_dim,state_dim>;
        using Matrix_yxd=Matrix<double,measure_dim,state_dim>;
        using Matrix_yyd=Matrix<double,measure_dim,measure_dim>;
        using Matrix_xyd=Matrix<double,state_dim,measure_dim>;//卡尔曼增益矩阵的尺寸是(state_dim,measure_dim)
//使用前面定义的矩阵类型去创建卡尔曼滤波所需的参数矩阵；
        Matrix_x1d Xe;//估计的状态变量
        Matrix_x1d Xp;//预测的状态变量；
        Matrix_xxd F;//预测的雅克比；
        Matrix_yxd H;//观测的雅克比，在测量量和预测量之间做转换
        Matrix_xxd Q;//预测过程中的协方差
        Matrix_yyd R;//测量过程中的协方差，需要通过测量实验获得；
        Matrix_xyd K;//kalman增益矩阵
        Matrix_xxd P;//状态量的协方差；
        Matrix_y1d Yp;//预测的观测量；
        
    public:
/**
 * @brief 通过对矩阵Q,R,状态量,P赋值来创建类；
 * 注意这里的估计状态变量最初赋值为全零矩阵，Q,R,P矩阵都初始化为单位矩阵
 * 
 */
        explicit EKF(const Matrix_x1d& state0=Matrix_x1d::Zero())
            :Xe(state0),
            P(Matrix_xxd::identity()),
            Q(Matrix_xxd::identity()),
            R(Matrix_yyd::identity()){}
/**
 * @brief 重新给估计状态量赋值；
 * 
 * @param state 
 * @param t 
 */
        void init(Matrix_x1d &state0=Matrix_x1d::Zero()){
            Xe=state0;
        }

/**
 * @brief 实现预测：
 * 
 */ 
        template<class Func>
        Matrix_x1d predict(Func &&func) {
            ceres::Jet<double, state_dim> Xe_auto_jet[state_dim];
            for (int i = 0; i < state_dim; i++) {
                Xe_auto_jet[i].a = Xe[i];
                Xe_auto_jet[i].v[i] = 1;
            }
            ceres::Jet<double, state_dim> Xp_auto_jet[state_dim];
            func(Xe_auto_jet, Xp_auto_jet);
            for (int i = 0; i < state_dim; i++) {
                Xp[i] = Xp_auto_jet[i].a;
                F.block(i, 0, 1, state_dim) = Xp_auto_jet[i].v.transpose();
            }
            P = F * P * F.transpose() + Q;
            return Xp;
        }
/**
 * @brief 实现更新：
 * 
 */
        template<class Func>
        Matrix_x1d update(Func &&func, const Matrix_y1d &Y) {
            ceres::Jet<double, state_dim> Xp_auto_jet[state_dim];
            for (int i = 0; i < state_dim; i++) {
                Xp_auto_jet[i].a = Xp[i];
                Xp_auto_jet[i].v[i] = 1;
            }
            ceres::Jet<double,state_dim> Yp_auto_jet[measure_dim];
            func(Xp_auto_jet, Yp_auto_jet);
            for (int i = 0; i < measure_dim; i++) {
                Yp[i] = Yp_auto_jet[i].a;
                H.block(i, 0, 1, state_dim) = Yp_auto_jet[i].v.transpose();
            }
            K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
            Xe = Xp + K * (Y - Yp);
            P = (Matrix_xxd::Identity() - K * H) * P;
            return Xe;
        }

        void estimate(){
            
        }
};
#endif