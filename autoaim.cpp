#include"../../include/autoaim/autoaim.hpp"

Autoaim::Autoaim(const int &color,CanSerial& u,const Pnp &solver):enemy_color(color),
state(SEARCHING),tracking_cnt(0),serial(u),pnpsolver(solver),detector("../parameter/detect.xml"){
    is_target_change=false;
    current_is_find_target=false;
    last_is_find_target=false;
    cv::Mat_<float> m_R;
    cv::Mat_<float> m_Q;
    FileStorage fs("../parameter/Kalman.xml",cv::FileStorage::READ);
    fs["R"]>>m_R;
    fs["Q"]>>m_Q;
    fs["same_armors_max_dis"]>>min_distance;
    fs["threshold"]>>threshold;
    fs.release();
    KF=std::make_shared<cv::KalmanFilter>(6,3,0);
    Mat H =(cv::Mat_<float>(3,6)<<1, 0,  0, 0,   0, 0,
                                0,  0,  1,   0,   0, 0,
                                0,  0,  0, 0,   1, 0);
    KF->processNoiseCov=m_Q;
    KF->measurementNoiseCov=m_R;
    //cv::setIdentity(KF->errorCovPost,cv::Scalar::all(1.0));
    KF->errorCovPost=(cv::Mat_<float>(6,6)<<1,0,0,0,0,0,
                                            0,500,0,0,0,0,
                                            0,0,1,0,0,0,
                                            0,0,0,500,0,0,
                                            0,0,0,0,1,0,
                                            0,0,0,0,0,500);
    cv::setIdentity(KF->transitionMatrix,cv::Scalar::all(1.0));
    KF->measurementMatrix=H;
}

/*运行时候状态的设置和变化：
初始时state设置为SEACHING
在SEACHING模式下如果找到了目标装甲板，就将状态设置为TRACKING;
结束TRACKING状态的条件：roi区域中找不到目标装甲板或跟踪的帧数超过500，结束跟踪后将状态设置为SEACHING;
*/
void Autoaim::run(cv::Mat &src){
    switch(state){

        case SEARCHING:
        {
            //cout<<"searching"<<endl;
            if(stateSearchingTarget(src)){//搜索目标装甲板
            //注意判断目标装甲板是否在截取的大图像中；
                if ((target_armor.rect_ & Rect2f(0,0,IMAGE_WIDTH,IMAGE_HEIGHT))==target_armor.rect_){
            //寻找到了目标装甲板，将机器人模式设置为跟踪模式；
                    //cout<<last_armor.rect_.tl()<<endl;
                    state=TRACKING;
                    tracking_cnt=0;
                }
            }
            break;
        }
        
       case TRACKING:
        {
            //cout<<"tracking"<<endl;
            //结束跟踪状态的条件：roi区域中找不到目标装甲板或跟踪的帧数超过500；
            if(!stateTrackingTarget(src)||++tracking_cnt>500){
                //cout<<tracking_cnt<<endl;
                state=SEARCHING;
            }
            break;
        }

        case STANDY:
        default:
            //cout<<"else state"<<endl;
            stateStandBy();
            break;//必须要加，不然跳不出去；
    }
    if (target_armor.rect_ !=cv::Rect2f()){
        if (iftargetchange()){
            is_target_change=true;
        }
        last_armor.rect=target_armor.rect;
        last_armor.rect_=target_armor.rect_;
        last_armor.id=target_armor.id;
        last_armor.armor_type=target_armor.armor_type;
        shoot_code=FIND_NO_SHOOT;
        //找到目标装甲板，做pnp解算；
        pnpsolver.pnp_get_pc(target_armor.rect,target_armor.armor_type);
    
    }
    
    else{
        is_target_change=true;
        shoot_code=NO_FIND_NO_SHOOT;
    }
    
}

/*不是类的成员函数所以不能调用该函数；
bool Autoaim::contrastTwoArmor(const armor_t &a,const armor_t &b){
    if (last_armor.rect_ != cv::Rect2f()){
            return getPointdistance(a.rect.center-last_armor.rect.center)<getPointdistance(b.rect.center-last_armor.rect.center);
        }
    else{
        return a<b;
    }
}*/
bool Autoaim::findtargetarmor(cv::Mat src,armor_t &target){
    detector.src_image=src.clone();
    detector.draw_img_=src.clone();
    Mat gray=detector.preprocess(src,enemy_color);
    if (!detector.findlight(gray)){
    #ifdef DETECT_CONFIG 
        cout<<"error1"<<endl;
    #endif
    /*if (state==TRACKING){
        imshow("lightblob",gray);
        imshow("problem roi",src);
        waitKey(0);
    }*/
        detector.armor_finded.clear();
        detector.armor_finded.shrink_to_fit();
        detector.armor_set.clear();
        detector.armor_set.shrink_to_fit();
        detector.light_set.clear();
        detector.light_set.shrink_to_fit();
        return false;
    }
    if (!detector.findarmor()){
    #ifdef DETECT_CONFIG
        cout<<"error2 :";
        cout<<detector.light_set.size()<<endl;
    #endif
        detector.armor_finded.clear();
        detector.armor_finded.shrink_to_fit();
        detector.armor_set.clear();
        detector.armor_set.shrink_to_fit();
        detector.light_set.clear();
        detector.light_set.shrink_to_fit();
        return false;
    }
    /*cout<<detector.armor_finded.size()<<endl;
    namedWindow("aim1",0);
    imshow("aim1",detector.draw_img_);
    waitKey(1);*/
//对识别出来的装甲板容器vector<armor_t> armor_finded排序
//排序的优先级：如果上一帧的装甲板不为空，那么和上一个装甲板距离更近的优先级更高，如果上一帧的装甲板为空，根据得分和距离图像中心的距离来判断优先级；
    sort(detector.armor_finded.begin(),detector.armor_finded.end(),[&](const armor_t &a,const armor_t &b){
    if (last_armor.rect_ != cv::Rect2f()){
            return detector.getDistance(a.rect.center,last_armor.rect.center)<detector.getDistance(b.rect.center,last_armor.rect.center);
        }
    else{
        return a<b;
    }
        });
    //判断id,如果id不是-1，0，6，7，8，9，10，就将容器中靠前的装甲板设置为目标装甲板；
    //装甲板的id应给初始化为什么???
    for (auto &armor:detector.armor_finded){
        if (armor.id!=-1&&armor.id!=0&&armor.id!=6&&armor.id!=7&&armor.id!=8&&armor.id!=9&&armor.id!=10){
            target.rect=armor.rect;
            target.rect_=armor.rect.boundingRect2f();
            target.id=armor.id;
            target.armor_type=armor.armor_type;
            break;
        }
    }
    if(target.rect_==cv::Rect2f()){
        if (state==TRACKING){
            target=detector.armor_finded[0];
        }
        else{
        #ifdef DETECT_CONFIG
            cout<<"error3"<<endl;
        #endif
            detector.armor_finded.clear();
            detector.armor_finded.shrink_to_fit();
            detector.armor_set.clear();
            detector.armor_set.shrink_to_fit();
            detector.light_set.clear();
            detector.light_set.shrink_to_fit();
            return false;
        }
    }
    detector.armor_finded.clear();
    detector.armor_finded.shrink_to_fit();
    detector.armor_set.clear();
    detector.armor_set.shrink_to_fit();
    detector.light_set.clear();
    detector.light_set.shrink_to_fit();
    return true;
}

//寻找；
bool Autoaim::stateSearchingTarget(cv::Mat &src){
    if (findtargetarmor(src,target_armor)){
        //能够避免胡乱切换目标；
        if (last_armor.rect_!=cv::Rect2f()&&
        (sqrt(pow(last_armor.rect.center.x-target_armor.rect.center.x,2)+pow(last_armor.rect.center.y-target_armor.rect.center.y,2))>last_armor.rect_.width*2.0)&&
        anti_switch_cnt++<3&&target_armor.id!=last_armor.id){
            //cout<<"error1"<<endl;
        //判断当前目标上一次有效目标是否为同一个目标:找到的目标装甲板和roi的装甲板的距离过大时
            current_is_find_target=false;
            target_armor.clear();//不是同一目标，给三帧的时间，找到相同的目标????
            last_armor.clear();
            return false;
        }
        else{
            current_is_find_target=true;
            anti_switch_cnt=0;
            return true;
        }
    }
    else{//当没有识别到装甲板时
        //cout<<"error2"<<endl;
        current_is_find_target=false;
        target_armor.clear();
        anti_switch_cnt++;
        return false;
    }
}
//矫正
bool Autoaim::stateStandBy(){
    state=SEARCHING;
    return true;
}
//跟踪
bool Autoaim::stateTrackingTarget(cv::Mat &src){
    Mat roi;
    auto t=last_armor.rect_;//这里出现问题；
    //cout<<last_armor.rect.center<<endl;
    //cout<<(last_armor.rect_.tl()+last_armor.rect_.br())/2<<endl;
//判断目标装甲板是否在所截取的大图像中；
    cv::Rect2f bigger_rect;//roi区域的正矩形；
    if(t.tl().x<0||t.tl().y<0||t.br().x>IMAGE_WIDTH||t.br().y>IMAGE_HEIGHT){//& cv::Rect2f(0,0,IMAGE_WIDTH,IMAGE_HEIGHT))!=t){
        cout<<"roi区域越界"<<endl;
        /*cout<<t.tl()<<endl;
        cout<<t.br()<<endl;
        imshow("problem roi",src);
        waitKey(0);*/
        current_is_find_target=false;
        target_armor.clear();
        return false;
    }
//创建roi区域：
    bigger_rect.x=t.x-0.8*t.width;
    bigger_rect.y=t.y-0.5*t.height;
    bigger_rect.height=t.height*2;
    bigger_rect.width=t.width*2.6;
    bigger_rect&=cv::Rect2f(0,0,IMAGE_WIDTH,IMAGE_HEIGHT);
    //函数clone()是完全的深度拷贝，在内存和src独立；
    roi=src(bigger_rect).clone();
    
//在区域内重新搜索：
    armor_t target;
    if (findtargetarmor(roi,target)){
    //如果成功获得目标，利用搜索区域重新更新目标装甲板的位置数据
        target_armor=target;
        current_is_find_target=true;
        //注意旋转矩形的四个角点都需要加偏移量：修改旋转矩阵的中心点的坐标，四个角点会自动添加偏移量
        target_armor.rect.center.x+=bigger_rect.x;//添加roi偏移量
        target_armor.rect.center.y+=bigger_rect.y;
        target_armor.rect_.x+=bigger_rect.x;
        target_armor.rect_.y+=bigger_rect.y;
        #ifdef SHOW_ROI
            namedWindow("roi1",0);
            imshow("roi1",roi);
            waitKey(1);
        #endif
    }
    //没有识别到，扩大roi区域的大小；
    else{
        //cout<<"bigger roi:"<<endl;
        bigger_rect.x =t.x-1.5*t.width;
        bigger_rect.y =t.y-1.0*t.height;
        bigger_rect.width =4*t.width;
        bigger_rect.height =3*t.height;
        /*bigger_rect.x -=1.5*bigger_rect.width/2.0*1.5;
        bigger_rect.y -=1.5*bigger_rect.width/2.0;
        bigger_rect.width *=2.5*1.5;
        bigger_rect.height *=2.5;*/
        /*bigger_rect.x-=0.75*bigger_rect.width;
        bigger_rect.y-=0.25*bigger_rect.height;
        bigger_rect.width*=2.5;
        bigger_rect.height*=1.5;*/
        //与符号&和=不能分开；
        bigger_rect &= cv::Rect2f(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT);
        roi=src(bigger_rect).clone();
       
        if (findtargetarmor(roi,target)){
            target_armor=target;
            current_is_find_target=true;
            target_armor.rect.center.x+=bigger_rect.x;//添加roi偏移量
            target_armor.rect.center.y+=bigger_rect.y; 
            target_armor.rect_.x+=bigger_rect.x;
            target_armor.rect_.y+=bigger_rect.y;
            #ifdef SHOW_ROI
                namedWindow("roi2",0);
                imshow("roi2",roi);
                waitKey(1);
            #endif
        }
        else{
            current_is_find_target=false;
            target_armor.clear();
            return false;
        }
    }
//设置上一个时刻的装甲板；可以用last_armor=target_armor来替代,结构体有默认实现的赋值运算符；
    /*last_armor.rect=target_armor.rect;
    last_armor.rect_=target_armor.rect_;
    last_armor.id=target_armor.id;
    last_armor.armor_type=target_armor.armor_type;*/
    //cout<<target_armor.rect.center<<endl;
    //cout<<"target"<<(target_armor.rect_.tl()+target_armor.rect_.br())/2<<endl;
    return true;
}

bool Autoaim::iftargetchange(){
    if(last_is_find_target==false&&current_is_find_target==true){
        return true;
    }
    else if (last_is_find_target==true&&current_is_find_target==true){
        float x1=target_armor.rect.center.x;
        float y1=target_armor.rect.center.y;
        float x2=last_armor.rect.center.x;
        float y2=last_armor.rect.center.y;
        float distance=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
        if (last_armor.id==target_armor.id||distance<min_distance){
                return false;
        }
        return true;
    }
}

void Autoaim::getpredict_angle(double timestamp,double q[],Mat img){
//根据四元数获得从云台到绝对坐标系的坐标转换矩阵；
    /*Eigen::Quaternionf q_raw(q[0],q[1],q[2],q[3]);
    Eigen::Quaternionf q_(q_raw.matrix().transpose());
    Eigen::Matrix3d P_world=q_.matrix().cast<double>();*/
//将pnp结算出来的坐标转换到绝对坐标系中,绝对坐标系中坐标为(x,y,数值方向的距离)
    cv::Point3f pworld=pnpsolver.camera_to_world1(pnpsolver.pcamera_,q,cv::Point3f(0.,24.86,103.54));
    if (is_target_change){//判断前后两帧的装甲板的中心的距离，来判断目标的切换；
        //cout<<"target change11111111111111111111111111111111"<<endl;
        cv::Point3f pworld1=pnpsolver.camera_to_world1(pnpsolver.pcamera_,q,cv::Point3f(0.,0.,0.));
        KF->statePost=(cv::Mat_<float>(6,1)<<pworld1.x,0,pworld1.y,0,pworld1.z,0);
        last_t=timestamp;
    }
    float dt=(float(timestamp-last_t))/1000.0;//单位是s;
    last_t=timestamp;
    
    //cout<<"camera:"<<pnpsolver.pcamera_(0,0)<<" "<<pnpsolver.pcamera_(1,0)<<" "<<pnpsolver.pcamera_(2,0)<<endl;
    //cout<<"world:"<<pworld.x<<" "<<pworld.y<<" "<<pworld.z<<endl;
    Mat measurement=(cv::Mat_<float>(3,1)<<pworld.x,pworld.y,pworld.z);
    KF->transitionMatrix.at<float>(0,1)=dt;
    KF->transitionMatrix.at<float>(2,3)=dt;
    KF->transitionMatrix.at<float>(4,5)=dt;
    Mat state1=KF->predict();
//在这两个函数之间需要判断目标是否发生切换，矩阵求逆用invert函数,矩阵转置用MAT.t()  
    Mat e=measurement-KF->measurementMatrix*state1;
    Mat_<float> r=e.t()*(KF->measurementNoiseCov+KF->measurementMatrix*KF->errorCovPost*KF->measurementMatrix.t()).inv()*e;
    //cout<<"r:"<<r.at<float>(0,0)<<endl;
    /*if (r.at<float>(0,0)>threshold){//目标发生切换，可以结合id等判断；
        cout<<"target change222222222222222222222222222222222222222222"<<endl;
        setIdentity(KF->errorCovPost,cv::Scalar::all(1.0));
        KF->statePre=(Mat_<float>(6,1)<<pworld.x,0,pworld.y,0,pworld.z,0);
        //KF->predict();
        //return ;
    }*/
    Mat corrected_state=KF->correct(measurement);

    antitoper.Recode_xyz(corrected_state.at<float>(0,0),corrected_state.at<float>(2,0),corrected_state.at<float>(4,0));
    antitoper.Is_toping_WithLostframe();
    
    if (antitoper.is_toping){//小陀螺模式的预测；
        cv::Point3f p(corrected_state.at<float>(0,0),corrected_state.at<float>(2,0),corrected_state.at<float>(4,0));
        double other_predict_time=shoot_delay+SEARIAL_DELAY+dt;
        bullet.setbs_coeff(sqrt(pow(corrected_state.at<float>(0,0),2)+pow(corrected_state.at<float>(2,0),2))/1000.0);
        double T=bullet.getpredictflytime(bullet_speed,corrected_state.at<float>(1,0),corrected_state.at<float>(3,0),p,other_predict_time);
        //cout<<"before predict:"<<T<<endl;
        if (isnan(T)){
            cout<<"结算不出来预测前的子弹飞行时间----------------------------------------"<<endl;
        }  
        double predict_time=T+other_predict_time;//预测时间：
        //cout<<predict_time<<endl;
        cv::Point3f predict_world;
        predict_world.x=corrected_state.at<float>(0,0);
        predict_world.y=corrected_state.at<float>(2,0);
        predict_world.z=corrected_state.at<float>(4,0);

        Mat show_img=img.clone();
        Point3f Pcamera=pnpsolver.project_point_draw(show_img,antitoper.point_xyz_means,Scalar(0,0,255),q);//P_world);
        Point2f rectpoint[4];
        //cout<<"camera:"<<Pcamera<<endl;
        target_armor.rect.points(rectpoint);
        for (int i=0;i<4;i++){
            cv::line(show_img,rectpoint[i],rectpoint[(i+1)%4],Scalar(255,255,255),2);
        }
        //cv::circle(show_img,(rectpoint[0]+rectpoint[2])/2,3,cv::Scalar(0,0,255),3);
        resize(show_img,show_img,Size(400,400));
        namedWindow("predict",0);
        imshow("predict",show_img);
        waitKey(1);
        bullet.setbs_coeff(sqrt(pow(corrected_state.at<float>(0,0),2)+pow(corrected_state.at<float>(2,0),2))/1000.0);
        double elevation=bullet.compensate(bullet_speed,predict_world,T,corrected_state.at<float>(1,0),corrected_state.at<float>(3,0));
        cout<<"仰角:"<<elevation<<endl;
        if (isnan(elevation)){
            cout<<"结算不出来不补偿角度-----------------------------------------"<<endl;
        }
        //float yaw1=atan2(Pcamera.x,Pcamera.z)*180.0/CV_PI;//加10    
        float pitch1=atan2(Pcamera.y,Pcamera.z)*180.0/CV_PI;//加80.48,60,10
        float target_pitch=atan(predict_world.z/sqrt(predict_world.x*predict_world.x+predict_world.y*predict_world.y));
        pnpsolver.pitch=pitch1+target_pitch-elevation;
        cout<<"send angle:"<<pnpsolver.yaw<<" "<<pnpsolver.pitch<<endl;
        if (pnpsolver.yaw<0.5&&pnpsolver.pitch<0.8){
            shoot_code=FIND_SHOOT;
        }
    }
    else{//非小陀螺模式的预测；
        Point3f p(corrected_state.at<float>(0,0),corrected_state.at<float>(2,0),corrected_state.at<float>(4,0));
        double other_predict_time=shoot_delay+SEARIAL_DELAY+dt;
        bullet.setbs_coeff(sqrt(pow(corrected_state.at<float>(0,0),2)+pow(corrected_state.at<float>(2,0),2))/1000.0);
        double T=bullet.getpredictflytime(bullet_speed,corrected_state.at<float>(1,0),corrected_state.at<float>(3,0),p,other_predict_time);
        //cout<<"before predict:"<<T<<endl;
        if (isnan(T)){
            cout<<"结算不出来预测前的子弹飞行时间----------------------------------------"<<endl;
        }  
        double predict_time=T+other_predict_time;//预测时间：要使预测明显，可以设置为1；
        //cout<<predict_time<<endl;
        Point3f predict_world;
        predict_world.x=corrected_state.at<float>(0,0)+corrected_state.at<float>(1,0)*predict_time;
        predict_world.y=corrected_state.at<float>(2,0)+corrected_state.at<float>(3,0)*predict_time;
        predict_world.z=corrected_state.at<float>(4,0)+corrected_state.at<float>(5,0)*predict_time;
        Mat show_img=img.clone();
        Point3f Pcamera=pnpsolver.project_point_draw(show_img,predict_world,Scalar(0,0,255),q);//P_world);
        Point2f rectpoint[4];
        //cout<<"x:"<<corrected_state.at<float>(0,0)<<" "<<corrected_state.at<float>(2,0)<<" "<<corrected_state.at<float>(4,0)<<endl;
        //cout<<predict_time<<endl;
        //cout<<"delta:"<<corrected_state.at<float>(1,0)*predict_time<<" "<<corrected_state.at<float>(3,0)*predict_time<<" "<<corrected_state.at<float>(5,0)*predict_time<<endl;
        //cout<<"predict:"<<predict_world<<endl;
        //cout<<"camera:"<<Pcamera<<endl;
        target_armor.rect.points(rectpoint);
        for (int i=0;i<4;i++){
            cv::line(show_img,rectpoint[i],rectpoint[(i+1)%4],Scalar(255,255,255),2);
        }
        cv::circle(show_img,(rectpoint[0]+rectpoint[2])/2,3,cv::Scalar(0,0,255),3);
        resize(show_img,show_img,Size(400,400));
        namedWindow("predict",0);
        imshow("predict",show_img);
        waitKey(1);
        bullet.setbs_coeff(sqrt(pow(corrected_state.at<float>(0,0),2)+pow(corrected_state.at<float>(2,0),2))/1000.0);
        double elevation=bullet.compensate(bullet_speed,predict_world,T,corrected_state.at<float>(1,0),corrected_state.at<float>(3,0));
        //cout<<"elevation:"<<elevation<<endl;
        if (isnan(elevation)){
            cout<<"结算不出来不补偿角度-----------------------------------------"<<endl;
        }
        //cout<<"camera:"<<Pcamera<<endl;
        float pitch1=atan2(Pcamera.y,Pcamera.z)*180.0/CV_PI;//加80.48,60,10
        float target_pitch=atan(predict_world.z/sqrt(predict_world.x*predict_world.x+predict_world.y*predict_world.y))*180.0/CV_PI;
        //cout<<"pitch:"<<pitch1<<endl;
        //cout<<"before predict:"<<pnpsolver.yaw<<" "<<pnpsolver.pitch<<endl;
        //cout<<"target pitch:"<<target_pitch<<endl;
        //cout<<"world"<<predict_world<<endl;
        pnpsolver.pitch=pitch1+target_pitch-elevation;
        //pnpsolver.pitch=-elevation+pitch1;
        //cout<<"current position: "<<x(0)<<" "<<x(2)<<" "<<x(4)<<endl;
        //cout<<"send angle1:"<<pnpsolver.yaw<<" "<<pnpsolver.pitch<<endl;
        //cout<<"speed:"<<corrected_state.at<float>(1,0)<<" "<<corrected_state.at<float>(3,0)<<" "<<corrected_state.at<float>(5,0)<<endl;
        if (pnpsolver.yaw<0.5&&pnpsolver.pitch<0.8){
            shoot_code=FIND_SHOOT;
        }
    }
    # ifdef DEBUG_ANTITOP
        cout<<"is_top:"<<antitoper.is_toping<<endl;
        cout<<"top_info xyz"<<antitoper.point_xyz_means<<endl;
    # endif
}
/*
结构体赋值：使用auto关键字可以获得结构体中的数据成员，auto &[获取的变量的序列]=结构体；
  刚体的旋转：可以使用四元数来表示，但是他的运算较为复杂，需要使用Eigen库；
       Eigen中四元数可以用Eigen::Quaterniond(double类型)或者Eigen::Quaternionf(float类型)表示
       构造方法：Eigen::Quaterniond/f 对象名(const Scalar &w,const Scalar *x,const Scalar &y,const Scalar &z);
            w为四元数的实部，x,y,z都是四元数的虚部，但在内存中四个元素的顺序是x,y,z,w，可以通过访问对象名的数据成员.coeffs;
            w表示一个角度分量，x,y,z表示轴的矢量；
       使用函数matrix（）函数可以将四元数对象转化为矩阵；
  转化矩阵中元素的类型：调用成员函数.cast<type>()
*/