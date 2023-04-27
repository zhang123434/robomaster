#include"../../include/Imageprocess/Imageprocess.hpp"
//#define CONFIG 
ImageProcesspredict::ImageProcesspredict(int accepted_num,
                                        int processed_num,
                                        int color,
                                        CanSerial &serial
                                        ):aimer(color,serial),
                                        accepted_number(accepted_num),
                                        processed_number(processed_num),
                                        BUFFER_LENGTH(3){
    cv::FileStorage fs("../parameter/debug.xml",cv::FileStorage::READ);
    cout<<"load debug parameter successfully"<<endl;
    if (!fs.isOpened()){
        cout<<"load debug parameter failed"<<endl;
    }
    fs["show_predict_photo"]>>aimer.SHOW_PREDICT_PHOTO;
    fs["angle_CONFIG"]>>ANGLE_CONFIG;
    fs["serial_DEBUG"]>>SERIAL_DEBUG;
    fs["WRITE_VIDEO"]>>WRITE_VIDEO;
    fs["send_DEBUG"]>>SEND_DEBUG;
    fs["COMPUTE_PROCESS_time"]>>COMPUTE_PROCESS_TIME;
    fs["predict_DEBUG"]>>predict_DEBUG;
    fs["SHOW_ROI"]>>aimer.SHOW_ROI;
    fs["show_predict_info"]>>aimer.SHOW_PREDICT_INFO;
    fs["autoaim_ANTITOP"]>>aimer.DEBUG_ANTITOP;
    fs["autoaim_DETECT_CONFIG"]>>aimer.DETECT_CONFIG;
    fs.release();
    send_time=0;
    pose_timestamp=0;
    string outpath="/home/nuc/Desktop/infantry_current_6.6/parameter/1.avi";
    writer=VideoWriter(outpath,VideoWriter::fourcc('M','J','P','G'),30,Size(IMAGE_WIDTH,IMAGE_HEIGHT));
}


String ImageProcesspredict::getVideoName(){//用录制时的时间来给视频命名；
    time_t t=time(NULL);
    char ch[22];
    strftime(ch,22,"%F-%H-%M-%S",localtime(&t));
    return "/home/nuc/Desktop/infantry_current_5.26/parameter/1.avi";
}


void ImageProcesspredict::get_image(){
    camera::MercureDriver cap;
    //cv::VideoCapture cap("/home/zzh/Desktop/infantry_current/infantry_current_5.26/BLUE2.avi");
    /*VideoWriter writer;
    if(WRITE_VIDEO==1){
        string outpath;
        outpath=getVideoName();
        writer=VideoWriter(outpath,VideoWriter::fourcc('M','J','P','G'),50,Size(IMAGE_WIDTH,IMAGE_HEIGHT));
    }
    Mat frame;*/
    Mat_t src;
    while(true){
        frame.create(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC3);
        //while (accepted_number-processed_number>=BUFFER_LENGTH)
        //***为了避免接受到帧过多来不及处理，当接受到的帧数减去处理过的帧数大于等于3，就休眠4ms，这段时间不在获取帧；
            //std::this_thread::sleep_for(std::chrono::milliseconds(4));

        //***获取图像和图像对应的时间戳时需要使用线程锁
        image_lock.lock();
        //gettickcount返回的时间的单位是秒，由于在后面的插值中图像的时间戳需要和电控发来的时间戳进行比较，需要将s,转化为ms;
        u_int32_t timestamp=(u_int32_t)((getTickCount()-start_time)/getTickFrequency()*1000);
        frame_mutex.lock();
        cap>>frame;
        frame_mutex.unlock();
        //imwrite("/home/nuc/Desktop/infantry_current_6.7/parameter/1.png",frame);
        image_lock.unlock();
        /*#ifdef IMAGEPROCESS_CONFIG
            namedWindow("photo",0);
            imshow("photo",frame);
            int c=waitKey(1);
            if (char(c)==27){
                break;
            }
        #endif*/
        //判断图像是否为空:用empty（）函数
        if (frame.empty()==true){
            //cout<<"image is empty---------------------------"<<endl;
            continue;
        }

        //设置图像的结构体：
        src.mat_timestamp=timestamp;
        src.image=frame.clone();
        gyro_mutex.lock();
        src.pose=newest_pose;
        gyro_mutex.unlock();
        image_queue.push(src);
        accepted_number++;
        if (WRITE_VIDEO==1)
            writer<<frame;
        
    }
}

void ImageProcesspredict::write_video(){//应该加上锁;
    frame_mutex.lock();
    writer<<frame;
    frame_mutex.unlock();
}


void ImageProcesspredict::sendData(){
    ArmorState last_shoot_code=NO_FIND_NO_SHOOT;
    int state;
    chrono::steady_clock::time_point last_send_time=chrono::steady_clock::now();
    while(1){//注意这里是个循环；
        //SendData send_data;
        //用于计数状态：NO_END_SOLVER状态发生的次数；
        int idex=0;
        //***访问ImageProcesspredict类的自瞄类成员时需要使用线程锁
        //image_lock.lock();//不加锁;
        //aimer.shoot_code=NO_END_SOLVER;
        int send_id;
        ArmorState send_state;
        if (aimer.target_armor.rect_!=cv::Rect2f()){
            send_state=FIND_NO_SHOOT;
            //完善了开火逻辑;
            if (aimer.pnpsolver.distance>=6.5){
                send_state=FIND_NO_SHOOT;
            } 
            else {
                if (fabs(send_pitch_yaw.y)<0.5&&fabs(send_pitch_yaw.x)<0.8)
                    send_state=FIND_SHOOT;
            }
        }
        else{
            send_state=NO_FIND_NO_SHOOT;
        }
        send_id=aimer.target_armor.id;
        if (SERIAL_DEBUG==1){
            //cout<<"send state:"<<send_state<<endl;
            //cout<<send_id<<endl;
            cout<<"send time:"<<send_time<<endl;
            //cout<<"send pitch: "<<send_pitch_yaw.x<<" yaw:"<<-1.0*send_pitch_yaw.y<<endl;
            //aimer.send_mutex.lock();
        }
        if (fabs(send_pitch_yaw.x)>15||fabs(send_pitch_yaw.y)>20){
            state=state=aimer.serial.send_Data(0.,0.,send_time);
        }
        else
            state=aimer.serial.send_Data(1.0*send_pitch_yaw.x,-1.0*send_pitch_yaw.y,send_time);
        if (state!=TIME_ERROR&&SEND_DEBUG){
            uint32_t temp=(uint32_t)(double(getTickCount())-start_time)/getTickFrequency()*1000;
            //cout<<"send frequency:"<<chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now()-last_send_time).count()<<" us"<<endl;
            last_send_time=chrono::steady_clock::now();
        }
        if (state==4||state==2||state==1){
            //cout<<"send failed"<<endl;
        }
        aimer.serial.send_state(send_state,send_id);
        this_thread::sleep_for(chrono::microseconds(200));
    }
}

//接受数据时候可能需要同步时间戳；
void ImageProcesspredict::receiveData(){
    start_time=getTickCount();
    uint32_t timestamp=0;
    uint receive_id=0;
    u_char buf[8]={0};
    u_char dlc=0;
    CanError s;
    u_char imu[8]={0};
    //先接受到receive_id较小处发送的数据；
    while(1){
        double time1;
        if(COMPUTE_PROCESS_TIME==1)
            time1=cv::getTickCount();
        int receiveflag[5]={0};
        ReceiveData receive_data_;
        while(receiveflag[0]!=1||receiveflag[1]!=1){//||receiveflag[3]!=1){//||receiveflag[4]!=1){//,接受不到第四个和第五个id发送的数据；
            s=(CanError)aimer.serial.can_receive(receive_id,buf,dlc);
            if (receive_id==RECEIVE_TIME&&s==0){
                receiveflag[0]=1;
                uint32_t t=0;
                for (int i = 0; i < 4; i++)
                {
                    t |=  ((u_int32_t)buf[i] << (i * 8));
                }
                timestamp=t;//发送的时间的单位是ms为单位
                //cout<<0<<endl;
            }
            else if (receive_id==IMU_RECEIVE_ID_2&&s==0){
                receiveflag[1]=1;
                for (int i=0;i<8;i++){
                    imu[i]=buf[i];
                }
                //cout<<1<<endl;
            }
            else if(receive_id==ROBOT_RECEIVE_ID&&s==0){
                //receiveflag[2]=1;
                int id=(uint8_t)(buf[0]);
                if(id<=9){
                    aimer.enemy_color=0;
                }
                else{
                    aimer.enemy_color=1;
                }
            }

        }
        if (SERIAL_DEBUG==1){
            cout<<"receive time:"<<timestamp<<" "<<"color:"<<aimer.enemy_color<<" "<<"speed:"<<aimer.bullet_speed<<endl;
            /*float q[4]={0.0};
            memcpy(q,imu,sizeof(u_char)*16);
            cout<<"q:"<<q[0]<<" "<<q[1]<<" "<<q[2]<<" "<<q[3]<<endl;*/
        }
        //必须接受到发送的时间和四元数才能做下面的过程;
        if (timestamp!=0){
            short int q[4]={0};
            memcpy(q,imu,8);
            //将四个四元数除以32767,然后在进行一次归一化，然后再使用;
            gyro_mutex.lock();
            Eigen::Quaterniond q1={double(q[0]),double(q[1]),double(q[2]),double(q[3])};
            q1.normalize();
            newest_pose.q[0]=float(q1.w());
            newest_pose.q[1]=float(q1.x());
            newest_pose.q[2]=float(q1.y());
            newest_pose.q[3]=float(q1.z());
            newest_pose.timestamp=timestamp;
            gyro_mutex.unlock();
            //cout<<"pose size:"<<history_quaternion.size()<<endl;
            if(SERIAL_DEBUG==1){
                cout<<"received q:"<<newest_pose.q[0]<<" "<<newest_pose.q[1]<<" "<<newest_pose.q[2]<<" "<<newest_pose.q[3]<<endl;
            }
            //history_quaternion.emplace_back(newest_pose);
        }
        if(COMPUTE_PROCESS_TIME==1){
            double time2=(double(getTickCount())-time1)/cv::getTickFrequency()*1000.0;
            cout<<"接受使用的时间: "<<time2<<" ms"<<endl;
            /*if (time2>10){
                cout<<"处理时间过长------------------------------------"<<endl;
            }*/
        }
        this_thread::sleep_for(chrono::microseconds(400));//接受的不够快，可以减少休眠的时间；
    }
}

void ImageProcesspredict::process_image(){
    
    Mat frame;
    while (true){
        double time1;
        if(COMPUTE_PROCESS_TIME==1)
            time1=cv::getTickCount();
    
        //当接受到的帧数小于等于处理过的帧数时，等待至接受的帧数大于处理过大的帧数再做处理；
        while (accepted_number<=processed_number){
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
        //获取图像队列中的元素需要使用线程锁
        //image_lock.lock();
        Mat_t src=image_queue.front();
        image_queue.pop();
        processed_number++;
        //cout<<"timestamp:"<<src.mat_timestamp<<endl;
        //image_lock.unlock();
        //寻找目标装甲板：
        //cout<<"图像的时间戳: "<<src.pose.timestamp<<endl;
        aimer.run(src.image,src.mat_timestamp);
        //cout<<aimer.target_armor.id<<endl;
        
        //double time3=(getTickCount()-time1)/cv::getTickFrequency()*1000.0;
        //cout<<"自瞄的时间:"<<time3<<" ms"<<endl;
        #ifdef CONFIG//roi区域中的target_armor在原始图像中对应的位置有偏移量
            Mat draw_img=src.image.clone();
            if(aimer.target_armor.rect_!=cv::Rect2f()){
                //cout<<"find target armor"<<endl;
                Point2f temp[4];
                aimer.target_armor.rect.points(temp);
                for (int i=0;i<4;i++){
                    cv::line(draw_img,temp[i],temp[(i+1)%4],cv::Scalar(255,255,255),2,8);
                }
                //cout<<aimer.last_armor.rect.center<<endl;
                //cout<<"last"<<(aimer.last_armor.rect_.tl()+aimer.last_armor.rect_.br())/2<<endl;
                //cv::rectangle(draw_img,aimer.target_armor.rect_,cv::Scalar(255,255,255),5,8);
            }
            else{
                //cout<<"can't find target armor----------------------"<<endl;
            }
            resize(draw_img,draw_img,Size(400,400));
            namedWindow("aim",0);
            imshow("aim",draw_img);
            waitKey(1);
        #endif
        if (ANGLE_CONFIG==1)
            cout<<"pnp:"<<aimer.pnpsolver.yaw<<" "<<aimer.pnpsolver.pitch<<" "<<aimer.pnpsolver.distance<<endl;
        //cout<<"pnp: "<<aimer.pnpsolver.pcamera_<<endl;
        if (aimer.current_is_find_target==true){
            //interpolate(src);
            //aimer.shoot_code=FIND_NO_SHOOT;
            //aimer.antitoper.Recode_lost_angle((aimer.target_armor.left_light_angle+aimer.target_armor.right_light_angle)*0.5,aimer.current_is_find_target,aimer.current_is_find_in_big_roi,src.mat_timestamp);
            //double time=(getTickCount()-start_time)/getTickFrequency()-src.mat_timestamp;
            if (predict_DEBUG==1){
                /*src.pose.q[0]=0.9711;
                src.pose.q[1]=-0.02814;
                src.pose.q[2]=0.0064;
                src.pose.q[3]=0.2296;*/
                Point2f pitch_yaw=aimer.getpredict_angle(src.pose.timestamp,src.pose.q,src.image);
                send_mutex.lock();
                send_pitch_yaw.x=pitch_yaw.x;
                send_pitch_yaw.y=pitch_yaw.y;
                send_time=src.pose.timestamp;
                send_mutex.unlock();
            }
            else if (predict_DEBUG==0){
                send_mutex.lock();
                send_pitch_yaw.x=-1.0*aimer.pnpsolver.pitch;
                send_pitch_yaw.y=aimer.pnpsolver.yaw;
                send_time=src.pose.timestamp;
                cout<<"send angle："<<aimer.pnpsolver.pitch<<" yaw: "<<aimer.pnpsolver.yaw<<endl;
                cout<<aimer.pnpsolver.pcamera_<<endl;
                send_mutex.unlock();
            }
        }
        aimer.is_target_change=false;
        aimer.last_is_find_target=aimer.current_is_find_target;
        if(COMPUTE_PROCESS_TIME==1){
            double time2=(double(getTickCount())-time1)/cv::getTickFrequency()*1000.0;
            cout<<"帧率:"<<time2<<" ms"<<endl;
            /*if (time2>10){
                cout<<"处理时间过长------------------------------------"<<endl;
            }*/
        }
    }
}

void ImageProcesspredict::receiveBS(){
    uint receive_id=0;
    u_char buf[8]={0};
    u_char dlc=0;
    int s;
    while (true){
        s=aimer.serial.can_receive(receive_id,buf,dlc);
        if (receive_id==BS_RECEIVE_ID&&s==0){
            aimer.bullet_speed=static_cast<uint16_t>(buf[4]|(buf[5]<<8));
        }
        else{
            aimer.bullet_speed=15;
        }
    }
}
/*void Imageprocess::receiveIMU(){

}*/
//插值的问题：插值是离散函数逼近的重要方法，假定区间[a,b]上的实值函数f(x)在该区间上 n+1个互不相同点x0,x1,……,xn 处的值是f (x0),……f(xn)，要求估算f(x)在[a,b]中某点x*的值。
//插值的基本思路：找到一个函数P(x)，在x0,x1,……,xn的节点上与f(x)函数值相同(有时，甚至一阶导数值也相同)，用P(x*)的值作为函数f(x*)的近似
//四元数的插值使用线性插值和球面线性插值的结合；
bool ImageProcesspredict::interpolate(Mat_t &src){
    u_int32_t image_time=src.mat_timestamp;
    cout<<"图像的时间戳:"<<src.mat_timestamp<<endl;
    cout<<"pose number:"<<history_quaternion.size()<<endl;
    cout<<"newest_pose:"<<newest_pose.timestamp<<endl;
    int num=history_quaternion.size();
    if (image_time<0||num<=5){
        //cout<<"error1"<<endl;
        return false;
    }
    GimblaPose start,end;
//寻找和图像生成的时刻前最接近的两个时刻的姿态来估计
    int flag = 0;
    for (int j = 1; j < num; j++)
    {
        if (image_time < history_quaternion.at(j).timestamp&& image_time>history_quaternion.at(j-1).timestamp)
        {
            flag = j;
            break;
        }
    }
    //cout<<flag<<endl;
    if(flag>=1&&flag<=num-1){
        //cout<<"find q:---------------------------"<<endl;
        //image_lock.lock();
        start=history_quaternion.at(flag-1);
        end=history_quaternion.at(flag);
        //image_lock.unlock();
        //这两个数据之间的时间差不能太大
        /*if(end.timestamp-start.timestamp<=0||end.timestamp-start.timestamp>100){//两个时间戳相差20ms;
            //cout<<"error2"<<endl;
            return false;
        }*/
    }
    else{
        //cout<<"error3"<<endl;
        return false;
    }
    double t=1.0*(image_time-start.timestamp)/(end.timestamp-start.timestamp);
    float cosa=start.q[0]*end.q[0]+start.q[1]*end.q[1]+start.q[2]*end.q[2]+start.q[3]*end.q[3];
    if (cosa<0.0f){
        end.q[0]=-end.q[0];
        end.q[1]=-end.q[1];
        end.q[2]=-end.q[2];
        end.q[3]=-end.q[3];
        cosa=-cosa;
    }
    float k0,k1;
    if (cosa>0.9995f){
        k0=1.0f-t;
        k1=t;
    }
    else{
        float sina=sqrt(1.0f-cosa*cosa);
        float a=atan2(sina,cosa);
        k0=sin((1.0f-t)*a)/sina;
        k1=sin(t*a)/sina;
    }
    src.pose.q[0]=start.q[0]*k0+end.q[0]*k1;
    src.pose.q[1]=start.q[1]*k0+end.q[1]*k1;
    src.pose.q[2]=start.q[2]*k0+end.q[2]*k1;
    src.pose.q[3]=start.q[3]*k0+end.q[3]*k1;
    cout<<"插值结果"<<src.pose.q[0]<<" "<<src.pose.q[1]<<" "<<src.pose.q[2]<<" "<<src.pose.q[3]<<endl;
//更新云台历史数据
    //image_lock.lock();
    /*vector<GimblaPose>::iterator iter=history_quaternion.begin();
    //当容器中首尾的数据的时间相差过大且容器中的元素过多时，删除队列中最靠前的数据；
    while(history_quaternion.at(history_quaternion.size()-1).timestamp-history_quaternion.at(0).timestamp>100&&history_quaternion.size()>=15){
        history_quaternion.erase(iter);
        iter=history_quaternion.begin();
    }*/
    if (flag>10){
        for (int i=0;i<flag-2;i++){
            history_quaternion.erase(history_quaternion.begin());
        }
    }
    //image_lock.unlock();

    return true;
}


void ImageProcesspredict::get_eneny_color(){
    uint receive_id=0;
    u_char buf[8]={0};
    u_char dlc=0;
    CanError s;
    while(1){
        s=(CanError)aimer.serial.can_receive(receive_id,buf,dlc);
        if(s==0&&receive_id==ROBOT_RECEIVE_ID){
            int id=(uint8_t)(buf[0]);
            if(id<=9){
                aimer.enemy_color=0;
            }
            else{
                aimer.enemy_color=1;
            }
            cout<<"读取到敌方的装甲板的颜色:"<<aimer.enemy_color<<endl;
            break;
        }
    }
}