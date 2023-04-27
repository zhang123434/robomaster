#include"../../include/detector/detector.hpp"

float Detector::getDistance(const cv::Point a, const cv::Point b) {
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}
Detector::Detector(string armor_config):classifier("../parameter/param/")
{
    FileStorage fs_armor(armor_config, cv::FileStorage::READ);
  // 初始化基本参数
    fs_armor["GRAY_EDIT"]          >> image_config_.gray_edit;
    fs_armor["COLOR_EDIT"]         >> image_config_.color_edit;

    fs_armor["BLUE_ARMOR_GRAY_TH"] >> image_config_.blue_armor_gray_th;
    fs_armor["RED_ARMOR_GRAY_TH"]  >> image_config_.red_armor_gray_th;
    fs_armor["RED_ARMOR_COLOR_TH"]   >> image_config_.red_armor_color_th;
    fs_armor["BLUE_ARMOR_COLOR_TH"]  >> image_config_.blue_armor_color_th;

    fs_armor["LIGHT_DRAW"]             >> light_config_.light_draw;
    fs_armor["LIGHT_EDTI"]             >> light_config_.light_edit;

    fs_armor["LIGHT_RATIO_W_H_MIN"]    >> light_config_.ratio_w_h_min;
    fs_armor["LIGHT_RATIO_W_H_MAX"]    >> light_config_.ratio_w_h_max;
    fs_armor["LIGHT_ANGLE_MIN"]        >> light_config_.angle_min;
    fs_armor["LIGHT_ANGLE_MAX"]        >> light_config_.angle_max;
    fs_armor["LIGHT_PERIMETER_MIN"]    >> light_config_.perimeter_min;
    fs_armor["LIGHT_PERIMETER_MAX"]    >> light_config_.perimeter_max;
    fs_armor["ARMOR_EDIT"]             >> armor_config_.armor_edit;
    fs_armor["ARMOR_DRAW"]             >> armor_config_.armor_draw;
    fs_armor["ARMOR_FORECAST"]         >> armor_config_.armor_forecast;
    fs_armor["ARMOR_HEIGHT_RATIO_MIN"] >> armor_config_.light_height_ratio_min;
    fs_armor["ARMOR_HEIGHT_RATIO_MAX"] >> armor_config_.light_height_ratio_max;
    fs_armor["ARMOR_WIDTH_RATIO_MIN"]  >> armor_config_.light_width_ratio_min;
    fs_armor["ARMOR_WIDTH_RATIO_MAX"]  >> armor_config_.light_width_ratio_max;
    fs_armor["ARMOR_Y_DIFFERENT"]      >> armor_config_.light_y_different;
    fs_armor["ARMOR_HEIGHT_DIFFERENT"] >> armor_config_.light_height_different;
    fs_armor["ARMOR_ANGLE_DIFFERENT"]  >> armor_config_.armor_angle_different;
    fs_armor["ARMOR_SMALL_ASPECT_MIN"] >> armor_config_.small_armor_aspect_min;
    fs_armor["ARMOR_TYPE_TH"]          >> armor_config_.armor_type_th;
    fs_armor["ARMOR_BIG_ASPECT_MAX"]   >> armor_config_.big_armor_aspect_max;
    cout<<"load detector param successfully"<<endl;
}
Mat Detector::preprocess(Mat src,int color){//color=0:蓝色灯条 1：红色灯条 2：两种颜色的灯条;
//1.灰度化之后直接做阈值化；
    Mat temp;
    cvtColor(src,temp,COLOR_BGR2GRAY);
    if (color==0){
        threshold(temp,thresh_gray_img,image_config_.blue_armor_gray_th,255,THRESH_BINARY);
    }
    else if (color==1){
        threshold(temp,thresh_gray_img,image_config_.red_armor_gray_th,255,THRESH_BINARY);
    }
    if(image_config_.gray_edit==1){
        namedWindow("thresh_gray_img",0);
        imshow("thresh_gray_img",thresh_gray_img);
        waitKey(1);
    }
//bgr_thresh_img;    
    vector<Mat> channels;
    split(src,channels);
    if (color==0){
        blue_red_img=channels[0]-channels[2];
        if (image_config_.gray_edit==1){
            namedWindow("blue-red",0);
            imshow("blue-red",blue_red_img);
        }
        threshold(blue_red_img,bgr_thresh_img,image_config_.blue_armor_color_th,255,THRESH_BINARY);
        if (image_config_.gray_edit==1){
            namedWindow("blue-red_thresh",0);
            imshow("blue-red_thresh",bgr_thresh_img);
        } 
    }
    else if (color==1){
        red_blue_img=channels[2]-channels[0];
        threshold(red_blue_img,bgr_thresh_img,image_config_.red_armor_color_th,255,THRESH_BINARY);
        if (image_config_.gray_edit==1){
            namedWindow("blue-red_thresh",0);
            imshow("blue-red_thresh",bgr_thresh_img);
        }
    }
//综合两种处理：
    bitwise_and(bgr_thresh_img,thresh_gray_img,final_gray);
//作膨胀操作；
    auto element=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
    morphologyEx(final_gray,final_gray,MORPH_DILATE,element);
    //不要使用中值滤波，因为他可能把灯条周围的亮斑和灯条融合，导致灯条的轮廓的旋转矩形出现较大偏差；medianBlur(final_gray,final_gray,5);
    if (image_config_.color_edit==1){
        namedWindow("preprocess_image_final",0);
        //resize(final_gray,final_gray,Size(400,400));
        imshow("preprocess_image_final",final_gray);
        
        //waitKey(1);
    }
    return final_gray;
}
bool Detector::findlight(Mat f_gray){
    int perimeter = 0;
    RotatedRect box;
    vector<std::vector<cv::Point>> contours;
    cv::findContours(f_gray,
                    contours,
                    cv::RETR_EXTERNAL,//只检测最外层的轮廓
                    cv::CHAIN_APPROX_NONE);

    if (contours.size() < 2) {
        cout<<"error11:"<<contours.size()<<endl;
        return false;
    }
    
    for (size_t i = 0; i != contours.size(); ++i ){
        perimeter = arcLength(contours[i], true);
//perimeter > light_config_.perimeter_max ||
        if (perimeter < light_config_.perimeter_min ||
            contours[i].size() < 5) {
        continue;
        }

        box = cv::fitEllipse(cv::Mat(contours[i]));

        if (box.angle > 90.0f) {
            box.angle = box.angle - 180.0f;
        }
//获得灯条的长度是拟合得到的椭圆的外接旋转矩形的宽度和长度的较大值，灯条的宽度是尺寸中的较小值
        static float _h        = MAX(box.size.width, box.size.height);
        static float _w        = MIN(box.size.width, box.size.height);
        static float light_w_h = _h / _w;
        // 判断灯条的条件
        if (box.angle < light_config_.angle_max     &&
            box.angle > -light_config_.angle_min    &&
            light_w_h < light_config_.ratio_w_h_max &&
            light_w_h > light_config_.ratio_w_h_min) {
            light_set.emplace_back(box);
            if (light_config_.light_draw == 1 || light_config_.light_edit == 1) {
                Point2f vertex[4];
                box.points(vertex);
                for (size_t l = 0; l != 4; ++l) {
                cv::line(draw_img_,
                        vertex[l], vertex[(l + 1) % 4],
                        cv::Scalar(0, 255, 255), 3, 8);
                }
            }
        }
    }
    if (light_set.size() < 2) {
        return false;
    }
    return true;
}
bool Detector::findarmor(){
  for (int i=0;i<light_set.size();i++){
      for (int j=0;j<light_set.size();j++){//这种遍历方法会导致同一对灯条被加入到armor_finded中两次；？？？
        int left=0,right=0;
        if (light_set[i].center.x<light_set[j].center.x){
            left=i;
            right=j;
        }
        else{
            left=j;
            right=i;
        }
        armor_data_.left_light=light_set[left];
        armor_data_.right_light=light_set[right];
        float angle1=atan((light_set[right].center.y-light_set[left].center.y)/(light_set[right].center.x-light_set[left].center.x));
        if (fabs(angle1)<CV_PI/4.0){
            angle1=angle1*180/CV_PI;
            armor_data_.tan_angle=angle1;
        }
        if (lightmatch(left,right)){
            cv::RotatedRect rect;
            int id=getid();
            armor_set.push_back(armor_data_);
            armor_finded.emplace_back(armor_data_.armor_rect,armor_data_.distinguish,id);
            if (armor_config_.armor_draw == 1 ||
                armor_config_.armor_edit == 1) {//这里是绘制出识别到的所有的装甲板；
                rectangle(draw_img_, armor_data_.armor_rect.boundingRect(),
                cv::Scalar(255,255, 255), 5, 8);
            }
        }
      }
  }
  if (armor_set.size() < 1) {
    return false;
  }
//cout<<armor_set.size()<<endl;
  return true;
}
bool Detector::lightmatch(int i, int j){
    armor_data_.left_light_height   =
      MAX(light_set[i].size.height, light_set[i].size.width);//height为灯条的长，width为灯条的宽
  armor_data_.left_light_width    =
      MIN(light_set[i].size.height, light_set[i].size.width);
  armor_data_.right_light_height  =
      MAX(light_set[j].size.height, light_set[j].size.width);
  armor_data_.right_light_width   =
      MIN(light_set[j].size.height, light_set[j].size.width);
  armor_data_.light_height_aspect =
      armor_data_.left_light_height / armor_data_.right_light_height;
  /*armor_data_.light_width_aspect  =
      armor_data_.left_light_width / armor_data_.right_light_width;*/
  // 左右灯条高宽比
  if (armor_data_.light_height_aspect < armor_config_.light_height_ratio_max * 0.1 &&
      armor_data_.light_height_aspect > armor_config_.light_height_ratio_min * 0.1 ) {
    //cout<<11<<endl;
    armor_data_.height =
      MIN(armor_data_.left_light.size.height, armor_data_.right_light.size.height);
    // 灯条 y 轴位置差
    if (fabs(armor_data_.left_light.center.y - armor_data_.right_light.center.y) <
        armor_data_.height * armor_config_.light_y_different * 0.1) {
      // 灯条高度差
      //cout<<22<<endl;
      if (fabs(armor_data_.left_light.size.height - armor_data_.right_light.size.height) <
          armor_data_.height * armor_config_.light_height_different * 0.1) {
        //cout<<33<<endl;
        armor_data_.width =
          getDistance(armor_data_.left_light.center,
                      armor_data_.right_light.center);
        armor_data_.aspect_ratio = armor_data_.width / MAX(armor_data_.left_light_height, armor_data_.right_light_height);
        // 灯条角度差
        if (fabs(armor_data_.left_light.angle - armor_data_.right_light.angle) <
            armor_config_.armor_angle_different * 0.1) {
        //cout<<44<<endl;
          cv::RotatedRect rects = cv::RotatedRect(
              (armor_data_.left_light.center + armor_data_.right_light.center)/2,
              cv::Size(armor_data_.width , armor_data_.height),
              armor_data_.tan_angle);
            /*？？？旋转矩形的尺寸的设置
            cv::RotatedRect rects = cv::RotatedRect(
              (armor_data_.left_light.center + armor_data_.right_light.center) / 2,
              cv::Size(armor_data_.width * 0.5 , armor_data_.height * 0.5 + 100),
              armor_data_.tan_angle);
            */
          armor_data_.armor_rect      = rects;
          /*装甲板保存灯条离中心点的距离
          armor_data_.distance_center =
              getDistance(armor_data_.armor_rect.center,
                          cv::Point(draw_img_.cols, draw_img_.rows));*/
          // 小装甲板比例范围
          //cout<<"fail to judge the size"<<endl;
          //cout<<armor_data_.aspect_ratio<<endl;
          //cout<<armor_data_.width<<endl;
          if (armor_data_.aspect_ratio > armor_config_.small_armor_aspect_min * 0.1 && armor_data_.aspect_ratio < armor_config_.armor_type_th * 0.1) {
            armor_data_.distinguish = 0;
            return true;
          // 大装甲板比例范围
          } else if (armor_data_.aspect_ratio > armor_config_.armor_type_th * 0.1 && armor_data_.aspect_ratio < armor_config_.big_armor_aspect_max * 0.1) {
            armor_data_.distinguish = 1;
            return true;
          }
        }
      }
    }
  }
  //cout<<"error"<<endl;
  return false;
}
int Detector::getid(){
    /*   armor_data_.left_light_height  =
        MAX(armor_data_.left_light_height, armor_data_.left_light_width);
    armor_data_.left_light_width   =
        MIN(armor_data_.left_light_height, armor_data_.left_light_width);
    armor_data_.right_light_height =
        MAX(armor_data_.right_light_height, armor_data_.right_light_width);
    armor_data_.right_light_width  =
        MIN(armor_data_.right_light_height, armor_data_.right_light_width);

    cv::RotatedRect rects =
        cv::RotatedRect((armor_data_.left_light.center + armor_data_.right_light.center) / 2,
        cv::Size(armor_data_.width - (armor_data_.left_light_width + armor_data_.right_light_width),
            ((armor_data_.left_light_height + armor_data_.right_light_height) / 2)),
        armor_data_.tan_angle);*/
    //cv::rectangle(draw_img_, rects.boundingRect(), cv::Scalar(255, 0, 0), 3, 8);

    //armor_data_.armor_rect = rects;
    cv::Rect rect_  = armor_data_.armor_rect.boundingRect();
    // ROI 安全条件
    int flag=0;
    if (rect_.x <= 0) {
        rect_.x = 0;
        flag++;
    }
    if (rect_.y <= 0) {
        rect_.y = 0;
        flag++;
    }
    if (rect_.y + rect_.height >=thresh_gray_img.rows) {
        rect_.height = thresh_gray_img.rows - rect_.y;
        flag++;
    }
    if (rect_.x + rect_.width >= thresh_gray_img.cols) {
        rect_.width = thresh_gray_img.cols - rect_.x;
        flag++;
    }
    // 计算颜色平均强度
    static cv::Mat roi1    = thresh_gray_img(rect_);
    int average_intensity = static_cast<int>(mean(roi1).val[0]);
    /*cout<<average_intensity<<endl;觉得没有必要设置亮度限制
    if (average_intensity<10){
        return 0;
    }*/
    rect_.y-=rect_.height/2.0*1.2;
    rect_.height*=2.0*1.2;
    rect_&=Rect(Point(0.,0.),src_image.size());
    Mat temp=src_image(Rect2d(rect_));
    Gamma(temp,temp,0.6);
    resize(temp,temp,Size(28,28),cv::INTER_LINEAR);
    cvtColor(temp,temp,COLOR_BGR2GRAY);
    //识别不到返回的是-1；
    return classifier(temp);
}


void Detector::Gamma(Mat src, Mat &dst, float fGamma)
{
    unsigned char lut[256];

    for (int i = 0; i < 256; i++)
    {
        float normalize = (float)(i/255.0);
        lut[i] = saturate_cast<uchar>(pow(normalize, fGamma) * 255.0f);
    }

    src.copyTo(dst);
    MatIterator_<Vec3b> it,end;
    for (it = dst.begin<Vec3b>(), end = dst.end<Vec3b>(); it != end; it++)
    {
        (*it)[0] = lut[((*it)[0])];
        (*it)[1] = lut[((*it)[1])];
        (*it)[2] = lut[((*it)[2])];
    }
    
}