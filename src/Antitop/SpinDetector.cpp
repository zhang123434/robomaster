#include "../../include/Antitop/SpinDetector.h"

void SpinDetector::reset()
{
    armor_info_window.clear();
    spin_timestamp_list.clear();
    coord_info_window.clear();
    x_realworld.clear();
    y_realworld.clear();
    z_realworld.clear();

    is_spin = false;

    mean_point = cv::Point3f(0, 0, 0);
}

bool SpinDetector::judgeSpin(armor_t armor,  uint32_t timestamp,bool if_spin=false)
{
    if (!spin_timestamp_list.empty())
    {
        if (timestamp - spin_timestamp_list[spin_timestamp_list.size() - 1] > param.spin_timeout)
            reset();
    }

    armor_info_window.push_back(armor);
    if (armor_info_window.size() >= ARMOR_TO_JUDGE_LENGTH)
    {
        if (ifLegal() || if_spin)
        {
            spin_timestamp_list.push_back(timestamp);
            is_spin = spin_timestamp_list.size() > param.min_spin_count;
        }
    }

    if (armor_info_window.size() > ARMOR_WINDOW_LENGTH)
    {
        armor_info_window.erase(armor_info_window.begin());
    }

    return is_spin;
}

bool SpinDetector::ifLegal()
{
    armor_t ct1 = armor_info_window[armor_info_window.size() - 1];
    armor_t ct2 = armor_info_window[armor_info_window.size() - 2];
    cv::Point2f coord_delta = ct1.rect.center - ct2.rect.center;

    float y_delta = fabs(coord_delta.y);
    float x_delta = fabs(coord_delta.x);
    float light_height = ct1.rect_.height;

    return (x_delta / light_height > param.min_x_delta_ratio &&
            x_delta / light_height < param.max_x_delta_ratio &&
            y_delta / light_height > param.min_y_delta_ratio &&
            y_delta / light_height < param.max_y_delta_ratio);
}

bool SpinDetector::solverSpin(cv::Point3f &coord)
{
    if (armor_info_window.size() < ARMOR_WINDOW_LENGTH || !is_spin)
        return false;
    
    float mean_distance = 0;
    for (int i = 0; i < coord_info_window.size(); i++)
    {
        mean_distance += sqrt(coord_info_window[i].x * coord_info_window[i].x + coord_info_window[i].y + coord_info_window[i].y);
    }
    mean_distance /= (float)coord_info_window.size();

    coord = cv::Point3f(0, 0, 0);
    int count = 0;
    for (int i = 0; i < coord_info_window.size(); i++)
    {
        if (sqrt(coord_info_window[i].x * coord_info_window[i].x + coord_info_window[i].y + coord_info_window[i].y) < mean_distance);
        {
            count++;
            coord += coord_info_window[i];
        }
    }
    coord /= (float)count;
    //计算反小陀螺的中心点；
    if(is_spin==0){
        mean_point=cv::Point3f(0,0,0);
        x_realworld.clear();
        y_realworld.clear();
        z_realworld.clear();
    }
    else{
        float x_mean=0;
        float y_mean=0;
        float z_mean=0;
        if(x_realworld.size()>=9){
            for (int i=0;i<9;i++){//选取后面的最后的九个元素计算中心点的位置；
                y_mean+=*(y_realworld.end()-i);
                x_mean+=*(x_realworld.end()-i);
                z_mean+=*(z_realworld.end()-i);
            }
        }
        else{
            int size=x_realworld.size();
            for (int i=0;i<size;i++){
                y_mean+=y_realworld[i];
                x_mean+=x_realworld[i];
                z_mean+=z_realworld[i];
            }
        }
        mean_point=cv::Point3f(x_mean/9.0,y_mean/9.0,z_mean/9.0);
        while(x_realworld.size()>10){
            x_realworld.erase(x_realworld.begin());
            y_realworld.erase(y_realworld.begin());
            z_realworld.erase(z_realworld.begin());
        }
    }
    return true;
}
void SpinDetector::Recode_xyz(double x,double y,double z){
    num_frame+=1;
    if (num_frame==4){
        x_realworld.push_back(x);
        y_realworld.push_back(y);
        z_realworld.push_back(z);
        num_frame=0;
    }
}
void SpinDetector::pushCoord(cv::Point3f coord)
{
    coord_info_window.push_back(coord);
    if (coord_info_window.size() > ARMOR_WINDOW_LENGTH)
    {
        coord_info_window.erase(coord_info_window.begin());
    }
}

SpinDetector::SpinDetector()
{
    reset();
}
