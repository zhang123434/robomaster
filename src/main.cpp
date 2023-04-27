/*
 * @Author: error: git config user.name && git config user.email & please set dead value or install git
 * @Date: 2022-05-27 22:10:52
 * @LastEditors: error: git config user.name && git config user.email & please set dead value or install git
 * @LastEditTime: 2022-05-28 15:23:12
 * @FilePath: /infantry_current_5.26/src/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include"../include/Imageprocess/Imageprocess.hpp"
#include"../include/detector/detector.hpp"
/*
加入到当前文件中的头文件的相对路径使用目标头文件相对于当前文件的相对路径
cpp、hpp的代码内容设计到的相对路径:这些代码经过编译后在build文件夹的exe文件中，相对路径为目标文件相对于exe文件位置的相对路径，不是相对于当前文件
*/
#define WRITE_VIDEO
#include<thread>
int main(){
    CanSerial serial;
    //Pnp solver;
    ImageProcesspredict main_progress(0,0,1,serial);//注意颜色设置；
    //main_progress.get_eneny_color();
    thread produce(&ImageProcesspredict::get_image,&main_progress);
    #ifdef WRITE_VIDEO
        thread recode(&ImageProcesspredict::write_video,&main_progress);
    #endif
    thread consume(&ImageProcesspredict::process_image,&main_progress);
    thread readport(&ImageProcesspredict::receiveData,&main_progress);
    thread receive_bullet_speed(&ImageProcesspredict::receiveBS,&main_progress);
    thread sendport(&ImageProcesspredict::sendData,&main_progress);
    produce.join();
    #ifdef WRITE_VIDEO
        recode.join();
    #endif
    consume.join();
    readport.join();
    receive_bullet_speed.join();
    sendport.join();
    return 0;
}