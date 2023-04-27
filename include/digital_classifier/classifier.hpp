#include<opencv2/opencv.hpp>
using namespace cv;
using namespace cv::dnn;
#include<iostream>
using namespace std;
#pragma once//后面不用加分号；
/*
使用blobFromImage函数来对图像做预处理：
    该函数主要包括的过程：
            整体像素值减去平均值（mean）
            通过缩放系数（scalefactor）对图片像素值进行缩放
            可以将元素类型转化为float
            交换蓝色和红色的通道
            从中心调整图像的大小和裁剪图像
blobFromImage(InputArray image, 
			  double scalefactor=1.0, 
		      const Size& size = Size(),
			  const Scalar& mean = Scalar(), 
			  bool swapRB = false, 
			  bool crop = false,
			  int ddepth = CV_32F)
mean:如果输入有三个通道且不同通道分别减去不同的值，可用Scalar(a,b,c)
    如果三个通道减去的值a相同，可以用Scalar(a)
    如果图像具有bgr的通道顺序并且参数swapRB为true,这个向量中元素的顺序为Scalar(R,G,B)
scalefactor:是对像素值所做的放缩，设置为1/255:0.00392156f
size表示神经网络对输入的图像的尺寸（height,width）
swapRB:opencv中图像的通道的顺序是BGR,但是如果设计网络时按照RGB的顺序，将该参数设置为true,可以交换R和B通道
crop：表示是否裁剪图像
    如果设置为false:会直接调整大小，不做剪裁，并保留横纵比
    如果设置为true:调整输入的大小(调整时保留横纵比，使得较短的边等于参数size中的分量的大小),然后对中心做剪裁；
实际调用该函数时不用写前面的参数，直接传入参数值；
该函数返回的是一个四维的矩阵，而且尺寸为(batch,channel,height,width)
然后分为三步：
1.读入模型：cv::dnn.readNetFromONNX(string filepath),返回的是一个dnn::Net类的对象；
    判断网络是否为空：empty()
    设置网络在支持的地方使用特定的计算后端：Net.setPreferableBackend(dnn::DNN_BACKEND_OPENCV)
    设置网络做计算的目标设备：Net.setPreferableBackend()
        参数：cpu:dnn::DNN_TARGET_CPU
        gpu:dnn::DNN_TARGET_CUDA
2.输入模型：调用dnn::Net类的对象的成员函数setInput(Mat mat)
3.做前向传播：调用dnn.Net类的对象的成员函数forward()，返回的是一个Mat类的对象
    注意这里坐前向传播后需要使用max函数来获得类别标记；
opencv中函数CV_Assert(条件表达式)，如果条件表达式的值为False,会返回一个错误信息；
c++中的函数assert(表达式)，这里的表达是条件表达式或者表示数值的表达式，如果表达式的值为0，输出错误信息，并终止程序；

调试出现的问题：
1.如果把onnx模型的文件放在和该文件相同的目录下，使用相对路径./digital_recognition.onnx来调用该文件，会报错，找不到Classifier类；
    可以把该模型的文件放在该工程最高一级的目录中
    由于exe文件的位置在build文件夹中，访问最高级目录用../
    注意相对目录的使用
2.不要使用blobFromImage函数中的参数scalefactor来对图像矩阵做归一化,可以在调用该函数之后做归一化；
3.使用cuda加速需要重新编译opencv;
*/
class Classifier{
    public:

        Classifier(string path);//初始化数据成员mat，net；

        ~Classifier()=default;

        int getid(Mat mat);

    private:

        dnn::Net net;

        Mat preprocess(Mat mat);

};