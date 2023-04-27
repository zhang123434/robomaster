#include"../../include/digital_classifier/classifier.hpp"
/*
待改进：数字识别区域的设置；
*/
Classifier::Classifier(string path){
    net=dnn::readNetFromONNX(path);//如果把onnx模型的文件放在和该文件相同的目录下，使用相对路径./digital_recognition.onnx来调用该文件，会报错，找不到Classifier类；
    CV_Assert(!this->net.empty());
    cout<<"load classifier param successfully"<<endl;
    //如果使用cuda加速，那么这两个函数的参数都应设置为CUDA;
    this->net.setPreferableBackend(dnn::DNN_BACKEND_CUDA);
    this->net.setPreferableTarget(dnn::DNN_TARGET_CUDA);
}

Mat Classifier::preprocess(Mat mat){
    cvtColor(mat,mat,COLOR_RGB2GRAY);
    mat=blobFromImage(mat,1,Size(16,16),Scalar(),false,true);
    mat=mat/255;
    mat=(mat-0.5)/0.5;
    return mat;
}

int Classifier::getid(Mat mat){
    mat=preprocess(mat);
    this->net.setInput(mat);
    Mat predict=this->net.forward();
    Point label_id;
    minMaxLoc(predict,0,0,0,&label_id);
    return label_id.x;
}