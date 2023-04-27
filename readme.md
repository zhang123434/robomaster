执行程序：先删掉build文件夹，然后mkdir build,cd build,cmake .. make -j9

    1.bash can.sh

    2.bash watchdog.sh

    启动的时候会读取裁判系统，看敌方颜色是否正确

主要调试的参数化：

1.parameter/mercure.xml:调整相机的参数：增益和曝光时间

2.当预测图像中框出的装甲板不好，可以调整parameter/detect.xml文件中的
<RED_ARMOR_GRAY_TH>147.5</RED_ARMOR_GRAY_TH>
<RED_ARMOR_COLOR_TH>20</RED_ARMOR_COLOR_TH>
<BLUE_ARMOR_GRAY_TH>113</BLUE_ARMOR_GRAY_TH>
<BLUE_ARMOR_COLOR_TH>40</BLUE_ARMOR_COLOR_TH>
这四个参数：red/blue_armor_gray_th:为预处理中第一步的灰度阈值化
red/blue_armor_color_th:为第二步中对通道相减图的阈值化；

如果出现结算不出来的特殊情况，直接重新启动程序；