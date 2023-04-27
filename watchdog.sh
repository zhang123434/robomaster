#! /bin/bash
PRO_NAME=/home/nuc/Desktop/infantry_current_6.6/src/main.cpp
while true
do

process=work              #监控的程序运行名

pid=$(ps -ef|grep $process |grep '/home/nuc/Desktop/infantry_current_6.6/build' | grep -v grep |awk '{print $2}')
echo $pid
NUM=`ps aux | grep -w ${PRO_NAME} | grep -v grep | wc -l`

if [ $pid -ne 0 ]
then                 #如果程序PID在，则程序在运行
        echo `date`
        echo "pid is exitable!"
elif["${NUM}" -lt "1"];
        echo "${PRO_NUM} was killed"
        cd /dev
        echo "nuc" | sudo ip link set can0 up type can bitrate 1000000    #设置波特率
        echo "nuc" | sudo ifconfig can0 up    # 使能can口
        cd    /home/nuc/Desktop/infantry_current_6.6/build
                                            ./exe
else                #否则程序没有运行，需要启动程序    启动的程序在run.sh文中。
        echo `date`
        echo "pid isnot exitable!"
        cd /dev
        echo "nuc" | sudo  ip link set can0 up type can bitrate 1000000    #设置波特率
        echo "nuc" | sudo  ifconfig can0 up    # 使能can口
        cd    /home/nuc/Desktop/infantry_current_6.6/build
                                            ./exe
fi

sleep 1s

done

exit 0
#通过"bash 文件名"来运行；
