#include <ros/ros.h>
#include "jhr_moto_hls.h"
#include "jhr_usart.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "jhr_odom_node");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub=nh.subscribe("cmd_node",1,Jhr_moto_hls::cmd_callback);//速度回调函数

    Jhr_usart::usart_init();
    sleep(1);
    Jhr_usart::close();
    sleep(1);
    Jhr_usart::usart_init();


    ros::Rate loop_rate(1);
    while (ros::ok()) {
    unsigned char buff[12]={0};
        Jhr_moto_hls::generateSpeed(1);
        // Jhr_moto_hls::generateEncoder(1);
        Jhr_usart::ReadFromUsart((const char*)buff,12);
        Jhr_moto_hls::usart_callback((const char*)buff,12);//串口回调函数，对串口数据进行解析




        
        ros::spinOnce();
        loop_rate.sleep();
    }
}