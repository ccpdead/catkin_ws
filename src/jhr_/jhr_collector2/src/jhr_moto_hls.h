#ifndef __JHR_MOTO_HLS_
#define __JHR_MOTO_HLS_
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <geometry_msgs/Twist.h>

class Jhr_moto_hls {
   public:
    static void moto_loop(void);
    static void rcv_callback_1(const char* buff_r, int iLen);

    static void moto_loop_pri(void);
    static unsigned short generateCRC16(unsigned char* puchMsg, unsigned short usDataLen);
    static void udp_send_data(const char* buf, int len);
    static void generateEnable(unsigned char addr, unsigned char enable);       // 设置马达使能/失能
    static void generateSpeed(unsigned char addr);                              // 发送读取速度指令
    static void generateSpeed(unsigned char addr, short speedL, short speedR);  // 发送电机速度 数据高字节在前
    static void generateEncoder(unsigned char addr);                            // 读取编码器的值，数据高字节在前
    static void calcOdom(void);                                                 // 计算pos
    static void usart_callback(const char* buff_r, int iLen);
    static void cmd_callback(const geometry_msgs::Twist &tw);
    static void odom_publish();

   private:
    static short lSpeed_r;  // 坐邮轮速度
    static short rSpeed_r;
    static short lCode_r;  // 左右轮编码器的值
    static short rCode_r;

    static float vLinear;
    static float vAngular;

    static int pulse_equivalent;
    static int wheel_spacing;

    static float posX;
    static float poseY;
    static float posRadian;
};

#endif