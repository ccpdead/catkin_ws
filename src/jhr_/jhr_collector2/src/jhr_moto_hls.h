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
    static void generateEnable(unsigned char addr, unsigned char enable);       // �������ʹ��/ʧ��
    static void generateSpeed(unsigned char addr);                              // ���Ͷ�ȡ�ٶ�ָ��
    static void generateSpeed(unsigned char addr, short speedL, short speedR);  // ���͵���ٶ� ���ݸ��ֽ���ǰ
    static void generateEncoder(unsigned char addr);                            // ��ȡ��������ֵ�����ݸ��ֽ���ǰ
    static void calcOdom(void);                                                 // ����pos
    static void usart_callback(const char* buff_r, int iLen);
    static void cmd_callback(const geometry_msgs::Twist &tw);
    static void odom_publish();

   private:
    static short lSpeed_r;  // �������ٶ�
    static short rSpeed_r;
    static short lCode_r;  // �����ֱ�������ֵ
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