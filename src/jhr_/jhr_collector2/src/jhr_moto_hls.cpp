#include "jhr_moto_hls.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include "jhr_usart.h"

short Jhr_moto_hls::lSpeed_r = 0;
short Jhr_moto_hls::rSpeed_r = 0;
short Jhr_moto_hls::lCode_r = 0;
short Jhr_moto_hls::rCode_r = 0;
float Jhr_moto_hls::vLinear = 0.0;
float Jhr_moto_hls::vAngular = 0.0;
int Jhr_moto_hls::pulse_equivalent = 0;
int Jhr_moto_hls::wheel_spacing = 0;
float Jhr_moto_hls::poseY = 0.0f;
float Jhr_moto_hls::posX = 0.0f;
float Jhr_moto_hls::posRadian = 0.0f;

short chg_big_small(const char* dat) {
    char ret[2];
    ret[0] = dat[1];
    ret[1] = dat[0];
    return *(short*)ret;
}
/*电机使能*/
/*01 44 21 00 31 00 00 01 00 01 74 34 [12]*/
void Jhr_moto_hls::generateEnable(unsigned char addr, unsigned char enable) {
    char buff[12] = {0x01, 0x44, 0x21, 0x00, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00};  // 驱动器电机是能指令
    buff[0] = addr;
    buff[7] = enable;
    buff[9] = enable;
    unsigned short crc16 = generateCRC16((unsigned char*)buff, sizeof(buff) - 2);
    char* szCrc = &buff[sizeof(buff) - 2];  // 将CRC的值放到buff的后两位
    *(unsigned short*)szCrc = crc16;
    Jhr_usart::WriteToUsart((const char*)buff, sizeof(buff));
    // printf("generateEnable: ");
    // for (int i = 0; i < 12; i++) {
    //     printf("%x ", buff[i]);
    // }
    // printf("\n");
}

/*发送读速度指令*/
/*01 43 50 00 51 00 68 95 [8]*/
void Jhr_moto_hls::generateSpeed(unsigned char addr) {
    char buff[8] = {0x01, 0x43, 0x50, 0x00, 0x51, 0x00};
    buff[0] = addr;
    unsigned short crc16 = generateCRC16((unsigned char*)buff, sizeof(buff) - 2);
    char* szCrc = &buff[strlen(buff) - 2];  // 将CRC的值放到buff的后两位
    *(unsigned short*)szCrc = crc16;
    Jhr_usart::WriteToUsart((const char*)buff, sizeof(buff));
    // printf("generateSpeed: ");
    // for (int i = 0; i < sizeof(buff); i++) {
    //     printf("%x ", buff[i]);
    // }
    // printf("\n");
}

/*设置电机速度*/
/*01 44 23 18 33 18 00 64 00 32 1d 06 [12]*/
void Jhr_moto_hls::generateSpeed(unsigned char addr, short speedL, short speedR) {
#define CON_MIN_SPEED 30
    short addSP = speedL - speedR;
    if (addSP < 0)
        addSP *= -1;

    if (addSP < 10 && speedL != 0) {
        // 电机速度限制30 ~ -30
        if (speedL > -CON_MIN_SPEED && speedL < CON_MIN_SPEED) {
            if (speedL < 0) {
                speedL = -CON_MIN_SPEED;
                speedR = -CON_MIN_SPEED;
            } else {
                speedL = CON_MIN_SPEED;
                speedR = CON_MIN_SPEED;
            }
        }
    }
    speedL /= 10;
    speedR /= 10;
    char buff[12] = {0x01, 0x44, 0x23, 0x18, 0x33, 0x18};  // 设置驱动器速度指令
    // 设置左右轮的速度
    buff[0] = addr;
    buff[6] = speedL >> 8;
    buff[7] = speedL & 0xff;

    buff[8] = speedR >> 8;
    buff[9] = speedR & 0xff;
    unsigned short crc16 = generateCRC16((unsigned char*)buff, strlen(buff) - 2);
    char* szCrc = &buff[strlen(buff) - 2];  // 将CRC的值放到buff的后两位
    *(unsigned short*)szCrc = crc16;
    printf("generateSpeed2: ");
    for (int i = 0; i < 12; i++) {
        printf("%x ", buff[i]);
    }
    printf("\n");
}

/*读电机编码器指令*/
/*01 43 50 15 51 15 00 00 f3` a8 [10]*/
void Jhr_moto_hls::generateEncoder(unsigned char addr) {
    char buff[10] = {0x01, 0x43, 0x50, 0x15, 0x51, 0x15, 0x00, 0x00};  // 50-15，51-15
    buff[0] = addr;
    unsigned short crc16 = generateCRC16((unsigned char*)buff, sizeof(buff) - 2);
    char* szCrc = &buff[sizeof(buff) - 2];  // 将CRC的值放到buff的后两位
    *(unsigned short*)szCrc = crc16;
    printf("generateEncoder: ");
    for (int i = 0; i < sizeof(buff); i++) {
        printf("%x ", buff[i]);
    }
    printf("\n");
}

/*里程计推演*/
void Jhr_moto_hls::calcOdom(void) {
    static short old_l = 0;
    static short old_r = 0;

    short d_l = lCode_r - old_l;
    short d_r = old_r - rCode_r;
    if ((abs(d_l) <= 2000) && (abs(d_r) <= 2000)) {
        float dL = d_l / Jhr_moto_hls::pulse_equivalent;
        float dR = d_r / Jhr_moto_hls::pulse_equivalent;
        float dTh = ((dR - dL) * 1.0f / Jhr_moto_hls::wheel_spacing);  // 电机轮间距
        dL += dR;
        dL /= 2;
        Jhr_moto_hls::posX += (dL * cosf(Jhr_moto_hls::posRadian));
        Jhr_moto_hls::poseY += (dL * sinf(Jhr_moto_hls::posRadian));
        Jhr_moto_hls::posRadian += dTh;
        while (Jhr_moto_hls::posRadian > 3.14156)
            Jhr_moto_hls::posRadian -= 6.2832;
        while (Jhr_moto_hls::posRadian < -3.14156)
            Jhr_moto_hls::posRadian += 6.2832;
    } else {
        perror("calcodom ERROR\n");
    }
    old_l = lCode_r;
    old_r = rCode_r;
}

/*读取电机速度或者编码器的值*/
/*电机转速和电机编码器值的返回值都是12个数据*/
void Jhr_moto_hls::usart_callback(const char* buff_r, int iLen) {
    if (iLen < 6)
        return;
    unsigned char crc16 = generateCRC16((unsigned char*)buff_r, iLen - 2);
    const char* szCrc = &buff_r[iLen - 2];
    if (*(unsigned short*)szCrc == crc16) {                 // 检查crc校验
        unsigned int iAddr = *(unsigned int*)(&buff_r[2]);  // 读取该地址的具体值
        if (buff_r[1] == 0x43) {
            switch (iAddr) {
                case 0x00510050:                           // 电机转速
                    lSpeed_r = chg_big_small(&buff_r[6]);  // 6,7
                    rSpeed_r = chg_big_small(&buff_r[8]);  // 8,9
                    Jhr_moto_hls::vLinear = (lSpeed_r - rSpeed_r) * 50.0f / Jhr_moto_hls::pulse_equivalent;
                    Jhr_moto_hls::vAngular = (lSpeed_r + rSpeed_r) / Jhr_moto_hls::pulse_equivalent / Jhr_moto_hls::wheel_spacing * -100;
                    break;
                case 0x15511550:  // 电机编码器
                    lCode_r = chg_big_small(&buff_r[6]);
                    rCode_r = chg_big_small(&buff_r[8]);
                    calcOdom();  // 里程计计算
                    odom_publish();
                    break;
            }
        }
    } else {
        perror("CRC Error \n");
    }

    moto_loop_pri();
}

void Jhr_moto_hls::moto_loop_pri(void) {
    static int iCnt = 0;
    switch (iCnt) {
        case 0:
            generateEncoder(1);  // 读编码器指令
            break;
        case 1:
            generateSpeed(1);  // 读速度指令
            break;

        case 2:
            if (Jhr_moto_hls::rSpeed_r != 0 || Jhr_moto_hls::lSpeed_r != 0) {
                generateEnable(1, 1);  // 当速度不为零时，开启电机
            } else {
                generateEnable(1, 0);  // 速度为零时，关闭电机
            }
            break;
        default:
            generateSpeed(1, Jhr_moto_hls::lSpeed_r, Jhr_moto_hls::rSpeed_r);  // 发送电机速度
            break;
    }

    iCnt++;
    if (iCnt > 3)
        iCnt = 0;
}

void Jhr_moto_hls::odom_publish(void) {

    float x = Jhr_moto_hls::getX();        // 获取坐标 X
    float y = Jhr_moto_hls::getY();        // 获取坐标 Y
    float th = Jhr_moto_hls::getRadian();  // 获取弧度值
    curr_time = ros::Time::now();

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    // //发布tf转化
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = curr_time;
    odom_trans.header.frame_id = "wheel_odom_link";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster->sendTransform(odom_trans);  // 发布tf转换。

    // 发布里程机wheel_odom
    nav_msgs::Odometry odom;
    odom.header.stamp = curr_time;
    odom.header.frame_id = "wheel_odom_link";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = Jhr_moto::getLinear();  // 获取线速度
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = Jhr_moto::getAngular();  // 获取角速度
    if ((odom.twist.twist.linear.x == 0) && (odom.twist.twist.angular.z == 0)) {
        odom.pose.covariance = {1e-3, 0, 0, 0,   0, 0, 0, 1e-3, 1e-9, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
                                0,    0, 0, 1e6, 0, 0, 0, 0,    0,    0, 1e6, 0, 0, 0, 0,   0, 0, 1e-9};
        odom.twist.covariance = {1e-3, 0, 0, 0,   0, 0, 0, 1e-3, 1e-9, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
                                 0,    0, 0, 1e6, 0, 0, 0, 0,    0,    0, 1e6, 0, 0, 0, 0,   0, 0, 1e-9};
    } else {
        odom.pose.covariance = {1e-3, 0, 0, 0,   0, 0, 0, 1e-3, 0, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
                                0,    0, 0, 1e6, 0, 0, 0, 0,    0, 0, 1e6, 0, 0, 0, 0,   0, 0, 1e-3};
        ;
        odom.twist.covariance = {1e-3, 0, 0, 0,   0, 0, 0, 1e-3, 0, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
                                 0,    0, 0, 1e6, 0, 0, 0, 0,    0, 0, 1e6, 0, 0, 0, 0,   0, 0, 1e-3};
        ;
    }
    odom_pub_ptr->publish(odom);

    // std::cout <<" th:" <<(int)(th*180/3.14)<<"  x:" <<(float)(th*180/3.14) <<"  y:" <<(float)(th*180/3.14) <<"\r         "<< std::flush;
}
}

// ********** modbos *********
/* 高位字节的CRC  值  */
const unsigned char auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40};

/* 低位字节的CRC  值  */
const unsigned char auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40};

/**
 * @brief CRC16位校验
 *
 * @param puchMsg
 * @param usDataLen
 * @return unsigned short
 */
unsigned short Jhr_moto_hls::generateCRC16(unsigned char* puchMsg, unsigned short usDataLen) {
    unsigned char uchCRCHi = 0xFF;  // CRC  的高字节初始化
    unsigned char uchCRCLo = 0xFF;  // CRC  的低字节初始化
    unsigned char uIndex;           // CRC 查询表索引

    while (usDataLen--)  // 完成整个报文缓冲区
    {
        uIndex = uchCRCLo ^ *puchMsg++;  // 计算  CRC
        uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
        uchCRCHi = auchCRCLo[uIndex];
    }
    return (uchCRCHi << 8 | uchCRCLo);
}