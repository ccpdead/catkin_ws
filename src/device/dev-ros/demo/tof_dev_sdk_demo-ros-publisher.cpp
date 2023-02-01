#ifdef WIN32

	#include <Windows.h>
	#include <direct.h>
	#include <psapi.h>
	#include <io.h>

	#define R_OK 4
	#define W_OK 2
	#define X_OK 1
	#define F_OK 0

#elif defined LINUX 

	#include <unistd.h>
	#include <sys/stat.h>
	#include <sys/time.h>
	#include <sys/sysinfo.h>

#endif
#include <time.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <iostream>
#include <thread>
#include <list>
#include <mutex>
#include <vector>
#include <chrono>
#include <algorithm>
#include "tof_error.h"
#include "typedef.h"
#include "tof_dev_sdk.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include<pcl_conversions/pcl_conversions.h>


#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


ros::Publisher pub_info;
ros::Publisher pub_pointCloud;
ros::Publisher pub_depthz;
ros::Publisher pub_gray;
ros::Publisher pub_rgbd;
ros::Publisher pub_rgb;

sensor_msgs::CameraInfoPtr camInfoMsg;
sensor_msgs::PointCloud2 cloudMsg;
sensor_msgs::ImagePtr depthzMsg;
sensor_msgs::ImagePtr grayMsg;
sensor_msgs::ImagePtr rgbdMsg;
sensor_msgs::ImagePtr rgbMsg;

pcl::PointCloud<pcl::PointXYZ> pclCloud;


//
#define SAFE_DELETE(p) if(p){delete p; p=NULL;}
#define SAFE_DELETE_ARRY(p) if(p){delete [] p; p=NULL;}
#define SAFE_FREE(p) if(p){free(p); p=NULL;}
#define SAFE_CLOSE_FP(fp) if(fp){fclose(fp); fp=NULL;}



#ifdef WIN32
static int gettimeofday(struct timeval *tp, void *tzp)
{
	time_t clock;
	struct tm tm;
	SYSTEMTIME wtm;
	GetLocalTime(&wtm);
	tm.tm_year = wtm.wYear - 1900;
	tm.tm_mon = wtm.wMonth - 1;
	tm.tm_mday = wtm.wDay;
	tm.tm_hour = wtm.wHour;
	tm.tm_min = wtm.wMinute;
	tm.tm_sec = wtm.wSecond;
	tm.tm_isdst = -1;
	clock = mktime(&tm);
	tp->tv_sec = (long)clock;
	tp->tv_usec = wtm.wMilliseconds * 1000;
	return (0);
}
#endif

static unsigned long long Utils_GetTickCount(void)
{
	unsigned long long tick = 0;

#ifdef WIN32
	//tick = GetTickCount();//实际精度只有15ms左右; 返回的是一个32位的无符号整数，Windows连续运行49.710天后，它将再次从零开始计时; 
	//tick = GetTickCount64();//返回一个64位的无符号整数。Windows连续运行5.8亿年后，其计时才会归零; 
	//tick = clock();//该程序从启动到函数调用占用CPU的时间, 是C/C++中的计时函数

	//struct timeval tv;
	//gettimeofday(&tv, 0);
	//tick = (tv.tv_sec * 1000 + tv.tv_usec / 1000);
		
	auto timePoint = std::chrono::steady_clock::now(); // std::chrono::time_point
	tick = std::chrono::duration_cast<std::chrono::milliseconds>(timePoint.time_since_epoch()).count();

#elif defined LINUX
	//struct timeval tv;
	//gettimeofday(&tv, 0);
	//tick = (tv.tv_sec * 1000 + tv.tv_usec/1000);
	
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
	tick = (tv.tv_sec * 1000 + tv.tv_nsec/1000000);
	
#else
	printf("unknown platform in getting tick cnt, error!\n");
#endif // WIN32

	return tick;
}


static long Utils_GetFileLen(const char * filename)
{
	if (NULL == filename)
	{
		printf("NULL == filename\n");
		return 0;
	}

	FILE * fd = fopen(filename, "rb");
	if (NULL == fd)
	{
		printf("open file (%s) failed, errno=%d(%s).\n", filename, errno, strerror(errno));
		return 0;
	}

	fseek(fd, 0L, SEEK_END); /* 定位到文件末尾 */
	const long len = ftell(fd);
	fclose(fd);

	return len;

}

static void Utils_SaveBufToFile(void* pData, const unsigned int nDataLen, const char* pFile, const bool bAppend)
{
	if ((NULL == pData) || (0 >= nDataLen) || (NULL == pFile))
	{
		return;
	}

	FILE* fp = fopen(pFile, (bAppend ? "ab" : "wb"));
	if (NULL == fp)
	{
		printf("open file(%s) failed, error=%d(%s).\n", pFile, errno, strerror(errno));
		return;
	}

	fwrite(pData, 1, nDataLen, fp);
	fclose(fp);
}

template <class T>
T Utils_FindMaxValue(T* pData, const int nCnt)
{
	T max = pData[0];

	for (int i = 0; i < nCnt; i++)
	{
		if (max < pData[i])
		{
			max = pData[i];
		}
	}

	return max;
}

template <class T>
T Utils_FindMinValue(T* pData, const int nCnt)
{
	T min = pData[0];

	for (int i = 0; i < nCnt; i++)
	{
		if (min > pData[i])
		{
			min = pData[i];
		}
	}

	return min;
}

static int CaculateGrayPixelBytes(const GRAY_FORMAT format)
{
	int grayByte = 1;

	switch (format)
	{
	case GRAY_FORMAT_UINT8: grayByte = 1; break;
	case GRAY_FORMAT_UINT16: grayByte = 2; break;
	case GRAY_FORMAT_FLOAT: grayByte = 4; break;
	case GRAY_FORMAT_BGRD: grayByte = 4; break;
	default:break;
	}

	return grayByte;

}
static const char* StringGrayFormat(const GRAY_FORMAT format)
{
	const char* pStr = "UnknownGRAY";

	switch (format)
	{
	case GRAY_FORMAT_UINT8: pStr = "U8"; break;
	case GRAY_FORMAT_UINT16: pStr = "U16"; break;
	case GRAY_FORMAT_FLOAT: pStr = "FLOAT"; break;
	case GRAY_FORMAT_BGRD: pStr = "BGRD"; break;

	default: break;
	}

	return pStr;
}
static float CalCenterPointDataZAvg(PointData *pPointData, const UINT32 width, const UINT32 height)
{
	if (NULL == pPointData)
	{
		return 0;
	}

	const int start_h = (10<height) ? ((height / 2) - 5) : 0;
	const int end_h = (10<height) ? ((height / 2) + 5) : (height);
	const int start_w = (10<width) ? ((width / 2) - 5) : 0;
	const int end_w = (10<width) ? ((width / 2) + 5) : (width);


	float sum = 0.0;
	int cnt = 0;
	for (int h = start_h; h < end_h; h++)
	{
		PointData *pTmp = pPointData + h*width;
		for (int w = start_w; w < end_w; w++)
		{
			if (0.00001 < pTmp[w].z)
			{
				sum += pTmp[w].z;
				cnt++;
			}
		}
	}

	return ((0 < cnt) ? (sum / cnt) : 0);
}

static const char* StringColorFormat(const COLOR_FORMAT type)
{
	const char* pStr = "UnknownRGB";

	switch (type)
	{
	case COLOR_FORMAT_MJPG: pStr = "JPG"; break;

	case COLOR_FORMAT_H264: pStr = "H264"; break;

	case COLOR_FORMAT_YUV422: pStr = "YUV422"; break;
	case COLOR_FORMAT_YUYV: pStr = "YUYV"; break;
	case COLOR_FORMAT_I420: pStr = "I420"; break;
	case COLOR_FORMAT_YV12: pStr = "YV12"; break;
	case COLOR_FORMAT_NV12: pStr = "NV12"; break;
	case COLOR_FORMAT_NV21: pStr = "NV21"; break;

	case COLOR_FORMAT_BGR: pStr = "BGR"; break;
	case COLOR_FORMAT_RGB: pStr = "RGB"; break;
	case COLOR_FORMAT_BGRA: pStr = "BGRA"; break;
	case COLOR_FORMAT_RGBA: pStr = "RGBA"; break;

	default: break;
	}

	return pStr;
}
static const char* TofMode2Str(const TOF_MODE mode)
{
	const char* pStr = "Unknown";

	switch (mode)
	{
	case TOF_MODE_STERO_5FPS: pStr = "STERO_5FPS"; break;
	case TOF_MODE_STERO_10FPS: pStr = "STERO_10FPS"; break;
	case TOF_MODE_STERO_15FPS: pStr = "STERO_15FPS"; break;
	case TOF_MODE_STERO_30FPS: pStr = "STERO_30FPS"; break;
	case TOF_MODE_STERO_45FPS: pStr = "STERO_45FPS"; break;
	case TOF_MODE_STERO_60FPS: pStr = "STERO_60FPS"; break;

	case TOF_MODE_MONO_5FPS: pStr = "MONO_5FPS"; break;
	case TOF_MODE_MONO_10FPS: pStr = "MONO_10FPS"; break;
	case TOF_MODE_MONO_15FPS: pStr = "MONO_15FPS"; break;
	case TOF_MODE_MONO_30FPS: pStr = "MONO_30FPS"; break;
	case TOF_MODE_MONO_45FPS: pStr = "MONO_45FPS"; break;
	case TOF_MODE_MONO_60FPS: pStr = "MONO_60FPS"; break;

	case TOF_MODE_HDRZ_5FPS: pStr = "HDRZ_5FPS"; break;
	case TOF_MODE_HDRZ_10FPS: pStr = "HDRZ_10FPS"; break;
	case TOF_MODE_HDRZ_15FPS: pStr = "HDRZ_15FPS"; break;
	case TOF_MODE_HDRZ_30FPS: pStr = "HDRZ_30FPS"; break;
	case TOF_MODE_HDRZ_45FPS: pStr = "HDRZ_45FPS"; break;
	case TOF_MODE_HDRZ_60FPS: pStr = "HDRZ_60FPS"; break;

	case TOF_MODE_5FPS: pStr = "5FPS"; break;
	case TOF_MODE_10FPS: pStr = "10FPS"; break;
	case TOF_MODE_20FPS: pStr = "20FPS"; break;
	case TOF_MODE_30FPS: pStr = "30FPS"; break;
	case TOF_MODE_45FPS: pStr = "45FPS"; break;
	case TOF_MODE_60FPS: pStr = "60FPS"; break;

	case TOF_MODE_ADI_1M5: pStr = "ADI_1M5"; break;
	case TOF_MODE_ADI_5M: pStr = "ADI_5M"; break;

	case TOF_MODE_CUSTOM_1: pStr = "CUSTOM_1"; break;
	case TOF_MODE_CUSTOM_2: pStr = "CUSTOM_2"; break;
	case TOF_MODE_CUSTOM_3: pStr = "CUSTOM_3"; break;
	case TOF_MODE_CUSTOM_4: pStr = "CUSTOM_4"; break;
	case TOF_MODE_CUSTOM_5: pStr = "CUSTOM_5"; break;

	case TOF_MODE_DEBUG: pStr = "DEBUG"; break;


	default: break;
	}

	return pStr;
}


class CGrayConvert
{
	CGrayConvert();
	~CGrayConvert();

public:
	static bool Gray_2_Bgr32(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned int* pBgr32);
	static bool Gray_2_U16(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned short* pU16);
	static bool Gray_2_U8(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned char* pU8);

private:
	static bool ToBgr32(unsigned char* pGray, const int width, const int height, unsigned int* pBgr32);
	static bool ToBgr32(unsigned short* pGray, const int width, const int height, unsigned int* pBgr32);
	static bool ToBgr32(float* pGray, const int width, const int height, unsigned int* pBgr32);
	static bool ToBgr32(unsigned int* pGray, const int width, const int height, unsigned int* pBgr32);
private:
	static bool ToU16(unsigned char* pGray, const int width, const int height, unsigned short* pU16);
	static bool ToU16(unsigned short* pGray, const int width, const int height, unsigned short* pU16);
	static bool ToU16(float* pGray, const int width, const int height, unsigned short* pU16);
	static bool ToU16(unsigned int* pGray, const int width, const int height, unsigned short* pU16);
private:
	static bool ToU8(unsigned char* pGray, const int width, const int height, unsigned char* pU8);
	static bool ToU8(unsigned short* pGray, const int width, const int height, unsigned char* pU8);
	static bool ToU8(float* pGray, const int width, const int height, unsigned char* pU8);
	static bool ToU8(unsigned int* pGray, const int width, const int height, unsigned char* pU8);
};
CGrayConvert::CGrayConvert()
{
}
CGrayConvert::~CGrayConvert()
{
}

bool CGrayConvert::Gray_2_Bgr32(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned int* pBgr32)
{
	bool retVal = false;

	switch (format)
	{
	case GRAY_FORMAT_UINT8:  retVal = ToBgr32((unsigned char*)pGray, width, height, pBgr32); break;
	case GRAY_FORMAT_UINT16: retVal = ToBgr32((unsigned short*)pGray, width, height, pBgr32); break;
	case GRAY_FORMAT_FLOAT:  retVal = ToBgr32((float*)pGray, width, height, pBgr32); break;
	case GRAY_FORMAT_BGRD:   retVal = ToBgr32((unsigned int*)pGray, width, height, pBgr32); break;
	default: break;
	}

	return retVal;
}

bool CGrayConvert::Gray_2_U16(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned short* pU16)
{
	bool retVal = false;

	switch (format)
	{
	case GRAY_FORMAT_UINT8:  retVal = ToU16((unsigned char*)pGray, width, height, pU16); break;
	case GRAY_FORMAT_UINT16: retVal = ToU16((unsigned short*)pGray, width, height, pU16); break;
	case GRAY_FORMAT_FLOAT:  retVal = ToU16((float*)pGray, width, height, pU16); break;
	case GRAY_FORMAT_BGRD:   retVal = ToU16((unsigned int*)pGray, width, height, pU16); break;
	default: break;
	}

	return retVal;
}

bool CGrayConvert::Gray_2_U8(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned char* pU8)
{
	bool retVal = false;

	switch (format)
	{
	case GRAY_FORMAT_UINT8:  retVal = ToU8((unsigned char*)pGray, width, height, pU8); break;
	case GRAY_FORMAT_UINT16: retVal = ToU8((unsigned short*)pGray, width, height, pU8); break;
	case GRAY_FORMAT_FLOAT:  retVal = ToU8((float*)pGray, width, height, pU8); break;
	case GRAY_FORMAT_BGRD:   retVal = ToU8((unsigned int*)pGray, width, height, pU8); break;
	default: break;
	}

	return retVal;
}
bool CGrayConvert::ToBgr32(unsigned char* pGray, const int width, const int height, unsigned int* pBgr32)
{
	const int pixel_cnt = width*height;

	for (int i = 0; i < pixel_cnt; i++)
	{
		pBgr32[i] = ((pGray[i] << 24) | (pGray[i] << 16) | (pGray[i] << 8) | pGray[i]);
	}

	return true;
}
bool CGrayConvert::ToBgr32(unsigned short* pGray, const int width, const int height, unsigned int* pBgr32)
{
#if 0
	const int pixel_cnt = width*height;
	//const unsigned short min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const unsigned short max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0 >= max)
	{
		memset(pBgr32, 0, pixel_cnt * sizeof(pBgr32[0]));
		return true;
	}

	const float K = (255 * 1.0f / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		const unsigned char tmp = (unsigned char)(pGray[i] * K);
		pBgr32[i] = ((tmp << 24) | (tmp << 16) | (tmp << 8) | tmp);
	}
#endif

#if 1
	const int pixel_cnt = width*height;

	//
	unsigned short* pGrayTmp = new unsigned short[pixel_cnt];
	memcpy(pGrayTmp, pGray, pixel_cnt * sizeof(pGray[0]));
	const int nth = (int)(0.995f * pixel_cnt);
	std::nth_element(pGrayTmp, pGrayTmp + nth, pGrayTmp + pixel_cnt);//找到第K小的数据
	const unsigned short max = pGrayTmp[nth];
	delete[] pGrayTmp;

	//
	if (0 >= max)
	{
		memset(pBgr32, 0, pixel_cnt * sizeof(pBgr32[0]));
		return true;
	}

	const float K = (255 * 1.0f / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		const unsigned char tmp = (unsigned char)((max < pGray[i]) ? 255 : (pGray[i] * K));
		pBgr32[i] = ((tmp << 24) | (tmp << 16) | (tmp << 8) | tmp);
	}
#endif

	return true;
}
bool CGrayConvert::ToBgr32(float* pGray, const int width, const int height, unsigned int* pBgr32)
{
#if 0
	const int pixel_cnt = width*height;
	//const float min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const float max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0.001f >= max)//0值用黑色表示
	{
		memset(pBgr32, 0, pixel_cnt * sizeof(pBgr32[0]));
		return true;
	}

	const float K = (255 * 1.0f / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char tmp = 0;//0值用黑色表示
		if (0.001f < pGray[i])
		{
			tmp = (unsigned char)(pGray[i] * K);
		}
		pBgr32[i] = ((tmp << 24) | (tmp << 16) | (tmp << 8) | tmp);
	}
#endif

#if 1
	const int pixel_cnt = width*height;

	//
	float* pGrayTmp = new float[pixel_cnt];
	memcpy(pGrayTmp, pGray, pixel_cnt * sizeof(pGray[0]));
	const int nth = (int)(0.995f * pixel_cnt);
	std::nth_element(pGrayTmp, pGrayTmp + nth, pGrayTmp + pixel_cnt);//找到第K小的数据
	const float max = pGrayTmp[nth];
	delete[] pGrayTmp;

	//
	if (0.001f >= max)//0值用黑色表示
	{
		memset(pBgr32, 0, pixel_cnt * sizeof(pBgr32[0]));
		return true;
	}

	const float K = (255 * 1.0f / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		const unsigned char tmp = (unsigned char)((max < pGray[i]) ? 255 : (pGray[i] * K));
		pBgr32[i] = ((tmp << 24) | (tmp << 16) | (tmp << 8) | tmp);
	}
#endif

	return true;
}
bool CGrayConvert::ToBgr32(unsigned int* pGray, const int width, const int height, unsigned int* pBgr32)
{
	const int pixel_cnt = width*height;

	memcpy(pBgr32, pGray, pixel_cnt * sizeof(pBgr32[0]));

	return true;
}

bool CGrayConvert::ToU16(unsigned char* pGray, const int width, const int height, unsigned short* pU16)
{
	const int pixel_cnt = width*height;
	//const unsigned char min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const unsigned char max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0 >= max)
	{
		memset(pU16, 0, pixel_cnt * sizeof(pU16[0]));
		return true;
	}

	const float K = (65535 * 1.0f / max);//最大值是65535的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		const unsigned short tmp = (unsigned short)(pGray[i] * K);
		pU16[i] = tmp;
	}

	return true;
}
bool CGrayConvert::ToU16(unsigned short* pGray, const int width, const int height, unsigned short* pU16)
{
	const int pixel_cnt = width*height;

	memcpy(pU16, pGray, pixel_cnt * sizeof(pU16[0]));

	return true;
}
bool CGrayConvert::ToU16(float* pGray, const int width, const int height, unsigned short* pU16)
{
#if 1
	//方法1：灰度直接数据类型强转

	const int pixel_cnt = width*height;

	for (int i = 0; i < pixel_cnt; i++)
	{
		pU16[i] = (unsigned short)(((65535.0f < pGray[i]) ? 65535 : pGray[i]));
	}

#else
	//方法2：灰度按照等比例压缩

	const int pixel_cnt = width*height;
	//const float min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const float max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0.001f >= max)//0值用黑色表示
	{
		memset(pU16, 0, pixel_cnt * sizeof(pU16[0]));
		return true;
	}

	const float K = (65535 * 1.0f / max);//最大值是65535的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned short tmp = 0;//0值用黑色表示
		if (0.001f < pGray[i])
		{
			tmp = (unsigned short)(pGray[i] * K);
		}
		pU16[i] = tmp;
	}
#endif
	return true;
}
bool CGrayConvert::ToU16(unsigned int* pGray, const int width, const int height, unsigned short* pU16)
{
	const int pixel_cnt = width*height;

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char* pTmp = (unsigned char*)(pGray + i);//BGRD排列
		pU16[i] = (pTmp[0] << 8);//放大到65535
	}

	return true;
}

bool CGrayConvert::ToU8(unsigned char* pGray, const int width, const int height, unsigned char* pU8)
{
	const int pixel_cnt = width*height;

	memcpy(pU8, pGray, pixel_cnt * sizeof(pU8[0]));

	return true;
}
bool CGrayConvert::ToU8(unsigned short* pGray, const int width, const int height, unsigned char* pU8)
{
#if 0
	const int pixel_cnt = width*height;
	//const unsigned short min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const unsigned short max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0 >= max)
	{
		memset(pU8, 0, pixel_cnt * sizeof(pU8[0]));
		return true;
	}

	const float K = (255 * 1.0f / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		const unsigned char tmp = (unsigned char)(pGray[i] * K);
		pU8[i] = tmp;
	}
#endif

#if 1
	const int pixel_cnt = width*height;

	//
	unsigned short* pGrayTmp = new unsigned short[pixel_cnt];
	memcpy(pGrayTmp, pGray, pixel_cnt * sizeof(pGray[0]));
	const int nth = (int)(0.995f * pixel_cnt);
	std::nth_element(pGrayTmp, pGrayTmp + nth, pGrayTmp + pixel_cnt);//找到第K小的数据
	const unsigned short max = pGrayTmp[nth];
	delete[] pGrayTmp;

	//
	if (0 >= max)
	{
		memset(pU8, 0, pixel_cnt * sizeof(pU8[0]));
		return true;
	}

	const float K = (255 * 1.0f / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		pU8[i] = (unsigned char)((max < pGray[i]) ? 255 : (pGray[i] * K));
	}
#endif

	return true;
}
bool CGrayConvert::ToU8(float* pGray, const int width, const int height, unsigned char* pU8)
{
#if 0
	const int pixel_cnt = width*height;
	//const float min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const float max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0.001f >= max)//0值用黑色表示
	{
		memset(pU8, 0, pixel_cnt * sizeof(pU8[0]));
		return true;
	}

	const float K = (255 * 1.0f / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char tmp = 0;//0值用黑色表示
		if (0.001f < pGray[i])
		{
			tmp = (unsigned char)(pGray[i] * K);
		}
		pU8[i] = tmp;
	}
#endif

#if 1
	const int pixel_cnt = width*height;

	//
	float* pGrayTmp = new float[pixel_cnt];
	memcpy(pGrayTmp, pGray, pixel_cnt * sizeof(pGray[0]));
	const int nth = (int)(0.995f * pixel_cnt);
	std::nth_element(pGrayTmp, pGrayTmp + nth, pGrayTmp + pixel_cnt);//找到第K小的数据
	const float max = pGrayTmp[nth];
	delete[] pGrayTmp;

	//
	if (0.001f >= max)//0值用黑色表示
	{
		memset(pU8, 0, pixel_cnt * sizeof(pU8[0]));
		return true;
	}

	const float K = (255 * 1.0f / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		pU8[i] = (unsigned char)((max < pGray[i]) ? 255 : (pGray[i] * K));
	}

#endif

	return true;
}
bool CGrayConvert::ToU8(unsigned int* pGray, const int width, const int height, unsigned char* pU8)
{
	const int pixel_cnt = width*height;

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char* pTmp = (unsigned char*)(pGray + i);//BGRD排列
		pU8[i] = pTmp[0];
	}

	return true;
}

static bool SaveGray_2_BGR32(void* pGray, const UINT32 width, const UINT32 height, const GRAY_FORMAT format, char* pFile)
{
	if ((NULL == pGray) || (0 >= width) || (0 >= height) || (NULL == pFile))
	{
		return false;
	}

	unsigned int* pData = new unsigned int[width * height];//bgra
	CGrayConvert::Gray_2_Bgr32(pGray, width, height, format, pData);
	Utils_SaveBufToFile(pData, width * height * sizeof(pData[0]), (const char*)pFile, false);
	SAFE_DELETE_ARRY(pData);

	return true;

}
static bool SaveGray_2_U16(void* pGray, const UINT32 width, const UINT32 height, const GRAY_FORMAT format, char* pFile)
{
	if ((NULL == pGray) || (0 >= width) || (0 >= height) || (NULL == pFile))
	{
		return false;
	}

	unsigned short* pData = new unsigned short[width * height];//U16
	CGrayConvert::Gray_2_U16(pGray, width, height, format, pData);
	Utils_SaveBufToFile(pData, width * height * sizeof(pData[0]), (const char*)pFile, false);
	SAFE_DELETE_ARRY(pData);

	return true;

}
static bool SaveGray_2_U8(void* pGray, const UINT32 width, const UINT32 height, const GRAY_FORMAT format, char* pFile)
{
	if ((NULL == pGray) || (0 >= width) || (0 >= height) || (NULL == pFile))
	{
		return false;
	}

	unsigned char* pData = new unsigned char[width * height];//U8
	CGrayConvert::Gray_2_U8(pGray, width, height, format, pData);
	Utils_SaveBufToFile(pData, width * height * sizeof(pData[0]), (const char*)pFile, false);
	SAFE_DELETE_ARRY(pData);

	return true;

}

static bool SaveDepthText(float* pDepthData, const UINT32 width, const UINT32 height, char* pTxtFile, const bool bWH)
{
	if ((NULL == pDepthData) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	if (bWH)//1：W列、H行排列
	{
		UINT32 nPos = 0;
		for (UINT32 h = 0; h < height; h++)
		{
			for (UINT32 w = 0; w < (width - 1); w++)
			{
				fprintf(fp, "%0.6f,", pDepthData[nPos]);
				nPos++;
			}
			fprintf(fp, "%0.6f\n", pDepthData[nPos]);
			nPos++;
		}
	}
	else//2：1行、W*H行排列
	{
		const UINT32 nCnt = width *height;
		for (UINT32 nPos = 0; nPos < nCnt; nPos++)
		{
			fprintf(fp, "%0.6f\n", pDepthData[nPos]);
		}
	}

	fclose(fp);
	return true;
}
static bool SavePointDataXYZText(PointData *pPointData, const UINT32 width, const UINT32 height, char* pTxtFile)
{
	if ((NULL == pPointData) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	const UINT32 nCnt = width *height;
	for (UINT32 nPos = 0; nPos < nCnt; nPos++)
	{
		fprintf(fp, "%0.6f;%0.6f;%0.6f\n", pPointData[nPos].x, pPointData[nPos].y, pPointData[nPos].z);
	}

	fclose(fp);
	return true;
}
static bool SavePointDataZWHText(PointData *pPointData, const UINT32 width, const UINT32 height, char* pTxtFile)
{
	if ((NULL == pPointData) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	UINT32 nPos = 0;
	for (UINT32 h = 0; h < height; h++)
	{
		for (UINT32 w = 0; w < width; w++)
		{
			fprintf(fp, "%0.6f", pPointData[nPos].z);
			nPos++;
		}
		fprintf(fp, "\n");
	}

	fclose(fp);
	return true;
}

static bool SaveColorPointDataXYZText(PointData *pPointData, RgbDData* pRgbD, const UINT32 width, const UINT32 height, char* pTxtFile)
{
	if ((NULL == pPointData) || (NULL == pRgbD) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	const UINT32 nCnt = width *height;
	for (UINT32 nPos = 0; nPos < nCnt; nPos++)
	{
		fprintf(fp, "v %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f\n", pPointData[nPos].x, pPointData[nPos].y, pPointData[nPos].z, pRgbD[nPos].r * 1.0, pRgbD[nPos].g * 1.0, pRgbD[nPos].b * 1.0);
	}

	fclose(fp);
	return true;
}



static bool SaveImuText(ImuFrameData *imuFrameData, const char* pFile, const bool bAppend)
{
	if ((NULL == imuFrameData) || (NULL == pFile))
	{
		return false;
	}

	FILE* fp = fopen(pFile, (bAppend ? "ab" : "wb"));
	if (NULL == fp)
	{
		printf("open file(%s) failed, error=%d(%s).\n", pFile, errno, strerror(errno));
		return false;
	}

	fprintf(fp, "timeStamp:%lld", imuFrameData->timeStamp);

	fprintf(fp, ",accelData:%0.6f,%0.6f,%0.6f", imuFrameData->accelData_x, imuFrameData->accelData_y, imuFrameData->accelData_z);
	fprintf(fp, ",gyrData:%0.6f,%0.6f,%0.6f", imuFrameData->gyrData_x, imuFrameData->gyrData_y, imuFrameData->gyrData_z);
	fprintf(fp, ",magData:%0.6f,%0.6f,%0.6f\n", imuFrameData->magData_x, imuFrameData->magData_y, imuFrameData->magData_z);

	fclose(fp);

	return true;
}

static void CaptureTofFrame(const std::string& strDir, const unsigned int nCaptureIndex, TofFrameData *tofFrameData)
{
	const unsigned int nPixelCnt = tofFrameData->frameWidth * tofFrameData->frameHeight;
	char szFile[512] = { 0 };

	//
	if (NULL != tofFrameData->pDepthData)
	{
		sprintf(szFile, "%s/%u-DepthData.dat", strDir.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pDepthData, nPixelCnt  * sizeof(tofFrameData->pDepthData[0]), szFile, false);

		sprintf(szFile, "%s/%u-DepthData.txt", strDir.c_str(), nCaptureIndex);
		SaveDepthText(tofFrameData->pDepthData, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile, false);
	}

	//
	if (NULL != tofFrameData->pDepthDataFilter)
	{
		sprintf(szFile, "%s/%u-DepthDataFilter.dat", strDir.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pDepthDataFilter, nPixelCnt  * sizeof(tofFrameData->pDepthDataFilter[0]), szFile, false);

		sprintf(szFile, "%s/%u-DepthDataFilter.txt", strDir.c_str(), nCaptureIndex);
		SaveDepthText(tofFrameData->pDepthDataFilter, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile, false);
	}

	//
	if (NULL != tofFrameData->pPointData)
	{
		sprintf(szFile, "%s/%u-PointData.dat", strDir.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pPointData, nPixelCnt  * sizeof(tofFrameData->pPointData[0]), szFile, false);

		sprintf(szFile, "%s/%u-PointData.txt", strDir.c_str(), nCaptureIndex);
		SavePointDataXYZText(tofFrameData->pPointData, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile);
	}

	//
	if (NULL != tofFrameData->pGrayData)
	{
		sprintf(szFile, "%s/%u-Gray.%s", strDir.c_str(), nCaptureIndex, StringGrayFormat(tofFrameData->grayFormat));
		Utils_SaveBufToFile(tofFrameData->pGrayData, nPixelCnt * CaculateGrayPixelBytes(tofFrameData->grayFormat), szFile, false);

		sprintf(szFile, "%s/%u-Gray.u8", strDir.c_str(), nCaptureIndex);
		SaveGray_2_U8(tofFrameData->pGrayData, tofFrameData->frameWidth, tofFrameData->frameHeight, tofFrameData->grayFormat, szFile);
	}
	//
	if (NULL != tofFrameData->pConfidence)
	{
		sprintf(szFile, "%s/%u-Confidence.dat", strDir.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pConfidence, nPixelCnt  * sizeof(tofFrameData->pConfidence[0]), szFile, false);

		sprintf(szFile, "%s/%u-Confidence.txt", strDir.c_str(), nCaptureIndex);
		SaveDepthText(tofFrameData->pConfidence, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile, false);

		sprintf(szFile, "%s/%u-Confidence.u8", strDir.c_str(), nCaptureIndex);
		SaveGray_2_U8(tofFrameData->pConfidence, tofFrameData->frameWidth, tofFrameData->frameHeight, GRAY_FORMAT_FLOAT, szFile);
	}
	//
	if (NULL != tofFrameData->pRgbD)
	{
		sprintf(szFile, "%s/%u-RgbD.%s", strDir.c_str(), nCaptureIndex, "bgr");
		Utils_SaveBufToFile(tofFrameData->pRgbD, nPixelCnt * sizeof(tofFrameData->pRgbD[0]), szFile, false);

		if (NULL != tofFrameData->pPointData)
		{
			sprintf(szFile, "%s/%u-PointDataColor.%s", strDir.c_str(), nCaptureIndex, "obj");
			SaveColorPointDataXYZText(tofFrameData->pPointData, tofFrameData->pRgbD, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile);
		}
	}
	//
	if ((NULL != tofFrameData->pRawData) && (0 < tofFrameData->nRawDataLen))
	{
		sprintf(szFile, "%s/%u-Tof.%s", strDir.c_str(), nCaptureIndex, "raw");
		Utils_SaveBufToFile(tofFrameData->pRawData, tofFrameData->nRawDataLen, szFile, false);
	}
	//
	if ((NULL != tofFrameData->pExtData) && (0 < tofFrameData->nExtDataLen))
	{
		sprintf(szFile, "%s/%u-Tof.%s", strDir.c_str(), nCaptureIndex, "extdata");
		Utils_SaveBufToFile(tofFrameData->pExtData, tofFrameData->nExtDataLen, szFile, false);
	}
}
static void CaptureRgbFrame(const std::string& strDir, const unsigned int nCaptureIndex, RgbFrameData *rgbFrameData)
{
	char szFile[512] = { 0 };

	//
	if ((NULL != rgbFrameData->pFrameData) && (0 < rgbFrameData->nFrameLen))
	{
		sprintf(szFile, "%s/%u-Rgb.%s", strDir.c_str(), nCaptureIndex, StringColorFormat(rgbFrameData->formatType));
		Utils_SaveBufToFile(rgbFrameData->pFrameData, rgbFrameData->nFrameLen, szFile, false);
	}
	//
	if ((NULL != rgbFrameData->pExtData) && (0 < rgbFrameData->nExtDataLen))
	{
		sprintf(szFile, "%s/%u-Rgb.%s", strDir.c_str(), nCaptureIndex, "extdata");
		Utils_SaveBufToFile(rgbFrameData->pExtData, rgbFrameData->nExtDataLen, szFile, false);
	}
}
static void CaptureImuFrame(const std::string& strDir, const unsigned int nCaptureIndex, ImuFrameData *imuFrameData)
{
	char szFile[512] = { 0 };

	//
	if (NULL != imuFrameData)
	{
		sprintf(szFile, "%s/%u-Imu.dat", strDir.c_str(), /*nCaptureIndex*/1);
		Utils_SaveBufToFile(imuFrameData, sizeof(ImuFrameData), szFile, /*false*/true);//固定写到同一个文件里就好了

		sprintf(szFile, "%s/%u-Imu.txt", strDir.c_str(), /*nCaptureIndex*/1);//固定写到同一个文件里就好了
		SaveImuText(imuFrameData, szFile, /*false*/true);
	}
}



static void CallBackTofDeviceStatus(TOFDEV_STATUS tofDevStatus, void *pUserData)
{
	printf("device status: %d.\n", tofDevStatus);
}

static void publishPointCloud(TofFrameData *tofFrameData, UINT32 frame_count)
{
	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;

	pclCloud.points.clear();
	pcl::PointXYZ singlePoint;

	for (int i = 0; i < width * height; ++i)
	{
		singlePoint.x = tofFrameData->pPointData[i].x;
		singlePoint.y = tofFrameData->pPointData[i].y;
		singlePoint.z = tofFrameData->pPointData[i].z;

		pclCloud.points.push_back(singlePoint);
	}

	pcl::toROSMsg(pclCloud, cloudMsg);
	cloudMsg.header.frame_id = "camera_point_cloud";
	cloudMsg.header.stamp = ros::Time::now();
	pub_pointCloud.publish(cloudMsg);
}

static void publishDepthzImage(TofFrameData *tofFrameData, UINT32 frame_count)
{
	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;

	cv::Mat depthzMat = cv::Mat(height, width, CV_16UC1);
	for (UINT32 i = 0; i < height; ++i)
	{
		for (UINT32 j = 0; j < width; ++j)
		{
			depthzMat.at<UINT16>(i, j) = static_cast<unsigned short>(tofFrameData->pPointData[i * width + j].z * 1000);
		}
	}
	depthzMsg.reset(new sensor_msgs::Image);
	depthzMsg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depthzMat).toImageMsg();
	depthzMsg->header.frame_id = "camera_depthz_frame";
	depthzMsg->header.stamp = ros::Time::now();
	depthzMsg->height = height;
	depthzMsg->width = width;
	depthzMsg->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
	depthzMsg->is_bigendian = false;
	depthzMsg->step = depthzMsg->width * 2; // 2 bytes per pixel
	pub_depthz.publish(depthzMsg);
}

static void publishGrayImage(TofFrameData *tofFrameData, UINT32 frame_count)
{
	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;
	const GRAY_FORMAT format = tofFrameData->grayFormat;
	void* pGray = tofFrameData->pGrayData;

	unsigned char* pGrayU8 = new unsigned char[width * height];//U8
	CGrayConvert::Gray_2_U8(pGray, width, height, format, pGrayU8);

	cv::Mat grayMat = cv::Mat(height, width, CV_8UC1);
	for (UINT32 i = 0; i < height; ++i)
	{
		for (UINT32 j = 0; j < width; ++j)
		{
			grayMat.at<UINT8>(i, j) = static_cast<unsigned char>(pGrayU8[i * width + j]);
		}
	}
	SAFE_DELETE_ARRY(pGrayU8);


	grayMsg.reset(new sensor_msgs::Image);
	grayMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", grayMat).toImageMsg();
	grayMsg->header.frame_id = "camera_gray_frame";
	grayMsg->header.stamp = ros::Time::now();
	grayMsg->height = height;
	grayMsg->width = width;
	grayMsg->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
	grayMsg->is_bigendian = false;
	grayMsg->step = grayMsg->width * 1; // 1 bytes per pixel
	pub_gray.publish(grayMsg);


}

static void publishRgbdImage(TofFrameData *tofFrameData, UINT32 frame_count)
{
	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;

	cv::Mat rgbMat = cv::Mat(height, width, CV_8UC3);
	for (UINT32 i = 0; i < height; ++i)
	{
		for (UINT32 j = 0; j < width; ++j)
		{
			rgbMat.at<cv::Vec3b>(i, j)[0] = tofFrameData->pRgbD[i * width + j].b; //B
			rgbMat.at<cv::Vec3b>(i, j)[1] = tofFrameData->pRgbD[i * width + j].g; //G
			rgbMat.at<cv::Vec3b>(i, j)[2] = tofFrameData->pRgbD[i * width + j].r; //R
		}
	}
	rgbdMsg.reset(new sensor_msgs::Image);
	rgbdMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbMat).toImageMsg();
	rgbdMsg->header.frame_id = "camera_rgbd_frame";
	rgbdMsg->header.stamp = ros::Time::now();
	rgbdMsg->height = height;
	rgbdMsg->width = width;
	rgbdMsg->encoding = sensor_msgs::image_encodings::TYPE_8UC3;
	rgbdMsg->is_bigendian = false;
	rgbdMsg->step = rgbdMsg->width * 3; // 3 bytes per pixel
	pub_rgbd.publish(rgbdMsg);
}


static void publishRgbImage(RgbFrameData *rgbFrameData, UINT32 frame_count)
{
	const UINT32 width = rgbFrameData->frameWidth;
	const UINT32 height = rgbFrameData->frameHeight;

	cv::Mat rgbMat = cv::Mat(height, width, CV_8UC3);
	//opencv默认bgr排列的，需要把sdk传出的rgb排列的数据转一下
	//memcpy(rgbMat.data, rgbFrameData.pFrameData, rgbFrameData.nFrameLen);
	for (int i = 0; i < width * height * 3; i += 3)
	{
		rgbMat.data[i + 0] = rgbFrameData->pFrameData[i + 2];
		rgbMat.data[i + 1] = rgbFrameData->pFrameData[i + 1];
		rgbMat.data[i + 2] = rgbFrameData->pFrameData[i + 0];
	}

	rgbMsg.reset(new sensor_msgs::Image);
	rgbMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbMat).toImageMsg();
	rgbMsg->header.frame_id = "camera_rgb_frame";
	rgbMsg->header.stamp = ros::Time::now();
	rgbMsg->height = height;
	rgbMsg->width = width;
	rgbMsg->encoding = sensor_msgs::image_encodings::TYPE_8UC3;
	rgbMsg->is_bigendian = false;
	rgbMsg->step = rgbMsg->width * 3; // 3 bytes per pixel
	pub_rgb.publish(rgbMsg);
}

static void PublishTofFrame(const unsigned int nCaptureIndex, TofFrameData *tofFrameData)
{
	if (NULL != tofFrameData->pPointData)
	{
		publishPointCloud(tofFrameData, nCaptureIndex);
		publishDepthzImage(tofFrameData, nCaptureIndex);
	}

	if (NULL != tofFrameData->pGrayData)
	{
		publishGrayImage(tofFrameData, nCaptureIndex);
	}

	if (NULL != tofFrameData->pRgbD)
	{
		publishRgbdImage(tofFrameData, nCaptureIndex);
	}
	
}
static void PublishRgbFrame(const unsigned int nCaptureIndex, RgbFrameData *rgbFrameData)
{
	if (NULL != rgbFrameData->pFrameData)
	{
		publishRgbImage(rgbFrameData, nCaptureIndex);
	}
	
}

static void fnTofStream(TofFrameData *tofFrameData, void* pUserData)
{
	UINT32* frame_count = (UINT32*)pUserData;
	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;

	(*frame_count)++;

	//
	const float fDepthZAvg = CalCenterPointDataZAvg(tofFrameData->pPointData, tofFrameData->frameWidth, tofFrameData->frameHeight);
	// printf("[%u], one TOF frame, time=%llu, center depthZ = %0.3f m.\n", *frame_count, Utils_GetTickCount(), fDepthZAvg);

	PublishTofFrame((*frame_count), tofFrameData);
	return;

	CaptureTofFrame(std::string("./Capture"), (*frame_count), tofFrameData);

}

static void fnRgbStream(RgbFrameData *rgbFrameData, void* pUserData)
{
	UINT32* frame_count = (UINT32*)pUserData;
	const UINT32 width = rgbFrameData->frameWidth;
	const UINT32 height = rgbFrameData->frameHeight;

	(*frame_count)++;

	printf("[%u], one RGB frame, time=%llu, formatType=%d, %d.\n", *frame_count, Utils_GetTickCount(), rgbFrameData->formatType, rgbFrameData->formatTypeOrg);

	PublishRgbFrame((*frame_count), rgbFrameData);
	return;

	CaptureRgbFrame(std::string("./Capture"), (*frame_count), rgbFrameData);

}

static void fnImuStream(ImuFrameData *imuFrameData, void* pUserData)
{
	UINT32* frame_count = (UINT32*)pUserData;

	(*frame_count)++;

	if (1 == ((*frame_count) % 100))//控制下打印频率
	{
		printf("[%u], one IMU frame, time=%llu.\n", *frame_count, Utils_GetTickCount());
	}
	return;

	CaptureImuFrame(std::string("./Capture"), (*frame_count), imuFrameData);

}

static void PrintDevInfo(TofDeviceInfo *pTofDeviceInfo)
{
	printf("Dev Info:==================================\n");
	printf(">>  szDevName=%s.\n", pTofDeviceInfo->szDevName);
	printf(">>  szDevId=%s.\n", pTofDeviceInfo->szDevId);
	printf(">>  szFirmwareVersion=%s.\n", pTofDeviceInfo->szFirmwareVersion);
	printf("Dev Info==================================\n\n");
}

static void GetTofModeList(const UINT32 supportedTOFMode, std::vector<TOF_MODE>& modeList)
{
	const int bitCnt = (sizeof(supportedTOFMode) * 8);
	int tofMode = 0;

	modeList.clear();
	for (int i = 0; i < bitCnt; i++)
	{
		tofMode = 1 << i;
		if (supportedTOFMode & tofMode)
		{
			modeList.push_back((TOF_MODE)tofMode);
		}
	}
}

static TOF_MODE ChoseTofMode(const UINT32 supportedTOFMode)
{
	std::vector<TOF_MODE> tofModeList;
	GetTofModeList(supportedTOFMode, tofModeList);

	//打印出所有支持的TOF模式，供用户选择
	printf("chose tof mode from list: \n");
	printf(">>  number: mode.\n");
	for (UINT32 i = 0; i < tofModeList.size(); i++)
	{
		printf(">>  %u: %s.\n", i, TofMode2Str(tofModeList[i]));
	}

	if (1 == tofModeList.size())//只有一种模式，直接返回，省去输入的过程
	{
		return tofModeList[0];
	}

#if 0
	//这里简单处理，选择第一种支持的模式
	return tofModeList[0];
#else
	//用于选择哪一种tof模式
	while (1)
	{
		printf("input mode (number) >>");

		std::string strInput;
		std::cin >> strInput;

		const UINT32 i = (UINT32)strtol(strInput.c_str(), NULL, 10);
		if (tofModeList.size() > i)
		{
			return tofModeList[i];
		}
		else
		{
			printf("the number is invalid.\n");
		}
	}
#endif

}


static bool OpenStream(HTOFD hTofD, TofDeviceInfo* pCaps, const TOF_MODE tofMode, 
	UINT32* tof_frame_count, UINT32* rgb_frame_count, UINT32* imu_frame_count)
{
	(*tof_frame_count) = 0;
	TOFRET retVal = TOFD_StartTofStream(hTofD, tofMode, fnTofStream, tof_frame_count);
	if ((TOFRET_SUCCESS != retVal) && (TOFRET_SUCCESS_READING_CALIB != retVal))
	{
		printf("start TOF stream, [ FAILED ], retVal=0x%08x!!!!!!!!!!!!!!!!!!!!!\n\n", retVal);
		return false;
	}

	if (pCaps->bRgbSupported)//支持RGB的情况下
	{
		(*rgb_frame_count) = 0;
		TOFRET retVal = TOFD_StartRgbStream(hTofD, fnRgbStream, rgb_frame_count);
		if (TOFRET_SUCCESS != retVal)
		{
			printf("start RGB stream, [ FAILED ], retVal=0x%08x!!!!!!!!!!!!!!!!!!!!!\n\n", retVal);
			return false;
		}
	}

	if (pCaps->bImuSupported)//支持IMU的情况下
	{
		(*imu_frame_count) = 0;
		TOFRET retVal = TOFD_StartImuStream(hTofD, fnImuStream, imu_frame_count);
		if (TOFRET_SUCCESS != retVal)
		{
			printf("start IMU stream, [ FAILED ], retVal=0x%08x!!!!!!!!!!!!!!!!!!!!!\n\n", retVal);
			return false;
		}
	}

	return true;

}

static void CloseStream(HTOFD hTofD, TofDeviceInfo* pCaps)
{
	TOFD_StopTofStream(hTofD);

	if (pCaps->bRgbSupported)//支持RGB的情况下
	{
		TOFD_StopRgbStream(hTofD);
	}
	if (pCaps->bImuSupported)//支持IMU的情况下
	{
		TOFD_StopImuStream(hTofD);
	}
}

static void PrintfTofLensParameter(TofModuleLensParameterV20* pTofLens)
{
	if (NULL == pTofLens)  return;

	const UINT32 nIndex = pTofLens->nIndex;

	if (1 == nIndex)
	{
		TofModuleLensGeneral* pTmp = &(pTofLens->uParam.general);

		printf("Lens Paramter (general):...............................\n");
		printf(">>   fx = %f.\n", pTmp->fx);
		printf(">>   fy = %f.\n", pTmp->fy);
		printf(">>   cx = %f.\n", pTmp->cx);
		printf(">>   cy = %f.\n", pTmp->cy);
		printf(">>   k1 = %f.\n", pTmp->k1);
		printf(">>   k2 = %f.\n", pTmp->k2);
		printf(">>   p1 = %f.\n", pTmp->p1);
		printf(">>   p2 = %f.\n", pTmp->p2);
		printf(">>   k3 = %f.\n", pTmp->k3);
	}
	else if (2 == nIndex)
	{
		TofModuleLensFishEye* pTmp = &(pTofLens->uParam.fishEye);

		printf("Lens Paramter (fishEye):...............................\n");
		printf(">>   fx = %f.\n", pTmp->fx);
		printf(">>   fy = %f.\n", pTmp->fy);
		printf(">>   cx = %f.\n", pTmp->cx);
		printf(">>   cy = %f.\n", pTmp->cy);
		printf(">>   k1 = %f.\n", pTmp->k1);
		printf(">>   k2 = %f.\n", pTmp->k2);
		printf(">>   k3 = %f.\n", pTmp->k3);
		printf(">>   k4 = %f.\n", pTmp->k4);
	}
	else
	{
		printf("Lens Paramter (index=%u):...............................\n", nIndex);
		printf(">>   unknown, not supported.\n");
	}
}
static void PrintfPixelOffset(TofFrameDataPixelOffset* pOffset)
{
	if (NULL == pOffset)  return;

	printf("Tof Frame Data Pixel Offset:...............................\n");
	printf(">>   nOffset = %u.\n", pOffset->nOffset);
}
static void PrintfDepthCalRoi(DepthCalRoi* Roi)
{
	if (NULL == Roi)  return;

	printf("Depth Cal Roi:...............................\n");
	printf(">>    struMax.left=%u.\n", Roi->struMax.left);
	printf(">>    struMax.right=%u.\n", Roi->struMax.right);
	printf(">>    struMax.top=%u.\n", Roi->struMax.top);
	printf(">>    struMax.bottom=%u.\n", Roi->struMax.bottom);
	printf(">>    struDefault.left=%u.\n", Roi->struDefault.left);
	printf(">>    struDefault.right=%u.\n", Roi->struDefault.right);
	printf(">>    struDefault.top=%u.\n", Roi->struDefault.top);
	printf(">>    struDefault.bottom=%u.\n", Roi->struDefault.bottom);
	printf(">>    struCurrent.left=%u.\n", Roi->struCurrent.left);
	printf(">>    struCurrent.right=%u.\n", Roi->struCurrent.right);
	printf(">>    struCurrent.top=%u.\n", Roi->struCurrent.top);
	printf(">>    struCurrent.bottom=%u.\n", Roi->struCurrent.bottom);

}


static void GetOrSetSomeParam(HTOFD hTofD, TofDeviceInfo* pCaps, const TOF_MODE tofMode)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;

#if 1
	if (pCaps->supportedTofExpMode & EXP_MODE_AUTO)
	{
		if (TOFRET_SUCCESS != (retVal = TOFD_SetTofAE(hTofD, true)))//按需，或者联系沟通后确认
		{
			printf("TOFD_SetTofAE failed, retVal=0x%08x.\n", retVal);
		}
	}

	if (TOFRET_SUCCESS != (retVal = TOFD_SetTofHDRZ(hTofD, false)))//按需，或者联系沟通后确认
	{
		printf("TOFD_SetTofHDRZ failed, retVal=0x%08x.\n", retVal);
	}
#endif

#if 1 //滤波，按需，或者联系沟通后确认
	for (UINT32 i = 0; i < 32; i++)
	{
		UINT32 type = (1 << i);
		if (0 != (pCaps->supportedTOFFilter & type))
		{
			if (TOFRET_SUCCESS != (retVal = TOFD_SetTofFilter(hTofD, (const TOF_FILTER)type, true)))
			{
				printf("TOFD_SetTofFilter failed, retVal=0x%08x.\n", retVal);
			}
		}
	}
#endif

#if 1 //按需，或者联系沟通后确认
	if (pCaps->bTofRemoveINSSupported)
	{
		bool bEnable = false;
		if ((TOF_MODE_HDRZ_5FPS == tofMode) || (TOF_MODE_HDRZ_10FPS == tofMode)
			|| (TOF_MODE_HDRZ_15FPS == tofMode) || (TOF_MODE_HDRZ_30FPS == tofMode)
			|| (TOF_MODE_HDRZ_45FPS == tofMode) || (TOF_MODE_HDRZ_60FPS == tofMode))
		{	//注意：目前只有17相位的才调试过效果，所以开放，其他相位的效果没调试过，所以不开放
			bEnable = true;
		}
		if (TOFRET_SUCCESS != (retVal = TOFD_SetTofRemoveINS(hTofD, bEnable)))
		{
			printf("TOFD_SetTofRemoveINS (%d) failed, retVal=0x%08x.\n", bEnable, retVal);
		}
	}
#endif
#if 0
	TofExpouse struExp;
	memset(&struExp, 0, sizeof(struExp));
	if (TOFRET_SUCCESS != (retVal = TOFD_GetTofExpTime(hTofD, &struExp)))
	{
		printf("TOFD_GetTofExpTime failed, retVal=0x%08x.\n", retVal);
	}

	if (TOFRET_SUCCESS != (retVal = TOFD_SetTofExpTime(hTofD, struExp.nCurrent)))
	{
		printf("TOFD_SetTofExpTime failed, retVal=0x%08x.\n", retVal);
	}
#endif

	TofDeviceParamV20 struParam;

	//打印出内参
	memset(&struParam, 0, sizeof(struParam));
	struParam.type = TOF_DEV_PARAM_TofLensParameterV20;
	if (TOFRET_SUCCESS == (retVal = TOFD_GetDeviceParamV20(hTofD, &struParam)))
	{
		PrintfTofLensParameter(&(struParam.uParam.struTofLensParameterV20));
	}

	//打印TOF回调函数里输出的TOF数据相对于RAW数据的像素偏移个数
	memset(&struParam, 0, sizeof(struParam));
	struParam.type = TOF_DEV_PARAM_TofFrameDataPixelOffset;
	if (TOFRET_SUCCESS == (retVal = TOFD_GetDeviceParamV20(hTofD, &struParam)))
	{
		PrintfPixelOffset(&(struParam.uParam.struPixelOffset));
	}

	//打印深度计算的区域
	memset(&struParam, 0, sizeof(struParam));
	struParam.type = TOF_DEV_PARAM_DepthCalRoi;
	if (TOFRET_SUCCESS == (retVal = TOFD_GetDeviceParamV20(hTofD, &struParam)))
	{
		PrintfDepthCalRoi(&(struParam.uParam.struDepthCalRoi));
	}

}

static void SaveSomeData(HTOFD hTofD, std::string& strSaveDir)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;
	TofDeviceParamV20 struParam;
	std::string strPath;

	//
	TofDeviceInfo struCaps;
	memset(&struCaps, 0, sizeof(struCaps));
	TOFD_GetDeviceInfo(hTofD, &struCaps);

	//保存标定文件
	memset(&struParam, 0, sizeof(struParam));
	struParam.type = TOF_DEV_PARAM_TofCalibData;
	if (TOFRET_SUCCESS == (retVal = TOFD_GetDeviceParamV20(hTofD, &struParam)))
	{
		TofCalibData* pTmp = &(struParam.uParam.struTofCalibData);
		strPath = (strSaveDir + std::string("/") + std::string(struCaps.szDevId) + std::string(".bin"));
		Utils_SaveBufToFile(pTmp->pData, pTmp->nDataLen, strPath.c_str(), false);
	}


}

static void PrintfCmdUsage()
{
	printf("\ntof camera started:==================================\n");
	// printf(">> s: exit demo.\n");
	// printf("\n");
}

static void ThreadTestDemo(HTOFD hTofD, std::string strSaveDir)
{
	TofDeviceInfo struCaps;
	memset(&struCaps, 0, sizeof(struCaps));
	TOFD_GetDeviceInfo(hTofD, &struCaps);
	PrintDevInfo(&struCaps);

	const TOF_MODE tofMode = ChoseTofMode(struCaps.supportedTOFMode);//选择其中一种TOF模式出TOF数据

	//
	UINT32 tof_frame_count = 0, rgb_frame_count = 0, imu_frame_count = 0;
	const bool bSuc = OpenStream(hTofD, &struCaps, tofMode, &tof_frame_count, &rgb_frame_count, &imu_frame_count);
	if (bSuc)
	{
		//成功后，等到数据流拿到后再去设置参数，这样才能保证生效
		while (0 >= tof_frame_count)
		{
			printf("Waiting for stream data...\n");
			std::this_thread::sleep_for(std::chrono::milliseconds(1000)); //单位是毫秒
		}

		printf("Get Or Set Some Params...\n");
		std::this_thread::sleep_for(std::chrono::milliseconds(2000)); //单位是毫秒
		GetOrSetSomeParam(hTofD, &struCaps, tofMode);
	}

	//处理键盘输入命令
	 const std::string strExitCode = "s";
	while (1)
	{
		PrintfCmdUsage();
	     std::string strInput;
		 std::cin >> strInput;
		 if (strExitCode == strInput)
		 {
			 break;
		 }
		 else
		 {
			// printf("unknown cmd: %s, skip.\n", strInput.c_str());
		 }
	}

	//等待线程退出
	CloseStream(hTofD, &struCaps);

	if ("" != strSaveDir)
	{
		SaveSomeData(hTofD, strSaveDir);//保存一些文件，便于问题排查
	}
}


static void DoTestDemo(TofDeviceDescriptor* pDevsDesc, std::string& strSaveDir)
{
	HTOFD hTofD = TOFD_OpenDevice(pDevsDesc, CallBackTofDeviceStatus, NULL);
	if (NULL == hTofD)
	{
		printf("Open Tof Device failed.\n");
		return;
	}

	std::thread thread_test = std::thread(ThreadTestDemo, hTofD, strSaveDir);
	thread_test.join();

	TOFD_CloseDevice(hTofD);

}

static UINT32 ChoseDev(const UINT32  dev_cnt)
{
	const UINT32 min_index = 1;
	const UINT32 max_index = dev_cnt;

	if (1 == max_index)//只有一个设备的话就不用选择了
	{
		return max_index;
	}

	UINT32 dev_index = 1;
	while (1)
	{
		printf("please chose a dev (min=%d, max:%d):\n", min_index, max_index);
		printf(">>");

		std::string strInput;
		std::cin >> strInput;

		dev_index = (UINT32)strtol(strInput.c_str(), NULL, 10);
		if ((min_index <= dev_index) && (max_index >= dev_index))
		{
			break;
		}
		printf("invalid dev index:%d.\n", dev_index);
	}

	return dev_index;
}

/*
用例功能简述：选择某一个设备的某一种模式后，进行取流测试，适用linux下的ROS系统。
*/
int main(int argc, char **argv)
{
	printf("*********************start test*************************\n");

	//初始化节点
	ros::init(argc, argv, "publisher_node");

	ros::NodeHandle nh;

	//init camera indo publisher
	pub_info = nh.advertise<sensor_msgs::CameraInfo>("sunny_topic/caminfo", 5);
	//init point clouds publisher
	pub_pointCloud = nh.advertise<sensor_msgs::PointCloud2>("sunny_topic/tof_frame/pointcloud", 5); 
	//init depthz publisher
	pub_depthz = nh.advertise<sensor_msgs::Image>("sunny_topic/tof_frame/depthz", 5);
	//init gray publisher
	pub_gray = nh.advertise<sensor_msgs::Image>("sunny_topic/tof_frame/gray", 5);
	//init rgbd publisher
	pub_rgbd = nh.advertise<sensor_msgs::Image>("sunny_topic/tof_frame/rgbd", 5);
	//init rgb publisher
	pub_rgb = nh.advertise<sensor_msgs::Image>("sunny_topic/rgb_frame/rgb", 5);


	std::string strSaveDir = ("");//(".");//用于保存文件的目录，可以自己指定，为空则表示不保存文件

	TofDevInitParam struInitParam;
	memset(&struInitParam, 0, sizeof(struInitParam));
	strncpy(struInitParam.szDepthCalcCfgFileDir, "/home/jhr/catkin_ws/devel/lib/tof_dev_sdk_demo/parameter", sizeof(struInitParam.szDepthCalcCfgFileDir) - 1);
	struInitParam.bSupUsb = true;
	struInitParam.bSupNetWork = true;
	struInitParam.bSupSerialCOM = true;
	if (struInitParam.bSupSerialCOM)
	{
#ifdef WIN32
		//strncpy(struInitParam.szSerialDev, "COM1", sizeof(struInitParam.szSerialDev));//windows下可以不用赋值
#elif defined LINUX 
		strncpy(struInitParam.szSerialDev, "/dev/ttyUSB0", sizeof(struInitParam.szSerialDev));//linux下必须赋值一个实际使用的串口设备，这里随便写了一个
#endif
	}
	struInitParam.bWeakAuthority = false;
	struInitParam.bDisablePixelOffset = false;
	strncpy(struInitParam.szLogFile, "./tof_dev_sdk_log.txt", sizeof(struInitParam.szLogFile));//不赋值则不记录日志到文件
	TOFD_Init(&struInitParam);

	printf("SDK Version: %s.\n", TOFD_GetSDKVersion());

	TofDeviceDescriptor* pDevsDescList = NULL;
	UINT32 dev_num = 0;
	TOFD_SearchDevice(&pDevsDescList, &dev_num);
	if (0 < dev_num )
	{
		const UINT32 dev_index = ChoseDev(dev_num) - 1;//决定测试哪一个设备
		DoTestDemo(pDevsDescList + dev_index, strSaveDir);
	}
	else
	{
		printf("can not find tof device!\n");
	}

	TOFD_Uninit();

	printf("*********************stop test*********************\n");

#ifdef WIN32 //防止控制台自动退出而看不到历史日志信息
	printf("please input anything to finish....");
	system("pause");
#endif

	return 0;
}




