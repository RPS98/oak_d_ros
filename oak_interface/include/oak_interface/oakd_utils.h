#ifndef OAKD_UTILS
#define OAKD_UTILS

#include "oakd_main.h"

class OakDUtils
{
public:
    // From Dai-core: From dai::ImgFrame to cv::Mat
    static cv::Mat getFrame(std::shared_ptr<dai::ImgFrame> frame_dai,bool deepCopy);

    static cv::Mat getCvFrame(std::shared_ptr<dai::ImgFrame> frame_dai);
    
    // From Dai-bridge: From dai::ImgFrame to sensor_msgs::Image
    static void planarToInterleaved(const std::vector<uint8_t> &srcData,
                            std::vector<uint8_t> &destData, int w,
                            int h, int numPlanes, int bpp);

    static void getRosMsg(std::shared_ptr<dai::ImgFrame> inData,
                sensor_msgs::Image &outImageMsg);

    // Own functions: From dai::ImgFrame to cv::Mat to sensor_msgs::Image
    static cv::Mat imgframe_to_mat(std::shared_ptr<dai::ImgFrame> frame, int data_type);

    static sensor_msgs::Image dai_to_ros(std_msgs::Header header,
                                std::shared_ptr<dai::ImgFrame> frame);

};

#endif