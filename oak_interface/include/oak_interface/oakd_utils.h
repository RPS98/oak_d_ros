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

    static void interleavedToPlanar(const std::vector<uint8_t> &srcData,
                                    std::vector<uint8_t> &destData, int w,
                                    int h, int numPlanes, int bpp);

    static void getRosMsg(std::shared_ptr<dai::ImgFrame> inData,
                         sensor_msgs::Image &outImageMsg, 
                         bool daiInterleaved);

};

#endif