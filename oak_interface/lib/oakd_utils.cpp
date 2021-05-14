#include <oak_interface/oakd_utils.h>

///// FROM DAI-CORE /////
// ENCOGIND TYPES 
/*
enum class Type {
        YUV422i,        // interleaved 8 bit
        YUV444p,        // planar 4:4:4 format
        YUV420p,        // planar 4:2:0 format
        YUV422p,        // planar 8 bit
        YUV400p,        // 8-bit greyscale
        RGBA8888,       // RGBA interleaved stored in 32 bit word
        RGB161616,      // Planar 16 bit RGB data
        RGB888p,        // Planar 8 bit RGB data
        BGR888p,        // Planar 8 bit BGR data
        RGB888i,        // Interleaved 8 bit RGB data
        BGR888i,        // Interleaved 8 bit BGR data
        RGBF16F16F16p,  // Planar FP16 RGB data
        BGRF16F16F16p,  // Planar FP16 BGR data
        RGBF16F16F16i,  // Interleaved FP16 RGB data
        BGRF16F16F16i,  // Interleaved FP16 BGR data
        GRAY8,          // 8 bit grayscale (1 plane)
        GRAYF16,        // FP16 grayscale (normalized)
        LUT2,           // 1 bit  per pixel, Lookup table
        LUT4,           // 2 bits per pixel, Lookup table
        LUT16,          // 4 bits per pixel, Lookup table
        RAW16,          // save any raw type (8, 10, 12bit) on 16 bits
        RAW14,          // 14bit value in 16bit storage
        RAW12,          // 12bit value in 16bit storage
        RAW10,          // 10bit value in 16bit storage
        RAW8,
        PACK10,  // 10bit packed format
        PACK12,  // 12bit packed format
        YUV444i,
        NV12,
        NV21,
        BITSTREAM,  // used for video encoder bitstream
        HDR,
        NONE
    }; */

// Get dai::ImgFrame frame and its info
cv::Mat OakDUtils::getFrame(std::shared_ptr<dai::ImgFrame> frame_dai,bool deepCopy) {
    // Convert to cv::Mat. If deepCopy enabled, then copy pixel data, otherwise reference only
    cv::Mat mat;
    cv::Size size = {0, 0};
    int type = 0;

    switch(frame_dai->getType()) {
        case dai::ImgFrame::Type::RGB888i:
        case dai::ImgFrame::Type::BGR888i:
        case dai::ImgFrame::Type::BGR888p:
        case dai::ImgFrame::Type::RGB888p:
            size = cv::Size(frame_dai->getWidth(), frame_dai->getHeight());
            type = CV_8UC3;
            break;

        case dai::ImgFrame::Type::YUV420p:
        case dai::ImgFrame::Type::NV12:
        case dai::ImgFrame::Type::NV21:
            size = cv::Size(frame_dai->getWidth(), frame_dai->getHeight() * 3 / 2);
            type = CV_8UC1;
            break;

        case dai::ImgFrame::Type::RAW8:
        case dai::ImgFrame::Type::GRAY8:
            size = cv::Size(frame_dai->getWidth(), frame_dai->getHeight());
            type = CV_8UC1;
            break;

        case dai::ImgFrame::Type::GRAYF16:
            size = cv::Size(frame_dai->getWidth(), frame_dai->getHeight());
            type = CV_16SC1; // cambiar S por F
            break;

        case dai::ImgFrame::Type::RAW16:
            size = cv::Size(frame_dai->getWidth(), frame_dai->getHeight());
            type = CV_16UC1;
            break;

        case dai::ImgFrame::Type::RGBF16F16F16i:
        case dai::ImgFrame::Type::BGRF16F16F16i:
        case dai::ImgFrame::Type::RGBF16F16F16p:
        case dai::ImgFrame::Type::BGRF16F16F16p:
            size = cv::Size(frame_dai->getWidth(), frame_dai->getHeight());
            type = CV_16SC3; // cambiar S por F
            break;

        case dai::RawImgFrame::Type::BITSTREAM:
        default:
            size = cv::Size(static_cast<int>(frame_dai->getData().size()), 1);
            type = CV_8UC1;
            break;
    }

    // Check if enough data
    long requiredSize = CV_ELEM_SIZE(type) * size.area();
    if(static_cast<long>(frame_dai->getData().size()) < requiredSize) {
        throw std::runtime_error("ImgFrame doesn't have enough data to encode specified frame. Maybe metadataOnly transfer was made?");
    }
    if(frame_dai->getWidth() <= 0 || frame_dai->getHeight() <= 0) {
        throw std::runtime_error("ImgFrame metadata not valid (width or height = 0)");
    }

    // Copy or reference to existing data
    if(deepCopy) {
        // Create new image data
        mat.create(size, type);
        // Copy number of bytes that are available by Mat space or by img data size
        std::memcpy(mat.data, frame_dai->getData().data(), std::min((long)(frame_dai->getData().size()), (long)(mat.dataend - mat.datastart)));
    } else {
        mat = cv::Mat(size, type, frame_dai->getData().data());
    }

    return mat;
}

// Get dai::ImgFrame frame and convert it into cv::Mat
cv::Mat OakDUtils::getCvFrame(std::shared_ptr<dai::ImgFrame> frame_dai) {
    cv::Mat frame = getFrame(frame_dai,true);
    cv::Mat output;

    switch(frame_dai->getType()) {
        case dai::RawImgFrame::Type::RGB888i:
            cv::cvtColor(frame, output, cv::ColorConversionCodes::COLOR_RGB2BGR);
            break;

        case dai::RawImgFrame::Type::BGR888i:
            output = frame.clone();
            break;

        case dai::RawImgFrame::Type::RGB888p: {
            cv::Size s(frame_dai->getWidth(), frame_dai->getHeight());
            std::vector<cv::Mat> channels;
            // RGB
            channels.push_back(cv::Mat(s, CV_8UC1, frame_dai->getData().data() + s.area() * 2));
            channels.push_back(cv::Mat(s, CV_8UC1, frame_dai->getData().data() + s.area() * 1));
            channels.push_back(cv::Mat(s, CV_8UC1, frame_dai->getData().data() + s.area() * 0));
            cv::merge(channels, output);
        } break;

        case dai::RawImgFrame::Type::BGR888p: {
            cv::Size s(frame_dai->getWidth(), frame_dai->getHeight());
            std::vector<cv::Mat> channels;
            // BGR
            channels.push_back(cv::Mat(s, CV_8UC1, frame_dai->getData().data() + s.area() * 0));
            channels.push_back(cv::Mat(s, CV_8UC1, frame_dai->getData().data() + s.area() * 1));
            channels.push_back(cv::Mat(s, CV_8UC1, frame_dai->getData().data() + s.area() * 2));
            cv::merge(channels, output);
        } break;

        case dai::RawImgFrame::Type::YUV420p:
            cv::cvtColor(frame, output, cv::ColorConversionCodes::COLOR_YUV420p2BGR);
            break;

        case dai::RawImgFrame::Type::NV12:
            cv::cvtColor(frame, output, cv::ColorConversionCodes::COLOR_YUV2BGR_NV12);
            break;

        case dai::RawImgFrame::Type::NV21:
            cv::cvtColor(frame, output, cv::ColorConversionCodes::COLOR_YUV2BGR_NV21);
            break;

        case dai::RawImgFrame::Type::RAW8:
        case dai::RawImgFrame::Type::RAW16:
        case dai::RawImgFrame::Type::GRAY8:
        case dai::RawImgFrame::Type::GRAYF16:
            output = frame.clone();
            break;

        default:
            output = frame.clone();
            break;
    }

    return output;
}


///// FROM DAI-BRIDGE /////
// Planar image to Interleaved (from dai::ImageConverter)
void OakDUtils::planarToInterleaved(const std::vector<uint8_t> &srcData,
                         std::vector<uint8_t> &destData, int w,
                         int h, int numPlanes, int bpp) {

  if (numPlanes == 3) {
    // optimization (cache)
    for (int i = 0; i < w * h; i++) {
      uint8_t b = srcData.data()[i + w * h * 0];
      destData[i * 3 + 0] = b;
    }
    for (int i = 0; i < w * h; i++) {
      uint8_t g = srcData.data()[i + w * h * 1];
      destData[i * 3 + 1] = g;
    }
    for (int i = 0; i < w * h; i++) {
      uint8_t r = srcData.data()[i + w * h * 2];
      destData[i * 3 + 2] = r;
    }
  } else {
    std::runtime_error("If you encounter the scenario where you need this "
                       "please create an issue on github");
  }
  return;
}

// dai::ImgFrame to sensor_msgs::Image (from dai::ImageConverter)
void OakDUtils::getRosMsg(std::shared_ptr<dai::ImgFrame> inData,
                              sensor_msgs::Image &outImageMsg) {
    
    bool _daiInterleaved = true;

    if (_daiInterleaved && dai::rosBridge::ImageConverter::encodingEnumMap.find(inData->getType()) == dai::rosBridge::ImageConverter::encodingEnumMap.end()){
        if (dai::rosBridge::ImageConverter::planarEncodingEnumMap.find(inData->getType()) != dai::rosBridge::ImageConverter::planarEncodingEnumMap.end())
            throw std::runtime_error("Encoding value found for planar dataformat but object was created with 'interleaved = true'. ");
        else
            throw std::runtime_error("Encoding value not found. ");
    } 
        
    if (!_daiInterleaved && dai::rosBridge::ImageConverter::planarEncodingEnumMap.find(inData->getType()) == dai::rosBridge::ImageConverter::planarEncodingEnumMap.end()){
        if (dai::rosBridge::ImageConverter::encodingEnumMap.find(inData->getType()) != dai::rosBridge::ImageConverter::encodingEnumMap.end())
            throw std::runtime_error("Encoding value found for Interleaved dataformat but object was created with 'Interleaved = false'. ");
        else
            throw std::runtime_error("Encoding convertion not found. ");
    }
    
    auto tstamp = inData->getTimestamp();
    int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(
                        tstamp.time_since_epoch())
                        .count();
    int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        tstamp.time_since_epoch())
                        .count() %
                    1000000000UL;

    outImageMsg.header.seq = inData->getSequenceNum();
    outImageMsg.header.stamp = ros::Time(sec, nsec);
    //outImageMsg.header.frame_id = _frameName;
        
    if (!_daiInterleaved) {

        std::istringstream f(dai::rosBridge::ImageConverter::planarEncodingEnumMap[inData->getType()]);
        std::vector<std::string> encoding_info;
        std::string s;

        while (getline(f, s, '_'))
        encoding_info.push_back(s);
        outImageMsg.height   = inData->getHeight();
        outImageMsg.width    = inData->getWidth();
        // FIXME(sachin): This might be wrong for NV12. Fix it
        outImageMsg.step     = inData->getData().size() / inData->getHeight(); 
        outImageMsg.is_bigendian = true;
        size_t size = inData->getData().size();
        outImageMsg.data.resize(size);
        if(dai::rosBridge::ImageConverter::planarEncodingEnumMap[inData->getType()] == "nv12"){
            outImageMsg.encoding = dai::rosBridge::ImageConverter::planarEncodingEnumMap[inData->getType()];
            outImageMsg.data = std::move(inData->getData());
        }
        else{
            outImageMsg.encoding = encoding_info[2];
            planarToInterleaved(inData->getData(), outImageMsg.data, outImageMsg.width,
                                outImageMsg.height, std::stoi(encoding_info[0]),
                                std::stoi(encoding_info[1]));
        }
    } else {
        // copying the data to ros msg
        // outImageMsg.header       = imgHeader;
        std::string temp_str(dai::rosBridge::ImageConverter::encodingEnumMap[inData->getType()]);
        outImageMsg.encoding = temp_str;
        outImageMsg.height = inData->getHeight();
        outImageMsg.width = inData->getWidth();
        outImageMsg.step = inData->getData().size() / inData->getHeight();
        if (outImageMsg.encoding == "16UC1")
        outImageMsg.is_bigendian = false;
        else
        outImageMsg.is_bigendian = true;

        size_t size = inData->getData().size();
        outImageMsg.data.resize(size);
        unsigned char *imageMsgDataPtr =
            reinterpret_cast<unsigned char *>(&outImageMsg.data[0]);
        unsigned char *daiImgData =
            reinterpret_cast<unsigned char *>(inData->getData().data());

        // TODO(Sachin): Try using assign since it is a vector
        // img->data.assign(packet.data->cbegin(), packet.data->cend());
        memcpy(imageMsgDataPtr, daiImgData, size);
    }
    return;
}


///// OWN FUNCTIONS /////
cv::Mat OakDUtils::imgframe_to_mat(std::shared_ptr<dai::ImgFrame> frame, int data_type=CV_16UC1){
    return cv::Mat(
        frame->getHeight(), 
        frame->getWidth(), 
        data_type, 
        frame->getData().data()
    );
}

sensor_msgs::Image OakDUtils::dai_to_ros(std_msgs::Header header, std::shared_ptr<dai::ImgFrame> frame){
    sensor_msgs::Image image;

    // From std::shared_ptr<dai::ImgFrame> to cv::Mat
    //cv::Mat frame_cv = imgframe_to_mat(frame);
    cv::Mat frame_cv = getCvFrame(frame);

    // From cv::Mat to cv_bridge::CvImage 
    cv_bridge::CvImage frame_cv_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, frame_cv); // "passthrough",

    // From cv_bridge::CvImage to sensor_msgs::Image
    frame_cv_bridge.toImageMsg(image);

    return image;
}