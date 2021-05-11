
#include "ros/ros.h"

#include <iostream>
#include <cstdio>
#include <chrono> // new

//#include "utility.hpp"
#include <depthai_examples/spatial_mobilenet_pipeline.hpp>

//#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include <vision_msgs/Detection2DArray.h>

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/ImgDetectionConverter.hpp>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#include "depthai_examples/BoundingBoxes.h"


// This node publishes in the topic called "/detections" the information of the elements detected by the mobilenet neural network which is running in
// the rgb camera.
// This node does not publish any photo, it only publishes the class of the elements detected, their probabilities, the coordinates of the
// top left and bottom right corners of the bounding boxes and their respective depths.
// This algorithm is based on the C++ API which can be found at "depthai-core" repository of luxonis.

int main(int argc, char** argv){

    using namespace std;
    using namespace std::chrono;

    ros::init(argc, argv, "spatial_mobilenet_node_test");
    ros::NodeHandle pnh("~");
    
    std::string deviceName;
    std::string camera_param_uri;
    std::string nnPath(BLOB_PATH);
    int bad_params = 0;

    bad_params += !pnh.getParam("camera_name", deviceName);
    // bad_params += !pnh.getParam("nnPath", nnPath);
    bad_params += !pnh.getParam("camera_param_uri", camera_param_uri);

    if (bad_params > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }

    SpatialMobileNetDetectionExample SpatialDetectionPipeline;
    // detectionPipeline.initDepthaiDev(nnPath); // deleted
    dai::Pipeline p = SpatialDetectionPipeline.createSpatialNNPipeline(nnPath); // new

    // Connect to device with above created pipeline
    dai::Device d(p);

    // Start the pipeline
    d.startPipeline();

    auto preview = d.getOutputQueue("preview", 4, false);
    auto detections = d.getOutputQueue("detections", 4, false);
    auto xoutBoundingBoxDepthMapping = d.getOutputQueue("boundingBoxDepthMapping", 4, false);
    auto depthQueue = d.getOutputQueue("depth", 4, false);

    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;
    auto color = cv::Scalar(255, 255, 255);

    // Publisher
        ros::Publisher node_publisher = pnh.advertise<depthai_examples::BoundingBoxes>("/detections", 10);
        ros::Rate loop_rate(10);

    while(ros::ok()) {
        auto imgFrame = preview->get<dai::ImgFrame>();
        auto det = detections->get<dai::SpatialImgDetections>();
        auto depth = depthQueue->get<dai::ImgFrame>();

        auto dets = det->detections;

        cv::Mat depthFrame = depth->getFrame();
        cv::Mat depthFrameColor;
        cv::normalize(depthFrame, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
        cv::equalizeHist(depthFrameColor, depthFrameColor);
        cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);

        if(!dets.empty()) {
            auto boundingBoxMapping = xoutBoundingBoxDepthMapping->get<dai::SpatialLocationCalculatorConfig>();
            auto roiDatas = boundingBoxMapping->getConfigData();

            for(auto roiData : roiDatas) {
                auto roi = roiData.roi;
                roi = roi.denormalize(depthFrameColor.cols, depthFrameColor.rows);
                auto topLeft = roi.topLeft();
                auto bottomRight = roi.bottomRight();
                auto xmin = (int)topLeft.x;
                auto ymin = (int)topLeft.y;
                auto xmax = (int)bottomRight.x;
                auto ymax = (int)bottomRight.y;

                cv::rectangle(depthFrameColor, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color, cv::FONT_HERSHEY_SIMPLEX);
            }
        }
        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        cv::Mat frame = imgFrame->getCvFrame();

        depthai_examples::BoundingBox bbox;
        depthai_examples::BoundingBoxes msg;

        int i = 0;

        for(const auto& d : dets) {
            int x1 = d.xmin * frame.cols;
            int y1 = d.ymin * frame.rows;
            int x2 = d.xmax * frame.cols;
            int y2 = d.ymax * frame.rows;

            int labelIndex = d.label;
            std::string labelStr = to_string(labelIndex);
            if(labelIndex < SpatialMobileNetDetectionExample::label_map.size()) {
                labelStr = SpatialMobileNetDetectionExample::label_map[labelIndex];
            }
            cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << d.confidence * 100;
            cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            std::stringstream depthX;
            depthX << "X: " << (int)d.spatialCoordinates.x << " mm";
            cv::putText(frame, depthX.str(), cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream depthY;
            depthY << "Y: " << (int)d.spatialCoordinates.y << " mm";
            cv::putText(frame, depthY.str(), cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream depthZ;
            depthZ << "Z: " << (int)d.spatialCoordinates.z << " mm";
            cv::putText(frame, depthZ.str(), cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);

            bbox.Class = (string)labelStr;
            bbox.probability = (float)d.confidence;
            bbox.xmin = (int)x1;
            bbox.ymin = (int)y1;
            bbox.xmax = (int)x2;
            bbox.ymax = (int)y2;
            bbox.depth = (float)d.spatialCoordinates.z;

            msg.bounding_boxes.push_back(bbox);

        }

        std::stringstream fpsStr;
        fpsStr << std::fixed << std::setprecision(2) << fps;
        cv::putText(frame, fpsStr.str(), cv::Point(2, imgFrame->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

        cv::imshow("depth", depthFrameColor);
        cv::imshow("preview", frame);
        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }

        // Publishing the info of detections in the topic 
        node_publisher.publish(msg);
        msg.bounding_boxes.clear();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} 


