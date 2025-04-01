#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rgb_depth_aligned.h"



// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

void runRGBDepth() {
    using namespace std;
    dai::Pipeline pipeline;
    dai::Device device;
    vector<string> queueNames;

    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto rgbOut = pipeline.create<dai::node::XLinkOut>();
    auto depthOut = pipeline.create<dai::node::XLinkOut>();

    rgbOut->setStreamName("rgb");
    queueNames.push_back("rgb");
    depthOut->setStreamName("depth");
    queueNames.push_back("depth");

    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setFps(30);

    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    left->setCamera("left");
    left->setFps(30);
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    right->setCamera("right");
    right->setFps(30);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::DEFAULT);
    stereo->setLeftRightCheck(true);
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);

    camRgb->isp.link(rgbOut->input);
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    stereo->disparity.link(depthOut->input);

    device.startPipeline(pipeline);

    for (const auto& name : queueNames) {
        device.getOutputQueue(name, 4, false);
    }

    unordered_map<string, cv::Mat> frames;

    cv::namedWindow("rgb");
    cv::namedWindow("depth");
    cv::namedWindow("rgb-depth");

    while (true) {
        unordered_map<string, shared_ptr<dai::ImgFrame>> latestPacket;
        auto queueEvents = device.getQueueEvents(queueNames);
        for (const auto& name : queueEvents) {
            auto packets = device.getOutputQueue(name)->tryGetAll<dai::ImgFrame>();
            if (!packets.empty()) {
                latestPacket[name] = packets.back();
            }
        }

        for (const auto& name : queueNames) {
            if (latestPacket.find(name) != latestPacket.end()) {
                frames[name] = latestPacket[name]->getCvFrame();
                if (name == "depth") {
                    auto maxDisparity = stereo->initialConfig.getMaxDisparity();
                    frames[name].convertTo(frames[name], CV_8UC1, 255.0 / maxDisparity);
                    cv::applyColorMap(frames[name], frames[name], cv::COLORMAP_HOT);
                }
                cv::imshow(name, frames[name]);
            }
        }

        if (frames.find("rgb") != frames.end() && frames.find("depth") != frames.end()) {
            if (frames["depth"].channels() < 3) {
                cv::cvtColor(frames["depth"], frames["depth"], cv::COLOR_GRAY2BGR);
            }
            cv::Mat blended;
            // You can adjust these weights as needed
            cv::addWeighted(frames["rgb"], 0.4, frames["depth"], 0.6, 0, blended);
            cv::imshow("rgb-depth", blended);
        }

        if (cv::waitKey(1) == 'q') break;
    }
}