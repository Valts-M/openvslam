#include <librealsense2/rs.hpp> // Include Intel RealSense Cross Platform API
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "util/realsense_util.h"



int main(int argc, char* argv[]) {
    // RealSense settings
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    rs2::pipeline_profile rs2_cfg = pipeline.start(config);

    rs2::align alignTo(RS2_STREAM_COLOR);
    cv::Mat frame1, frame2;

    for(;;){
        rs2::frameset data = pipeline.wait_for_frames();
        rs2::frameset alignedFrame = alignTo.process(data);
        rs2::depth_frame depth = alignedFrame.get_depth_frame();

        frame1 = funcFormat::frame2Mat(alignedFrame.get_color_frame());
        frame2 = funcFormat::frame2Mat(depth);

        cv::imshow("Road facing camera", frame1);
        cv::imshow("Depth", frame2);
        cv::waitKey(50);
    }
}