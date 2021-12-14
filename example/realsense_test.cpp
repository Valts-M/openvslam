#include <librealsense2/rs.hpp> // Include Intel RealSense Cross Platform API
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "util/realsense_util.h"

#include "openvslam/system.h"
#include "openvslam/config.h"
#include "openvslam/util/stereo_rectifier.h"
#include "openvslam/util/image_converter.h"
#include "openvslam/util/yaml.h"

int main(int argc, char* argv[]) {
    // RealSense settings
    rs2::pipeline pipeline;
    rs2::config config;
    // config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    // config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    config.enable_stream(RS2_STREAM_FISHEYE, 1);
    config.enable_stream(RS2_STREAM_FISHEYE, 2);
    rs2::pipeline_profile rs2_cfg = pipeline.start(config);

    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>("/home/valts/openvslam-1/example/realsense/realsense-T265.yaml");
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    const openvslam::util::stereo_rectifier rectifier(cfg);

    // rs2::align alignTo(RS2_STREAM_COLOR);
    cv::Mat frame1, frame2, frame1_rect, frame2_rect;

    for(;;){
        rs2::frameset data = pipeline.wait_for_frames();
        // rs2::frameset alignedFrame = alignTo.process(data);
        // rs2::depth_frame depth = alignedFrame.get_depth_frame();

        frame1 = funcFormat::frame2Mat(data.get_fisheye_frame(1));
        frame2 = funcFormat::frame2Mat(data.get_fisheye_frame(2));

        rectifier.rectify(frame1, frame2, frame1_rect, frame2_rect);

        cv::imshow("Fish1", frame1_rect);
        cv::imshow("Fish2", frame2_rect);
        cv::waitKey(50);
    }
}