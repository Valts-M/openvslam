#include "util/realsense_util.h"

#ifdef USE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#elif USE_SOCKET_PUBLISHER
#include "socket_publisher/publisher.h"
#endif

#include "openvslam/system.h"
#include "openvslam/config.h"
#include "openvslam/util/stereo_rectifier.h"
#include "openvslam/util/image_converter.h"
#include "openvslam/util/yaml.h"

#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <stdio.h>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>
#include <filesystem>

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

namespace fs = std::filesystem;

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *gmtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%H:%M:%S", &tstruct);

    return buf;
}

void rgbd_tracking(const std::shared_ptr<openvslam::config>& cfg,
                   const std::string& vocab_file_path,
                   const std::string& save_dir,
                   const bool localization_mode) {
    // initialize a SLAM system
    openvslam::system SLAM(cfg, vocab_file_path);
    std::cerr << "here" << std::endl;

    if (localization_mode) {
        // startup the SLAM process (don't need to initialize a map since we're only localizing)
        SLAM.startup(false);
        SLAM.disable_mapping_module();
        SLAM.disable_loop_detector();
        SLAM.load_map_database(save_dir + "/latest/map.msg");
    }
    else {
        // startup the SLAM process
        SLAM.startup();
    }

    std::cerr << "here" << std::endl;

    // RealSense settings
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    rs2::pipeline_profile rs2_cfg = pipeline.start(config);

    rs2::align alignTo(RS2_STREAM_COLOR);

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(openvslam::util::yaml_optional_ref(cfg->yaml_node_, "PangolinViewer"), &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif

    cv::Mat frame1, frame2, input1, input2;
    double tframe;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point t2 = t1;
    typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;

    // run the SLAM in another thread
    std::thread thread([&]() {
        while (true) {
            rs2::frameset data = pipeline.wait_for_frames();

              rs2::frameset alignedFrame = alignTo.process(data);
              rs2::depth_frame depth = alignedFrame.get_depth_frame();

              frame1 = funcFormat::frame2Mat(alignedFrame.get_color_frame());
              frame2 = funcFormat::frame2Mat(depth);

            input1 = frame1.clone();
            input2 = frame2.clone();

            t1 = std::chrono::steady_clock::now();
            tframe = std::chrono::duration_cast<ms>(t1 - t2).count();

            SLAM.feed_RGBD_frame(input1, input2, tframe);

            t2 = t1;
            std::ostringstream strs;
            strs << tframe;
            std::string str = strs.str() + " ms";

            cv::Size size = input1.size();
            cv::putText(input1, str, cv::Point(10, size.height - 10), inforerFontA, 1, inforerColorFA, 1, cv::LINE_AA);
            cv::putText(input1, str, cv::Point(10, size.height - 10), inforerFontB, 1, inforerColorFB, 1, cv::LINE_AA);

            //cv::imshow("Road facing camera", input1);
            //cv::imshow("Depth", input2);

            if (SLAM.terminate_is_requested())
                break;
        }

        // wait until the loop BA is finished
        while (SLAM.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
    });

    // run the viewer in the current thread
#ifdef USE_PANGOLIN_VIEWER
    viewer.run();
#elif USE_SOCKET_PUBLISHER
    publisher.run();
#endif

    thread.join();

    // shutdown the SLAM process
    SLAM.shutdown();

    fs::path savePath{save_dir};
    if (!fs::exists(savePath))
        fs::create_directories(savePath);

    fs::path latestSymlink{savePath};
    latestSymlink.append("latest");

    if (!localization_mode) {
        fs::path newMapPath{savePath};
        newMapPath.append("map-" + currentDateTime());
        fs::create_directory(newMapPath);
        if (fs::exists(latestSymlink))
            fs::remove(latestSymlink);
        fs::create_symlink(newMapPath, latestSymlink);
        // output the map database
        SLAM.save_map_database(latestSymlink.string() + "/map.msg");
        SLAM.save_frame_trajectory(latestSymlink.string() + "/mapping_trajectory.txt", "KITTI");
    }
    else {
        savePath.append("latest");
        fs::path fullSavePath{fs::read_symlink(savePath)};
        fs::path mapSavePath{fs::read_symlink(savePath)};
        std::string dirName = "loc-" + currentDateTime();
        savePath.append(dirName);
        fullSavePath.append(dirName);
        fs::create_directory(savePath);
        latestSymlink.append("latest");
        if (fs::exists(latestSymlink))
            fs::remove(latestSymlink);
        fs::create_symlink(fullSavePath, latestSymlink);
        // output the trajectories for evaluation
        SLAM.save_frame_trajectory(latestSymlink.string() + "/trajectory.txt", "KITTI");
#ifdef USE_PANGOLIN_VIEWER
        viewer.create_report_config_file(mapSavePath.string(), fullSavePath.string());
#elif USE_SOCKET_PUBLISHER
        // publisher.run();
#endif
    }
}

void stereo_tracking(const std::shared_ptr<openvslam::config>& cfg,
                   const std::string& vocab_file_path,
                   const std::string& save_dir,
                   const bool localization_mode) {
    // initialize a SLAM system
    openvslam::system SLAM(cfg, vocab_file_path);
    std::cerr << "here" << std::endl;

    if (localization_mode) {
        // startup the SLAM process (don't need to initialize a map since we're only localizing)
        SLAM.startup(false);
        SLAM.disable_mapping_module();
        SLAM.disable_loop_detector();
        SLAM.load_map_database(save_dir + "/latest/map.msg");
    }
    else {
        // startup the SLAM process
        SLAM.startup();
    }

    std::cerr << "here" << std::endl;

    // RealSense settings
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8, 30);
    config.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8, 30);

    rs2::pipeline_profile rs2_cfg = pipeline.start(config);

    const openvslam::util::stereo_rectifier rectifier(cfg);

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(openvslam::util::yaml_optional_ref(cfg->yaml_node_, "PangolinViewer"), &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif

    cv::Mat frame1, frame2, input1, input2;
    double tframe;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point t2 = t1;
    typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;

    // run the SLAM in another thread
    std::thread thread([&]() {
        while (true) {
            rs2::frameset data = pipeline.wait_for_frames();

            frame1 = funcFormat::frame2Mat(data.get_fisheye_frame(1));
            frame2 = funcFormat::frame2Mat(data.get_fisheye_frame(2));

            rectifier.rectify(frame1, frame2, input1, input2);

            t1 = std::chrono::steady_clock::now();
            tframe = std::chrono::duration_cast<ms>(t1 - t2).count();

            // SLAM.feed_RGBD_frame(input1, input2, tframe);
            SLAM.feed_stereo_frame(input1, input2, tframe);

            t2 = t1;
            std::ostringstream strs;
            strs << tframe;
            std::string str = strs.str() + " ms";

            cv::Size size = input1.size();
            cv::putText(input1, str, cv::Point(10, size.height - 10), inforerFontA, 1, inforerColorFA, 1, cv::LINE_AA);
            cv::putText(input1, str, cv::Point(10, size.height - 10), inforerFontB, 1, inforerColorFB, 1, cv::LINE_AA);

            //cv::imshow("Road facing camera", input1);
            //cv::imshow("Depth", input2);

            if (SLAM.terminate_is_requested())
                break;
        }

        // wait until the loop BA is finished
        while (SLAM.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
    });

    // run the viewer in the current thread
#ifdef USE_PANGOLIN_VIEWER
    viewer.run();
#elif USE_SOCKET_PUBLISHER
    publisher.run();
#endif

    thread.join();

    // shutdown the SLAM process
    SLAM.shutdown();

    fs::path savePath{save_dir};
    if (!fs::exists(savePath))
        fs::create_directories(savePath);

    fs::path latestSymlink{savePath};
    latestSymlink.append("latest");

    if (!localization_mode) {
        fs::path newMapPath{savePath};
        newMapPath.append("map-" + currentDateTime());
        fs::create_directory(newMapPath);
        if (fs::exists(latestSymlink))
            fs::remove(latestSymlink);
        fs::create_symlink(newMapPath, latestSymlink);
        // output the map database
        SLAM.save_map_database(latestSymlink.string() + "/map.msg");
        SLAM.save_frame_trajectory(latestSymlink.string() + "/mapping_trajectory.txt", "KITTI");
    }
    else {
        savePath.append("latest");
        fs::path fullSavePath{fs::read_symlink(savePath)};
        fs::path mapSavePath{fs::read_symlink(savePath)};
        std::string dirName = "loc-" + currentDateTime();
        savePath.append(dirName);
        fullSavePath.append(dirName);
        fs::create_directory(savePath);
        latestSymlink.append("latest");
        if (fs::exists(latestSymlink))
            fs::remove(latestSymlink);
        fs::create_symlink(fullSavePath, latestSymlink);
        // output the trajectories for evaluation
        SLAM.save_frame_trajectory(latestSymlink.string() + "/trajectory.txt", "KITTI");
#ifdef USE_PANGOLIN_VIEWER
        viewer.create_report_config_file(mapSavePath.string(), fullSavePath.string());
#elif USE_SOCKET_PUBLISHER
        // publisher.run();
#endif
    }
}

int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
#endif

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto save_dir = op.add<popl::Value<std::string>>("s", "save-dir", "root save directory");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    auto localization_mode = op.add<popl::Switch>("l", "localization-only", "run SLAM in localization mode");

    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !config_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    if (!save_dir->is_set()) {
        std::cerr << "save_dir is not set";
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(config_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

    // run tracking
    if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::RGBD) {
        rgbd_tracking(cfg, vocab_file_path->value(), save_dir->value(), localization_mode->is_set());
    }
    else if(cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Stereo)
    {
        stereo_tracking(cfg, vocab_file_path->value(), save_dir->value(), localization_mode->is_set());
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}
