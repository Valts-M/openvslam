// #define USE_SOCKET_PUBLISHER

#include "util/box_util.hpp"

#ifdef USE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#elif USE_SOCKET_PUBLISHER
#include "socket_publisher/publisher.h"
#endif

#include "openvslam/system.h"
#include "openvslam/config.h"
#include "openvslam/util/image_converter.h"
#include "openvslam/util/yaml.h"

#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <stdio.h>
#include <time.h>
#include <filesystem>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>
#include <nlohmann/json.hpp>

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

void start_tracker(const std::shared_ptr<openvslam::config>& cfg,
                   const std::string& vocabFilePath,
                   const std::string& saveDir,
                   const bool localizationMode) {
    // initialize a SLAM system
    std::shared_ptr<openvslam::system> SLAM = std::make_shared<openvslam::system>(cfg, vocabFilePath);
    std::shared_ptr<box::PoseLogger> poseLogger = std::make_shared<box::PoseLogger>(SLAM);
    std::unique_ptr<box::FrameFeeder> frameFeeder = std::make_unique<box::FrameFeeder>(SLAM, cfg, poseLogger, saveDir, localizationMode);

    poseLogger->startNewLog(saveDir);

// #ifdef USE_PANGOLIN_VIEWER
//     pangolin_viewer::viewer viewer(
//         openvslam::util::yaml_optional_ref(cfg->yaml_node_, "PangolinViewer"), SLAM.get(), SLAM->get_frame_publisher(), SLAM->get_map_publisher());
// #elif USE_SOCKET_PUBLISHER
//     socket_publisher::publisher publisher(
//         openvslam::util::yaml_optional_ref(cfg->yaml_node_, "SocketPublisher"), SLAM.get(), SLAM->get_frame_publisher(), SLAM->get_map_publisher());
// #endif

    std::thread slamThread([&]() {
        frameFeeder->run();
    });

//     //can't remove this otherwise CPU usage jumps to 100% for no explicable reason
//     std::thread uselessViewerThread([&]() {
// #ifdef USE_PANGOLIN_VIEWER
//         viewer.run();
// #elif USE_SOCKET_PUBLISHER
//         publisher.run();
// #endif
//     });

    //Run server communication is this thread
    std::thread serverClientThread([&]() {
        box::StatusSender serverClient{poseLogger};
        serverClient.run();
    });
    
    box::TelemetrySender serverListener{poseLogger};
    serverListener.run();

    SLAM->request_terminate();
// #ifdef USE_PANGOLIN_VIEWER
//     viewer.request_terminate();
// #elif USE_SOCKET_PUBLISHER
//     publisher.request_terminate();
// #endif

    slamThread.join();
    serverClientThread.join();
    // uselessViewerThread.join();

    // shutdown the SLAM process
    SLAM->shutdown();

    // poseLogger->saveTraj(saveDir, localizationMode);
}


int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
#endif

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocabFilePath = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto saveDir = op.add<popl::Value<std::string>>("s", "save-dir", "root save directory");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    auto localizationMode = op.add<popl::Switch>("l", "localization-only", "run SLAM in localization mode");

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
    if (!vocabFilePath->is_set() || !config_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    if (!saveDir->is_set()) {
        std::cerr << "saveDir is not set";
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
    if(cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Stereo || cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular)
    {
        start_tracker(cfg, vocabFilePath->value(), saveDir->value(), localizationMode->is_set());
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}
