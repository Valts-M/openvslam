#include "util/image_util.h"

#ifdef USE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#elif USE_SOCKET_PUBLISHER
#include "socket_publisher/publisher.h"
#endif

#include "openvslam/system.h"
#include "openvslam/config.h"
#include "openvslam/util/yaml.h"
#include "openvslam/util/stereo_rectifier.h"

#include <iostream>
#include <chrono>
#include <fstream>
#include <numeric>
#include <filesystem>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

namespace fs = std::filesystem;

     const std::string currDateTime()
    {
        const time_t now = time(0);
        struct tm tstruct;
        char buf[80];
        tstruct = *gmtime(&now);
        // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
        // for more information about date/time format
        strftime(buf, sizeof(buf), "%Y-%m-%d.%H-%M-%S", &tstruct);

        return buf;
    }

    /**
     * @brief Get curent date
     * 
     * @return format YYYY-MM-DD
     */
     const std::string currDate()
    {
        const time_t now = time(0);
        struct tm tstruct;
        char buf[80];
        tstruct = *gmtime(&now);
        // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
        // for more information about date/time format
        strftime(buf, sizeof(buf), "%Y-%m-%d", &tstruct);

        return buf;
    }

    /**
     * @brief Get ms since 1st of Jan 1970
     * 
     * @return uint64_t 
     */
    uint64_t getTs()
    {
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    }

#pragma pack(1)
struct Pose2D
{
    int32_t x; //cm
    int32_t y; //cm
    int16_t yaw; //0.1 deg
    uint64_t ts; //ms

    /**
     * @brief Construct a new Pose 2 D object with automatic conversion of units from meters and radians to cm and centidegrees
     * 
     * @param _x m
     * @param _y m
     * @param _yaw rad 
     * @param _ts ms since 1st of Jan 1970
     */
    Pose2D(const double _x, const double _y, const double _yaw, const uint64_t _ts)
    {
        x = static_cast<int32_t>(_x * 100);
        y = static_cast<int32_t>(_y * 100);
        yaw = static_cast<int16_t>(_yaw * 1800 / M_PI); //0.1 deg resolution
        ts = _ts;
    }
    
    /**
     * @brief Construct a new Pose 2 D object with zeros
     * 
     */
    Pose2D()
    {
        x = 0;
        y = 0;
        yaw = 0;
        ts = 0;
    }

    /**
     * @brief Method for easy printing to console
     * 
     * @return { x, y, yaw, ts }
     */
    std::string toString() const
    {
        return "{ " + std::to_string(x) 
                    + ", " + std::to_string(y) 
                    + ", " + std::to_string(yaw) 
                    + ", " + std::to_string(ts) 
                    + " }";
    }
};
#pragma pack()

/**
 * @brief Object for logging 2D poses to a binary file. 
 * This objest is also used to get the current position from other threads
 * 
 */
class PoseLogger
{
    public:
        PoseLogger(const std::shared_ptr<openvslam::system> SLAM);
        //void saveTraj(const std::string& saveDir, const bool localizationMode);
        void updateLog(std::shared_ptr<openvslam::Mat44_t> poseMat);
        
        /**
         * @brief Finishes the current log
         * 
         * @return Path to finished log file
         */
        std::string finishLog();

        /**
         * @brief Starts a new log in the specified directory. 
         * The directory structure is saveDir->date->date.time.log
         * 
         */
        void startNewLog(const std::string& saveDir);

        Pose2D getCurrPose();

        /**
         * @brief Check if a new pose is available
         * 
         */
        bool getPoseAvailability();

        /**
         * @brief Set the pose availablity. 
         * PoseLogger sets to true if there has been enough movement to be logged
         * TelemetrySender sets to false if it has sent the newest pose
         */
        void setPoseAvailability(const bool newPoseAvailable);
    private:
        const std::shared_ptr<openvslam::system> SLAM;

        /**
         * @brief This object is used to get the current pose from the slam system.
         */
        //std::shared_ptr<openvslam::publish::map_publisher> mapPublisher;
        // const std::shared_ptr<spdlog::logger> logger;

        /**
         * @brief Path to the current log file if one has been started. 
         * Is set by the startNewLog() method. 
         * Unset by the finishLog() method.
         * 
         */
        std::string currLogPath{};
        Pose2D currPose{};
        std::mutex currPoseMtx;
        std::mutex newPoseAvailableMtx;
        bool newPoseAvailable = false;
        
        /**
         * @brief Compares the passed pose and currPose to see if the distance deltas in xor y are greater than UPDATE_DIST
         * as well as checks if the yay delta is greater than UPDATE_ANG
         * 
         * @param pose pose to compare to
         */
        bool shouldUpdatePose(const Pose2D& pose);
        void updateCurrPose(const double x, const double y, const double yaw, const uint64_t ts);
        void updateCurrPose(const Pose2D& pose);
};

PoseLogger::PoseLogger(const std::shared_ptr<openvslam::system> SLAM) : 
                                SLAM(SLAM)//, mapPublisher(SLAM->get_map_publisher())//, logger(logger)
{
    
}

void PoseLogger::updateLog(std::shared_ptr<openvslam::Mat44_t> poseMat)
{
    if(currLogPath.empty())
    {
        // spdlog::error("Called updateLog with no new log file specified");
        std::cerr << "no log path\n";
        return;
    }

    const auto pose = *poseMat;
    const double y = -pose(0, 3);
    const double x = pose(2, 3);
    const double yaw = atan2(-pose(2, 0), pose(2, 2));
    const uint64_t ts = getTs();
    const Pose2D tmpPose(x, y, yaw, ts);
    // std::cout << tmpPose.toString() << '\n';

    std::ofstream ofs(currLogPath, std::ios::app | std::ios::binary);
    if (!ofs.is_open()) {
        // spdlog::critical("cannot create a file at {}", currLogPath);
        throw std::runtime_error("cannot create a file at " + currLogPath);
    }

    if(shouldUpdatePose(tmpPose))
    {
        // std::cout << tmpPose.x - currPose.x << ' ' << tmpPose.y - currPose.y << ' ' << tmpPose.yaw - currPose.yaw << '\n';
        // std::cout << currPose.yaw << '\n';
        updateCurrPose(tmpPose);
        setPoseAvailability(true);

        std::lock_guard<std::mutex> lock(currPoseMtx);
        ofs.write((char*)&currPose, sizeof(currPose));

        std::cout << std::setprecision(9)
            << "x = " << x << "\t\ty = " << y
            << "\t\tyaw = " << yaw
            << "\t\tts = " << std::setprecision(15) << getTs()
            << '\n';
    }
}

bool PoseLogger::shouldUpdatePose(const Pose2D& pose)
{
    std::lock_guard<std::mutex> lock(currPoseMtx);
    return (std::abs(pose.x - currPose.x) > 20 || 
            std::abs(pose.y - currPose.y) > 20 || 
            std::abs(pose.yaw - currPose.yaw) > 0.01);
}

void PoseLogger::updateCurrPose(const double x, const double y, const double yaw, const uint64_t ts)
{
    std::lock_guard<std::mutex> lock(currPoseMtx);
    currPose.x = static_cast<int32_t>(x * 100);
    currPose.y = static_cast<int32_t>(y * 100);
    currPose.yaw = static_cast<int16_t>(yaw * 1800 / M_PI); //0.1 deg resolution
    currPose.ts = ts;
}

void PoseLogger::updateCurrPose(const Pose2D& pose)
{
    std::lock_guard<std::mutex> lock(currPoseMtx);
    currPose = pose;
}

Pose2D PoseLogger::getCurrPose()
{
    const std::lock_guard<std::mutex> lock(currPoseMtx);
    return currPose;
}

void PoseLogger::setPoseAvailability(const bool value)
{
    const std::lock_guard<std::mutex> lock(newPoseAvailableMtx);
    newPoseAvailable = value;
}

bool PoseLogger::getPoseAvailability()
{
    const std::lock_guard<std::mutex> lock(newPoseAvailableMtx);
    return newPoseAvailable;
}

void PoseLogger::startNewLog(const std::string& saveDir)
{
    fs::path savePath{saveDir};
    if (!fs::exists(savePath))
        fs::create_directories(savePath);
    
    fs::path DateDir{savePath / currDate()};

    if(!fs::exists(DateDir))
    {
        fs::create_directories(DateDir);
    }

    if(!currLogPath.empty())
        finishLog();

    currLogPath = fs::path(DateDir / (currDateTime() + ".log").c_str());
}

std::string PoseLogger::finishLog()
{
    const std::string returnVal = currLogPath;
    
    //create an empty file to indicate that the finished log has not yet been sent to the server
    std::ofstream ofs(returnVal + ".notsent", std::ios::binary);
    if (!ofs.is_open()) {
        // spdlog::critical("cannot create a file at {}", returnVal + ".notsent");
        throw std::runtime_error("cannot create a file at " + returnVal + ".notsent");
    }

    currLogPath = "";
    return returnVal;
}

void mono_tracking(const std::shared_ptr<openvslam::config>& cfg,
                   const std::string& vocab_file_path, const std::string& image_dir_path, const std::string& mask_img_path,
                   const unsigned int frame_skip, const bool no_sleep, const bool auto_term,
                   const bool eval_log, const std::string& map_db_path) {
    // load the mask image
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    const image_sequence sequence(image_dir_path, cfg->camera_->fps_);
    const auto frames = sequence.get_frames();

    // build a SLAM system
    openvslam::system SLAM(cfg, vocab_file_path);
    // startup the SLAM process
    SLAM.startup();

#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(
        openvslam::util::yaml_optional_ref(cfg->yaml_node_, "PangolinViewer"), &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(
        openvslam::util::yaml_optional_ref(cfg->yaml_node_, "SocketPublisher"), &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif

    std::vector<double> track_times;
    track_times.reserve(frames.size());

    // run the SLAM in another thread
    std::thread thread([&]() {
        for (unsigned int i = 0; i < frames.size(); ++i) {
            const auto& frame = frames.at(i);
            const auto img = cv::imread(frame.img_path_, cv::IMREAD_UNCHANGED);

            const auto tp_1 = std::chrono::steady_clock::now();

            if (!img.empty() && (i % frame_skip == 0)) {
                // input the current frame and estimate the camera pose
                SLAM.feed_monocular_frame(img, frame.timestamp_, mask);
            }

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            if (i % frame_skip == 0) {
                track_times.push_back(track_time);
            }

            // wait until the timestamp of the next frame
            if (!no_sleep && i < frames.size() - 1) {
                const auto wait_time = frames.at(i + 1).timestamp_ - (frame.timestamp_ + track_time);
                if (0.0 < wait_time) {
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                }
            }

            // check if the termination of SLAM system is requested or not
            if (SLAM.terminate_is_requested()) {
                break;
            }
        }

        // wait until the loop BA is finished
        while (SLAM.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }

        // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
        if (auto_term) {
            viewer.request_terminate();
        }
#elif USE_SOCKET_PUBLISHER
        if (auto_term) {
            publisher.request_terminate();
        }
#endif
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

    if (eval_log) {
        // output the trajectories for evaluation
        SLAM.save_frame_trajectory("frame_trajectory.txt", "TUM");
        SLAM.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
        // output the tracking times for evaluation
        std::ofstream ofs("track_times.txt", std::ios::out);
        if (ofs.is_open()) {
            for (const auto track_time : track_times) {
                ofs << track_time << std::endl;
            }
            ofs.close();
        }
    }

    if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
    }

    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
}

void stereo_tracking(const std::shared_ptr<openvslam::config>& cfg,
                   const std::string& vocab_file_path, const std::string& image_dir_path, const std::string& mask_img_path,
                   const unsigned int frame_skip, const bool no_sleep, const bool auto_term,
                   const bool eval_log, const std::string& map_db_path, const unsigned int startFrame, const std::string load_map) {
    // load the mask image
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    const image_sequence sequenceLeft(image_dir_path + "/left", cfg->camera_->fps_);
    const auto framesLeft = sequenceLeft.get_frames();

    const image_sequence sequenceRight(image_dir_path + "/right", cfg->camera_->fps_);
    const auto framesRight = sequenceRight.get_frames();

    const openvslam::util::stereo_rectifier rectifier(cfg);

    // build a SLAM system
    std::shared_ptr<openvslam::system> SLAM = std::make_shared<openvslam::system>(cfg, vocab_file_path);

    PoseLogger poseLogger{SLAM};
    poseLogger.startNewLog("/home/valts");

    if(!load_map.empty())
    {
        SLAM->load_map_database(load_map);
        SLAM->startup(false);
        SLAM->disable_mapping_module();
    }
    else
    {
        // startup the SLAM process
        SLAM->startup();
    }


#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(
        openvslam::util::yaml_optional_ref(cfg->yaml_node_, "PangolinViewer"), SLAM.get(), SLAM->get_frame_publisher(), SLAM->get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(
        openvslam::util::yaml_optional_ref(cfg->yaml_node_, "SocketPublisher"), &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif

    std::vector<double> track_times;
    track_times.reserve(framesLeft.size());

    // run the SLAM in another thread
    std::thread thread([&]() {
        for (unsigned int i = startFrame; i < framesLeft.size(); ++i) {
            const auto& frameLeft = framesLeft.at(i);
            const auto imgLeft = cv::imread(frameLeft.img_path_, cv::IMREAD_UNCHANGED);

            const auto& frameRight = framesRight.at(i);
            const auto imgRight = cv::imread(frameRight.img_path_, cv::IMREAD_UNCHANGED);

            cv::Mat unImgLeft, uImgRight;

            rectifier.rectify(imgLeft, imgRight, unImgLeft, uImgRight);
            
            const auto tp_1 = std::chrono::steady_clock::now();

            if (!imgLeft.empty() && (i % frame_skip == 0)) {
                // input the current frame and estimate the camera pose
                poseLogger.updateLog(SLAM->feed_stereo_frame(unImgLeft, uImgRight, frameLeft.timestamp_, mask));
            }

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            if (i % frame_skip == 0) {
                track_times.push_back(track_time);
            }

            // wait until the timestamp of the next frame
            if (!no_sleep && i < framesLeft.size() - 1) {
                const auto wait_time = framesLeft.at(i + 1).timestamp_ - (frameLeft.timestamp_ + track_time);
                if (0.0 < wait_time) {
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                }
            }

            // check if the termination of SLAM system is requested or not
            if (SLAM->terminate_is_requested()) {
                break;
            }
        }

        // wait until the loop BA is finished
        while (SLAM->loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }

        // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
        if (auto_term) {
            viewer.request_terminate();
        }
#elif USE_SOCKET_PUBLISHER
        if (auto_term) {
            publisher.request_terminate();
        }
#endif
    });

    // run the viewer in the current thread
#ifdef USE_PANGOLIN_VIEWER
    viewer.run();
#elif USE_SOCKET_PUBLISHER
    publisher.run();
#endif

    thread.join();

    // shutdown the SLAM process
    SLAM->shutdown();

    if (eval_log) {
        // output the trajectories for evaluation
        SLAM->save_frame_trajectory("frame_trajectory.txt", "TUM");
        SLAM->save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
        // output the tracking times for evaluation
        std::ofstream ofs("track_times.txt", std::ios::out);
        if (ofs.is_open()) {
            for (const auto track_time : track_times) {
                ofs << track_time << std::endl;
            }
            ofs.close();
        }
    }

    if (!map_db_path.empty()) {
        // output the map database
        SLAM->save_map_database(map_db_path);
    }

    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
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
    auto img_dir_path = op.add<popl::Value<std::string>>("i", "img-dir", "directory path which contains images");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    auto frame_skip = op.add<popl::Value<unsigned int>>("", "frame-skip", "interval of frame skip", 1);
    auto no_sleep = op.add<popl::Switch>("", "no-sleep", "not wait for next frame in real time");
    auto auto_term = op.add<popl::Switch>("", "auto-term", "automatically terminate the viewer");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    auto eval_log = op.add<popl::Switch>("", "eval-log", "store trajectory and tracking times for evaluation");
    auto map_db_path = op.add<popl::Value<std::string>>("p", "map-db", "store a map database at this path after SLAM", "");
    auto load_map = op.add<popl::Value<std::string>>("l", "", "load a map database from this path", "");
    auto start_frame = op.add<popl::Value<unsigned int>>("s", "start-frame", "Frame to start at", 0);
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
    if (!vocab_file_path->is_set() || !img_dir_path->is_set() || !config_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
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
    if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
        mono_tracking(cfg, vocab_file_path->value(), img_dir_path->value(), mask_img_path->value(),
                      frame_skip->value(), no_sleep->is_set(), auto_term->is_set(),
                      eval_log->is_set(), map_db_path->value());
    }
    else if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Stereo){
        stereo_tracking(cfg, vocab_file_path->value(), img_dir_path->value(), mask_img_path->value(),
                    frame_skip->value(), no_sleep->is_set(), auto_term->is_set(),
                    eval_log->is_set(), map_db_path->value(), start_frame->value(), load_map->value());
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}
