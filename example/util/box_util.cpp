
#include "box_util.hpp"

#include "realsense_util.h"
#include "openvslam/publish/map_publisher.h"

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <popl.hpp>

namespace box{

    namespace fs = std::filesystem;

    const std::string currDateTime() {
        time_t now = time(0);
        struct tm tstruct;
        char buf[80];
        tstruct = *gmtime(&now);
        // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
        // for more information about date/time format
        strftime(buf, sizeof(buf), "%Y-%m-%d.%H-%M-%S", &tstruct);

        return buf;
    }

    PoseLogger::PoseLogger(const std::shared_ptr<openvslam::system> SLAM): SLAM(SLAM), mapPublisher(SLAM->get_map_publisher()){
        
    }

    void PoseLogger::updateCurrPose(){
        auto pose = mapPublisher->get_current_cam_pose();

        std::ofstream ofs("/home/valts/power_test.txt", std::ios::app);
        if (!ofs.is_open()) {
            spdlog::critical("cannot create a file at {}", "/home/valts/power_test.txt");
            throw std::runtime_error("cannot create a file at  /home/valts/power_test.txt");
        }

        const double y = -pose(0, 3);
        const double x = pose(2, 3);
        const double yaw = atan2(-pose(2, 0), pose(2, 2));
        if(std::abs(x - prev_x) > UPDATE_DIST || std::abs(y - prev_y) > UPDATE_DIST || std::abs(yaw - prev_yaw) > UPDATE_ANG)
        {
            ofs << std::setprecision(9)
                << "x = " << x << "\t\ty = " << y
                << "\t\tyaw = " << yaw
                << "\t\tts = " << std::setprecision(15) << getTs()
                << '\n';
            prev_x=x;
            prev_y=y;
            prev_yaw=yaw;
        }

        // std::cout << std::setprecision(9)
        //         << "x = " << x << "\t\ty = " << y
        //         << "\t\tyaw = " << yaw
        //         << "\t\tts = " << std::setprecision(15) << getTs()
        //         << '\n';
    }

    void PoseLogger::saveTraj(const std::string& save_dir, const bool localization_mode){
        fs::path savePath{save_dir};
        if (!fs::exists(savePath))
            fs::create_directories(savePath);

        fs::path latestSymlink{savePath};
        latestSymlink.append("latest");

        if (!localization_mode) 
        {
            fs::path newMapPath{savePath};
            newMapPath.append("map-" + currDateTime());
            fs::create_directory(newMapPath);
            if (fs::exists(latestSymlink))
                fs::remove(latestSymlink);
            fs::create_symlink(newMapPath, latestSymlink);
            // output the map database
            SLAM->save_map_database(latestSymlink.string() + "/map.msg");
            SLAM->save_frame_trajectory(latestSymlink.string() + "/mapping_trajectory.txt", "KITTI");
        }
        else 
        {
            savePath.append("latest");
            fs::path fullSavePath{fs::read_symlink(savePath)};
            fs::path mapSavePath{fs::read_symlink(savePath)};
            std::string dirName = "loc-" + currDateTime();
            savePath.append(dirName);
            fullSavePath.append(dirName);
            fs::create_directory(savePath);
            latestSymlink.append("latest");
            if (fs::exists(latestSymlink))
                fs::remove(latestSymlink);
            fs::create_symlink(fullSavePath, latestSymlink);
            // output the trajectories for evaluation
            SLAM->save_frame_trajectory(latestSymlink.string() + "/trajectory.txt", "BOX");
        }
    }

    FrameFeeder::FrameFeeder(const std::shared_ptr<openvslam::system> SLAM, 
                                const std::shared_ptr<openvslam::config> cfg,
                                const std::shared_ptr<box::PoseLogger> PoseLogger,
                                const std::string& save_dir,
                                const bool localizationMode
                                ): SLAM(SLAM), rectifier(cfg), poseLogger(PoseLogger){
         if (localizationMode) 
         {
            // startup the SLAM process (don't need to initialize a map since we're only localizing)
            SLAM->load_map_database(save_dir + "/latest/map.msg");
            SLAM->startup(false);
            SLAM->disable_mapping_module();
            SLAM->disable_loop_detector();
        }
        else 
        {
            // startup the SLAM process
            SLAM->startup();
        }
        
        switch(cfg->camera_->setup_type_){
            case(openvslam::camera::setup_type_t::Monocular):
                stereo = false;
                break;
            case(openvslam::camera::setup_type_t::Stereo):
                stereo = true;
                break;
            default:
                exit(-1);
        }
    }

    void FrameFeeder::pauseFeed(const bool pause)
    {
        paused = pause;
    }

    void FrameFeeder::run(){
        // RealSense settings
        rs2::pipeline pipeline;
        rs2::config config;
        config.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8, 30);
        config.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8, 30);

        pipeline.start(config);

        rs2::frameset data;
        unsigned long long prev_frame_num = 0;

        while (true) {
            if(!paused)
            {
                auto t1 = getTs();
                data = pipeline.wait_for_frames();
                rs2::video_frame curr_frame = data.get_fisheye_frame(1);

                if(curr_frame.get_frame_number() == prev_frame_num && !stereo)
                    continue;

                frame1 = funcFormat::frame2Mat(curr_frame);
                if(stereo) frame2 = funcFormat::frame2Mat(data.get_fisheye_frame(2));

                if(stereo){
                    rectifier.rectify(frame1, frame2, input1, input2);
                    SLAM->feed_stereo_frame(input1, input2, box::getTs());
                }
                else{
                    rectifier.undistort(frame1, input1);       
                    SLAM->feed_monocular_frame(input1, box::getTs());
                }

                poseLogger->updateCurrPose();

                prev_frame_num = curr_frame.get_frame_number();
                std::cout << getTs() - t1 << '\n';
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (SLAM->terminate_is_requested())
                break;
        }

        // wait until the loop BA is finished
        while (SLAM->loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
    }

    ServerCom::ServerCom(const std::shared_ptr<openvslam::system> SLAM, 
                            const std::shared_ptr<box::PoseLogger> poseLogger) : 
                                SLAM(SLAM), poseLogger(poseLogger){
        
    }
}