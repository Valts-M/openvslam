
#include "box_util.hpp"

#include "realsense_util.h"
#include "openvslam/publish/map_publisher.h"

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <filesystem>
#include <mutex>
#include <future>
#include <stdlib.h>    // free
#include <string.h>    // strlen, strcat, memset
#include "zstd_helper.h"    // Helper functions, CHECK(), and CHECK_ZSTD()

#include <spdlog/spdlog.h>
#include <popl.hpp>
#include <curlpp/Infos.hpp>
#include <zstd.h>

namespace box{

    namespace fs = std::filesystem;
    std::string serverUrl{"https://server.squadrobotic.eu"};
    std::string serverAddr{"server.squadrobotic.eu"};
    std::string guid{"b69065ef-0a5e-4a17-a8b6-cb95ea7b76ca"};

    const std::string currDateTime() 
    {
        time_t now = time(0);
        struct tm tstruct;
        char buf[80];
        tstruct = *gmtime(&now);
        // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
        // for more information about date/time format
        strftime(buf, sizeof(buf), "%Y-%m-%d.%H-%M-%S", &tstruct);

        return buf;
    }

    const std::string currDate() 
    {
        time_t now = time(0);
        struct tm tstruct;
        char buf[80];
        tstruct = *gmtime(&now);
        // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
        // for more information about date/time format
        strftime(buf, sizeof(buf), "%Y-%m-%d", &tstruct);

        return buf;
    }

    PoseLogger::PoseLogger(const std::shared_ptr<openvslam::system> SLAM): SLAM(SLAM), mapPublisher(SLAM->get_map_publisher())
    {
        
    }

    void PoseLogger::updateLog()
    {
        if(currLogPath.empty())
        {
            spdlog::error("Called updateLog with no new log file specified");
            return;
        }

        const auto pose = mapPublisher->get_current_cam_pose();
        const double y = -pose(0, 3);
        const double x = pose(2, 3);
        const double yaw = atan2(-pose(2, 0), pose(2, 2));
        const unsigned long long ts = getTs();
        const Pose2D tmpPose(x, y, yaw, ts);

        std::ofstream ofs(currLogPath, std::ios::app | std::ios::binary);
        if (!ofs.is_open()) {
            spdlog::critical("cannot create a file at {}", currLogPath);
            throw std::runtime_error("cannot create a file at " + currLogPath);
        }

        if(shouldUpdatePose(tmpPose))
        {
            // std::cout << tmpPose.x - currPose.x << ' ' << tmpPose.y - currPose.y << ' ' << tmpPose.yaw - currPose.yaw << '\n';
            std::cout << currPose.yaw << '\n';
            updateCurrPose(tmpPose);
            setPoseAvailability(true);

            //Have to write indevidual parts of struct to file to avoid padding
            std::lock_guard<std::mutex> lock(currPoseMtx);
            ofs.write((char*)&currPose, sizeof(currPose));
        }

        // std::cout << std::setprecision(9)
        //         << "x = " << x << "\t\ty = " << y
        //         << "\t\tyaw = " << yaw
        //         << "\t\tts = " << std::setprecision(15) << getTs()
        //         << '\n';
    }

    bool PoseLogger::shouldUpdatePose(const Pose2D& pose)
    {
        std::lock_guard<std::mutex> lock(currPoseMtx);
        return (std::abs(pose.x - currPose.x) > UPDATE_DIST || 
                std::abs(pose.y - currPose.y) > UPDATE_DIST || 
                std::abs(pose.yaw - currPose.yaw) > UPDATE_ANG);
    }

    void PoseLogger::updateCurrPose(const double x, const double y, const double yaw, const unsigned long long ts)
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

        currLogPath = fs::path(DateDir / (currDateTime() + ".log").c_str());
    }

    std::string PoseLogger::finishLog()
    {
        const std::string returnVal = currLogPath;
        currLogPath = "";
        return returnVal;
    }

    void PoseLogger::saveTraj(const std::string& save_dir, const bool localization_mode)
    {
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
                                ): SLAM(SLAM), rectifier(cfg), poseLogger(PoseLogger)
    {
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
        paused = false;
        
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

    void FrameFeeder::run()
    {
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
                const double startTime = getTs();
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

                poseLogger->updateLog();

                prev_frame_num = curr_frame.get_frame_number();
                spdlog::debug("Frame processing time: {} ms", startTime - getTs());
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

    StatusSender::StatusSender(const std::shared_ptr<box::PoseLogger> poseLogger) : poseLogger(poseLogger)
    {
        curlpp::initialize();
        // compressLog("/home/valts/test/2022-02-08/2022-02-08.08-29-23.log");
        // sendLog("/home/valts/test/2022-02-08/2022-02-08.08-29-23.zst");
    }

    StatusSender::~StatusSender()
    {
        curlpp::terminate();
    }

    

    bool StatusSender::sendStatus()
    {
        try
        {
            std::list<std::string> header;
            header.push_back("Id: " + guid);

            curlpp::Cleanup clean;
            curlpp::Easy request;
            request.setOpt(new curlpp::options::Url(serverUrl + "/Slambox/PostSlamboxIp"));
            request.setOpt(new curlpp::options::HttpHeader(header));
            request.setOpt(new curlpp::options::PostFieldSize(0));

            request.setOpt(new curlpp::options::WriteStream(&std::cout));

            request.perform();

            auto http_code = curlpp::infos::ResponseCode::get(request);
            if (http_code == 200) 
            {
                return true;
            } 
            else 
            {
                return false;
            }
                
        }
        catch( curlpp::RuntimeError &e )
        {
            // std::cerr << e.what() << std::endl;
            spdlog::error(e.what());
            return false;
        }
        catch( curlpp::LogicError &e )
        {
            // std::cerr << e.what() << std::endl;
            spdlog::error(e.what());
            return false;
        }
    }

    void StatusSender::run()
    {
        
    }

    void TelemetrySender::sendTelemetry()
    {
        const Pose2D pose = poseLogger->getCurrPose();

        spdlog::debug("Sending telemetry {}", pose.toString());

        // udpClient.send((char*)&pose, sizeof(pose));

        try
        {
            std::list<std::string> header;
            header.push_back("Id: " + guid);
            header.push_back("Content-Type: application/octet-stream");

            curlpp::Cleanup clean;
            curlpp::Easy request;
            request.setOpt(new curlpp::options::Url(serverUrl + "/Slambox/PostLog"));
            request.setOpt(new curlpp::options::HttpHeader(header));
            request.setOpt(new curlpp::options::Verbose(true));
            request.setOpt(new curlpp::options::PostFields((char*)&pose));
            request.setOpt(new curlpp::options::PostFieldSize(sizeof(pose)));
            request.setOpt(new curlpp::options::Timeout(100));
            request.setOpt(new curlpp::options::WriteStream(&std::cout));

            request.perform();

            auto http_code = curlpp::infos::ResponseCode::get(request);
            std::cout << http_code << '\n';
            if (http_code == 200) 
            {
            } 
            else 
            {
            }
                
        }
        catch( curlpp::RuntimeError &e )
        {
            std::cerr << e.what() << std::endl;
        }
        catch( curlpp::LogicError &e )
        {
            std::cerr << e.what() << std::endl;
        }
    }

    bool TelemetrySender::sendLog(const std::string& filename)
    {
        try
        {
            std::string compressedFileName = compressLog(filename);
            std::list<std::string> header;
            header.push_back("Id: " + guid);
            header.push_back("Content-Type: application/octet-stream");

            curlpp::Forms form;
            form.push_back(new curlpp::FormParts::File("Log", compressedFileName));

            curlpp::Cleanup clean;
            curlpp::Easy request;
            request.setOpt(new curlpp::options::Url(serverUrl + "/Slambox/PostLog"));
            request.setOpt(new curlpp::options::HttpHeader(header));
            // request.setOpt(new curlpp::options::HttpPost(form));
            request.setOpt(new curlpp::options::Upload(true));
            request.setOpt(new curlpp::options::Verbose(true));
            request.setOpt(new curlpp::options::ReadFile(fopen(compressedFileName.c_str(), "rb")));
            request.setOpt(new curlpp::options::Timeout(100));
            request.setOpt(new curlpp::options::WriteStream(&std::cout));

            request.perform();

            auto http_code = curlpp::infos::ResponseCode::get(request);
            std::cout << http_code << '\n';
            if (http_code == 200) 
            {
                return true;
            } 
            else 
            {
                return false;
            }
                
        }
        catch( curlpp::RuntimeError &e )
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
        catch( curlpp::LogicError &e )
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
    }

    std::string TelemetrySender::compressLog(const std::string& filename)
    {
        size_t fSize;
        void* const fBuff = mallocAndLoadFile_orDie(filename.c_str(), &fSize);
        size_t const cBuffSize = ZSTD_compressBound(fSize);
        void* const cBuff = malloc_orDie(cBuffSize);

        size_t const cSize = ZSTD_compress(cBuff, cBuffSize, fBuff, fSize, 1);
        CHECK_ZSTD(cSize);

        std::string outfilename = filename.substr(0,filename.find_last_of('.')) + ".zst";

        saveFile_orDie(outfilename.c_str(), cBuff, cSize);

        /* success */
        printf("%25s : %6u -> %7u - %s \n", filename.c_str(), (unsigned)fSize, (unsigned)cSize, outfilename.c_str());
        spdlog::info("Compressed log {} : {} -> {}. Saved to {}", filename, fSize, cSize, outfilename);

        free(fBuff);
        free(cBuff);
        return outfilename;
    }

    TelemetrySender::TelemetrySender(const std::shared_ptr<box::PoseLogger> poseLogger) : poseLogger(poseLogger)
    {

    }

    void TelemetrySender::run()
    {
        while(true)
        {
            if(poseLogger->getPoseAvailability())
            {
                sendTelemetry();
                poseLogger->setPoseAvailability(false);
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if(shutdownRequested())
                break;
        }
    }

    bool TelemetrySender::shutdownRequested()
    {
        return _shutdownRequested;
    }
  
}