#ifndef BOX_UTIL_H
#define BOX_UTIL_H

#ifndef UPDATE_DIST
#define UPDATE_DIST 20 //20 cm
#endif

#ifndef UPDATE_ANG
#define UPDATE_ANG 150 //15 deg
#endif

#include "openvslam/system.h"
#include "openvslam/config.h"
#include "openvslam/util/stereo_rectifier.h"
#include "openvslam/io/map_database_io.h"

#include <chrono>
#include <string>
#include <time.h>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

namespace box{

    const std::string logDir = "/logs";

    // Get current date/time, format is YYYY-MM-DD.HH-mm-ss
    const std::string currDateTime();

    //Get current date, format is YYYY-MM-DD
    const std::string currDate();

    inline unsigned long long getTs(){
        return static_cast<unsigned long long>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    }
    
    #pragma pack(1)
    struct Pose2D
    {
        int32_t x; //cm
        int32_t y; //cm
        int16_t yaw; //0.1 deg
        unsigned long long ts; //ms

        Pose2D(const double _x, const double _y, const double _yaw, const unsigned long long _ts)
        {
            x = static_cast<int32_t>(_x * 100);
            y = static_cast<int32_t>(_y * 100);
            yaw = static_cast<int16_t>(_yaw * 1800 / M_PI); //0.1 deg resolution
            ts = _ts;
        }

        Pose2D()
        {
            x = 0;
            y = 0;
            yaw = 0;
            ts = 0;
        }

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

    class PoseLogger{
        public:
            PoseLogger(const std::shared_ptr<openvslam::system> SLAM);
            void saveTraj(const std::string& saveDir, const bool localizationMode);
            void updateLog();
            
            //returns filepath to log
            std::string finishLog();
            void startNewLog(const std::string& saveDir);
            Pose2D getCurrPose();
            bool getPoseAvailability();
            void setPoseAvailability(const bool newPoseAvailable);
        private:
            const std::shared_ptr<openvslam::system> SLAM;
            std::shared_ptr<openvslam::publish::map_publisher> mapPublisher;
            std::string currLogPath{};
            Pose2D currPose{};
            std::mutex currPoseMtx;
            std::mutex newPoseAvailableMtx;
            bool newPoseAvailable = false;
            
            bool shouldUpdatePose(const Pose2D& pose);
            void updateCurrPose(const double x, const double y, const double yaw, const unsigned long long ts);
            void updateCurrPose(const Pose2D& pose);
            void updateLogPath();
            void createLogDir();
    };

    class FrameFeeder{
        public:
            FrameFeeder(const std::shared_ptr<openvslam::system> SLAM, 
                            const std::shared_ptr<openvslam::config> cfg,
                            const std::shared_ptr<box::PoseLogger> poseLogger,
                            const std::string& saveDir,
                            const bool localizationMode
                            );
            void run();
            void pauseFeed(const bool pause);
        private:
            bool paused;
            bool stereo;
            const std::shared_ptr<openvslam::system> SLAM;
            const openvslam::util::stereo_rectifier rectifier;
            const std::shared_ptr<box::PoseLogger> poseLogger;
            cv::Mat frame1, frame2, input1, input2;
    };

    class ServerCom
    {
        public:
            ServerCom(const std::shared_ptr<box::PoseLogger> poseLogger);
            virtual void run();
            void requestShutdown();

        protected:
            std::mutex shutdownMtx;
            bool shutdownRequested();
            bool _shutdownRequested;
            const std::shared_ptr<box::PoseLogger> poseLogger;
    };

    class StatusSender : public ServerCom
    {
        public:
            StatusSender(const std::shared_ptr<box::PoseLogger> poseLogger);
            void run() override;
        private:


    };

    class TelemetrySender : public ServerCom
    {
        public:
            TelemetrySender(const std::shared_ptr<box::PoseLogger> poseLogger);
            void run() override;
            bool sendLog(const std::string& filename);
        private:
            //returns the name of the compressed log file
            std::string compressLog(const std::string& filename);
            void sendTelemetry();
    };
}

#endif