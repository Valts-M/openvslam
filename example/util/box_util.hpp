#ifndef BOX_UTIL_H
#define BOX_UTIL_H

#ifndef UPDATE_DIST
#define UPDATE_DIST 0.05
#endif

#ifndef UPDATE_ANG
#define UPDATE_ANG 0.174533
#endif

#include "openvslam/system.h"
#include "openvslam/config.h"
#include "openvslam/util/stereo_rectifier.h"
#include "openvslam/io/map_database_io.h"

#include <chrono>
#include <string>
#include <time.h>


namespace box{

    // Get current date/time, format is YYYY-MM-DD.HH-mm-ss
    const std::string currDateTime();

    inline double getTs(){
        return static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    }

    class PoseLogger{
        public:
            PoseLogger(const std::shared_ptr<openvslam::system> SLAM);
            void saveTraj(const std::string& save_dir, const bool localization_mode);
            void updateCurrPose();
        private:
            const std::shared_ptr<openvslam::system> SLAM;
            std::shared_ptr<openvslam::publish::map_publisher> mapPublisher;
            double prev_x = 0;
            double prev_y = 0;
            double prev_yaw = 0;
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

    class ServerCom{
        public:
            ServerCom(const std::shared_ptr<openvslam::system> SLAM, 
                const std::shared_ptr<box::PoseLogger> poseLogger);
            void run();
        private:
            const std::shared_ptr<openvslam::system> SLAM;
            const std::shared_ptr<box::PoseLogger> poseLogger;
    };
}

#endif