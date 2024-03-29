#ifndef PYLONNODE_H
#define PYLONNODE_H

#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <basler_ros_driver/grabbing.h>
#include <basler_ros_driver/saveDirectory.h>
#include <basler_ros_driver/setCameraOptions.h>
#include <basler_ros_driver/trigger.h>
#include <basler_ros_driver/cameraInfo.h>

#include <iostream>
#include <thread>
#include <vector>
#include <map>

#include <unistd.h>
#include <sys/stat.h>

#include "pylon_unit.h"

struct iCamera
{
    std::string name;
    std::string serial;
};

struct baslerStatus
{
    std::vector<std::string>  connectType;
    std::vector<std::string>  capturePath;
    std::vector<int>          captureCount;
    std::vector<int>          fileCount;
    int                       triggerCount;
};

class pylonNode
{
public:
    pylonNode();
    ~pylonNode();

    void getParameters();
    void subscribeAndPublish();
    void loadCameraData(int id, std::string path);
    void addCamera(iCamera camera);
    void initStatus();

    bool connect();
    bool disconnect();

    void setImageQuality(int idx, double brightness, int funcProfile, double gainLower, double gainUpper, double timeLower, double timeUpper);
    void getImageQuality(int idx, double &brightness, int &funcProfile, double &gainLower, double &gainUpper, double &timeLower, double &timeUpper);
    std::string getLastFilePath(int id);

private:
    bool grabbingService(basler_ros_driver::grabbingRequest& req, basler_ros_driver::grabbingResponse& res);
    bool saveDirectoryService(basler_ros_driver::saveDirectoryRequest& req, basler_ros_driver::saveDirectoryResponse& res);
    bool setCameraOptionsService(basler_ros_driver::setCameraOptionsRequest& req, basler_ros_driver::setCameraOptionsResponse& res);
    bool triggerService(basler_ros_driver::triggerRequest& req, basler_ros_driver::triggerResponse& res);

    void imageMonitoring(int id);
    void publishCamInfo();
    void sendTrigger();

private:
    ros::NodeHandle nh_;

    ros::ServiceServer srvGrabbing_;
    ros::ServiceServer srvSaveDirectory_;
    ros::ServiceServer srvSetCamOptions_;
    ros::ServiceServer srvTrigger_;

    ros::Publisher captureInfoPub_;

    baslerStatus *status_;
    std::thread  *pubThread_;
    std::thread  *triggerThread_;
    std::thread  **statusThread_;
    std::vector<iCamera> iCamList_;
    std::vector<pylonUnit*> pylonUnitList_;
    sensor_msgs::CameraInfo *camera_info_;

    Pylon::DeviceInfoList baslerList_;

    int cameraCount_;
    int connected_;
    int frame_rate_;
    bool grabbing_;
    int preTotalFileCnt_;
    bool existStoreDir_;
    bool checkFile_;

    std::map<std::string, int> classToEnum_;
};

#endif // PYLONNODE_H
