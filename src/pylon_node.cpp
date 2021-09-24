#include "../include/pylon_node.h"

#include <fstream>
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>

#define KERNEL_BUFFER_SZ 2000

#define NORMAL_DELAY 333*1000
#define PRETEST_DELAY 133*1000
#define PRETEST_COUNT 10

using namespace std;

//sudo gpasswd -a [user] video
pylonNode::pylonNode() :
    cameraCount_(0),
    connected_(0),
    grabbing_(false),
    preTotalFileCnt_(0),
    existStoreDir_(false),
    checkFile_(false)
{
    status_ = new baslerStatus();

    classToEnum_["BaslerUsb"] = (int)DeviceType::USB;
    classToEnum_["BaslerGigE"] = (int)DeviceType::GIGE;

    getParameters();

    subscribeAndPublish();

    if(!connect())
      exit(0);
}

pylonNode::~pylonNode()
{
    disconnect();

    delete statusThread_;
    delete pubThread_;
    delete cameraImagePub_;

    statusThread_ = nullptr;
    pubThread_ = nullptr;
    cameraImagePub_ = nullptr;
}

void pylonNode::getParameters()
{
    int camera_cnt;
    nh_.param<int>("/camera_count", camera_cnt, 0);

    for(int i=0; i<camera_cnt; i++)
    {
        iCamera temp;
        int serial_num;

        nh_.param<std::string>("/camera" + std::to_string(i+1) + "/name", temp.name, "");
        nh_.param<int>("/camera" + std::to_string(i+1) + "/serial_number", serial_num, 0);
        temp.serial = std::to_string(serial_num);

        addCamera(temp);
    }
}

void pylonNode::subscribeAndPublish()
{
    srvGrabbing_      = nh_.advertiseService("basler_ros_driver/grabbing", &pylonNode::grabbingService, this);
    srvTrigger_       = nh_.advertiseService("basler_ros_driver/trigger", &pylonNode::triggerService, this);
    srvSaveDirectory_ = nh_.advertiseService("basler_ros_driver/set_save_dir", &pylonNode::saveDirectoryService, this);

    cameraInfoPub_    = nh_.advertise<basler_ros_driver::cameraInfo>("basler_ros_driver/camera_info", 1);
    cameraImagePub_   = new image_transport::Publisher[iCamList_.size()];
    for(int i=0; i<iCamList_.size(); i++)
    {
        image_transport::ImageTransport it(nh_);
        cameraImagePub_[i] = it.advertise("basler_ros_driver/camera" + to_string(i+1) + "/image_raw", 1);
    }
}

void pylonNode::addCamera(iCamera camera)
{
    iCamList_.push_back(camera);
}

void pylonNode::initStatus()
{
    preTotalFileCnt_ = 0;
    existStoreDir_ = false;
    checkFile_ = false;

    status_->triggerCount = 0;

    for(int i=0; i<cameraCount_; i++)
    {
        pylonUnitList_.at(i)->initCnt();
    }
}

//-1: impossible, 0: failed.. retry, 1: success
bool pylonNode::connect()
{
    if(connected_)
        return true;

    // Status Initialize
    preTotalFileCnt_ = 0;
    existStoreDir_ = false;

    // Pylon Initialize
    Pylon::PylonInitialize();
    Pylon::CTlFactory &camSystem = Pylon::CTlFactory::GetInstance();
    camSystem.EnumerateDevices(baslerList_, false);

    cameraCount_ = baslerList_.size();

    if(cameraCount_ <= 0)
    {
        ROS_ERROR_STREAM("Not found Basler camera!");
        connected_ = -1;
        return false;
    }

    // Check USB Memory
    int usb_count = 0;
    for (size_t i=0; i<baslerList_.size(); i++)
    {
        if(baslerList_[i].GetDeviceClass() == "BaslerUsb")
            usb_count++;
    }

    ifstream usbfsParamPath("/sys/module/usbcore/parameters/usbfs_memory_mb");

    uint16_t usbFsSz;
    usbfsParamPath >> usbFsSz;
    usbfsParamPath.close();

    if (usbFsSz < KERNEL_BUFFER_SZ * usb_count)
    {
        ROS_ERROR_STREAM("USBFS kernel parameter is too small. It must be bigger than " << KERNEL_BUFFER_SZ * usb_count << ". Fix Kernel setting!");
        connected_ = -1;
        return false;
    }

    // Allocate Memory
    statusThread_ = new std::thread*[cameraCount_];
    for(int i=0; i<cameraCount_; i++)
    {
        statusThread_[i] = nullptr;
    }
    pubThread_ = nullptr;

    // Make pretest folder
    chdir(getenv("HOME"));
    mkdir("tmp", 0777);
    chdir("tmp");

    ROS_INFO_STREAM("Found Camera: " << cameraCount_);

    // Check detect basler count with xml
    if(cameraCount_ != iCamList_.size())
    {
        ROS_ERROR_STREAM("Camera count is not matched with parameter");
        connected_ = -1;
        return false;
    }

    // Set pylon unit params
    pylonUnitList_.resize(cameraCount_);

    for(int i=0; i<cameraCount_; i++)
    {
        auto pylon = new Pylon::CBaslerUniversalInstantCamera(camSystem.CreateDevice(baslerList_[i]));
        pylonUnit *temp = new pylonUnit(pylon);

        // Matching serial number
        bool matching = false;
        for(int j=0; j<iCamList_.size(); j++)
        {
            // Re-order camera sequence by xml info
            if(iCamList_[j].serial.compare(temp->getDeviceSerialNumber()) == 0)
            {
                pylonUnitList_.at(j) = temp; //copy

                string cam_path = get_current_dir_name();
                cam_path += "/";
                cam_path += iCamList_[j].name;
                mkdir(cam_path.c_str(), 0777);

                pylonUnitList_.at(j)->setStoreDir(cam_path);
                pylonUnitList_.at(j)->setGrabbing(true);

                ROS_INFO_STREAM("Serial number: " << temp->getDeviceSerialNumber());
                ROS_INFO_STREAM("Connect Type: " << temp->getDeviceClass());
                ROS_INFO_STREAM("Camera store path: " << cam_path);

                matching = true;
                break;
            }
        }

        if(!matching)
        {
            ROS_ERROR_STREAM("Cannot find matching serial: " << temp->getDeviceSerialNumber());
        }
    }
    chdir("..");

    // Make connect state true
    connected_ = 1;

    // Delay for send message to controller
    usleep(NORMAL_DELAY);

    // Pre-test for correct image data
    for(int i=0; i<PRETEST_COUNT; i++)
    {
        for(int j=0; j<cameraCount_; j++)
            pylonUnitList_.at(j)->softwareTrigger();

        cout << "Triggering.. [" << setfill('0') << setw(2) << (i + 1) << "/" << PRETEST_COUNT << "]";
        fflush(stdout);
        for (int j = 0; j < 21; j++)
        {
          cout << "\033[D";
        }
        fflush(stdout);

        usleep(PRETEST_DELAY);
    }

    usleep(NORMAL_DELAY);

    bool pretest_flag = true;

    for(int i=0; i<cameraCount_; i++)
    {
        int temp_cnt = pylonUnitList_.at(i)->checkCurrentFileCnt();

        if(temp_cnt < PRETEST_COUNT)
        {
            pretest_flag = false;
            ROS_ERROR_STREAM(i+1 << " >> pre-test failed .. (" << temp_cnt << "/" << PRETEST_COUNT << ")");
            continue;
        }

        ROS_INFO_STREAM(i+1 << " >> camera connected! (" << temp_cnt << "/" << PRETEST_COUNT << ")");
    }

    for(int i=0; i<cameraCount_; i++)
        pylonUnitList_.at(i)->setGrabbing(false);

    if(!pretest_flag)
    {
        disconnect();
        return false;
    }
    else
    {
        // Delete pre-test folder
        system("rm -rf ./tmp");

        // Initialize status
        if(status_ != nullptr)
        {
            status_->connectType.resize(cameraCount_);
            status_->capturePath.resize(cameraCount_);
            status_->captureCount.resize(cameraCount_);
            status_->fileCount.resize(cameraCount_);
        }

        for(int i=0; i<cameraCount_; i++)
        {
            pylonUnitList_.at(i)->initCnt();
            status_->connectType.at(i) = pylonUnitList_.at(i)->getDeviceClass();
            statusThread_[i] = new std::thread(&pylonNode::imageMonitoring, this, i);
        }
        pubThread_ = new std::thread(&pylonNode::publishCamInfo, this);

        return connected_;
    }
}

bool pylonNode::disconnect()
{
    if(!connected_)
        return true;

    // Disconnect cameras
    connected_ = 0;

    for(int i=0; i<cameraCount_; i++)
    {
        if(statusThread_[i] != nullptr && statusThread_[i]->joinable())
        {
            statusThread_[i]->join();
            delete statusThread_[i];
            statusThread_[i] = nullptr;
        }

        delete pylonUnitList_[i];
        pylonUnitList_[i] = nullptr;
    }
    pylonUnitList_.clear();

    Pylon::PylonTerminate();

    return true;
}

void pylonNode::setImageQuality(int idx, double brightness, int funcProfile, double gainLower, double gainUpper, double timeLower, double timeUpper)
{
    pylonUnitList_.at(idx)->setImageQuality(brightness, funcProfile, gainLower, gainUpper, timeLower, timeUpper);
}

void pylonNode::getImageQuality(int idx, double &brightness, int &funcProfile, double &gainLower, double &gainUpper, double &timeLower, double &timeUpper)
{
    pylonUnitList_.at(idx)->getImageQuality(brightness, funcProfile, gainLower, gainUpper, timeLower, timeUpper);
}

string pylonNode::getLastFilePath(int id)
{
    return pylonUnitList_.at(id)->getSavePath();
}

bool pylonNode::grabbingService(basler_ros_driver::grabbingRequest &req, basler_ros_driver::grabbingResponse &res)
{
    grabbing_ = req.flag;

    for(int i=0; i<cameraCount_; i++)
        pylonUnitList_.at(i)->setGrabbing(req.flag);

    res.success = true;
    return res.success;
}

bool pylonNode::triggerService(basler_ros_driver::triggerRequest &req, basler_ros_driver::triggerResponse &res)
{
    if(req.signal)
    {
        status_->triggerCount++;

        for(int i=0; i<cameraCount_; i++)
        {
            pylonUnitList_.at(i)->setBusyFlag(true);
            pylonUnitList_.at(i)->softwareTrigger();
        }
    }

    res.success = true;
    return res.success;
}

bool pylonNode::saveDirectoryService(basler_ros_driver::saveDirectoryRequest &req, basler_ros_driver::saveDirectoryResponse &res)
{
    std::string cam_path;

    checkFile_ = true;

    if(existStoreDir_)
    {
        preTotalFileCnt_ += pylonUnitList_.front()->checkCurrentFileCnt();
        existStoreDir_ = false;
    }

    usleep(100000);

    for(int i=0; i<cameraCount_; i++)
    {
        cam_path = req.path + "/" + iCamList_.at(i).name;

        //mkdir(cam_path.c_str(), 0777);
        system((std::string("mkdir -p ") + cam_path).c_str());
        pylonUnitList_.at(i)->setStoreDir(cam_path);
    }

    existStoreDir_ = true;

    res.success = true;
    return res.success;
}

void pylonNode::imageMonitoring(int id)
{
    while(connected_)
    {
        if(status_ == NULL)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if(checkFile_)
            status_->fileCount.at(id) = pylonUnitList_.at(id)->checkCurrentFileCnt();
        else
            status_->fileCount.at(id) = 0;

        status_->captureCount.at(id) = status_->fileCount.at(id) + preTotalFileCnt_;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void pylonNode::publishCamInfo()
{
    while(ros::ok() && connected_)
    {
        // Publish Info
        basler_ros_driver::cameraInfo msg;

        msg.trigger_count = status_->triggerCount;
        for(int i=0; i<cameraCount_; i++)
        {
            msg.status.push_back(basler_ros_driver::baslerStatus());
            msg.status.back().connect_type = status_->connectType.at(i);
            msg.status.back().capture_count = status_->captureCount.at(i);

            std::string path = getLastFilePath(i);
            if(!path.empty() && (path != status_->capturePath.at(i)) && (msg.status.back().capture_count > 0))
            {
                while(pylonUnitList_.at(i)->getBusyFlag())
                {
                    usleep(10);
                }

                cv::Mat cv_img = cv::imread(path, cv::IMREAD_COLOR);
                if(!cv_img.empty())
                {
                    std_msgs::Header header;
                    header.stamp = ros::Time::now();

                    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", cv_img).toImageMsg();

                    cameraImagePub_[i].publish(img_msg);
                    cameraInfoPub_.publish(msg);

                    status_->capturePath.at(i) = path;
                }
            }
        }        

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pylon_node");

    ROS_INFO("\033[1;32m----> Pylon node Started.\033[0m");

    pylonNode PN;

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    return 0;
}
