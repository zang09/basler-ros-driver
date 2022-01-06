#include "pylon_unit.h"

#include <iostream>
#include <dirent.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <unistd.h>

using namespace std;
using namespace Basler_UniversalCameraParams;

pylonUnit::pylonUnit(Pylon::CBaslerUniversalInstantCamera* pylon)
{
    pylon_ = pylon;

    configuration_ = new CCustomConfiguration();
    eventHandler_  = new CCustomEvtHandler();

    pylon_->RegisterConfiguration(configuration_, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_Delete);
    pylon_->RegisterImageEventHandler(eventHandler_, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_Delete);

    classToEnum_["BaslerUsb"] = (int)DeviceType::USB;
    classToEnum_["BaslerGigE"] = (int)DeviceType::GIGE;

    configuration_->setType((DeviceType)classToEnum_[pylon_->GetDeviceInfo().GetDeviceClass().c_str()]);
    pylon_->Open();
}

pylonUnit::~pylonUnit()
{
    if(pylon_->IsGrabbing())
        pylon_->StopGrabbing();

    if(pylon_->IsOpen())
        pylon_->Close();

    pylon_->DeregisterConfiguration(configuration_);
    pylon_->DeregisterImageEventHandler(eventHandler_);

    configuration_ = nullptr;
    eventHandler_  = nullptr;
}

void pylonUnit::initCnt()
{
    eventHandler_->initCnt();
}

void pylonUnit::setID(int id)
{
    eventHandler_->setID(id);
}

void pylonUnit::setCameraData(sensor_msgs::CameraInfo cam_info)
{
    eventHandler_->setCameraData(cam_info);
}

void pylonUnit::setReverseOption(bool x, bool y)
{
    while(eventHandler_->getBusyFlag())
    {
        usleep(100);
    }

    pylon_->ReverseX.SetValue(x);
    pylon_->ReverseY.SetValue(y);

    return;
}

void pylonUnit::setImageQuality(double brightness, int funcProfile, double gainLower, double gainUpper, double timeLower, double timeUpper)
{
    while(eventHandler_->getBusyFlag())
    {
        usleep(100);
    }

    pylon_->StopGrabbing();

    switch (configuration_->getType()) {
    case DeviceType::USB:
    {
        pylon_->AutoTargetBrightness.SetValue(brightness);
        pylon_->AutoFunctionProfile.SetValue(AutoFunctionProfileEnums(funcProfile));
        pylon_->AutoGainLowerLimit.SetValue(gainLower);
        pylon_->AutoGainUpperLimit.SetValue(gainUpper);
        pylon_->AutoExposureTimeLowerLimit.SetValue(timeLower);
        pylon_->AutoExposureTimeUpperLimit.SetValue(timeUpper);
        break;
    }
    case DeviceType::GIGE:
    {
        pylon_->AutoTargetValue.SetValue(brightness);
        pylon_->AutoFunctionProfile.SetValue(AutoFunctionProfileEnums(funcProfile));
        pylon_->AutoGainRawLowerLimit.SetValue(gainLower);
        pylon_->AutoGainRawUpperLimit.SetValue(gainUpper);
        pylon_->AutoExposureTimeLowerLimitRaw.SetValue(timeLower);
        pylon_->AutoExposureTimeUpperLimitRaw.SetValue(timeUpper);
        break;
    }
    default:
        break;
    }

    pylon_->StartGrabbing(Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByInstantCamera);

    return;
}

void pylonUnit::getImageQuality(double &brightness, int &funcProfile, double &gainLower, double &gainUpper, double &timeLower, double &timeUpper)
{
    switch (configuration_->getType()) {
    case DeviceType::USB:
    {
        brightness = pylon_->AutoTargetBrightness.GetValue();
        AutoFunctionProfileEnums e = pylon_->AutoFunctionProfile.GetValue();
        funcProfile = (int)e;
        gainLower = pylon_->AutoGainLowerLimit.GetValue();
        gainUpper = pylon_->AutoGainUpperLimit.GetValue();
        timeLower = pylon_->AutoExposureTimeLowerLimit.GetValue();
        timeUpper = pylon_->AutoExposureTimeUpperLimit.GetValue();
        break;
    }
    case DeviceType::GIGE:
    {
        brightness = pylon_->AutoTargetValue.GetValue();
        AutoFunctionProfileEnums e = pylon_->AutoFunctionProfile.GetValue();
        funcProfile = (int)e;
        gainLower = pylon_->AutoGainRawLowerLimit.GetValue();
        gainUpper = pylon_->AutoGainRawUpperLimit.GetValue();
        timeLower = pylon_->AutoExposureTimeLowerLimitRaw.GetValue();
        timeUpper = pylon_->AutoExposureTimeUpperLimitRaw.GetValue();
        break;
    }
    default:
        break;
    }
}

void pylonUnit::setGrabbing(bool flag)
{    
    while(eventHandler_->getBusyFlag())
    {
        usleep(100);
    }

    try
    {
        if(flag)
        {
            if(pylon_->IsGrabbing()) return;
            pylon_->StartGrabbing(Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByInstantCamera);
        }
        else
        {
            if(!pylon_->IsGrabbing()) return;
            pylon_->StopGrabbing();
        }
    }
    catch(const Pylon::GenericException &e)
    {
        std::cerr << "Exception at: " << __FILE__ << " | " << __LINE__ << endl;
        std::cerr << e.what() << '\n';
    }
}

void pylonUnit::setStoreDir(const std::string &path)
{
    eventHandler_->setStoreDir(path);
}

void pylonUnit::setBusyFlag(bool flag)
{
    eventHandler_->setBusyFlag(flag);
}

bool pylonUnit::getBusyFlag()
{
    return eventHandler_->getBusyFlag();
}

string pylonUnit::getDeviceSerialNumber()
{
    return pylon_->GetDeviceInfo().GetSerialNumber().c_str();
}

string pylonUnit::getDeviceClass()
{
    return pylon_->GetDeviceInfo().GetDeviceClass().c_str();
}

string pylonUnit::getSavePath() const
{
    return eventHandler_->getStorePath();
}

void pylonUnit::softwareTrigger()
{
    if(pylon_->WaitForFrameTriggerReady(5000, Pylon::TimeoutHandling_ThrowException)) //timeout: 5s
        pylon_->ExecuteSoftwareTrigger();
}

// include <dirent.h>
int pylonUnit::checkCurrentFileCnt()
{
    string store_dir = eventHandler_->getStoreDir();

    DIR *dp = opendir(store_dir.c_str());
    dirent *ep;

    int cnt = 0;

    if (dp != NULL)
    {
        while ((ep = readdir(dp)))
        {
            if (strcmp(ep->d_name, ".") == 0 || strcmp(ep->d_name, "..") == 0)
            {
                continue;
            }

            cnt++;
        }
        closedir(dp);
        return cnt;
    }
    else
    {
        return 0;
    }
}

void CCustomConfiguration::OnGrabStarted(Pylon::CBaslerUniversalInstantCamera &camera)
{
    ROS_INFO_STREAM("Start Grabbing... ");
    camera.WaitForFrameTriggerReady(100, Pylon::TimeoutHandling_ThrowException); //1000
}

void CCustomConfiguration::OnGrabStopped(Pylon::CBaslerUniversalInstantCamera &camera)
{
    ROS_INFO_STREAM("Stop Grabbing... ");
    camera.StopGrabbing();
}

void CCustomConfiguration::OnOpened(Pylon::CBaslerUniversalInstantCamera &camera)
{
    camera.StopGrabbing();

    ROS_INFO_STREAM("Pylon Open!");

    switch (type_) {
    case DeviceType::USB:
    {
        //cout << "Image Format Control..." << endl;
        //camera.WidthMax();
        //camera.HeightMax();
        //camera.OffsetX.SetValue(0);
        //camera.OffsetY.SetValue(0);
        camera.ReverseX.SetValue(false);
        camera.ReverseY.SetValue(false);
        camera.PixelFormat.SetValue(PixelFormat_YCbCr422_8);

        //cout << "Analog Control..." << endl;
        camera.GainAuto.SetValue(GainAuto_Continuous);

        //cout << "Image Quality Control..." << endl;
        camera.DemosaicingMode.SetValue(DemosaicingMode_Simple);
        //camera.DemosaicingMode.SetValue(DemosaicingMode_BaslerPGI);
        //camera.NoiseReduction.SetValue(2.00000);
        //camera.SharpnessEnhancement.SetValue(1.0);

        camera.LightSourcePreset.SetValue(LightSourcePreset_Daylight5000K);
        camera.BalanceWhiteAuto.SetValue(BalanceWhiteAuto_Continuous);
        camera.ExposureAuto.SetValue(ExposureAuto_Continuous);

        //cout << "Auto Function Control..." << endl;
        camera.AutoTargetBrightness.SetValue(0.2);
        camera.AutoFunctionProfile.SetValue(AutoFunctionProfile_MinimizeGain);
        camera.AutoGainLowerLimit.SetValue(0.0);
        camera.AutoGainUpperLimit.SetValue(25.0);
        camera.AutoExposureTimeLowerLimit.SetValue(50.0);
        camera.AutoExposureTimeUpperLimit.SetValue(50000.0);

        //cout << "Setting Software Trigger...\n";
#ifdef SOFTWARE_TRIGGER
        camera.TriggerSelector.SetValue(TriggerSelector_FrameStart);
        camera.TriggerMode.SetValue(TriggerMode_On);
        camera.TriggerSource.SetValue(TriggerSource_Software);
#else
        //cout << "Digital I/O Control..." << endl;
        camera.LineSelector.SetValue(LineSelector_Line1);
        camera.LineMode.SetValue(LineMode_Input);

        camera.LineSelector.SetValue(LineSelector_Line4);
        camera.LineMode.SetValue(LineMode_Output);
        camera.LineSource.SetValue(LineSource_ExposureActive);
        camera.LineInverter.SetValue(true);
        camera.LineMinimumOutputPulseWidth.SetValue(1.000);

        //cout << "Setting Hardware Trigger..." << endl;
        camera.TriggerSelector.SetValue(TriggerSelector_FrameStart);
        camera.TriggerMode.SetValue(TriggerMode_On);
        camera.TriggerSource.SetValue(TriggerSource_Line1);
        camera.TriggerActivation.SetValue(TriggerActivation_RisingEdge);
#endif

        camera.AcquisitionMode.SetValue(AcquisitionMode_Continuous);
        break;
    }

    case DeviceType::GIGE:
    {
        //cout << "Image Format Control..." << endl;
        camera.ReverseX.SetValue(false);
        camera.ReverseY.SetValue(false);
        camera.PixelFormat.SetValue(PixelFormat_BayerRG8); //PixelFormat_BayerRG12

        //cout << "Set Analog Controls...\n";
        camera.GainAuto.SetValue(GainAuto_Continuous);

        //cout << "Set Acquisition Controls...\n";
        camera.ExposureAuto.SetValue(ExposureAuto_Continuous);

        //cout << "Set Image Quality Control...\n";
        camera.LightSourceSelector.SetValue(LightSourceSelector_Daylight);
        camera.BalanceWhiteAuto.SetValue(BalanceWhiteAuto_Continuous);

        //cout << "Set Image Quality Control...\n";
        camera.AutoTargetValue.SetValue(50); //800
        camera.AutoGainRawLowerLimit.SetValue(0.0);
        camera.AutoGainRawUpperLimit.SetValue(240.0);
        camera.AutoExposureTimeLowerLimitRaw.SetValue(80.0);
        camera.AutoExposureTimeUpperLimitRaw.SetValue(100000.0);
        camera.AutoFunctionProfile.SetValue(AutoFunctionProfile_GainMinimum);

        //cout << "Set Transport Layer...\n";
        camera.GevStreamChannelSelector.SetValue(GevStreamChannelSelector_StreamChannel0);
        camera.GevSCPSPacketSize.SetValue(8192);

        //cout << "Setting Software Trigger...\n";
#ifdef SOFTWARE_TRIGGER
        camera.TriggerSelector.SetValue(TriggerSelector_FrameStart);
        camera.TriggerMode.SetValue(TriggerMode_On);
        camera.TriggerSource.SetValue(TriggerSource_Software);
#else
        //cout << "Digital I/O Control..." << endl;
        camera.LineSelector.SetValue(LineSelector_Line1);
        camera.LineMode.SetValue(LineMode_Input);

        camera.LineSelector.SetValue(LineSelector_Line3);
        camera.LineMode.SetValue(LineMode_Output);
        camera.LineInverter.SetValue(true);

        //cout << "Setting Hardware Trigger..." << endl;
        camera.TriggerSelector.SetValue(TriggerSelector_FrameStart);
        camera.TriggerMode.SetValue(TriggerMode_On);
        camera.TriggerSource.SetValue(TriggerSource_Line1);
        camera.TriggerActivation.SetValue(TriggerActivation_RisingEdge);
#endif

        camera.AcquisitionMode.SetValue(AcquisitionMode_Continuous);
        break;
    }

    default:
        break;
    }
}

void CCustomConfiguration::OnClosed(Pylon::CBaslerUniversalInstantCamera &camera)
{
    ROS_INFO_STREAM("Pylon Close");
}

void CCustomConfiguration::setType(DeviceType type)
{
    type_ = type;
}

DeviceType CCustomConfiguration::getType()
{
    return type_;
}

CCustomEvtHandler::CCustomEvtHandler()
{
    image_transport::ImageTransport it(nh_);
    cameraImagePub_ = it.advertise("basler_ros_driver/camera1/image_raw", 1);
    cameraInfoPub_ = nh_.advertise<sensor_msgs::CameraInfo>("basler_ros_driver/camera1/camera_info", 1);

    camBusy_ = false;
}

void CCustomEvtHandler::OnImageGrabbed(Pylon::CBaslerUniversalInstantCamera &camera, const Pylon::CBaslerUniversalGrabResultPtr &grabResult)
{
    using namespace Pylon;

    //cout << "Grabbed at: " << camera.GetDeviceInfo().GetSerialNumber() << endl;

    if (grabResult->GrabSucceeded())
    {
        // Access the image data.
        //cout << "SizeX: " << grabResult->GetWidth() << endl;
        //cout << "SizeY: " << grabResult->GetHeight() << endl;
        //auto x = grabResult->GetGrabResultDataImpl();
        CImageFormatConverter formatConverter;
        formatConverter.OutputPixelFormat = PixelType_BGR8packed;
        CPylonImage pylonImage;
        formatConverter.Convert(pylonImage, grabResult);

        // Save image by SDK
        /*
        char buf[100];
        std::string image_path;

        sprintf(buf, "/im[%06d].png", (int)grabResult->GetImageNumber());
        image_path = "/home/stryx/SETUP_TOOL/Pylon_Test/output";
        image_path += buf;
        pylonImage.Save(ImageFileFormat_Png, image_path.c_str());
        */

        // Create an OpenCV image out of pylon image
        cv::Mat cv_img;
        cv_img = cv::Mat(grabResult->GetHeight(), grabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer()); //me

        string image_path = storeDir_;

        char buf[100];
        sprintf(buf, "/im[%06d].bmp", imageCnt_);
        image_path += buf;
        storePath_ = image_path;

        if(!storeDir_.empty())
        {
            cv::imwrite(storePath_, cv_img);
            imageCnt_++;
        }

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        // cv::Mat resize_img;
        // cv::resize(cv_img, resize_img, cv::Size(610, 512));
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", cv_img).toImageMsg();
        cameraImagePub_.publish(img_msg);
        cam_info_.header = header;
        cameraInfoPub_.publish(cam_info_);

        camBusy_ = false;
    }
    else
    {
        cerr << "[Error] " << grabResult->GetErrorCode() << " " << grabResult->GetErrorDescription() << endl;
    }
}

void CCustomEvtHandler::initCnt()
{
    imageCnt_ = 0;
}

void CCustomEvtHandler::setID(int id)
{
    id_ = id;
}

void CCustomEvtHandler::setCameraData(sensor_msgs::CameraInfo cam_info)
{
    cam_info_ = cam_info;
}

void CCustomEvtHandler::setStoreDir(const string path)
{
    storeDir_ = path;
}

void CCustomEvtHandler::setBusyFlag(bool flag)
{
    camBusy_ = flag;
}

const string CCustomEvtHandler::getStoreDir() const
{
    return storeDir_;
}

const string CCustomEvtHandler::getStorePath() const
{
    return storePath_;
}

const bool CCustomEvtHandler::getBusyFlag() const
{
    return camBusy_;
}
