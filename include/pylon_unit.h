#ifndef PYLONUNIT_H
#define PYLONUNIT_H

#include <ros/ros.h>

//SDK
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/PylonIncludes.h>
#include <pylon/_BaslerUniversalCameraParams.h>

#include <map>

//#undef SOFTWARE_TRIGGER
#define SOFTWARE_TRIGGER

enum class DeviceType
{
    USB = 1,
    GIGE
};

class CCustomConfiguration : public Pylon::CBaslerUniversalConfigurationEventHandler
{
    // CBaslerUniversalConfigurationEventHandler interface
public:
    void OnGrabStarted(Pylon::CBaslerUniversalInstantCamera &camera);
    void OnGrabStopped(Pylon::CBaslerUniversalInstantCamera &camera);
    void OnOpened(Pylon::CBaslerUniversalInstantCamera &camera);
    void OnClosed(Pylon::CBaslerUniversalInstantCamera &camera);

public:
    void setType(DeviceType type);
    DeviceType getType();

private:
    DeviceType type_;
};

class CCustomEvtHandler: public Pylon::CBaslerUniversalImageEventHandler
{
    // CBaslerUniversalImageEventHandler interface
public:
    CCustomEvtHandler();
    void OnImageGrabbed(Pylon::CBaslerUniversalInstantCamera &camera, const Pylon::CBaslerUniversalGrabResultPtr &grabResult);

public:
    void initCnt();
    void setStoreDir(const std::string path);
    void setBusyFlag(bool flag);

    const std::string getStoreDir() const;
    const std::string getStorePath() const;
    const bool getBusyFlag() const;

private:
    int imageCnt_;
    bool camBusy_;

    std::string storeDir_;
    std::string storePath_;
};


class pylonUnit
{
public:
    pylonUnit(Pylon::CBaslerUniversalInstantCamera *pylon);
    virtual ~pylonUnit();

    void initCnt();
    void setReverseOption(bool x, bool y);
    void setImageQuality(double brightness, int funcProfile, double gainLower, double gainUpper, double timeLower, double timeUpper);
    void setGrabbing(bool flag);
    void setStoreDir(const std::string &path);
    void setBusyFlag(bool flag);

    bool getBusyFlag();
    std::string getDeviceSerialNumber();
    std::string getDeviceClass();
    void getImageQuality(double &brightness, int &funcProfile, double &gainLower, double &gainUpper, double &timeLower, double &timeUpper);
    //const std::string &getStoreDir() const;
    std::string getSavePath() const;

    void softwareTrigger();
    int checkCurrentFileCnt();

private:
    Pylon::CBaslerUniversalInstantCamera *pylon_;

    CCustomConfiguration *configuration_;
    CCustomEvtHandler    *eventHandler_;

    std::map<std::string, int> classToEnum_;
};

#endif // PYLONUNIT_H
