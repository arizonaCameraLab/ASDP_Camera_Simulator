#include <fstream>
#include <map>
#include <string>
#include <sstream>
#include <filesystem>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <list>
//#include <iostream>   ///< @todo Remove when done debugging
#include <BspInclude.hpp>
#include <ASDP_Core_API.h>
#include <ASDP_SpinFreeAccurateTimer.h>
#include <Trigger.h>
#include <ImageSource.h>

//==============================================================================
// Helper functions.


/// @brief Trim whitespace from the beginning and end of a string.
/// @param str The string to trim.
/// @return The trimmed string.
static std::string TrimWhiteSpace(const std::string& str) {
  std::string s = str;
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char uc) {return !std::isspace(uc); }));
  s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char uc) {return !std::isspace(uc); }).base(), s.end());
  return s;
}

/// @brief Parse a configuration file.
/// @param filename The name of the file to parse.
/// @return A map of maps of strings. The first map is from section name to a map of key-value pairs.
/// Returns an empty map if the file cannot be opened.
static std::map<std::string, std::map<std::string, std::string>> ParseConfFile(const std::string& filename) {
  std::map<std::string, std::map<std::string, std::string>> confMap;
  std::ifstream file(filename);
  std::string line;
  std::string section;

  while (std::getline(file, line)) {
    if (line[0] == '[') {
      section = TrimWhiteSpace(line.substr(1, line.find(']') - 1));
    }
    else {
      std::istringstream is_line(line);
      std::string key;
      if (std::getline(is_line, key, '=')) {
        std::string value;
        if (std::getline(is_line, value)) {
          confMap[section][TrimWhiteSpace(key)] = TrimWhiteSpace(value);
        }
      }
    }
  }

  return confMap;
}

//==============================================================================
// Helper classes that implement behaviors.

/// @brief Implements the TemperatureSensor behavior
class TemperatureSensor {
public:
  TemperatureSensor(uint16_t camera, uint16_t ID, float temperature, float period)
    : camera(camera), ID(ID), temperature(temperature), period(period) {}

  bool ReportTemperature(uint16_t &camera, uint16_t &ID, float& temperature,
                         uint32_t &seconds, uint32_t &microseconds) {
    /// @todo Implement this by pushing a trigger time into the future so we don't slip each time.
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapsed = now - lastReportTime;
    if (elapsed.count() < period) {
      return false;
    }
    lastReportTime = now;
    camera = this->camera;
    ID = this->ID;
    temperature = this->temperature;
    seconds = (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() / 1000000;
    microseconds = (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000;
    return true;
  }

protected:
  uint16_t camera;
  uint16_t ID;
  float temperature;
  float period;
  std::chrono::time_point<std::chrono::steady_clock> lastReportTime;
};

/// @brief Implements the Pose reporter behavior
class Pose {
public:
  Pose(uint16_t ID, float period) : ID(ID), period(period) {}

  bool ReportPose(Bsp::Pose::pose_s& report) {
    /// @todo Implement this by pushing a trigger time into the future so we don't slip each time.
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapsed = now - lastReportTime;
    if (elapsed.count() < period) {
      return false;
    }
    lastReportTime = now;
    report.Seconds = (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() / 1000000;
    report.Microseconds = (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000;
    report.Longitude = 0;
    report.Latitude = 0;
    report.Altitude = 0;
    for (size_t i = 0; i < 3; i++) {
      report.Rotation[i] = 0.0;
      report.Velocity[i] = 0.0;
      report.RotationVelocity[i] = 0.0;
    }
    report.DeviceID = ID;
    return true;
  }

protected:
  uint16_t ID;
  float period;
  std::chrono::time_point<std::chrono::steady_clock> lastReportTime;
};

/// @brief Implements the Camera behavior
class Camera {
public:

  /// @brief Construct a camera object.
  Camera() : cv(std::make_shared<std::condition_variable>()) {
    // Construct the images that will be sent from the camera.
    /// @todo Read the number of images from the configuration file.
    size_t frames = 60;
    IS::MovingBarsSource source(1280, 1024, frames, 20, 48, 5);
    for (size_t i = 0; i < frames; i++) {
      images.push_back(source.getNextImage());
    }
  }

  /// @brief Destroy a camera object.
  ~Camera() {}

  /// @brief Set up the camera with a trigger.
  /// @param trigger The trigger to associate with the camera.
  /// @return true if the camera was set up successfully, false otherwise.
  bool Setup(std::shared_ptr<Trigger> trigger) {
    if (this->trigger) {
      if (!this->trigger->UnregisterClient(cv)) {
        return false;
      }
    }
    this->trigger = trigger;
    trigger->RegisterClient(cv);
    return true;
  }

  /// @brief Insert a stream into the camera.
  /// @param Endpoint The endpoint to send the stream to.
  /// @param Roi The region of interest to send.
  /// @param SkipFrames How many frames to skip between sends, 0 for none.
  /// @param StartSeconds The time to start sending frames, ignore any frames before this.
  /// @param StartMicroseconds The time to start sending frames, ignore any frames before this.
  /// @return true if the stream was inserted, false otherwise.
  bool InsertStream(Bsp::UdpInserter::endpoint_s Endpoint, Bsp::UdpInserter::roi_s Roi, uint32_t SkipFrames,
        uint32_t StartSeconds, uint32_t StartMicroseconds)
  {
    Stream stream;
    stream.Endpoint = Endpoint;
    stream.Roi = Roi;
    stream.SkipFrames = SkipFrames;
    stream.StartSeconds = StartSeconds;
    stream.StartMicroseconds = StartMicroseconds;
    asdp::StreamEndpoint streamEndpoint(Endpoint.IP, Endpoint.Port);
    stream.sender = std::make_shared<asdp::SenderUDP>(streamEndpoint);
    if (asdp::OKAY != stream.sender->GetConstructorStatus()) {
      return false;
    }
    streams.push_back(stream);
    return true;
  }

  /// @brief Remove a stream from the camera.
  /// @param Endpoint The endpoint to remove the stream from.
  /// @return true if the stream was removed, false otherwise.
  bool RemoveStream(Bsp::UdpInserter::endpoint_s Endpoint)
  {
    for (auto it = streams.begin(); it != streams.end(); it++) {
      if (it->Endpoint == Endpoint) {
        streams.erase(it);
        return true;
      }
    }
    return false;
  }

protected:

  /// @brief The images that will be sent from the camera.
  std::vector< std::shared_ptr<IS::Image> > images;
  /// @brief The index of the next image to be sent.
  size_t nextImage = 0;

  /// @brief The internal trigger each camera is associated with (empty for none).
  std::shared_ptr<Trigger> trigger;

  /// @brief A condition variable to signal when the camera is triggered.
  std::shared_ptr<std::condition_variable> cv;

  struct Stream {
    Bsp::UdpInserter::endpoint_s Endpoint;
    Bsp::UdpInserter::roi_s Roi;
    uint32_t SkipFrames = 0;                  ///< How may frames to skip between sends, 0 for none.
    uint32_t StartSeconds = 0;                ///< The time to start sending frames. Ignore any frames before this
    uint32_t StartMicroseconds = 0;           ///< The time to start sending frames. Ignore any frames before this
    size_t curFrame = 0;  ///< The current frame number, starting with 0, used modulo SkipFrames to determine when to send a frame.
    std::shared_ptr<asdp::SenderUDP> sender;  ///< The sender being used to send the stream.
  };

  /// The streams that are currently enabled for this camera.
  std::list<Stream> streams;
};

//==============================================================================
// Start-up code that reads the configuration file and sets up things that are
// needed for the rest of the code.

/// @brief The path to the configuration file. Defined in the CMakeLists file.
std::filesystem::path dirPath = CONFIG_FILE_PATH;
std::filesystem::path baseName = "ASDP_Camera_Simulator.conf";
/// @brief The configuration file name.
static const std::string ConfigFileName = (dirPath / baseName).string();

/// @brief Read the configuration map from the configuration file.
/// @details This map is used to set up the state of the system and it is configured
/// when the library is loaded.
static std::map<std::string, std::map<std::string, std::string> > ConfigMap = ParseConfFile(ConfigFileName);

/// @brief Loads the configuration file and sets up the state.
class State {
public:
  /// @brief Construct a state and start any required threads to operate the system.
  State(const std::map<std::string, std::map<std::string, std::string>>& configMap);

  /// @brief Stop any threads and destruct the state cleanly.
  ~State();

  uint32_t NumInternalTriggers = 0;
  uint32_t NumExternalTriggers = 0;
  uint32_t NumTemperatureSensorsPerCamera = 0;
  uint32_t NumSystemTemperatureSensors = 0;
  std::vector<Bsp::Pose::pose_report_s> PoseDeviceConfigurations;
  uint32_t NumCameras = 0;
  float MinCameraPeriodSeconds = 1.0f/60;
  float MaxCameraPeriodSeconds = 1.0f/50;
  std::vector<TemperatureSensor> TemperatureSensors;
  float TemperaturePeriod = 0.25;
  std::vector<Pose> Posers;
  float PosePeriod = 0.25;

  /// @brief the timer object to avoid spin-waiting inside every thread.
  asdp::SpinFreeAccurateTimer timer;

  /// @brief The triggers for the system.
  std::vector< std::shared_ptr<Trigger> > triggers;
  /// @brief the external trigger each internal trigger is tied to (0 for none).
  std::vector<uint8_t> externalTriggers;

  /// @brief the cameras in the system
  std::vector< std::shared_ptr<Camera> > cameras;
};

State::State(const std::map<std::string, std::map<std::string, std::string>>& configMap)
{
  if (configMap.find("Timing") != configMap.end()) {
    std::map<std::string, std::string> timing = configMap.at("Timing");
    try {
      NumInternalTriggers = std::stoi(timing.at("NumInternalTriggers"));
    }
    catch (...) {}
    try {
      NumExternalTriggers = std::stoi(timing.at("NumExternalTriggers"));
    }
    catch (...) {}
  }

  if (configMap.find("Temperature") != configMap.end()) {
    std::map<std::string, std::string> temperature = configMap.at("Temperature");
    try {
      NumTemperatureSensorsPerCamera = std::stoi(temperature.at("NumSensorsPerCamera"));
    } catch (...) {};
    try {
      NumSystemTemperatureSensors = std::stoi(temperature.at("NumSystemSensors"));
    }
    catch (...) {};
    try {
      TemperaturePeriod = std::stof(temperature.at("ReportingPeriodSeconds"));
    }
    catch (...) {};
  }

  uint32_t numPosers = 0;
  if (configMap.find("Pose") != configMap.end()) {
    std::map<std::string, std::string> pose = configMap.at("Pose");
    try {
      numPosers = std::stoi(pose.at("NumPoses"));
    } catch (...) {}
    try {
      PosePeriod = std::stof(pose.at("ReportingPeriodSeconds"));
    } catch (...) {}
    for (uint16_t i = 0; i < numPosers; i++) {
      Bsp::Pose::pose_report_s report;
      std::string key = "Pose" + std::to_string(i) + "Orientation";
      if (pose.find(key) != pose.end()) {
        report.ReturnsOrientation = (0 != std::stoi(pose.at(key)));
      }
      key = "Pose" + std::to_string(i) + "Position";
      if (pose.find(key) != pose.end()) {
        report.ReturnsPosition = (0 != std::stoi(pose.at(key)));
      }
      PoseDeviceConfigurations.push_back(report);
    }
  }

  if (configMap.find("Camera") != configMap.end()) {
    std::map<std::string, std::string> camera = configMap.at("Camera");
    try {
      NumCameras = std::stoi(camera.at("NumCameras"));
    } catch (...) {}
    try {
      MinCameraPeriodSeconds = std::stof(camera.at("MaxFrameRateSeconds"));
    } catch (...) {}
    try {
      MaxCameraPeriodSeconds = std::stof(camera.at("MinFrameRateSeconds"));
    } catch (...) {}
  }

  // Create objects for the temperature sensors.
  for (size_t i = 1; i <= NumTemperatureSensorsPerCamera; i++) {
    for (size_t c = 1; c <= NumCameras; c++) {
      TemperatureSensors.push_back(TemperatureSensor(c, i, 25.0, TemperaturePeriod));
    }
  }
  for (size_t i = 1; i <= NumSystemTemperatureSensors; i++) {
    TemperatureSensors.push_back(TemperatureSensor(0, i, 25.0, TemperaturePeriod));
  }

  // Create objects for the Posers
  for (size_t i = 1; i <= numPosers; i++) {
    Posers.push_back(Pose(i, PosePeriod));
  }

  // Create the internal triggers which will by default be disabled and not tied to any external trigger.
  for (size_t i = 0; i < NumInternalTriggers; i++) {
    triggers.push_back( std::shared_ptr<Trigger>(new Trigger(timer)));
    externalTriggers.push_back(0);
  }

  // Cameras are initially not triggered.
  for (size_t i = 0; i < NumCameras; i++) {
    cameras.push_back(std::shared_ptr<Camera>(new Camera));
  }

  /// @todo Implement below here.
}

State::~State()
{
  /// @todo Implement this.
}

/// @brief The state that is shared between the different functions.
/// @details The constructor for this object is called when the library is loaded
/// and the destructor is called when the library is unloaded.
static State state(ConfigMap);

//==============================================================================
// Code that implements the Bsp namespace.

/// Overall namespace for the BSP.
namespace Bsp {

  std::string GetVersion(void) {
    return "1.0.0";
  }

  /// Namespace for timing and trigger functions.
  namespace Timing {

    uint32_t GetNumberOfInternalTriggers(void) {
      return state.NumInternalTriggers;
    }

    uint32_t GetNumberOfExternalTriggers(void) {
      return state.NumExternalTriggers;
    }

    void GetCurrentSystemTime(uint32_t& Seconds, uint32_t& Microseconds) {
      auto now = std::chrono::system_clock::now();
      Seconds = (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() / 1000000;
      Microseconds = (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000;
    }

    bool ConfigureTrigger(uint16_t ID, uint8_t Mode, uint8_t ExternalID, float Period, float TrackingFactor) {
      if (ID >= state.triggers.size()) {
        return false;
      }
      state.triggers[ID]->Configure(Mode, Period, TrackingFactor);
      state.externalTriggers[ID] = ExternalID;
      return true;
    }

    bool SoftwareTrigger(uint8_t ExternalID, uint32_t Seconds, uint32_t Microseconds) {
      bool failed = false;

      // See if any of the internal triggers are tied to this external trigger.  If so, fire them.
      for (size_t i = 0; i < state.triggers.size(); i++) {
        if (state.externalTriggers[i] == ExternalID) {
          if (!state.triggers[i]->Fire(
            std::chrono::steady_clock::time_point(std::chrono::seconds(Seconds) + std::chrono::microseconds(Microseconds)))) {
            failed = true;
          }
        }
      }

      return !failed;
    }

  }

  /// Namespace for temperature functions.
  namespace Temperature {

    uint16_t GetNumberOfTemperatureSensorsPerCamera(void) {
      return state.NumTemperatureSensorsPerCamera;
    }

    uint16_t GetNumberOfSystemTemperatureSensors(void) {
      return state.NumSystemTemperatureSensors;
    }

    std::vector<temp_s> GetNewTemperatures(void) {
      std::vector<temp_s> temps;
      for (auto& sensor : state.TemperatureSensors) {
        uint16_t camera, ID;
        float temperature;
        uint32_t seconds, microseconds;
        if (sensor.ReportTemperature(camera, ID, temperature, seconds, microseconds)) {
          temp_s temp;
          temp.Seconds = 0;
          temp.Microseconds = 0;
          temp.Temperature = temperature * 1000;
          temp.CameraIndex = camera;
          temp.SensorIndex = ID;
          temps.push_back(temp);
        }
      }
      return temps;
    }

  }

  /// Namespace for pose functions.
  namespace Pose {

    std::vector<pose_report_s> GetPoseDeviceConfigurations(void) {
      return state.PoseDeviceConfigurations;
    }

    std::vector<pose_s> GetNewPoses(void) {
      std::vector<pose_s> poses;
      Bsp::Pose::pose_s report;
      for (auto& p : state.Posers) {
        if (p.ReportPose(report)) {
          poses.push_back(report);
        }
      }
      return poses;
    }

  }

  /// Namespace for camera functions.
  namespace Camera {

    uint16_t GetNumberOfCameras(void) {
      return state.NumCameras;
    }

    bool SetupCamera(uint16_t Index, uint32_t Trigger) {
      if (Index >= state.NumCameras) {
        return false;
      }
      if (Trigger >= state.triggers.size()) {
        return false;
      }

      return state.cameras[Index]->Setup(state.triggers[Trigger]);
    }

    std::map<std::string, std::string> GetStatus(uint16_t Index) {
      std::map<std::string, std::string> status;
      /// @todo
      return status;
    }

  }

  /// Namespace for UDP inserter functions.
  namespace UdpInserter {

    bool EnableStream(uint16_t CameraIndex, endpoint_s Endpoint, roi_s Roi, uint32_t SkipFrames,
      uint32_t StartSeconds, uint32_t StartMicroseconds) {
      if (CameraIndex >= state.cameras.size()) {
        return false;
      }
      state.cameras[CameraIndex]->InsertStream(Endpoint, Roi, SkipFrames, StartSeconds, StartMicroseconds);
      return true;
    }

    bool DisableStream(uint32_t CameraIndex, endpoint_s Endpoint) {
      if (CameraIndex >= state.cameras.size()) {
        return false;
      }
      return state.cameras[CameraIndex]->RemoveStream(Endpoint);
    }

  }

}
