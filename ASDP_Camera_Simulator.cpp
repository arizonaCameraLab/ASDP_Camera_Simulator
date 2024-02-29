#include <fstream>
#include <map>
#include <string>
#include <sstream>
#include <filesystem>
#include <vector>
#include <chrono>
#include <BspInclude.hpp>

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

//==============================================================================
// Start-up code that reads the configuration file and sets up things that are
// needed for the rest of the code.

/// @brief The path to the configuration file. Defined in the CMakeLists file.
std::filesystem::path dirPath = CONFIG_FILE_PATH;
std::filesystem::path baseName = "ASDP_Camera_Simulator.conf";
/// @brief The configuration file name.
static const std::string ConfigFileName = (dirPath / baseName).string();

/// @brief Read the configuration map from the configuration file.
static std::map<std::string, std::map<std::string, std::string> > ConfigMap = ParseConfFile(ConfigFileName);

/// @brief Loads the configuration file and sets up the state.
class State {
public:
  State(const std::map<std::string, std::map<std::string, std::string>>& configMap);

  uint32_t NumInternalTriggers = 0;
  uint32_t NumExternalTriggers = 0;
  uint32_t NumTemperatureSensorsPerCamera = 0;
  uint32_t NumSystemTemperatureSensors = 0;
  std::vector<Bsp::Pose::pose_report_s> PoseDeviceConfigurations;
  uint32_t NumCameras = 0;
  std::vector<TemperatureSensor> TemperatureSensors;
  std::vector<Pose> Posers;
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
  }

  uint32_t numPosers = 0;
  if (configMap.find("Pose") != configMap.end()) {
    std::map<std::string, std::string> pose = configMap.at("Pose");
    try {
      numPosers = std::stoi(pose.at("NumPoses"));
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
    NumCameras = 0;
    try {
      NumCameras = std::stoi(camera.at("NumCameras"));
    } catch (...) {}
  }

  // Create objects for the temperature sensors.
  for (size_t i = 1; i <= NumTemperatureSensorsPerCamera; i++) {
    for (size_t c = 1; c <= NumCameras; c++) {
      TemperatureSensors.push_back(TemperatureSensor(c, i, 25.0, 0.25));
    }
  }
  for (size_t i = 1; i <= NumSystemTemperatureSensors; i++) {
    TemperatureSensors.push_back(TemperatureSensor(0, i, 25.0, 0.25));
  }

  // Create objects for the Posers
  for (size_t i = 1; i <= numPosers; i++) {
    Posers.push_back(Pose(i, 0.25));
  }

  /// @todo Implement below here.
}

/// @brief The state that is shared between the different functions.
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
      /// @todo
      return false;
    }

    bool SoftwareTrigger(uint8_t ID, uint32_t Seconds, uint32_t Microseconds) {
      /// @todo
      return false;
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
      /// @todo
      return false;
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
      /// @todo
      return false;
    }

    bool DisableStream(uint32_t CameraIndex, endpoint_s Endpoint) {
      /// @todo
      return false;
    }

  }

}
