#include <BspInclude.hpp>

/// Overall namespace for the BSP.
namespace Bsp {

  std::string GetVersion(void) {
    return "1.0.0";
  }

  /// @todo Implement below here.

  /// Namespace for timing and trigger functions.
  namespace Timing {

    uint32_t GetNumberOfInternalTriggers(void) {
      return 0;
    }

    uint32_t GetNumberOfExternalTriggers(void) {
      return 0;
    }

    void GetCurrentSystemTime(uint32_t& Seconds, uint32_t& Microseconds) {
      Seconds = 0;
      Microseconds = 0;
    }

    bool ConfigureTrigger(uint16_t ID, uint8_t Mode, uint8_t ExternalID, float Period, float TrackingFactor) {
      return false;
    }

    bool SoftwareTrigger(uint8_t ID, uint32_t Seconds, uint32_t Microseconds) {
      return false;
    }

  }

  /// Namespace for temperature functions.
  namespace Temperature {

    uint16_t GetNumberOfTemperatureSensorsPerCamera(void) {
      return 0;
    }

    uint16_t GetNumberOfSystemTemperatureSensors(void) {
      return 0;
    }

    std::vector<temp_s> GetNewTemperatures(void) {
      std::vector<temp_s> temps;
      return temps;
    }

  }

  /// Namespace for pose functions.
  namespace Pose {

    std::vector<pose_report_s> GetPoseDeviceConfigurations(void) {
      std::vector<pose_report_s> poseReports;
      return poseReports;
    }

    std::vector<pose_s> GetNewPoses(void) {
      std::vector<pose_s> poses;
      return poses;
    }

  }

  /// Namespace for camera functions.
  namespace Camera {

    uint16_t GetNumberOfCameras(void) {
      return 0;
    }

    bool SetupCamera(uint16_t Index, uint32_t Trigger) {
      return false;
    }

    std::map<std::string, std::string> GetStatus(uint16_t Index) {
      std::map<std::string, std::string> status;
      return status;
    }

  }

}
