/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <fstream>
#include <map>
#include <string>
#include <sstream>
#include <filesystem>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <list>
#include <deque>
#include <algorithm>
#include <BspInclude.hpp>
#include <ASDP_Core_API.h>
#include <ASDP_SpinFreeQueue.hpp>
#include <ASDP_SpinFreeAccurateTimer.hpp>
#include <Trigger.h>
#include <ASDP_ImageSource.h>

using namespace asdp;

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

static std::string ToStringWithPrecision(double value, int precision = 20) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
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
    seconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() / 1000000);
    microseconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000);
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
    report.Seconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() / 1000000);
    report.Microseconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000);
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
  /// @param id The ID of the camera.
  Camera(
    uint32_t cameraID = 0, uint32_t cameraType = 0, uint32_t sensorWidth = 1280, uint32_t sensorHeight = 1024,
    float exposure = 0.001f,  float gain = 1.0f, double minPeriodSeconds = 1/60.0, double maxPeriodSeconds = 1/50.0);

  /// @brief Destroy a camera object.
  ~Camera();

  /// @brief Set up the camera with a trigger.
  /// @param trigger The trigger to associate with the camera. Nullptr to disable triggering.
  /// @return true if the camera was set up successfully, false otherwise.
  bool Setup(std::shared_ptr<Trigger> trigger);

  /// @brief Insert a stream into the camera.
  /// @param Endpoint The endpoint to send the stream to.
  /// @param Roi The region of interest to send.
  /// @param SkipFrames How many frames to skip between sends, 0 for none.
  /// @param StartSeconds The time to start sending frames, ignore any frames before this.
  /// @param StartMicroseconds The time to start sending frames, ignore any frames before this.
  /// @return true if the stream was inserted, false otherwise.
  bool InsertStream(Bsp::UdpInserter::endpoint_s Endpoint, Bsp::UdpInserter::roi_s Roi, uint32_t SkipFrames,
    uint32_t StartSeconds, uint32_t StartMicroseconds);

  /// @brief Remove a stream from the camera.
  /// @param Endpoint The endpoint to remove the stream from.
  /// @return true if the stream was removed, false otherwise.
  bool RemoveStream(Bsp::UdpInserter::endpoint_s Endpoint);

  uint32_t GetCameraID() const { return m_cameraID; }
  uint32_t GetCameraType() const { return m_cameraType; }
  uint32_t GetSensorWidth() const { return m_sensorWidth; }
  uint32_t GetSensorHeight() const { return m_sensorHeight; }
  float GetExposure() const { return m_exposure; }
  float GetGain() const { return m_gain; }
  double GetMinPeriodSeconds() const { return m_minPeriodSeconds; }
  double GetMaxPeriodSeconds() const { return m_maxPeriodSeconds; }

protected:

  //========================
  // Camera configuration.
  uint32_t m_cameraID;
  uint32_t m_cameraType;
  uint32_t m_sensorWidth;
  uint32_t m_sensorHeight;
  float m_exposure;
  float m_gain;
  double m_minPeriodSeconds;
  double m_maxPeriodSeconds;

  /// @brief The images that will be sent from the camera stored in a loopable source.
  std::shared_ptr<asdp::ImageSource::ImageSource> m_images;

  /// @brief The internal trigger each camera is associated with (empty for none).
  std::shared_ptr<Trigger> m_trigger;

  /// @brief Mutex to use when modifying internal state.
  std::mutex m_mut;

  //========================
  // Trigger thread that receives per-frame triggers and constructs lists of slices to send.

  /// @brief Data structure holding information about a stream
  struct Stream {
    Bsp::UdpInserter::endpoint_s Endpoint = {}; ///< The endpoint to send the stream to.
    Bsp::UdpInserter::roi_s Roi = {};           ///< The region of interest to send.
    uint32_t SkipFrames = 0;                    ///< How may frames to skip between sends, 0 for none.
    uint32_t StartSeconds = 0;                  ///< The time to start sending frames. Ignore any frames before this
    uint32_t StartMicroseconds = 0;             ///< The time to start sending frames. Ignore any frames before this
    /// The current frame number, starting with 0, used modulo SkipFrames to determine when to send a frame.
    /// Must be a pointer so that when we copy the stream we can still update the value it points to.
    std::shared_ptr<size_t> curFrame = std::make_shared<size_t>(0);  
    std::shared_ptr<StreamWriter> writer;       ///< The StreamWriter being used to send the stream.
  };

  /// The streams that are currently enabled for this camera.
  std::list<Stream> m_streams;

  /// @brief The thread that pushes slices for each image.
  std::thread m_triggerThread;

  /// @brief The function that pushes slices for each image.
  void TriggerThread();

  /// @brief Atomic flag to signal the trigger thread to stop.
  std::atomic_bool m_triggerThreadStop = false;

  /// @brief A queue to push on when the camera is triggered.
  std::shared_ptr< asdp::SpinFreeQueue< std::chrono::steady_clock::time_point> > m_triggerQ;
};

Camera::Camera(uint32_t cameraID, uint32_t cameraType, uint32_t sensorWidth, uint32_t sensorHeight,
  float exposure, float gain, double minPeriodSeconds, double maxPeriodSeconds)
  : m_cameraID(cameraID)
  , m_cameraType(cameraType)
  , m_sensorWidth(sensorWidth)
  , m_sensorHeight(sensorHeight)
  , m_exposure(exposure)
  , m_gain(gain)
  , m_minPeriodSeconds(minPeriodSeconds)
  , m_maxPeriodSeconds(maxPeriodSeconds)
  , m_triggerQ(std::make_shared< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> >())
{
}

Camera::~Camera()
{
  // Stop and join our threads
  m_triggerThreadStop = true;
  if (m_triggerThread.joinable()) {
    m_triggerThread.join();
  }

  // Clear our streams and slice queue, then reset our trigger and timer.
  m_streams.clear();
  m_triggerQ.reset();
  m_trigger.reset();
}

bool Camera::Setup(std::shared_ptr<Trigger> trigger)
{
  {
    std::lock_guard<std::mutex> lock(m_mut);

    // Construct the images that will be sent from the camera.
    // We must do this here because we can't run parallel threads in the constructor that
    // is called during the dynamic library load in the State constructor.  At least on
    // Windows, the threads do not start then.
    /// @todo Read the number of images from the configuration file.
    /// @todo Enable specifying a directory to read the images from as files.
    size_t frames = 64;
    m_images = std::make_shared<asdp::ImageSource::MovingBarsSource>(1280, 1024, frames, 20, 48, 2);

    // Intentionally de-synchronize the cameras so that we won't be surprised when they don't start
    // at the same time.  There is nothing requiring them to remain synchronized but they often were,
    // which was confusing when a Storage Module connects and they are not.
    for (size_t i = 0; i < m_cameraID * 3; i++) {
      // Toss images
      m_images->getNextImage();
    }

    // If we're already associated with a trigger, unregister us from it.
    if (m_trigger) {
      if (!m_trigger->UnregisterClient(m_triggerQ)) {
        return false;
      }
    }
  }

  // Start the threads that will send the images to the streams if we're configured
  // to use a trigger. Stop it if we're not using a trigger and it is already running.
  // We can't hold the mutex during this code or the subthread will never exit.
  if (trigger) {
    m_triggerThreadStop = false;
    m_triggerThread = std::thread([this] { TriggerThread(); });
  } else {
    m_triggerThreadStop = true;
    if (m_triggerThread.joinable()) {
      m_triggerThread.join();
    }
  }

  // Clear the trigger queue so it will be ready when re-started.
  {
    std::lock_guard<std::mutex> lock(m_mut);

    std::chrono::steady_clock::time_point timePt;
    while (m_triggerQ->size()) {
      m_triggerQ->dequeue(timePt, std::chrono::milliseconds(10));
    }

    // Register us with the trigger, if there is one.
    m_trigger = trigger;
    if (m_trigger) {
      m_trigger->RegisterClient(m_triggerQ);
    }
  }
  return true;
}

bool Camera::InsertStream(Bsp::UdpInserter::endpoint_s Endpoint, Bsp::UdpInserter::roi_s Roi, uint32_t SkipFrames,
  uint32_t StartSeconds, uint32_t StartMicroseconds)
{
  std::lock_guard<std::mutex> lock(m_mut);

  Stream stream;
  stream.Endpoint = Endpoint;
  stream.Roi = Roi;
  stream.SkipFrames = SkipFrames;
  stream.StartSeconds = StartSeconds;
  stream.StartMicroseconds = StartMicroseconds;
  StreamEndpoint streamEndpoint(Endpoint.IP, Endpoint.Port);
  std::shared_ptr<SenderUDP> sender = std::make_shared<SenderUDP>(StreamEndpoint(Endpoint.IP, Endpoint.Port));
  stream.writer = std::make_shared<StreamWriter>(sender);
  if (asdp::OKAY != stream.writer->GetConstructorStatus()) {
    return false;
  }
  m_streams.push_back(stream);
  return true;
}

bool Camera::RemoveStream(Bsp::UdpInserter::endpoint_s Endpoint)
{
  std::lock_guard<std::mutex> lock(m_mut);

  for (auto it = m_streams.begin(); it != m_streams.end(); it++) {
    if (it->Endpoint == Endpoint) {
      m_streams.erase(it);
      return true;
    }
  }
  return false;
}

void Camera::TriggerThread()
{
  while (!m_triggerThreadStop) {
    // See if we have been triggered.  Time out if not so we can check the stop flag.
    std::chrono::steady_clock::time_point triggerTime;
    if (!m_triggerQ->dequeue(triggerTime, std::chrono::milliseconds(50))) {
      continue;
    }

    uint32_t seconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(triggerTime.time_since_epoch()).count() / 1000000);
    uint32_t microseconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(triggerTime.time_since_epoch()).count() % 1000000);

    // We have been triggered.  Get the next image and send it to any streams it should
    // go to.  Grab the mutex while getting our list of streams so that it does not get
    // changed out from under us.  Release the lock before we start looking at the
    // streams so that we give the RemoveStream() method time to grab it when needed.
    std::list<Stream> streams;
    {
      std::lock_guard<std::mutex> lock(m_mut);
      streams = m_streams;
    }

    // Check all of the streams to see if we should send an image to them.
    for (auto& stream : streams) {
      // Don't send if it is not yet time for this stream.
      if (seconds < stream.StartSeconds || ((seconds == stream.StartSeconds) && (microseconds < stream.StartMicroseconds))) {
        continue;
      }

      // Get the next image from the source.
      std::shared_ptr<asdp::ImageSource::Image> image = m_images->getNextImage();
      if (!image) {
        continue;
      }

      // Don't send if we are skipping frames.  Increment the frame ID in any case.
      if ((*stream.curFrame)++ % (stream.SkipFrames+1) != 0) {
        continue;
      }

      // Compare each image subset to the ROI and send it if it is in the ROI.
      // The time to send each is evenly divided across the duration of the frame.
      // Remember that the last slice may not have as many rows as the others.
      /// @todo When simulating a 3kx3k camera, every line will be its own slice.
      uint32_t cols = image->getWidth();
      uint32_t rows = image->getHeight();
      uint32_t numSlices = (rows+2) / 3;      ///< At most 3 rows per slice, but be sure to capture any partial write.
      uint32_t sliceTimeMicroseconds = 1e6 * m_trigger->GetPeriod() / (numSlices + 1);  ///< Period gap to send each slice in seconds to be done before next
      auto startSendTime = triggerTime + std::chrono::microseconds(sliceTimeMicroseconds);
      uint32_t startSeconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(startSendTime.time_since_epoch()).count() / 1000000);
      uint32_t startMicroseconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(startSendTime.time_since_epoch()).count() % 1000000);
      for (uint32_t i = 0; i < numSlices; i++) {
        uint32_t firstLine = i * 3;
        uint32_t lastLine = (i + 1) * 3 - 1;
        if (lastLine >= rows) {
          lastLine = rows - 1;
        }
        if (!((firstLine > stream.Roi.end_y) || (lastLine < stream.Roi.start_y))) {
          // Calculate the time to send this slice.
          auto sendTime = triggerTime + std::chrono::microseconds(i * sliceTimeMicroseconds);
          uint32_t seconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(sendTime.time_since_epoch()).count() / 1000000);
          uint32_t microseconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(sendTime.time_since_epoch()).count() % 1000000);

          // Trim the region of interest to fit inside the current slice.
          auto roi = stream.Roi;
          if (firstLine > stream.Roi.start_y) {
            roi.start_y = firstLine;
          }
          if (lastLine < stream.Roi.end_y) {
            roi.end_y = lastLine;
          }

          // Send the ROI if it is not empty.
          if (roi.end_y >= roi.start_y) {
            // Wait until it is time to send the slice message.
            while (std::chrono::steady_clock::now() < sendTime) { }

            bool beginFrame = (roi.start_y == stream.Roi.start_y);
            bool endFrame = (roi.end_y == stream.Roi.end_y);

            // The time for the frame is the time it was send unless the begin/end frame bits are set.
            if (beginFrame) {
              if (endFrame) {
                // Both begin and end frame are set, record the time halfway through the frame.
                auto halfFrame = triggerTime + std::chrono::microseconds(static_cast<uint32_t>(m_trigger->GetPeriod() / 2 + 0.5f));
                seconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(halfFrame.time_since_epoch()).count() / 1000000);
                microseconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(halfFrame.time_since_epoch()).count() % 1000000);
              } else {
                // Use the start time of the frame even if we are not sending that pixel.
                seconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(triggerTime.time_since_epoch()).count() / 1000000);
                microseconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(triggerTime.time_since_epoch()).count() % 1000000);
              }
            } else if (endFrame) {
              // Only end frame is set, record the time of the end of the total frame even if we are not sending that pixel.
              auto endOfFrame = triggerTime + std::chrono::microseconds(static_cast<uint32_t>(m_trigger->GetPeriod()+0.5f));
              seconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(endOfFrame.time_since_epoch()).count() / 1000000);
              microseconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(endOfFrame.time_since_epoch()).count() % 1000000);
            }
            Time myTime(seconds, microseconds);

            std::shared_ptr<StreamPacket> packetData;
            stream.writer->GetCurrentPacket(packetData);
            uint8_t* dataPtr = reinterpret_cast<uint8_t*>(image->getData()->data());
            MessageConsolidatedFrameData message(*packetData, myTime,
              m_cameraID, m_cameraType, image->getWidth(), image->getHeight(),
              roi.start_x, roi.start_y, roi.end_x, roi.end_y,
              beginFrame, endFrame, dataPtr, m_sensorWidth, m_exposure, m_gain);
            stream.writer->Flush();
          }
        }
      }
    }
  }
}

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

  /// @brief Creates triggers.
  /// @details This cannot be done during the constructor of the State, which fires before the
  /// dynamic libraries are all loaded. At least on Windows, the thread function is never called.
  /// This function should be called before access to the timers is needed, in one of the user-
  /// called functions.  It is idempotent (it has no effect if they are already created).
  void CreateTriggers();

  /// @brief Stop any threads and destruct the state cleanly.
  ~State();

  uint32_t SerialNumber = 0;
  uint32_t NumInternalTriggers = 0;
  uint32_t NumExternalTriggers = 0;
  uint32_t NumTemperatureSensorsPerCamera = 0;
  uint32_t NumSystemTemperatureSensors = 0;
  std::vector<Bsp::Pose::pose_report_s> PoseDeviceConfigurations;
  uint32_t NumCameras = 0;
  double MinCameraPeriodSeconds = 1.0/60;
  double MaxCameraPeriodSeconds = 1.0/50;
  std::vector<TemperatureSensor> TemperatureSensors;
  float TemperaturePeriod = 0.25f;
  std::vector<Pose> Posers;
  float PosePeriod = 0.25f;

  /// @brief Timer object to avoid spin-waiting inside every thread.
  std::shared_ptr< asdp::SpinFreeAccurateTimer< std::chrono::steady_clock::time_point> > timer;

  /// @brief The triggers for the system.
  std::vector< std::shared_ptr<Trigger> > triggers;
  /// @brief the external trigger each internal trigger is tied to (0 for none).
  std::vector<uint8_t> externalTriggers;

  /// @brief the cameras in the system
  std::vector< std::shared_ptr<Camera> > cameras;
};

State::State(const std::map<std::string, std::map<std::string, std::string>>& configMap)
{
  // Set the serial number if it is specified.
  if (configMap.find("General") != configMap.end()) {
    std::map<std::string, std::string> general = configMap.at("General");
    try {
      SerialNumber = std::stoi(general.at("SerialNumber"));
    } catch (...) {}
  }

  // Set up timer
  timer = std::make_shared< asdp::SpinFreeAccurateTimer< std::chrono::steady_clock::time_point> >();

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
      MinCameraPeriodSeconds = std::stod(camera.at("MinFramePeriodSeconds"));
    } catch (...) {}
    try {
      MaxCameraPeriodSeconds = std::stod(camera.at("MaxFramePeriodSeconds"));
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

  // Cameras are initially not triggered.
  for (uint32_t i = 0; i < NumCameras; i++) {
    cameras.push_back(std::make_shared<Camera>(i+1, 0, 1280, 1024, 0.001f, 1.0f,
      MinCameraPeriodSeconds, MaxCameraPeriodSeconds));
  }
}

void State::CreateTriggers()
{
  // Create the internal triggers which will by default be disabled and not tied to any external trigger.
  // Round-robin the timers onto the timers to spread the load.
  /// @todo Consider a single timer to handle all triggers that is separate from the ones used for streaming.
  if (triggers.size() < NumInternalTriggers) {
    for (size_t i = triggers.size(); i < NumInternalTriggers; i++) {
      triggers.push_back(std::make_shared<Trigger>(timer));
      externalTriggers.push_back(0);
    }
  }
}

State::~State()
{
  // Get rid of the cameras (and their threads), then the triggers (and their threads), then the timer.
  cameras.clear();
  triggers.clear();
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
    return "4.0.0";
  }

  uint32_t GetSerialNumber(void) {
    return state.SerialNumber;
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
      auto now = std::chrono::steady_clock::now();
      Seconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() / 1000000);
      Microseconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000);
    }

    bool ConfigureTrigger(uint16_t ID, uint8_t Mode, uint8_t ExternalID, double Period, float TrackingFactor) {
      state.CreateTriggers();
      if ((ID == 0) || (ID > state.triggers.size())) {
        return false;
      }
      // The array is 0-indexed but the ID is 1-indexed.
      state.triggers[ID - 1]->Configure(Mode, Period, TrackingFactor);
      state.externalTriggers[ID - 1] = ExternalID;
      return true;
    }

    bool SoftwareTrigger(uint8_t ExternalID, uint32_t Seconds, uint32_t Microseconds) {
      if ((ExternalID == 0) || (ExternalID > GetNumberOfExternalTriggers())) {
        return false;
      }

      bool failed = false;

      // See if any of the internal triggers are tied to this external trigger.  If so, fire them.
      // We stored the external trigger ID as a 1-indexed value, so we don't adjust it here.
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
          temp.Seconds = seconds;
          temp.Microseconds = microseconds;
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
      // Ensure that our triggers exist.
      state.CreateTriggers();

      if ((Index == 0) || (Index > state.NumCameras)) {
        return false;
      }
      if (Trigger > state.triggers.size()) {
        return false;
      }

      // The arrays are 0-indexed whereas our numbers are 1-indexed.
      // Round-robin the cameras onto the Timer objects so that they will not get overloaded.
      if (Trigger == 0) {
        return state.cameras[Index - 1]->Setup(nullptr);
      } else {
        return state.cameras[Index - 1]->Setup(state.triggers[Trigger - 1]);
      }
    }

    std::map<std::string, std::string> GetStatus(uint16_t Index) {
      std::map<std::string, std::string> status;
      if ((Index == 0) || (Index > state.NumCameras)) {
        // Return an empty status if the camera index is out of range.
        return status;
      }

      // Fill in camera inforamtion.
      // The arrays are 0-indexed whereas our numbers are 1-indexed.
      status["type"] = std::to_string(state.cameras[Index - 1]->GetCameraType());
      status["width"] = std::to_string(state.cameras[Index - 1]->GetSensorWidth());
      status["height"] = std::to_string(state.cameras[Index - 1]->GetSensorHeight());
      status["minTriggerPeriod"] = ToStringWithPrecision(state.cameras[Index - 1]->GetMinPeriodSeconds());
      status["maxTriggerPeriod"] = ToStringWithPrecision(state.cameras[Index - 1]->GetMaxPeriodSeconds());

      return status;
    }

    bool SetNUCFlagState(bool Flipped) {
      /// @todo Implement this if it is configured.
      return false;
    }

    bool StartOnCameraNUC() {
      /// @todo Implement this if it is configured.
      return false;
    }
  }

  /// Namespace for UDP inserter functions.
  namespace UdpInserter {

    bool EnableStream(uint16_t CameraIndex, endpoint_s Endpoint, roi_s Roi, uint32_t SkipFrames,
      uint32_t StartSeconds, uint32_t StartMicroseconds) {
      if ((CameraIndex == 0) || (CameraIndex > state.cameras.size())) {
        return false;
      }
      // The arrays are 0-indexed whereas our numbers are 1-indexed.
      state.cameras[CameraIndex - 1]->InsertStream(Endpoint, Roi, SkipFrames, StartSeconds, StartMicroseconds);
      return true;
    }

    bool DisableStream(uint32_t CameraIndex, endpoint_s Endpoint) {
      if ((CameraIndex == 0) || (CameraIndex > state.cameras.size())) {
        return false;
      }
      // The arrays are 0-indexed whereas our numbers are 1-indexed.
      return state.cameras[CameraIndex - 1]->RemoveStream(Endpoint);
    }

  }

}
