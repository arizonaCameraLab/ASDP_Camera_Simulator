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
#include <iostream>   ///< @todo Remove when done debugging
#include <BspInclude.hpp>
#include <ASDP_Core_API.h>
#include <ASDP_SpinFreeQueue.hpp>
#include <ASDP_SpinFreeAccurateTimer.hpp>
#include <Trigger.h>
#include <ImageSource.h>

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
  /// @param id The ID of the camera.
  Camera(
    uint32_t cameraID = 0, uint32_t cameraType = 0, uint32_t sensorWidth = 1280, uint32_t sensorHeight = 1024,
    float exposure = 0.001f,  float gain = 1.0f);

  /// @brief Destroy a camera object.
  ~Camera();

  /// @brief Set up the camera with a trigger.
  /// @param timer The timer to associate with the camera.
  /// @param trigger The trigger to associate with the camera. Nullptr to disable triggering.
  /// @return true if the camera was set up successfully, false otherwise.
  bool Setup(std::shared_ptr< SpinFreeAccurateTimer<std::chrono::steady_clock::time_point> > timer,
    std::shared_ptr<Trigger> trigger);

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

protected:

  //========================
  // Camera configuration.
  uint32_t m_cameraID = 0;
  uint32_t m_cameraType = 2;
  uint32_t m_sensorWidth = 1280;
  uint32_t m_sensorHeight = 1024;
  float m_exposure = 0.001f;
  float m_gain = 1.0f;

  /// @brief The images that will be sent from the camera stored in a loopable source.
  std::shared_ptr<IS::ImageSource> m_images;

  /// @brief The internal trigger each camera is associated with (empty for none).
  std::shared_ptr<Trigger> m_trigger;

  /// @brief Timer used for slice timing.
  std::shared_ptr< SpinFreeAccurateTimer<std::chrono::steady_clock::time_point> > m_timer;

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
    size_t curFrame = 0;  ///< The current frame number, starting with 0, used modulo SkipFrames to determine when to send a frame.
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

  //========================
  // Sending thread that receives per-slice triggers and sends the slices.
  // Handles the case where it misses a trigger by using the time to send each slice.

  /// @brief Data structure holding information about a slice
  struct Slice {
    std::shared_ptr<StreamWriter> writer;           ///< The StreamWriter being used to send the stream.
    std::chrono::steady_clock::time_point sendTime; ///< The time to send this slice.
    std::shared_ptr<IS::Image> image;               ///< The image to send a portion of.
    Bsp::UdpInserter::roi_s roi;                    ///< The region of interest to send. All zeros if empty (send frame end).
  };
  /// @brief The queue of slices that are ready to send, in increasing time order.
  asdp::SpinFreeQueue<Slice> m_sliceQueue;

  /// @brief The thread that sends the slices to the streams.
  std::thread m_sliceThread;

  /// @brief The mutex to use with the slice queue to avoid writing packets in both queues.
  std::mutex m_sliceMut;

  /// @brief The function that sends the slices to the streams.
  void SliceThread();

  /// @brief Atomic flag to signal the slice thread to stop.
  std::atomic_bool m_sliceThreadStop = false;

  /// Queue to push on when a slice is triggered.
  /// After an entry has been added to the m_sliceQueue, an entry is added to this queue.
  /// When the slice thread receives an entry on the m_sliceQ, it reads the m_sliceQueue
  /// to find the corresponding entry.  There are always the same number of entries pushed
  /// onto each queue.
  std::shared_ptr< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> > m_sliceQ;
};

Camera::Camera(uint32_t cameraID, uint32_t cameraType, uint32_t sensorWidth, uint32_t sensorHeight,
  float exposure, float gain)
  : m_cameraID(cameraID)
  , m_cameraType(cameraType)
  , m_sensorWidth(sensorWidth)
  , m_sensorHeight(sensorHeight)
  , m_exposure(exposure)
  , m_gain(gain)
  , m_triggerQ(std::make_shared< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> >())
  , m_sliceQ(std::make_shared< asdp::SpinFreeQueue<std::chrono::steady_clock::time_point> >())
{
}

Camera::~Camera()
{
  // Stop and join our threads
  m_triggerThreadStop = true;
  if (m_triggerThread.joinable()) {
    m_triggerThread.join();
  }
  m_sliceThreadStop = true;
  if (m_sliceThread.joinable()) {
    m_sliceThread.join();
  }

  // Clear our streams and slice queue, then reset our trigger and timer.
  m_streams.clear();
  m_sliceQ.reset();
  m_triggerQ.reset();
  m_trigger.reset();
  m_timer.reset();
}

bool Camera::Setup(std::shared_ptr< SpinFreeAccurateTimer<std::chrono::steady_clock::time_point> > timer,
  std::shared_ptr<Trigger> trigger)
{
  {
    std::lock_guard<std::mutex> lock(m_mut);

    m_timer = timer;

    // Construct the images that will be sent from the camera.
    // We must do this here because we can't run parallel threads in the constructor that
    // is called during the dynamic library load in the State constructor.  At least on
    // Windows, the threads do not start then.
    /// @todo Read the number of images from the configuration file.
    /// @todo Enable specifying a directory to read the images from as files.
    size_t frames = 60;
    m_images = std::make_shared<IS::MovingBarsSource>(1280, 1024, frames, 20, 48, 5);

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
    m_sliceThreadStop = false;
    m_sliceThread = std::thread([this] { SliceThread(); });
  } else {
    m_triggerThreadStop = true;
    if (m_triggerThread.joinable()) {
      m_triggerThread.join();
    }
    m_sliceThreadStop = true;
    if (m_sliceThread.joinable()) {
      m_sliceThread.join();
    }
  }

  // Clear the trigger and slice queues so they will be ready when re-started.
  std::chrono::steady_clock::time_point timePt;
  while (m_triggerQ->size()) {
    m_triggerQ->dequeue(timePt, std::chrono::milliseconds(10));
  }
  while (m_sliceQ->size()) {
    m_sliceQ->dequeue(timePt, std::chrono::milliseconds(10));
  }
  while (m_sliceQueue.size()) {
    Slice slice;
    m_sliceQueue.dequeue(slice, std::chrono::milliseconds(10));
  }

  // Register us with the trigger.
  m_trigger = trigger;
  if (trigger) {
    trigger->RegisterClient(m_triggerQ);
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

    uint32_t seconds = (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(triggerTime.time_since_epoch()).count() / 1000000;
    uint32_t microseconds = (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(triggerTime.time_since_epoch()).count() % 1000000;

    // We have been triggered.  Get the next image and send it to any streams it should
    // go to.
    std::lock_guard<std::mutex> lock(m_mut);

    // Check all of the streams to see if we should send an image to them.
    for (auto& stream : m_streams) {
      // Don't send if it is not yet time for this stream.
      if (seconds < stream.StartSeconds || (seconds == stream.StartSeconds && microseconds < stream.StartMicroseconds)) {
        continue;
      }
      // Don't send if we are skipping frames.  Increment the frame ID in any case.
      if (stream.curFrame++ % (stream.SkipFrames+1) != 0) {
        continue;
      }

      // Get the next image from the source.
      std::shared_ptr<IS::Image> image = m_images->getNextImage();
      if (!image) {
        continue;
      }

      /// Wait until the slice queues are empty to prevent us from writing overlapping frames
      m_sliceQ->awaitEmpty(0, std::chrono::milliseconds(1000));
      m_sliceQueue.awaitEmpty(0, std::chrono::milliseconds(1000));

      // Send a begin-frame message to the stream.
      {
        std::lock_guard<std::mutex> lock(m_sliceMut);
        std::shared_ptr<StreamPacket> packetBegin;
        stream.writer->GetCurrentPacket(packetBegin);
        MessageFrameBegin message(*packetBegin, Time(seconds, microseconds),
          m_cameraID, m_cameraType, image->getWidth(), image->getHeight(), m_exposure, m_gain);
        stream.writer->Flush();
      }

      // Compare each image subset to the ROI and send it if it is in the ROI.
      // The time to send each is evenly divided across the duration of the frame.
      // Remember that the last slice may not have as many rows as the others.
      // Put all of the slices into a vector, then schedule them in the Timer to be sent.
      // The sending thread will send them when triggered.
      // Store the time to send each slice inside the slice object so that the sending thread can use it to ensure that
      // it doesn't miss slices even if it misses a condition variable trigger.
      uint32_t cols = image->getWidth();
      uint32_t rows = image->getHeight();
      uint32_t numSlices = rows / 3;      ///< At most 3 rows per slice.
      uint32_t sliceTime = m_trigger->GetPeriod() / (numSlices + 1);  ///< Period gap to send each slice in seconds to be done before next
      for (uint32_t i = 0; i < numSlices; i++) {
        uint32_t firstLine = i * 3;
        uint32_t lastLine = (i + 1) * 3 - 1;
        if (lastLine >= rows) {
          lastLine = rows - 1;
        }
        if (!((firstLine > stream.Roi.end_y) || (lastLine < stream.Roi.start_y))) {
          Slice slice;
          slice.writer = stream.writer;
          slice.sendTime = triggerTime + std::chrono::seconds(i * sliceTime);
          slice.image = image;
          // Trim the region of interest to fit inside the current slice.
          slice.roi = stream.Roi;
          if (firstLine > stream.Roi.start_y) {
            slice.roi.start_y = firstLine;
          }
          if (lastLine < stream.Roi.end_y) {
            slice.roi.end_y = lastLine;
          }

          // Push the slice onto the slice queue.
          m_sliceQueue.enqueue(slice);

          // Add an entry to the timer to send the slice at the appropriate time.
          m_timer->AddEntry(slice.sendTime, slice.sendTime, m_sliceQ);
        }
      }

      // Always push a frame end message to the stream at the time that the last line will be sent.
      // it will have an empty ROI so that no frame-data message will be sent.
      {
        Slice slice;
        slice.writer = stream.writer;
        slice.sendTime = triggerTime + std::chrono::seconds(numSlices * sliceTime);
        slice.image = image;
        slice.roi = {};

        // Push the slice onto the slice queue.
        m_sliceQueue.enqueue(slice);

        // Add an entry to the timer to send the slice at the appropriate time.
        m_timer->AddEntry(slice.sendTime, slice.sendTime, m_sliceQ);
      }
    }
  }
}

void Camera::SliceThread()
{
  while (!m_sliceThreadStop) {

    // If there is an entry available on the sliceQ from the timer, read the corresponding entry from
    // the sliceQueue and send it to the stream. Otherwise, time out and keep looking.
    std::chrono::steady_clock::time_point sliceTime;
    if (!m_sliceQ->dequeue(sliceTime, std::chrono::milliseconds(50))) {
      continue;
    }

    // Get the entry.
    Slice slice ;
    if (!m_sliceQueue.dequeue(slice, std::chrono::milliseconds(1))) {
      continue;
    }

    uint32_t seconds = (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(sliceTime.time_since_epoch()).count() / 1000000;
    uint32_t microseconds = (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(sliceTime.time_since_epoch()).count() % 1000000;

    // If it has zero size, we instead send a frame end message. In any case, lock the slice mutex so
    // we don't try to write a packet while the trigger thread is writing a packet.
    std::lock_guard<std::mutex> lock(m_sliceMut);
    if (slice.roi.start_y == 0 && slice.roi.end_y == 0) {
      std::shared_ptr<StreamPacket> packetEnd;
      slice.writer->GetCurrentPacket(packetEnd);
      MessageFrameEnd message(*packetEnd, Time(seconds, microseconds), m_cameraID);
      slice.writer->Flush();
    } else {
      std::shared_ptr<StreamPacket> packetData;
      slice.writer->GetCurrentPacket(packetData);
      uint8_t *dataPtr = reinterpret_cast<uint8_t*>(slice.image->getData()->data());
      MessageFrameData message(*packetData, Time(seconds, microseconds), m_cameraID,
        slice.roi.start_x, slice.roi.start_y, slice.roi.end_x, slice.roi.end_y, dataPtr, m_sensorWidth);
      slice.writer->Flush();
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

  /// @brief Configures timer and triggers.
  /// @details This cannot be done during the constructor of the State, which fires before the
  /// dynamic libraries are all loaded. At least on Windows, the thread function is never called.
  /// This function should be called before access to the timers is needed, in one of the user-
  /// called functions.  It is idempotent (it has no effect if they are already set up).
  void ConfigureTriggers();

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
  std::shared_ptr< asdp::SpinFreeAccurateTimer<std::chrono::steady_clock::time_point> > timer;

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

  // Cameras are initially not triggered.
  for (uint32_t i = 0; i < NumCameras; i++) {
    cameras.push_back(std::make_shared<Camera>(i+1));
  }
}

void State::ConfigureTriggers()
{
  if (!timer) {
    timer = std::make_shared< asdp::SpinFreeAccurateTimer<std::chrono::steady_clock::time_point> >();
  }

  // Create the internal triggers which will by default be disabled and not tied to any external trigger.
  if (triggers.size() != NumInternalTriggers) {
    for (size_t i = 0; i < NumInternalTriggers; i++) {
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
  timer.reset();
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

  uint32_t GetSerialNumber(void) {
    return 0;
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
      Seconds = (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() / 1000000;
      Microseconds = (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000;
    }

    bool ConfigureTrigger(uint16_t ID, uint8_t Mode, uint8_t ExternalID, float Period, float TrackingFactor) {
      state.ConfigureTriggers();
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
      // Ensure that our triggers are configured.
      state.ConfigureTriggers();

      if ((Index == 0) || (Index > state.NumCameras)) {
        return false;
      }
      if (Trigger > state.triggers.size()) {
        return false;
      }

      // The arrays are 0-indexed whereas our numbers are 1-indexed.
      if (Trigger == 0) {
        return state.cameras[Index - 1]->Setup(state.timer, nullptr);
      } else {
        return state.cameras[Index - 1]->Setup(state.timer, state.triggers[Trigger - 1]);
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
      status["minTriggerPeriod"] = "0.0166666";
      status["maxTriggerPeriod"] = "0.0200001";

      return status;
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
