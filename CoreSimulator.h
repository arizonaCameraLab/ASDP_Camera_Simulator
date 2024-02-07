/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#pragma once
#include <asdp_api.h>
#include <ImageSource.h>

namespace asdp {

class CoreSimulator : public CoreServer {
public:
  /// @brief Constructor
  /// @param serialNumber The serial number of the server
  /// @param IP The IP address of the server
  /// @param sources The image sources to be used by the server
  CoreSimulator(uint32_t serialNumber, const std::string &IP,
    std::vector< std::shared_ptr< asdp::IS::ImageSource> > sources,
    int verbosity = 0);

  /// @brief Run the server, not returning unless an error occurs.
  ///
  /// This function runs the server, handling all incoming commands and
  /// returning only when an error occurs.  The error message is returned.
  /// @return Error message when the server stops running.
  std::string run();

protected:
  std::vector< std::shared_ptr< asdp::IS::ImageSource> > m_sources; ///< The image sources to be used by the server
  int m_verbosity; ///< The verbosity level of the server, 0 for no verbosity, higher for more verbosity.
  std::string m_error; ///< The error message when the server stops running.

  float m_keepAliveInterval; ///< The keep alive interval in seconds.

  // Member functions to implement the commands.

  /// @brief Implement the specified command.
  void doReset(const CommandPacketReset&);

  /// @brief Implement the specified command.
  void doCancelAllStreams(const CommandPacketCancelAllStreams&);

  /// @brief Implement the specified command.
  void doStartRecording(const CommandPacketStartRecording&);

  /// @brief Implement the specified command.
  void doStopRecording(const CommandPacketStopRecording&);

  /// @brief Implement the specified command.
  void doSetStartUpRecordingState(const CommandPacketSetStartUpRecordingState&);

  /// @brief Implement the specified command.
  void doStartReplay(const CommandPacketStartReplay&);

  /// @brief Implement the specified command.
  void doPauseReplay(const CommandPacketPauseReplay&);

  /// @brief Implement the specified command.
  void doStopReplay(const CommandPacketStopReplay&);

  /// @brief Implement the specified command.
  void doSetKeepAliveInterval(const CommandPacketKeepaliveInterval&);

  /// @brief Implement the specified command.
  void doStreamState(const CommandPacketStreamState&);

  /// @brief Implement the specified command.
  void doCancelState(const CommandPacketCancelState&);

  /// @brief Implement the specified command.
  void doConfigureTrigger(const CommandPacketConfigureTrigger&);

  /// @brief Implement the specified command.
  void doSoftwareTrigger(const CommandPacketSoftwareTrigger&);

  /// @brief Implement the specified command.
  void doStreamEvents(const CommandPacketStreamEvents&);

  /// @brief Implement the specified command.
  void doCancelEvents(const CommandPacketCancelEvents&);

  /// @brief Implement the specified command.
  void doStreamSubregions(const CommandPacketStreamSubregions&);

  /// @brief Implement the specified command.
  void doCancelSubregions(const CommandPacketCancelSubregions&);

  /// @brief Implement the specified command.
  void doEraseAllStoredStreams(const CommandPacketEraseAllStoredStreams&);

  /// @brief Implement the specified command.
  void doListStoredStreams(const CommandPacketStreamStoredList&);

  /// @brief Implement the specified command.
  void doEraseStoredStream(const CommandPacketEraseStoredStream&);

  /// @brief Implement the specified command.
  void doStreamTemperatures(const CommandPacketStreamTemperatures&);

  /// @brief Implement the specified command.
  void doCancelTemperatures(const CommandPacketCancelTemperatures&);

  /// @brief Implement the specified command.
  void doStreamPoses(const CommandPacketStreamPoses&);

  /// @brief Implement the specified command.
  void doCancelPoses(const CommandPacketCancelPoses&);
};

} // namespace asdp
