/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#pragma once
#include <asdp_api.h>
#include <ImageSource.h>
#include <map>
#include <mutex>
#include <list>

namespace asdp {

class CoreSimulator : public CoreServer {
public:
  /// @brief Constructor
  /// @param serialNumber The serial number of the server
  /// @param IP The IP address of the server
  /// @param sources The image sources to be used by the server
  /// @param verbosity The verbosity level of the server, 0 for no verbosity, higher for more verbosity.
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
  int m_verbosity; ///< The verbosity level of the server, 0 for no verbosity, higher for more verbosity.
  std::string m_error; ///< The error message when the server stops running.
  std::recursive_mutex m_mutex; ///< Mutex to protect the state variables.

  // State variables
  std::vector< std::shared_ptr< asdp::IS::ImageSource> > m_sources; ///< The image sources to be used by the server
  std::vector<uint16_t> m_features; ///< The features of the server (filled in when added)
  float m_keepAliveInterval;        ///< The keep alive interval in seconds.
  float m_framePeriod;              ///< The frame period in seconds.

  //=============================================================================
  // There are a number of sets of StreamWriters that are created and managed by the server.
  // These are the StreamWriters for the various types of streams.  There can be more
  // than one writer for each type of stream.

  /// @brief Information about a writer for a stream.
  ///
  /// This structure contains the endpoint for the writer and the StreamWriter for the stream.
  /// It also contains entries specific to each time of stream, with only the ones that are
  /// relevant to the stream being used.  This enables us to use common code to handle the
  /// different types of streams.
  struct WriterInfo {
    // Common things
    StreamEndpoint                endpoint; ///< The endpoint for the writer.
    std::shared_ptr<StreamWriter> writer;   ///< The StreamWriter for the stream.
    Time                          timeOut;  ///< When the stream will time out.

    // Stream-specific things
    uint32_t                      verbosity; ///< The verbosity level for event stream.
  };

  /// @todo implement period of streaming for those that have it
  /// @todo Implement KeepAlive by looking through the lists of writers and removing any that have timed out.

  std::list<WriterInfo> m_stateWriters;       ///< The StreamWriters for state, if any.
  std::list<WriterInfo> m_eventWriters;       ///< The StreamWriters for events, if any.
  std::list<WriterInfo> m_subregionWriters;   ///< The StreamWriters for subregions, if any.

  /// @brief Determine the new timeout for a stream based on the current time and
  /// the keep-alive interval.
  /// @return The new timeout for the stream.
  /// This function returns the current time plus
  /// the keep-alive interval, unless the keep-alive interval is zero or negative, in
  /// which case it is set to end end of time.
  Time getNewTimeout();

  /// @brief Create or get an existing StreamWriter for the specified endpoint.
  /// 
  /// This function looks through all of the existing StreamWriters to see if there is
  /// one for the specified endpoint.  If there is, it re-uses it.  If not, it creates
  /// a new one and adds it to the list.
  /// 
  /// This function also sets the timeout for the StreamWriter to the current time plus
  /// the keep-alive interval, unless the keep-alive interval is zero or negative, in
  /// which case it is set to end end of time.
  /// @param endpoint The endpoint for the sender.
  /// @param baseList The list that the writer will be added to.  If the writer is
  /// already in this particular list, it is not added again.
  /// @return The StreamWriter for the endpoint in the specified list, so that its
  /// specific entries can be filled in.
  WriterInfo &getWriter(const StreamEndpoint& endpoint, std::list<WriterInfo> &baseList);

  //=============================================================================
  // Helper methods.

  /// @brief Send an invalid-command event to the event stream.
  /// @param opCode The invalid command opcode.
  void sendInvalidCommandMessage(OpCode opCode);

  //=============================================================================
  // Methods to implement the commands.

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
