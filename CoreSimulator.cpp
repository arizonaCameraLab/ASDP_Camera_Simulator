/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include "CoreSimulator.h"
#include <iostream>
#include <algorithm>
#include <limits>

using namespace asdp;

/// @brief Helper function to translate from OpCode to a descriptive string.
/// @param opCode The OpCode to translate.
/// @return A descriptive string for the OpCode.
static std::string OpCodeName(OpCode opCode)
{
  switch (opCode) {
  case RESET: return "RESET";
  case CANCEL_ALL_STREAMS: return "CANCEL_ALL_STREAMS";
  case START_RECORDING: return "START_RECORDING";
  case STOP_RECORDING: return "STOP_RECORDING";
  case SET_START_UP_RECORDING_STATE: return "SET_START_UP_RECORDING_STATE";
  case START_REPLAY: return "START_REPLAY";
  case PAUSE_REPLAY: return "PAUSE_REPLAY";
  case STOP_REPLAY: return "STOP_REPLAY";
  case SET_KEEPALIVE_INTERVAL: return "SET_KEEPALIVE_INTERVAL";
  case STREAM_STATE: return "STREAM_STATE";
  case CANCEL_STATE: return "CANCEL_STATE";
  case CONFIGURE_TRIGGER: return "CONFIGURE_TRIGGER";
  case SOFTWARE_TRIGGER: return "SOFTWARE_TRIGGER";
  case STREAM_EVENTS: return "STREAM_EVENTS";
  case CANCEL_EVENTS: return "CANCEL_EVENTS";
  case STREAM_SUBREGIONS: return "STREAM_SUBREGIONS";
  case CANCEL_SUBREGIONS: return "CANCEL_SUBREGIONS";
  case ERASE_ALL_STORED_STREAMS: return "ERASE_ALL_STORED_STREAMS";
  case LIST_STORED_STREAMS: return "LIST_STORED_STREAMS";
  case ERASE_STORED_STREAM: return "ERASE_STORED_STREAM";
  case STREAM_TEMPERATURES: return "STREAM_TEMPERATURES";
  case CANCEL_TEMPERATURES: return "CANCEL_TEMPERATURES";
  case STREAM_POSES: return "STREAM_POSES";
  case CANCEL_POSES: return "CANCEL_POSES";
  default: return "UNKNOWN";
  }
}

CoreSimulator::CoreSimulator(uint32_t serialNumber, const std::string& IP,
  std::vector< std::shared_ptr< asdp::IS::ImageSource> > sources,
  int verbosity)
  : CoreServer(serialNumber, IP)
  , m_sources(sources)
  , m_verbosity(verbosity)
  , m_keepAliveInterval(0)
  , m_framePeriod(1/60.0f)
{
}

asdp::Time CoreSimulator::getNewTimeout()
{
  Time time = { std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max() };
  if (m_keepAliveInterval <= 0) {
    return time;
  }

  // Get the current time.
  Status status = m_timer->GetCoreTime(time);
  if (status != OKAY) {
    m_error = "Failed to get core time: " + ErrorMessage(status);
    return time;
  }

  // See how much to increase the current time by.
  uint32_t seconds = static_cast<uint32_t>(m_keepAliveInterval);
  uint32_t microseconds = static_cast<uint32_t>(
    (m_keepAliveInterval - static_cast<float>(static_cast<uint32_t>(m_keepAliveInterval))) * 1000000);
  Time increase = { seconds, microseconds };
  return time + increase;
}

asdp::CoreSimulator::WriterInfo &CoreSimulator::getWriter(
  const StreamEndpoint& endpoint, std::list<WriterInfo>& baseList)
{
  std::lock_guard<std::recursive_mutex> lock(m_mutex);

  // See if there is already an entry for this endpoint in the list we're trying to add
  // it to.  If there is, we're done.
  auto it = std::find_if(baseList.begin(), baseList.end(),
       [&endpoint](const WriterInfo& info) { return info.endpoint == endpoint; });
  if (it != baseList.end()) {
    it->timeOut = getNewTimeout();
    return *it;
  }

  // See if there is already an entry for this endpoint in the other lists.  If there is,
  // add it to the list above.
  if (&baseList != &m_stateWriters) if (m_stateWriters.size() > 0) {
    it = std::find_if(m_stateWriters.begin(), m_stateWriters.end(),
              [&endpoint](const WriterInfo& info) { return info.endpoint == endpoint; });
    if (it != m_stateWriters.end()) {
      baseList.push_back(*it);
      it->timeOut = getNewTimeout();
      return *it;
    }
  }
  if (&baseList != &m_eventWriters) if (m_eventWriters.size() > 0) {
    it = std::find_if(m_eventWriters.begin(), m_eventWriters.end(),
                   [&endpoint](const WriterInfo& info) { return info.endpoint == endpoint; });
    if (it != m_eventWriters.end()) {
      baseList.push_back(*it);
      it->timeOut = getNewTimeout();
      return *it;
    }
  }
  if (&baseList != &m_subregionWriters) if (m_subregionWriters.size() > 0) {
    it = std::find_if(m_subregionWriters.begin(), m_subregionWriters.end(),
                        [&endpoint](const WriterInfo& info) { return info.endpoint == endpoint; });
    if (it != m_subregionWriters.end()) {
      baseList.push_back(*it);
      it->timeOut = getNewTimeout();
      return *it;
    }
  }

  // Create a new sender and writer for the endpoint because there is not one.
  // Add it to the list.
  std::shared_ptr<Sender> sender(new SenderUDP(endpoint));
  Status status = sender->GetConstructorStatus();
  if (status != OKAY) {
    m_error = "Failed to create SenderUDP: " + ErrorMessage(status);
  }
  std::shared_ptr<StreamWriter> writer(new StreamWriter(sender, m_timer));
  status = writer->GetConstructorStatus();
  if (status != OKAY) {
    m_error = "Failed to create StreamWriter: " + ErrorMessage(status);
  }
  WriterInfo wi;
  wi.endpoint = endpoint;
  wi.writer = writer;
  if (m_verbosity >= 1) {
    std::cout << "Created StreamWriter for " << endpoint << std::endl;
  }
  baseList.push_back(wi);
  wi.timeOut = getNewTimeout();
  return baseList.back();
}

void CoreSimulator::sendInvalidCommandMessage(OpCode opCode)
{
  if (m_verbosity >= 1) {
    std::cout << "Invalid opcode received: " << opCode << std::endl;
  }

  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (auto writer : m_eventWriters) {
    uint8_t priority = 0;
    EventID type = INVALID_OPERATION;
    if (writer.verbosity < type) {
      continue;
    }
    std::shared_ptr<StreamPacket> packet;
    Status status = writer.writer->GetCurrentPacket(packet);
    if (status != OKAY) {
      m_error = "Error getting current packet from m_eventWriters: " + ErrorMessage(status);
      return;
    }
    Time timeCode;
    m_timer->GetCoreTime(timeCode);
    MessageEvent message(*packet, timeCode, priority, type, std::to_string(opCode));
    if (message.GetConstructorStatus() != OKAY) {
      m_error = "Error constructing MessageEvent: " + ErrorMessage(message.GetConstructorStatus());
      return;
    }

    // Send the packet immediately.
    status = writer.writer->Flush();
    if (status != OKAY) {
      m_error = "Error flushing m_eventWriters: " + ErrorMessage(status);
      return;
    }
  }
}

std::string CoreSimulator::run()
{
  // Get the Receiver we'll use to get Commands from the client.
  std::shared_ptr<Receiver> receiver;
  Status status = GetReceiver(receiver);
  if (status != OKAY) {
    return "Failed to get receiver: " + ErrorMessage(status);
  }

  // Loop forever, getting Commands from the client and acting on them.
  while (true) {

    // If the discovery thread fails, we should stop.
    Status discoveryStatus;
    status = GetDiscoveryThreadStatus(discoveryStatus);
    if (status != OKAY) {
      return "Failed to get discovery thread status: " + ErrorMessage(status);
    }
    if (discoveryStatus != OKAY) {
      return "Discovery thread failed: " + ErrorMessage(discoveryStatus);
    }

    // Wait up to half a second for receiving a command.
    std::shared_ptr<CommandPacket> command;
    status = receiver->ReceiveCommandPacket(0.5, command);
    if (status == TIMEOUT) {
      // Loop again to try to get a command.
      continue;
    }
    if (status != OKAY) {
      return "Failed to get command: " + ErrorMessage(status);
    }

    // Act on the command.
    OpCode opCode;
    status = command->GetOpCode(opCode);
    if (status != OKAY) {
      return "Failed to get op code: " + ErrorMessage(status);
    }
    if (m_verbosity >= 1) {
      std::cout << "Received command " << OpCodeName(opCode) << std::endl;
    }
    switch (opCode) {
    case RESET:
      {
        CommandPacketReset resetCommand(*command);
        status = resetCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct reset command: " + ErrorMessage(status);
        }
        doReset(resetCommand);
      }
      break;
    case CANCEL_ALL_STREAMS:
      {
        CommandPacketCancelAllStreams cancelAllStreamsCommand(*command);
        status = cancelAllStreamsCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct cancel all streams command: " + ErrorMessage(status);
        }
        doCancelAllStreams(cancelAllStreamsCommand);
      }
      break;
    case START_RECORDING:
      {
        CommandPacketStartRecording startRecordingCommand(*command);
        status = startRecordingCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct start recording command: " + ErrorMessage(status);
        }
        doStartRecording(startRecordingCommand);
      }
      break;
    case STOP_RECORDING:
      {
        CommandPacketStopRecording stopRecordingCommand(*command);
        status = stopRecordingCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct stop recording command: " + ErrorMessage(status);
        }
        doStopRecording(stopRecordingCommand);
      }
      break;
    case SET_START_UP_RECORDING_STATE:
      {
        CommandPacketSetStartUpRecordingState setStartUpRecordingStateCommand(*command);
        status = setStartUpRecordingStateCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct set start up recording state command: " + ErrorMessage(status);
        }
        doSetStartUpRecordingState(setStartUpRecordingStateCommand);
      }
      break;
    case START_REPLAY:
      {
        CommandPacketStartReplay startReplayCommand(*command);
        status = startReplayCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct start replay command: " + ErrorMessage(status);
        }
        doStartReplay(startReplayCommand);
      }
      break;
    case PAUSE_REPLAY:
      {
        CommandPacketPauseReplay pauseReplayCommand(*command);
        status = pauseReplayCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct pause replay command: " + ErrorMessage(status);
        }
        doPauseReplay(pauseReplayCommand);
      }
      break;
    case STOP_REPLAY:
      {
        CommandPacketStopReplay stopReplayCommand(*command);
        status = stopReplayCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct stop replay command: " + ErrorMessage(status);
        }
        doStopReplay(stopReplayCommand);
      }
      break;
    case SET_KEEPALIVE_INTERVAL:
      {
        CommandPacketKeepaliveInterval setKeepAliveIntervalCommand(*command);
        status = setKeepAliveIntervalCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct set keep alive interval command: " + ErrorMessage(status);
        }
        doSetKeepAliveInterval(setKeepAliveIntervalCommand);
      }
      break;
    case STREAM_STATE:
      {
        CommandPacketStreamState streamStateCommand(*command);
        status = streamStateCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct stream state command: " + ErrorMessage(status);
        }
        doStreamState(streamStateCommand);
      }
      break;
    case CANCEL_STATE:
      {
        CommandPacketCancelState cancelStateCommand(*command);
        status = cancelStateCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct cancel state command: " + ErrorMessage(status);
        }
        doCancelState(cancelStateCommand);
      }
      break;
    case CONFIGURE_TRIGGER:
      {
        CommandPacketConfigureTrigger configureTriggerCommand(*command);
        status = configureTriggerCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct configure trigger command: " + ErrorMessage(status);
        }
        doConfigureTrigger(configureTriggerCommand);
      }
      break;
    case SOFTWARE_TRIGGER:
      {
        CommandPacketSoftwareTrigger softwareTriggerCommand(*command);
        status = softwareTriggerCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct software trigger command: " + ErrorMessage(status);
        }
        doSoftwareTrigger(softwareTriggerCommand);
      }
      break;
    case STREAM_EVENTS:
      {
        CommandPacketStreamEvents streamEventsCommand(*command);
        status = streamEventsCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct stream events command: " + ErrorMessage(status);
        }
        doStreamEvents(streamEventsCommand);
      }
      break;
    case CANCEL_EVENTS:
      {
        CommandPacketCancelEvents cancelEventsCommand(*command);
        status = cancelEventsCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct cancel events command: " + ErrorMessage(status);
        }
        doCancelEvents(cancelEventsCommand);
      }
      break;
    case STREAM_SUBREGIONS:
      {
        CommandPacketStreamSubregions streamSubregionsCommand(*command);
        status = streamSubregionsCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct stream subregions command: " + ErrorMessage(status);
        }
        doStreamSubregions(streamSubregionsCommand);
      }
      break;
    case CANCEL_SUBREGIONS:
      {
        CommandPacketCancelSubregions cancelSubregionsCommand(*command);
        status = cancelSubregionsCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct cancel subregions command: " + ErrorMessage(status);
        }
        doCancelSubregions(cancelSubregionsCommand);
      }
      break;
    case ERASE_ALL_STORED_STREAMS:
      {
        CommandPacketEraseAllStoredStreams eraseAllStoredStreamsCommand(*command);
        status = eraseAllStoredStreamsCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct erase all stored streams command: " + ErrorMessage(status);
        }
        doEraseAllStoredStreams(eraseAllStoredStreamsCommand);
      }
      break;
    case LIST_STORED_STREAMS:
      {
        CommandPacketStreamStoredList listStoredStreamsCommand(*command);
        status = listStoredStreamsCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct list stored streams command: " + ErrorMessage(status);
        }
        doListStoredStreams(listStoredStreamsCommand);
      }
      break;
    case ERASE_STORED_STREAM:
      {
        CommandPacketEraseStoredStream eraseStoredStreamCommand(*command);
        status = eraseStoredStreamCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct erase stored stream command: " + ErrorMessage(status);
        }
        doEraseStoredStream(eraseStoredStreamCommand);
      }
      break;
    case STREAM_TEMPERATURES:
      {
        CommandPacketStreamTemperatures streamTemperaturesCommand(*command);
        status = streamTemperaturesCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct stream temperatures command: " + ErrorMessage(status);
        }
        doStreamTemperatures(streamTemperaturesCommand);
      }
      break;
    case CANCEL_TEMPERATURES:
      {
        CommandPacketCancelTemperatures cancelTemperaturesCommand(*command);
        status = cancelTemperaturesCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct cancel temperatures command: " + ErrorMessage(status);
        }
        doCancelTemperatures(cancelTemperaturesCommand);
      }
      break;
    case STREAM_POSES:
      {
        CommandPacketStreamPoses streamPosesCommand(*command);
        status = streamPosesCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct stream poses command: " + ErrorMessage(status);
        }
        doStreamPoses(streamPosesCommand);
      }
      break;
    case CANCEL_POSES:
      {
        CommandPacketCancelPoses cancelPosesCommand(*command);
        status = cancelPosesCommand.GetConstructorStatus();
        if (status != OKAY) {
          return "Failed to construct cancel poses command: " + ErrorMessage(status);
        }
        doCancelPoses(cancelPosesCommand);
      }
      break;
    default:
      return "Unrecognized OpCode: " + std::to_string(opCode);
    }

    // If we have an error, return it.
    if (!m_error.empty()) {
      return m_error;
    }
  }
}

void CoreSimulator::doReset(const CommandPacketReset& command)
{
  m_error = "@todo implement doReset";
}

void CoreSimulator::doCancelAllStreams(const CommandPacketCancelAllStreams& command)
{
  m_error = "@todo implement doCancelAllStreams";
}

void CoreSimulator::doStartRecording(const CommandPacketStartRecording& command)
{
  // This server does not yet support this operation.
  sendInvalidCommandMessage(START_RECORDING);
}

void CoreSimulator::doStopRecording(const CommandPacketStopRecording& command)
{
  // This server does not yet support this operation.
  sendInvalidCommandMessage(STOP_RECORDING);
}

void CoreSimulator::doSetStartUpRecordingState(const CommandPacketSetStartUpRecordingState& command)
{
  // This server does not yet support this operation.
  sendInvalidCommandMessage(SET_START_UP_RECORDING_STATE);
}

void CoreSimulator::doStartReplay(const CommandPacketStartReplay& command)
{
  // This server does not yet support this operation.
  sendInvalidCommandMessage(START_REPLAY);
}

void CoreSimulator::doPauseReplay(const CommandPacketPauseReplay& command)
{
  // This server does not yet support this operation.
  sendInvalidCommandMessage(PAUSE_REPLAY);
}

void CoreSimulator::doStopReplay(const CommandPacketStopReplay& command)
{
  // This server does not yet support this operation.
  sendInvalidCommandMessage(STOP_REPLAY);
}

void CoreSimulator::doSetKeepAliveInterval(const CommandPacketKeepaliveInterval& command)
{
  Status status = command.GetInterval(m_keepAliveInterval);
  if (status != OKAY) {
    m_error = "Failed to get keep alive interval: " + ErrorMessage(status);
  }
}

void CoreSimulator::doStreamState(const CommandPacketStreamState& command)
{
  m_error = "@todo implement doStreamState";
}

void CoreSimulator::doCancelState(const CommandPacketCancelState& command)
{
  m_error = "@todo implement doCancelState";
}

void CoreSimulator::doConfigureTrigger(const CommandPacketConfigureTrigger& command)
{
  m_error = "@todo implement doConfigureTrigger";
}

void CoreSimulator::doSoftwareTrigger(const CommandPacketSoftwareTrigger& command)
{
  m_error = "@todo implement doSoftwareTrigger";
}

void CoreSimulator::doStreamEvents(const CommandPacketStreamEvents& command)
{
  StreamEndpoint endpoint;
  Status status = command.GetEndpoint(endpoint);
  if (status != OKAY) {
    m_error = "Failed to get endpoint: " + ErrorMessage(status);
    return;
  }

  uint32_t verbosity;
  status = command.GetVerbosity(verbosity);
  if (status != OKAY) {
    m_error = "Failed to get verbosity: " + ErrorMessage(status);
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(m_mutex);

  // Create or get the basic writer for the endpoint.
  WriterInfo &wi = getWriter(endpoint, m_eventWriters);

  // Fill in the specific entries for streaming events.
  wi.verbosity = verbosity;
}

void CoreSimulator::doCancelEvents(const CommandPacketCancelEvents& command)
{
  // Get the endpoint to cancel.
  StreamEndpoint endpoint;
  Status status = command.GetEndpoint(endpoint);
  if (status != OKAY) {
    m_error = "Failed to get endpoint: " + ErrorMessage(status);
    return;
  }

  // Remove it from the list if it is there.
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  auto it = std::find_if(m_eventWriters.begin(), m_eventWriters.end(),
          [&endpoint](const WriterInfo& info) { return info.endpoint == endpoint; });
}

void CoreSimulator::doStreamSubregions(const CommandPacketStreamSubregions& command)
{
  m_error = "@todo implement doStreamSubregions";
}

void CoreSimulator::doCancelSubregions(const CommandPacketCancelSubregions& command)
{
  m_error = "@todo implement doCancelSubregions";
}

void CoreSimulator::doEraseAllStoredStreams(const CommandPacketEraseAllStoredStreams& command)
{
  // This server does not yet support this operation.
  sendInvalidCommandMessage(ERASE_ALL_STORED_STREAMS);
}

void CoreSimulator::doListStoredStreams(const CommandPacketStreamStoredList& command)
{
  // This server does not yet support this operation.
  sendInvalidCommandMessage(LIST_STORED_STREAMS);
}

void CoreSimulator::doEraseStoredStream(const CommandPacketEraseStoredStream& command)
{
  // This server does not yet support this operation.
  sendInvalidCommandMessage(ERASE_STORED_STREAM);
}

void CoreSimulator::doStreamTemperatures(const CommandPacketStreamTemperatures& command)
{
  // This server does not yet support streaming temperatures.
  sendInvalidCommandMessage(STREAM_TEMPERATURES);
}

void CoreSimulator::doCancelTemperatures(const CommandPacketCancelTemperatures& command)
{
  // This server does not yet support streaming temperatures.
  sendInvalidCommandMessage(CANCEL_TEMPERATURES);
}

void CoreSimulator::doStreamPoses(const CommandPacketStreamPoses& command)
{
  // This server does not yet support streaming poses.
  sendInvalidCommandMessage(STREAM_POSES);
}

void CoreSimulator::doCancelPoses(const CommandPacketCancelPoses& command)
{
  // This server does not yet support streaming poses.
  sendInvalidCommandMessage(CANCEL_POSES);
}
