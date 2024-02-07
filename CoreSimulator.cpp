/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include "CoreSimulator.h"
#include <iostream>

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
{
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
  m_error = "@todo implement doStartRecording";
}

void CoreSimulator::doStopRecording(const CommandPacketStopRecording& command)
{
  m_error = "@todo implement doStopRecording";
}

void CoreSimulator::doSetStartUpRecordingState(const CommandPacketSetStartUpRecordingState& command)
{
  m_error = "@todo implement doSetStartUpRecordingState";
}

void CoreSimulator::doStartReplay(const CommandPacketStartReplay& command)
{
  m_error = "@todo implement doStartReplay";
}

void CoreSimulator::doPauseReplay(const CommandPacketPauseReplay& command)
{
  m_error = "@todo implement doPauseReplay";
}

void CoreSimulator::doStopReplay(const CommandPacketStopReplay& command)
{
  m_error = "@todo implement doStopReplay";
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
  m_error = "@todo implement doStreamEvents";
}

void CoreSimulator::doCancelEvents(const CommandPacketCancelEvents& command)
{
  m_error = "@todo implement doCancelEvents";
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
  m_error = "@todo implement doEraseAllStoredStreams";
}

void CoreSimulator::doListStoredStreams(const CommandPacketStreamStoredList& command)
{
  m_error = "@todo implement doListStoredStreams";
}

void CoreSimulator::doEraseStoredStream(const CommandPacketEraseStoredStream& command)
{
  m_error = "@todo implement doEraseStoredStream";
}

void CoreSimulator::doStreamTemperatures(const CommandPacketStreamTemperatures& command)
{
  m_error = "@todo implement doStreamTemperatures";
}

void CoreSimulator::doCancelTemperatures(const CommandPacketCancelTemperatures& command)
{
  m_error = "@todo implement doCancelTemperatures";
}

void CoreSimulator::doStreamPoses(const CommandPacketStreamPoses& command)
{
  m_error = "@todo implement doStreamPoses";
}

void CoreSimulator::doCancelPoses(const CommandPacketCancelPoses& command)
{
  m_error = "@todo implement doCancelPoses";
}
