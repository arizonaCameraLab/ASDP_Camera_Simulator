/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include "CoreSimulator.h"
#include <iostream>

using namespace asdp;

CoreSimulator::CoreSimulator(uint32_t serialNumber, const std::string& IP,
  std::vector< std::shared_ptr< asdp::IS::ImageSource> > sources,
  int verbosity)
  : CoreServer(serialNumber, IP)
  , m_sources(sources)
  , m_verbosity(verbosity)
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
      std::cout << "Received command code: " << opCode << std::endl;
    }
  }
}