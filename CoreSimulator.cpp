/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include "CoreSimulator.h"
#include <iostream>
#include <algorithm>
#include <limits>

using namespace asdp;

CoreSimulator::CoreSimulator(uint32_t serialNumber, const std::string& IP,
  std::vector< std::shared_ptr< asdp::IS::ImageSource> > sources,
  int verbosity)
  : CoreServerBase(serialNumber, IP, 10102, 10101, 9000 - 28, verbosity)
  , m_sources(sources)
  , m_framePeriod(1/60.0f)
{
}

void CoreSimulator::doReset(const CommandPacketReset& command, ClientState& client)
{
  m_error = "@todo implement doReset";
}

void CoreSimulator::doConfigureTrigger(const CommandPacketConfigureTrigger& command, ClientState& client)
{
  m_error = "@todo implement doConfigureTrigger";
}

void CoreSimulator::doSoftwareTrigger(const CommandPacketSoftwareTrigger& command, ClientState& client)
{
  m_error = "@todo implement doSoftwareTrigger";
}

void CoreSimulator::doStreamSubregion(const CommandPacketStreamSubregion& command, ClientState& client)
{
  m_error = "@todo implement doStreamSubregion";
}

void CoreSimulator::doCancelSubregion(const CommandPacketCancelSubregion& command, ClientState& client)
{
  m_error = "@todo implement doCancelSubregion";
}
