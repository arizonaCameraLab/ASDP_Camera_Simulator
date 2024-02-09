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
  : CoreServerBase(serialNumber, IP, verbosity)
  , m_sources(sources)
  , m_framePeriod(1/60.0f)
{
}

void CoreSimulator::doReset(const CommandPacketReset& command)
{
  m_error = "@todo implement doReset";
}

void CoreSimulator::doCancelAllStreams(const CommandPacketCancelAllStreams& command)
{
  m_error = "@todo implement doCancelAllStreams";
}

void CoreSimulator::doConfigureTrigger(const CommandPacketConfigureTrigger& command)
{
  m_error = "@todo implement doConfigureTrigger";
}

void CoreSimulator::doSoftwareTrigger(const CommandPacketSoftwareTrigger& command)
{
  m_error = "@todo implement doSoftwareTrigger";
}

void CoreSimulator::doStreamSubregions(const CommandPacketStreamSubregions& command)
{
  m_error = "@todo implement doStreamSubregions";
}

void CoreSimulator::doCancelSubregions(const CommandPacketCancelSubregions& command)
{
  m_error = "@todo implement doCancelSubregions";
}
