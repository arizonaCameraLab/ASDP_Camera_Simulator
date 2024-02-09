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

class CoreSimulator : public CoreServerBase {
public:
  /// @brief Constructor
  /// @param serialNumber The serial number of the server
  /// @param IP The IP address of the server
  /// @param sources The image sources to be used by the server
  /// @param verbosity The verbosity level of the server, 0 for no verbosity, higher for more verbosity.
  CoreSimulator(uint32_t serialNumber, const std::string &IP,
    std::vector< std::shared_ptr< asdp::IS::ImageSource> > sources,
    int verbosity = 0);

protected:

  // State variables
  std::vector< std::shared_ptr< asdp::IS::ImageSource> > m_sources; ///< The image sources to be used by the server
  float m_framePeriod;              ///< The frame period in seconds.

  //=============================================================================
  // Overrise methods to implement the commands as needed.

  /// @brief Implement the specified command.
  void doReset(const CommandPacketReset&) override;

  /// @brief Implement the specified command.
  void doCancelAllStreams(const CommandPacketCancelAllStreams&) override;

  /// @brief Implement the specified command.
  void doConfigureTrigger(const CommandPacketConfigureTrigger&) override;

  /// @brief Implement the specified command.
  void doSoftwareTrigger(const CommandPacketSoftwareTrigger&) override;

  /// @brief Implement the specified command.
  void doStreamSubregions(const CommandPacketStreamSubregions&) override;

  /// @brief Implement the specified command.
  void doCancelSubregions(const CommandPacketCancelSubregions&) override;


};

} // namespace asdp
