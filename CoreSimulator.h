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
  /// @return Error message when the server stops running.
  std::string run();

protected:
  std::vector< std::shared_ptr< asdp::IS::ImageSource> > m_sources; ///< The image sources to be used by the server
  int m_verbosity; ///< The verbosity level of the server, 0 for no verbosity, higher for more verbosity.
};

} // namespace asdp