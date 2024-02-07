/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

// This is a basic server that listens for commands from a client.  It opens a server
// on the specified IP address and listens for commands from the client.  It prints
// the op code of each command it receives.

#include <cstdlib>
#include <iostream>
#include <chrono>
#include <asdp_api.h>
#include <ImageSource.h>
#include <CoreSimulator.h>

using namespace asdp;
using namespace asdp::IS;

void Usage(const char* programName, int code)
{
  std::cerr << "Usage: " << programName << " [--serial N] [--width w] [--height h]"
    << " [--numCamsX x] [--numCamsY y] [--verbosity V] <ip_address>" << std::endl;
  std::cerr << "  --serial N: Set the serial number of the server to N (default 1)" << std::endl;
  std::cerr << "  --width w: Set the width of the images to w (default 1920)" << std::endl;
  std::cerr << "  --height h: Set the height of the images to h (default 1080)" << std::endl;
  std::cerr << "  --numCamsX x: Set the number of cameras in the x direction to x (default 7)" << std::endl;
  std::cerr << "  --numCamsY y: Set the number of cameras in the y direction to y (default 3)" << std::endl;
  std::cerr << "  --verbosity V: Set the verbosity level to V (default 0)" << std::endl;
  std::cerr << "  <ip_address>: The IP address or DNS name of the NIC to broadcast on" << std::endl;
  std::exit(code);
}

int main(int argc, char** argv)
{
  std::string ip_address;
  uint32_t serial_number = 1;
  size_t realParams = 0;
  uint16_t width = 1920;
  uint16_t height = 1080;
  size_t numCamsX = 7;
  size_t numCamsY = 3;
  int verbosity = 0;

  // Parse the command line arguments, with the first non-flag argument being the
  // name of the IP address to broadcast on.  There is a --serial flag to specify
  // the serial number of the server, which defaults to 1.
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--serial") {
      if (i + 1 < argc) {
        serial_number = std::stoi(argv[i + 1]);
        ++i;
      } else {
        std::cerr << "--serial flag requires an argument" << std::endl;
        return 1;
      }
    } else if (std::string(argv[i]) == "--width") {
      if (i + 1 < argc) {
        width = std::stoi(argv[i + 1]);
        ++i;
      } else {
        std::cerr << "--width flag requires an argument" << std::endl;
        return 1;
      }
    } else if (std::string(argv[i]) == "--height") {
      if (i + 1 < argc) {
        height = std::stoi(argv[i + 1]);
        ++i;
      } else {
        std::cerr << "--height flag requires an argument" << std::endl;
        return 1;
      }
    } else if (std::string(argv[i]) == "--numCamsX") {
      if (i + 1 < argc) {
        numCamsX = std::stoi(argv[i + 1]);
        ++i;
      } else {
        std::cerr << "--numCamsX flag requires an argument" << std::endl;
        return 1;
      }
    } else if (std::string(argv[i]) == "--numCamsY") {
      if (i + 1 < argc) {
        numCamsY = std::stoi(argv[i + 1]);
        ++i;
      } else {
        std::cerr << "--numCamsY flag requires an argument" << std::endl;
        return 1;
      }
    } else if (std::string(argv[i]) == "--verbosity") {
    if (i + 1 < argc) {
        verbosity = std::stoi(argv[i + 1]);
        ++i;
      } else {
        std::cerr << "--verbosity flag requires an argument" << std::endl;
        return 1;
      }
    } else if (argv[i][0] == '-' ) {
      std::cerr << "Unknown flag: " << argv[i] << std::endl;
      return 1;
    } else switch (realParams++) {
      case 0:
        ip_address = argv[i];
        break;
      default:
        Usage(argv[0], 2);
    }
  }
  if (realParams != 1) {
    Usage(argv[0], 2);
  }

  // Construct a vector of MovingBarsSource objects, one per camera.  The darkness should
  // vary from 0 to 21000 with Y, and the brightness should vary from 43000 to 65535 with X.
  std::cout << "Creating " << numCamsX << "x" << numCamsY << " moving bars sources..." << std::endl;
  std::vector< std::shared_ptr<ImageSource> > sources;
  for (size_t y = 0; y < numCamsY; y++) {
    for (size_t x = 0; x < numCamsX; x++) {
      std::cout << "." << std::flush;
      uint16_t darkValue = 21000 * y / (numCamsY - 1);
      uint16_t brightValue = 43000 + 22535 * x / (numCamsX - 1);
      sources.push_back(std::make_shared<MovingBarsSource>(width, height, 64, 20, 96, 1,
        darkValue, brightValue));
    }
    std::cout << std::endl;
  }

  // Open a server, specifying the serial number and IP address to broadcast on.
  std::cout << "Opening server on " << ip_address << " with serial number " << serial_number << std::endl;
  CoreSimulator server(serial_number, ip_address, sources, verbosity);
  if (server.GetConstructorStatus() != OKAY) {
    std::cerr << "Failed to open server: " << ErrorMessage(server.GetConstructorStatus()) << std::endl;
    return 3;
  }

  // Start the server.  Print the error message if it fails.
  std::cerr << "Server failed: " << server.run() << std::endl;

  return 0;
}
