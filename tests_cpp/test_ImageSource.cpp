/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <ImageSource.h>

int main(int argc, char** argv)
{
  std::string ret = asdp::IS::Test();
  if (ret.size() > 0) {
    std::cerr << "Error: " << ret << std::endl;
    return 1;
  }
  std::cout << "Success" << std::endl;
  return 0;
}
