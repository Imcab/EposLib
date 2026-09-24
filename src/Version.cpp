#include "epos4/Version.hpp"

namespace epos4
{

Version
GetVersion()
{
  return Version{EPOS4_VERSION_MAJOR, EPOS4_VERSION_MINOR, EPOS4_VERSION_PATCH};
}

const char *
GetVersionString()
{
  return EPOS4_VERSION_STRING;
}

}  // namespace epos4
