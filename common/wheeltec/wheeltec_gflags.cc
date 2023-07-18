#include "common/wheeltec/wheeltec_gflags.h"

DEFINE_string(raw_cgi610_bestpos, "/zhito/rawcgi610/bestpos",
              "zhito::drivers::gnss:rawcgi610::Bestpos");
DEFINE_string(raw_cgi610_bestvel, "/zhito/rawcgi610/bestvel",
              "zhito::drivers::gnss:rawcgi610::BestVel");
DEFINE_string(raw_cgi610_gpgga, "/zhito/rawcgi610/gpgga",
              "zhito::drivers::gnss:rawcgi610::GPGGA");
DEFINE_string(raw_cgi610_gpvtg, "/zhito/rawcgi610/gpvtg",
              "zhito::drivers::gnss:rawcgi610::GPVTG");
DEFINE_string(raw_cgi610_heading2, "/zhito/rawcgi610/heading2",
              "zhito::drivers::gnss:rawcgi610::Heading2");
// DEFINE_string(raw_wheeltec_rawimu, "/civ/raw_wheeltec/rawimu",
//               "civ::drivers::imu:CorrectedImu");
DEFINE_string(raw_wheeltec_rawimu, "/WHEELTEC",
              "civ::drivers::imu:CorrectedImu");