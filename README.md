# Soapy SDR plugins for UHD devices

## Dependencies

* UHD >= 4.0 - https://github.com/EttusResearch/uhd/wiki
* SoapySDR >= 0.7 - https://github.com/pothosware/SoapySDR/wiki
* C++20 compiler

## Documentation

* https://github.com/pothosware/SoapyUHD/wiki

## What's different in this fork

This fork stages modernization work for upstream review. Compared to
pothosware/SoapyUHD master:

* Requires C++20 and UHD >= 4.0 (drops UHD 3.x conditional code paths)
* Boost no longer a direct dependency (replaced with std library equivalents)
* Surfaces UHD 4.10 `mtu` stream argument
* Several latent bug fixes (`hasIQBalanceMode` fallback, `LATE_COMMAND` mapping)
* Build hygiene: target-scoped CMake, `BUILD_UHD_SUPPORT` / `BUILD_SOAPY_SUPPORT` toggles

The fork does not issue releases of its own. Tagged releases come from
upstream pothosware/SoapyUHD.

## Licensing information

* GPLv3: http://www.gnu.org/licenses/gpl-3.0.html
