# MAVLink Generator (`mavgen`)

This directory contains the MAVLink library and scripts to automatically generate it from MAVLink definitions.

In order to run it, you will need [Python and some other libraries installed](https://mavlink.io/en/getting_started/installation.html).

Then, run the appropriate script (`generate.sh` for Linux, `generate.bat` for Windows) to automatically re-generate the library.

## IMPORTANT NOTE

By default, the MAVLink library declares all of its functions as `inline`, which results in common functions being duplicated many times.
After generating the library, `protocol.h` must be modified, and all `inline` keywords removed.
This is performed automatically by `generate.sh`, but not by `generate.bat` as Windows batch files do not have an equivalent to `sed`. So, this must be done manually on Windows.
