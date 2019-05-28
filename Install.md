
## PC-MSDM installation guide (Linux and Windows)

## Dependencies
Mandatory dependencies :
 - CMake (External download)
 - Boost >= 1.59
 - Eigen 3

## Linux Ubuntu installation

### Build stage

## Windows installation

### Building stage

 - Get PC-MSDM source code using your favourite Git client

 - Run cmake-gui.exe

 - Choose 'Visual Studio 14 2015 Win64' as the compiler version

 - Where is the source code = ".../PC-MSDM"

 - Where to build the binaries = ".../PC-MSDM/build"

 - Click "Configure"

 - Click "Generate"

 - Open ".../PC-MSDM/build/PC-MSDM.sln" solution with MSVC 2015, select 'Release' mode, then generate the 'ALL_BUILD' target


## Documentation

You can build the documentation by yourself using doxygen.

## Known issues
