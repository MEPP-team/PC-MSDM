
## PC-MSDM installation guide (Linux and Windows)

## Dependencies
Mandatory dependencies :
 - CMake (External download)
 - Eigen 3 (Included)
 - Nanoflann (Included)

## Linux Ubuntu installation

### Building stage

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

The documentation can be build using [Doxygen](http://www.doxygen.nl/).

## Known issues
