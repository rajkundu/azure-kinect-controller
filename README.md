# Azure Kinect DK Controller Application
This C++ application is used to control one or more Azure Kinect DK cameras in parallel.

## Prerequisites
- [Azure Kinect SDK (K4A)](https://github.com/microsoft/Azure-Kinect-Sensor-SDK)
  - This application has was built using v1.4.1; other versions of the SDK may or may not be compatible.
  - Once installed, add the appropriate `bin` folder (e.g. `C:\Program Files\Azure Kinect SDK v1.4.1\sdk\windows-desktop\amd64\release\bin`) to your PATH
- [OpenGL v3.3+](https://www.khronos.org/opengl/wiki/Getting_Started#Downloading_OpenGL)

## Downloading
- (TODO: Upload & link binaries)

## Building from Source
### Dependencies
- All prerequisites (see above)
- [libjpeg-turbo](https://libjpeg-turbo.org)
  - Instructions for building `libjpeg-turbo` can be found [here](https://github.com/libjpeg-turbo/libjpeg-turbo/blob/main/BUILDING.md)
  - Once installed, adjust paths in `./CMakeLists.txt` as necessary
  - Add the appropriate `bin` folder (e.g., `C:\libjpeg-turbo-gcc64\bin`) to your PATH
- [BS::thread_pool](https://github.com/bshoshany/thread-pool) - used for multithreading
  - Download [`BS_thread_pool.hpp`](https://raw.githubusercontent.com/bshoshany/thread-pool/master/BS_thread_pool.hpp) into this directory (`./BS_thread_pool.hpp`)
- [rigtorp::SPSCQueue](https://github.com/rigtorp/SPSCQueue) - used for communication between image processing & rendering threads
  - Download [`SPSCQueue.h`](https://raw.githubusercontent.com/rigtorp/SPSCQueue/master/include/rigtorp/SPSCQueue.h) into this directory (`./SPSCQueue.h`)

### Compilation
- This application is currently being written on Windows and has been tested using both MinGW-w64/GCC and MSVC. After installing all dependencies (below), **remember to adjust `./CMakeLists.txt` as necessary to match your installation.**
