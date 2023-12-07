# 1. Code style guide
- We follow [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) as our fundamental coding standard, incorporating any exceptions or additional rules documented in this Code of Conduct.

## 1.1. File extentions
- For code and header files, please use the extensions `cpp` and `hpp`, respectively.

# 2. Code check tool
## Environment setup
- `clang-format` 17.0.2
- `cpplint` 1.6.1
- `cppcheck` 1.90
- `doxygen` v1.9.8
- `cmake` >= 3.16.3
### Install CMake >= 3.16.3
* Download CMake from https://cmake.org/download/ for the suitable platform
``` shell
# ex) Current Make version is 3.16.3
cd ~
wget https://github.com/Kitware/CMake/releases/download/v3.16.3/cmake-3.16.3-linux-x86_64.tar.gz
tar -xzvf cmake-3.16.3-linux-x86_64.tar.gz

# Backup previous CMake just in case
sudo mv /usr/bin/cmake /usr/bin/cmake.old
sudo mv /usr/bin/ctest /usr/bin/ctest.old
sudo mv /usr/bin/cpack /usr/bin/cpack.old

# Make softlink to /usr/bin
sudo ln -s ${HOME}/cmake-3.16.3-linux-x86_64/bin/cmake /usr/bin/cmake
sudo ln -s ${HOME}/cmake-3.16.3-linux-x86_64/bin/ctest /usr/bin/ctest
sudo ln -s ${HOME}/cmake-3.16.3-linux-x86_64/bin/cpack /usr/bin/cpack
```


## 2.1. Formating
```shell
./run_check.sh format
```

## 2.2. Linting
```shell
./run_check.sh lint
```

## 2.3. Documentation
```shell
./run_check doc_check
```

## 2.4. Formating, Linting, and documentation checking all at once
```shell
./run_check.sh all
```

# 3. Unit testing
```shell
cd ~/catkin_ws
catkin test slambox_ros_driver
```

# 4. Commit
* **DO NOT** commit on `main` branch. Make a new branch and commit and create a PR request.
* Formatting, linting, documentation check, and unit test is auto-called when you `push`.
We advise you to run `./run_check all` occasionally.

# 5. Documentation
## Install Doxygen
```shell
cd ~
git clone -b Release_1_9_8 https://github.com/doxygen/doxygen.git
cd doxygen
mkdir build
cd build
cmake -G "Unix Makefiles" .. 
make
make install
```

## 5.1. Generate API document
* This will generate `html` directory.
```shell
doxygen
```

## 5.2. Run local documentation web server
```shell
cd html
python -m http.server
```

