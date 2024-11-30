# Update
- v1.0 (Sep 24, 2024): First public version, using GTSAM 4.0.3 in Ubuntu 20.04, pre-built binary gtsam_gnss distributed.
- v2.0 (Nov 30, 2024): Upgraded to GTSAM 4.3a in Ubuntu 22.04, [gtsam_gnss](https://github.com/taroz/gtsam_gnss) was released in a separate repository, and Windows environment support added.

# Overview
- This repository provides the source code to reproduce the results of the [Google Smartphone Decimeter Challenge 2023](https://www.kaggle.com/competitions/smartphone-decimeter-2023) on Kaggle
- Decimeter accurate position estimation was achieved using smartphone GNSS and IMU
  - Public score: 0.789 m (1st)
  - Private score:  0.928 m (2nd)
- Factor Graph Optimization of GNSS and IMU for Smartphones
- For details of the method, please refer to the following papers, presentations, and source code

# Paper/Presentation
```
Taro Suzuki, "An Open-Source Factor Graph Optimization Package for GNSS and IMU Integration in Smartphones," in Proceedings of the 37th International Technical Meeting of the Satellite Division of The Institute of Navigation (ION GNSS+ 2024), 2024.
```
- [Paper](http://www.taroz.net/paper/IONGNSS2024_GSDC.pdf)
- [Presentation](http://www.taroz.net/paper/IONGNSS2024_GSDC_ppt.pdf)

# Test environments
- Ubuntu 22.04 / Windows 11
- MATLAB R2024a
  - (Optional) Parallel Computing Toolbox

# Requirements
- [GTSAM](https://github.com/borglab/gtsam):
Factor graph optimization library. Due to a problem with the MATLAB wrapper, please clone [GTSAM from my repository](https://github.com/taroz/gtsam-4.3a) instead of the original GTSAM and build it using the following procedure. For Windows, please refer to [this](https://github.com/taroz/gtsam_gnss/BUILD_WINDOWS.md) build procedure.
```shell
sudo apt-get install -y git build-essential cmake libboost-all-dev libtbb-dev python3-pip
pip install pyparsing
git clone https://github.com/taroz/gtsam-4.3a.git
cd gtsam-4.3a
mkdir build && cd build
cmake .. -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_INSTALL_MATLAB_TOOLBOX=ON
make -j$(nproc)
sudo make install
```

- [gtsam-gnss](https://github.com/taroz/gtsam_gnss): A small set of custom factors and MATLAB wrappers that use GTSAM for GNSS processing. For Windows, please refer to [this](https://github.com/taroz/gtsam_gnss/BUILD_WINDOWS.md) build procedure.
```shell
git clone https://github.com/taroz/gtsam_gnss.git
cd gtsam_gnss
mkdir build && cd build
cmake ..
make
sudo make install
```

- [MatRTKLIB](https://github.com/taroz/MatRTKLIB):
MATLAB wrapper for RTKLIB.
```shell
git clone https://github.com/taroz/MatRTKLIB.git
```
Add the MatRTKLIB installation directory to the MATLAB search path.
- [MatlabProgressBar](https://github.com/JAAdrian/MatlabProgressBar):
tqdm like MATLAB progress bar.
```shell
git clone https://github.com/JAAdrian/MatlabProgressBar.git
```
Add the MatlabProgressBar installation directory to the MATLAB search path.

# How to run
1. Build and install all of the above dependent packages.
2. Clone this repository.
```shell
git clone https://github.com/taroz/gsdc2023.git
```
3. Download the pre-processed GSDC2023 data set (2.7GB).
```shell
cd gsdc2023
wget http://www.taroz.net/data/dataset_2023.zip
unzip dataset_2023.zip
```
4. In the case of Linux, due to the linker issue shown [here](https://github.com/borglab/gtsam/blob/develop/matlab/README.md), you need to run the following shell line before starting MATLAB from the same shell.
```shell
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
matlab
```
4. Run `run_fgo.m` in MATLAB.
5. The submission file for Kaggle is generated in `results` directory. If you have Kaggle account, let's submit the estimation result to the [Google Smartphone Decimeter Challenge 2023](https://www.kaggle.com/competitions/smartphone-decimeter-2023) and evaluate the accuracy. You should get the following results.
![](https://github.com/taroz/Misc/blob/master/data/kaggle/gsdc2023_score.jpg?raw=true)

