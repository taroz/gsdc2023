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
- Paper link: TBD
- Presentation link: TBD

# Test environments
- Ubuntu 20.04
- MATLAB R2024a
  - (Optional) Parallel Computing Toolbox

# Requirements
- [GTSAM](https://github.com/borglab/gtsam):
Factor graph optimization library. Due to a problem with the MATLAB wrapper, it is necessary to build GTSAM version 4.0.3 by following steps. Please refer to the GTSAM github for details on the build procedure.
```shell
sudo apt-get install build-essential cmake libboost-all-dev libtbb-dev
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.0.3
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_UNSTABLE:OPTION=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS:OPTION=OFF -DGTSAM_INSTALL_MATLAB_TOOLBOX:OPTION=ON
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
1. Install all of the above dependent packages
2. Clone this repository
```shell
git clone https://github.com/taroz/gsdc2023.git
```
3. Download the pre-processed GSDC2023 data set (2.7GB)
```shell
cd gsdc2023
wget http://www.taroz.net/data/dataset_2023.zip
unzip dataset_2023.zip

```
4. Run `run_fgo.m` in MATLAB
5. The submission file for Kaggle is generated in `results` directory. If you have Kaggle account, let's submit the estimation result to the [Google Smartphone Decimeter Challenge 2023](https://www.kaggle.com/competitions/smartphone-decimeter-2023) and evaluate the accuracy. You should get the following results.
![](https://github.com/taroz/Misc/blob/master/data/kaggle/gsdc2023_score.jpg?raw=true)

