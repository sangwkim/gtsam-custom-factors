# gtsam-custom-factors

The library including the custom factors.

## Setup Manual

This manual assumes using the conda environment with Python 3.8.5, but you can use another environment or Python version (>=3.6). If you are using another environment, some details might be different.

- Setup the conda environment

  ```   
  conda create -n "gtsam" python=3.8.5
  conda activate gtsam
  conda install pytorch torchvision torchaudio cudatoolkit=11.3 -c pytorch
  ```
  
- Install prerequisites for the GTSAM library

  ```
  sudo apt-get install libboost-all-dev
  sudo apt-get install cmake
  sudo ldconfig
  ```
  
  If you face a cmake version related error later, you might have to remove the current cmake and reinstall a new version of cmake (!be careful to do this if you are using ROS!, since it can break your ROS setup. In that case, please refer to: https://answers.ros.org/question/293119/how-can-i-updateremove-cmake-without-partially-deleting-my-ros-distribution/)
  
  ```
  sudo apt remove cmake
  sudo apt autoremove
  (Download a new version of cmake eg. 3.22.2)
  cd Downloads/
  sudo cp cmake-3.22.2-linux-x86_64.sh /opt/
  chmod +x /opt/cmake-3.22.2-linux-x86_64.sh 
  sudo bash /opt/cmake-3.22.2-linux-x86_64.sh
  ```
  
- Install GTSAM

  ```
  git clone https://github.com/borglab/gtsam.git
  git checkout 69a3a75
  cd gtsam
  pip install -r python/requirements.txt 
  mkdir build
  cd build
  cmake .. -DGTSAM_PYTHON_VERSION=3.8 -DGTSAM_BUILD_PYTHON=1 -DCMAKE_INSTALL_PREFIX="./install"
  make
  make install
  make python-install
  ```
  
- Install custom factors

  ```
  git clone https://github.com/sangwkim/gtsam-custom-factors.git
  cd gtsam-custom-factors
  mkdir build
  cd build
  cmake .. -DCMAKE_PREFIX_PATH="/home/sangwoon/github/gtsam/build/install"
  make
  make python-install
  ```

## DOCUMENTATION

For instruction on how to use this template, please refer to the [tutorial](TUTORIAL.md) and also https://dongjing3309.github.io/files/gtsam-tutorial.pdf.
