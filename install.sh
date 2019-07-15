# Simple apt-get/pip packages.
sudo apt-get install libopencv-dev
sudo apt-get install ros-melodic-desktop-full
sudo pip install flask
sudo pip install tensorflow

# Install tensorflow for C.
cd ~/Downloads && wget https://storage.googleapis.com/tensorflow/libtensorflow/libtensorflow-cpu-linux-x86_64-1.14.0.tar.gz
cd ~/Downloads && sudo tar -C /usr/local -xzf libtensorflow-cpu-linux-x86_64-1.14.0.tar.gz
