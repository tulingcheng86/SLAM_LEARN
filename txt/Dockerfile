FROM jianchong/vins-fusion

RUN apt-get update && apt-get install -y vim

RUN apt-get install -y ninja-build exiftool protobuf-compiler libeigen3-dev genromfs xmlstarlet libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev python-pip python3-pip gawk

RUN python -m pip install --upgrade setupTools && python -m pip install --upgrade pip
RUN pip2 install pandas jinja2 pyserial cerberus pyulog==0.7.0 numpy toml pyquaternion empy pyyaml

RUN pip3 install packaging numpy empy toml pyyaml jinja2 pyargparse

# 安装ROS
# 初始化rosdep
RUN sudo apt-get install python-pip
RUN sudo pip install rosdepc
RUN sudo rosdepc init
RUN rosdepc update
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
#RUN . ~/.bashrc
RUN apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential


RUN apt-get remove -y gazebo*
RUN apt-get remove -y libgazebo*
RUN apt-get remove -y ros-melodic-gazebo*

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

RUN apt-get update
RUN apt-get install -y gazebo9
RUN apt-get install -y libgazebo9-dev
RUN apt upgrade -y 

RUN apt-get install -y ros-melodic-moveit-msgs ros-melodic-object-recognition-msgs ros-melodic-octomap-msgs ros-melodic-camera-info-manager ros-melodic-control-toolbox ros-melodic-polled-camera ros-melodic-controller-manager ros-melodic-transmission-interface ros-melodic-joint-limits-interface

RUN git clone https://gitee.com/robin_shaun/XTDrone.git
RUN cd XTDrone && git submodule update --init --recursive
RUN ls /XTDrone/sitl_config/gazebo_ros_pkgs
RUN apt-get update && \
    apt-get install -y python-catkin-tools
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN mkdir -p ~/catkin_ws1/src
RUN cd ~/catkin_ws1 \
    && cp -r /XTDrone/sitl_config/gazebo_ros_pkgs ~/catkin_ws1/src/ \
    && catkin build

RUN cd ~/.gazebo && git clone https://github.com/osrf/gazebo_models.git && mv gazebo_models models

RUN apt install -y ros-melodic-mavros ros-melodic-mavros-extras

RUN wget https://gitee.com/robin_shaun/XTDrone/raw/master/sitl_config/mavros/install_geographiclib_datasets.sh
RUN chmod a+x ./install_geographiclib_datasets.sh
RUN ./install_geographiclib_datasets.sh

RUN git clone https://github.com/PX4/PX4-Autopilot.git
RUN mv PX4-Autopilot PX4_Firmware
RUN cd PX4_Firmware && git checkout -b xtdrone/dev v1.11.0-beta1 && git submodule update --init --recursive

RUN cd PX4_Firmware && make px4_sitl_default gazebo

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default" >> ~/.bashrc
RUN echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware" >> ~/.bashrc
RUN echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo" >> ~/.bashrc

RUN cd XTDrone && git submodule update --init --recursive

RUN cp sensing/gimbal/gazebo_gimbal_controller_plugin.cpp ~/PX4_Firmware/Tools/sitl_gazebo/src/
RUN cp sitl_config/init.d-posix/rcS ~/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/
RUN cp sitl_config/worlds/* ~/PX4_Firmware/Tools/sitl_gazebo/worlds/
RUN cp -r sitl_config/models/* ~/PX4_Firmware/Tools/sitl_gazebo/models/
RUN cp -r sitl_config/launch/* ~/PX4_Firmware/launch/

RUN cd ~/.gazebo/models/ && rm -r stereo_camera/ 3d_lidar/ 3d_gpu_lidar/ hokuyo_lidar/

RUN pip3 uninstall empy
RUN pip3 install empy==3.3.2

RUN cd ~/PX4_Firmware && make px4_sitl_default gazebo

