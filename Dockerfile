FROM osrf/ros:noetic-desktop-full
ARG USER=user
ARG DEBIAN_FRONTEND=noninteractive

COPY requirements.txt requirements.txt
COPY packages.txt packages.txt

RUN apt-get update && apt-get install -y \
    $(cat packages.txt) \
    && rm -rf /var/lib/apt/lists/* && apt-get clean
RUN pip install --upgrade pip
RUN pip install -r requirements.txt
RUN rosdep update
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "alias die='tmux kill-server'" >> ~/.bashrc
RUN echo "alias sim_start='python3 ./scripts/sim_start.py && tmux attach'" >> ~/.bashrc
RUN echo "alias source_all='source /opt/ros/$ROS_DISTRO/setup.bash && source catkin_ws/devel/setup.bash'" >> ~/.bashrc 
RUN echo "alias clean='catkin_make clean'" >> ~/.bashrc
RUN echo "alias vs_start='rostopic pub /vs_start std_msgs/Empty --once"
WORKDIR /home/${USER}
RUN mkdir project428
