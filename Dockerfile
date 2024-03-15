FROM osrf/ros:noetic-desktop-full
ARG USER=user
ARG DEBIAN_FRONTEND=noninteractive

COPY requirements.txt requirements.txt
COPY packages.txt packages.txt

RUN rm -rf /var/lib/apt/lists/* && apt-get clean
RUN apt-get update && apt-get install -y \
    $(cat packages.txt) 
RUN pip install --upgrade pip
RUN pip install -r requirements.txt
RUN conan config set general.revisions_enabled=1 && \
    conan profile new default --detect > /dev/null && \
    conan profile update settings.compiler.libcxx=libstdc++11 default
RUN rosdep update
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "alias die='tmux kill-server'" >> ~/.bashrc
RUN echo "alias sim_start='python3 ./scripts/sim_start.py && tmux attach'" >> ~/.bashrc
RUN echo "alias source_all='source /opt/ros/$ROS_DISTRO/setup.bash && source catkin_ws/devel/setup.bash'" >> ~/.bashrc 
RUN echo "alias clean='rm -rf /build && rm -rf /devel'" >> ~/.bashrc
RUN echo "alias install_rosdeps='rosdep install --from-paths src --ignore-src -y'" >> ~/.bashrc
RUN echo "alias careful_make='rosdep install --from-paths src --ignore-src -y && catkin_make'"
