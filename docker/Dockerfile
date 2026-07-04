FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

WORKDIR /waybionic_ws
COPY . /waybionic_ws/src

RUN apt update
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install

COPY ./docker/entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
