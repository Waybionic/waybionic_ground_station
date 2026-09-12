FROM ros@sha256:386d06ec6d4188f731bae5678e07b4cb64a4e4d4152090c0bd1f881dcf7706f5 AS development

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR /waybionic_ws

COPY waybionic_description/package.xml src/waybionic_ground_station/waybionic_description/package.xml
COPY waybionic_bringup/package.xml src/waybionic_ground_station/waybionic_bringup/package.xml
COPY waybionic_rviz_plugins/package.xml src/waybionic_ground_station/waybionic_rviz_plugins/package.xml

RUN apt-get update \
	&& rosdep update --rosdistro jazzy \
	&& rosdep install --from-paths src --ignore-src --rosdistro jazzy -y \
	&& rm -rf /var/lib/apt/lists/*

RUN if ! id -u ubuntu >/dev/null 2>&1; then \
	  useradd --create-home --user-group --uid 1000 --shell /bin/bash ubuntu; \
	fi \
	&& install -d -o ubuntu -g ubuntu /home/ubuntu/.waybionic_ws/{build,install,log} \
	&& ln -s /home/ubuntu/.waybionic_ws/{build,install,log} /waybionic_ws/

COPY --chmod=755 docker/entrypoint.sh /entrypoint.sh

RUN printf '\nsource /opt/ros/jazzy/setup.bash\nif [[ -f /waybionic_ws/install/setup.bash ]]; then\n  source /waybionic_ws/install/setup.bash\nfi\n' >> /etc/bash.bashrc

USER ubuntu
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

FROM development AS build

COPY . /waybionic_ws/src/waybionic_ground_station

RUN source /opt/ros/jazzy/setup.bash \
	&& colcon build --symlink-install

FROM build AS test

RUN source install/setup.bash \
	&& colcon test --return-code-on-test-failure --event-handlers console_direct+ \
	&& colcon test-result --verbose
