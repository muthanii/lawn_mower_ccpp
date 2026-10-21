FROM ros:lyrical
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip openssh-client
# Encrypts the logins at rest on the local container (see unlock-logins.sh)
RUN apt-get install -y gocryptfs
# Simulation (Gazebo through ros_gz), robot description and coverage planning tools
RUN apt-get install -y \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-fields2cover \
    python3-numpy \
    python3-matplotlib \
    python3-scipy \
    python3-shapely \
    python3-deap
# scikit-fuzzy has no apt package
RUN pip install --break-system-packages scikit-fuzzy
# LaTeX for the journal paper in paper/: latexmk and the packages that elsarticle needs
RUN apt-get install -y --no-install-recommends \
    latexmk \
    texlive-latex-recommended \
    texlive-latex-extra \
    texlive-publishers \
    texlive-fonts-recommended \
    texlive-science
ENV SHELL=/bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
# VS Code overrides the image entrypoint, so source ROS in every interactive shell
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
# The first terminal asks for the passphrase when the local vault volume is mounted and locked
RUN echo 'if mountpoint -q ~/.vault-cipher && ! mountpoint -q ~/.vault; then bash /home/ws/.devcontainer/unlock-logins.sh; fi' >> ~/.bashrc

# Claude Code (native installer)
RUN curl -fsSL https://claude.ai/install.sh | bash
# Made here so the vault volume mounted over ~/.vault-cipher is owned by the user
RUN mkdir -m 700 ~/.vault-cipher ~/.vault
ENV PATH=/home/$USERNAME/.local/bin:$PATH
CMD ["/bin/bash"]