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
ENV SHELL=/bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
# VS Code overrides the image entrypoint, so source ROS in every interactive shell
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Claude Code (native installer). ~/.claude is created here so the volume mounted over it is owned by the user
RUN curl -fsSL https://claude.ai/install.sh | bash && mkdir -p ~/.claude
# Made here so the GitHub CLI and SSH volumes mounted over them are owned by the user
RUN mkdir -p ~/.config/gh && mkdir -m 700 ~/.ssh
ENV PATH=/home/$USERNAME/.local/bin:$PATH
CMD ["/bin/bash"]