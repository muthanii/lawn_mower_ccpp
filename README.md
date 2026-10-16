# ROS 2 Quickstart

[![CI](https://github.com/muthanii/ros2_quickstart/actions/workflows/ci.yml/badge.svg)](https://github.com/muthanii/ros2_quickstart/actions/workflows/ci.yml)

A ready-to-use ROS 2 development workspace in a VS Code Dev Container.
It uses ROS 2 **Lyrical Luth** and runs on Windows through WSL2, with GPU-accelerated GUI tools such as RViz.

## What you get

- A Docker image based on `ros:lyrical`, with a non-root user and `sudo`.
- ROS sourced in every shell.
- VS Code extensions for C++, CMake, Python and ROS.
- GUI apps that show on your Windows desktop through WSLg.
- Hardware OpenGL on an NVIDIA GPU.
- `rosdep` dependency install when the container is first made.
- Gazebo with the `ros_gz` bridge, plus `xacro`, `robot_state_publisher`, RViz and `teleop_twist_keyboard`.
- Coverage path planning tools: Fields2Cover, and the Python packages NumPy, SciPy, Matplotlib, Shapely, DEAP (genetic algorithms) and scikit-fuzzy (fuzzy logic).
- Claude Code and the GitHub CLI preinstalled.

## Requirements

- Windows 11 with WSL2 and WSLg.
- Docker (Docker Desktop with the WSL2 backend, or Docker Engine inside WSL).
- An NVIDIA GPU with the current Windows driver. See [Other setups](#other-setups) if you do not have one.
- VS Code with the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.

## Setup

1. Clone the repository into your WSL file system:

   ```bash
   git clone https://github.com/muthanii/ros2_quickstart.git ~/ros2_ws
   ```

2. Open the folder in VS Code from the WSL terminal:

   ```bash
   code ~/ros2_ws
   ```

3. When VS Code asks, click **Reopen in Container**. You can also run **Dev Containers: Reopen in Container** from the Command Palette.

The first build takes some minutes. After it finishes, the workspace is at `/home/ws` in the container.

The container user gets the same name as your WSL user. The name comes from the `USER` variable in WSL. Because of this, open the folder from WSL, not from a Windows path.

## Check the install

Open a terminal in VS Code and run:

```bash
ros2 doctor --report
```

The base image has no demo packages. Install them to test topics and GUI output:

```bash
sudo apt-get install -y ros-lyrical-demo-nodes-cpp ros-lyrical-turtlesim
```

Start a talker in one terminal:

```bash
ros2 run demo_nodes_cpp talker
```

Start a listener in a second terminal:

```bash
ros2 run demo_nodes_cpp listener
```

The listener prints each message that the talker sends.

To test the GUI, run:

```bash
ros2 run turtlesim turtlesim_node
```

A turtle window opens on your Windows desktop.

## Use Git and GitHub

The container has an SSH client and the GitHub CLI (`gh`).

### Sign in one time

The container has its own SSH key, separate from the key in WSL. The key and the `gh` login are kept in Docker volumes, so you sign in one time and they stay after a rebuild.

1. In the container, run:

   ```bash
   gh auth login
   ```

   When `gh` asks for the Git protocol, select **SSH**. When it asks to upload an SSH key, select `~/.ssh/id_ed25519.pub`.

2. Check that SSH works:

   ```bash
   ssh -T git@github.com
   ```

   GitHub replies with your user name. Then you can clone with SSH, for example `git clone git@github.com:OWNER/REPO.git`.

Each time the container starts, `.devcontainer/setup-ssh.sh` makes the key if there is none and adds it to GitHub if it is not there yet. The key shows in your [GitHub SSH keys](https://github.com/settings/keys) as **ros2-devcontainer**. To remove the key from the container, delete the `ssh-config-*` Docker volume and that key on GitHub.

## Make your first package

1. Go to the source folder:

   ```bash
   cd /home/ws/src
   ```

2. Make a Python package with one node:

   ```bash
   ros2 pkg create --build-type ament_python --node-name my_node my_package
   ```

3. Go back to the workspace root and build:

   ```bash
   cd /home/ws
   colcon build --symlink-install
   ```

4. Source the workspace overlay:

   ```bash
   source install/setup.bash
   ```

5. Run the node:

   ```bash
   ros2 run my_package my_node
   ```

For a C++ package, use `--build-type ament_cmake` in step 2.

If a package needs system dependencies, add them to its `package.xml`. Then run:

```bash
rosdep install --from-paths src --ignore-src -y
```

## Repository layout

```text
.
├── .github/
│   ├── release-please/     # Release Please config, manifest and version
│   └── workflows/          # CI, PR title check and release workflows
├── .devcontainer/
│   ├── Dockerfile          # Image: ROS 2 Lyrical, user, sudo, Claude Code
│   └── devcontainer.json   # Container settings, mounts, GPU, extensions
├── .vscode/
│   └── settings.json       # Tells the ROS extension which distro to use
└── src/                    # Your ROS 2 packages go here
```

`colcon` makes `build/`, `install/` and `log/` in the workspace root. Git ignores these folders.

## Container settings

| Setting | Value | Why |
| --- | --- | --- |
| `ROS_DOMAIN_ID` | `42` | Keeps this workspace apart from other ROS 2 systems on the network. |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `LOCALHOST` | Nodes find only nodes on this computer. |
| `--net=host`, `--ipc=host`, `--pid=host` | on | DDS discovery and shared memory work as on the host. |
| `GALLIUM_DRIVER=d3d12` | on | Mesa renders OpenGL on the GPU through WSL. |

To talk to robots or computers on your network, remove `ROS_AUTOMATIC_DISCOVERY_RANGE`. Then set the same `ROS_DOMAIN_ID` on all machines.

## Other setups

**No NVIDIA GPU.** In `devcontainer.json`, remove `--gpus=all` and `MESA_D3D12_DEFAULT_ADAPTER_NAME`. GUI apps still run on WSLg.

**Native Linux, not WSL.** Remove the `/usr/lib/wsl` mount and the `LD_LIBRARY_PATH`, `GALLIUM_DRIVER` and `MESA_D3D12_DEFAULT_ADAPTER_NAME` variables. Run `xhost +local:` on the host so the container can open windows.

**No Claude Code.** Remove the `curl ... install.sh` line from the `Dockerfile`. Also remove the `anthropic.claude-code` extension, the `CLAUDE_CONFIG_DIR` variable and the `claude-code-config` volume from `devcontainer.json`.

After you change these files, run **Dev Containers: Rebuild Container**.

## Troubleshooting

**GUI windows do not open.** In WSL, run `echo $DISPLAY`. It must not be empty. Run `wsl --update` in PowerShell and restart WSL.

**Nodes do not see each other.** Make sure all terminals use the same `ROS_DOMAIN_ID`. Run `ros2 daemon stop`, then try again.

**`Permission denied` in `/home/ws`.** Run `sudo chown -R $(whoami) /home/ws`.

## Learn more

- [ROS 2 documentation](https://docs.ros.org/en/lyrical/)
- [ROS 2 tutorials](https://docs.ros.org/en/lyrical/Tutorials.html)
- [Dev Containers documentation](https://code.visualstudio.com/docs/devcontainers/containers)

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).

## License

[MIT](LICENSE)
