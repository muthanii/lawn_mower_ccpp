# Lawn Mower CCPP

[![CI](https://github.com/muthanii/lawn_mower_ccpp/actions/workflows/ci.yml/badge.svg)](https://github.com/muthanii/lawn_mower_ccpp/actions/workflows/ci.yml)

Intelligent complete coverage path planning (CCPP) for an autonomous 4WD skid-steer lawn mower robot, using soft computing techniques.
The project compares a boustrophedon planner with genetic algorithm or particle swarm optimization and a fuzzy logic controller against random-bounce mowing. It measures coverage, time and energy in Gazebo simulation.

## Environment

The workspace runs in a VS Code Dev Container on Windows, through WSL2. The container has:

- ROS 2 **Lyrical Luth**, sourced in every shell.
- Gazebo with the `ros_gz` bridge, `xacro`, `robot_state_publisher`, RViz and `teleop_twist_keyboard`.
- Fields2Cover, and the Python packages NumPy, SciPy, Matplotlib, Shapely, DEAP and scikit-fuzzy.
- Hardware OpenGL on the NVIDIA GPU. GUI windows show on the Windows desktop through WSLg.
- Claude Code, the GitHub CLI and an SSH client.

## Setup

You need Windows 11 with WSL2, Docker, an NVIDIA GPU and VS Code with the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.

1. Clone the repository into the WSL file system:

   ```bash
   git clone https://github.com/muthanii/lawn_mower_ccpp.git ~/lawn_mower_ccpp
   ```

2. Open the folder from the WSL terminal:

   ```bash
   code ~/lawn_mower_ccpp
   ```

3. Click **Reopen in Container**.

The workspace is at `/home/ws` in the container. The container user gets the name of your WSL user, so open the folder from WSL, not from a Windows path.

To check that the GUI works, run `gz sim shapes.sdf`.

## Build

ROS 2 packages go in `src/`. To build them and use the result in the current shell:

```bash
cd /home/ws
colcon build --symlink-install
source install/setup.bash
```

Before the first build, and after you add a dependency to a `package.xml`, install the system dependencies:

```bash
rosdep install --from-paths src --ignore-src -y
```

## GitHub sign-in

The container has its own SSH key. The key and the `gh` login stay in Docker volumes after a rebuild, so you sign in one time.

1. Run `gh auth login`.
2. For the Git protocol, select **SSH**.
3. For the SSH key, select `~/.ssh/id_ed25519.pub`.
4. Run `ssh -T git@github.com` to check the connection.

At each start, `.devcontainer/setup-ssh.sh` makes the key if there is none and adds it to GitHub as **ros2-devcontainer**.

## Container settings

| Setting | Value | Why |
| --- | --- | --- |
| `ROS_DOMAIN_ID` | `42` | Keeps this workspace apart from other ROS 2 systems on the network. |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `LOCALHOST` | Nodes find only nodes on this computer. |
| `--net=host`, `--ipc=host`, `--pid=host` | on | DDS discovery and shared memory work as on the host. |
| `GALLIUM_DRIVER=d3d12` | on | Mesa renders OpenGL on the GPU through WSL. |

To connect to a real robot, remove `ROS_AUTOMATIC_DISCOVERY_RANGE` and use the same `ROS_DOMAIN_ID` on both machines.

## Troubleshooting

**GUI windows do not open.** In WSL, run `echo $DISPLAY`. It must not be empty. Run `wsl --update` in PowerShell and restart WSL.

**Nodes do not see each other.** Make sure all terminals use the same `ROS_DOMAIN_ID`. Run `ros2 daemon stop`, then try again.

**`Permission denied` in `/home/ws`.** Run `sudo chown -R $(whoami) /home/ws`.

## Commits and releases

See [CONTRIBUTING.md](CONTRIBUTING.md).

## License

[MIT](LICENSE)
