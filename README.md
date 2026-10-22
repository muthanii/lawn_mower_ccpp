# Lawn Mower CCPP

[![CI](https://github.com/muthanii/lawn_mower_ccpp/actions/workflows/ci.yml/badge.svg)](https://github.com/muthanii/lawn_mower_ccpp/actions/workflows/ci.yml) [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.23554288.svg)](https://doi.org/10.5281/zenodo.23554288)


Intelligent complete coverage path planning (CCPP) for an autonomous 4WD skid-steer lawn mower robot, using soft computing techniques.
The project compares a boustrophedon planner with genetic algorithm or particle swarm optimization and a fuzzy logic controller against random-bounce mowing. It measures coverage, time and energy in Gazebo simulation.

## Environment

The project runs in a dev container. The container has:

- ROS 2 **Lyrical Luth**, sourced in every shell.
- Gazebo with the `ros_gz` bridge, `xacro`, `robot_state_publisher`, RViz and `teleop_twist_keyboard`.
- Fields2Cover, and the Python packages NumPy, SciPy, Matplotlib, Shapely, DEAP and scikit-fuzzy.
- Claude Code and the GitHub CLI.

You can run the container in two places:

| | GitHub Codespaces | Your computer |
| --- | --- | --- |
| You need | A GitHub account and a browser | Windows 11 with WSL2, Docker, an NVIDIA GPU and VS Code |
| GUI apps show | In a desktop in a browser tab | On the Windows desktop, through WSLg |
| Graphics | Software rendering on the CPU | Hardware OpenGL on the GPU |
| Workspace path | `/workspaces/lawn_mower_ccpp` | `/home/ws` |
| Config | `.devcontainer/codespaces/devcontainer.json` | `.devcontainer/devcontainer.json` |

## Run on GitHub Codespaces

[![Open in GitHub Codespaces](https://github.com/codespaces/badge.svg)](https://codespaces.new/muthanii/lawn_mower_ccpp?quickstart=1&devcontainer_path=.devcontainer%2Fcodespaces%2Fdevcontainer.json)

1. Click **Open in GitHub Codespaces** at the top of this page.
2. Click **Create new codespace**.

   The first start builds the container image. This takes 10 to 15 minutes.

3. To see GUI apps, open the **Ports** tab in VS Code.
4. On the **Desktop (noVNC)** row (port 6080), click the globe icon.
5. In the browser tab that opens, click **Connect**.
6. In a VS Code terminal, run:

   ```bash
   gz sim shapes.sdf
   ```

   The Gazebo window opens in the noVNC desktop.

The codespace uses a 4-core machine, because Gazebo is slow on 2 cores. A GitHub account gets free Codespaces hours each month. Then GitHub bills you. To stop the billing, stop or delete the codespace at [github.com/codespaces](https://github.com/codespaces).

## Run on your computer

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
4. When VS Code asks for a configuration, select **Local (WSL2 and NVIDIA GPU)**.
5. In a VS Code terminal, run:

   ```bash
   gz sim shapes.sdf
   ```

   The Gazebo window opens on the Windows desktop.

The container user gets the name of your WSL user, so open the folder from WSL, not from a Windows path.

## Build

ROS 2 packages go in `src/`. A new VS Code terminal opens in the workspace root. In the workspace root, build the packages and use the result in the current shell:

```bash
colcon build --symlink-install
source install/setup.bash
```

Before the first build, and after you add a dependency to a `package.xml`, install the system dependencies:

```bash
rosdep install --from-paths src --ignore-src -y
```

## Paper

The journal paper is in `paper/`. It uses the Elsevier `elsarticle` class with numbered references. The target journal is *Biomimetic Intelligence and Robotics*.

| Path | Content |
| --- | --- |
| `paper/main.tex` | Front matter, keywords and the order of the sections. |
| `paper/sections/` | One `.tex` file for each section. |
| `paper/references.bib` | References, exported from Zotero. Do not edit by hand. |
| `paper/figures/` | Figures. Plotting scripts write their output here. |

In the workspace root, build the PDF:

```bash
latexmk -pdf paper/main.tex
```

The PDF is `paper/main.pdf`. To delete the build files, run `latexmk -c paper/main.tex`.

In VS Code, the LaTeX Workshop extension builds the PDF each time you save a `.tex` file. To see the PDF, open `paper/main.tex` and press `Ctrl+Alt+V`.

To keep `paper/references.bib` up to date from Zotero:

1. Install the [Better BibTeX](https://retorque.re/zotero-better-bibtex/) plugin in Zotero.
2. Right-click the paper's collection and click **Export Collection**.
3. For the format, select **Better BibTeX** and select **Keep updated**.
4. Save the file as `paper/references.bib` in this repository.

Zotero then writes the file again each time the collection changes.

CI builds the PDF on each pull request. To download it, open the workflow run and find the **paper** artifact.

## GitHub sign-in

In Codespaces, Git and `gh` are signed in when the codespace starts. Do not do these steps.

On your computer, the container has its own SSH key. The key and the `gh` login stay in Docker volumes after a rebuild, so you sign in one time.

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
