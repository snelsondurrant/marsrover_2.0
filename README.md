[Get Started](https://github.com/BYUMarsRover/marsrover_2.0?tab=readme-ov-file#get-started)

[Essential Tutorials](https://github.com/BYUMarsRover/marsrover_2.0?tab=readme-ov-file#essential-tutorials)

[How-To Videos](https://github.com/BYUMarsRover/marsrover_2.0?tab=readme-ov-file#how-to-videos)

[Software Diagrams](https://github.com/BYUMarsRover/marsrover_2.0?tab=readme-ov-file#software-diagrams)

[Contributing](https://github.com/BYUMarsRover/marsrover_2.0?tab=readme-ov-file#contributing)

--

### Get Started

![ASCII Art](https://github.com/user-attachments/assets/b9a8e8b7-9f1c-44c6-b7b7-b2add7c5d788)

> **NOTE:** Newer Macs that use ARM64 architecture (M1, M2, etc) are not able to run the Gazebo simulation due to missing package support, and have not been extensively tested.

**Windows:**

- Install WSL2 on your Windows machine by following the instructions [here](https://docs.microsoft.com/en-us/windows/wsl/install).

- Install Docker Desktop on your Windows machine by following the instructions [here](https://docs.docker.com/desktop/), and enable the WSL2 backend by following the instructions [here](https://docs.docker.com/desktop/windows/wsl/).

- Open a WSL2 terminal and clone the marsrover_2.0 repo into your WSL2 environment using `git clone https://github.com/BYUMarsRover/marsrover_2.0.git`.

- Run `cd marsrover_2.0 && bash compose.sh` to pull and launch the latest Docker image from DockerHub.

**Linux:**

- Install Docker Engine on your Linux machine by following the instructions [here](https://docs.docker.com/engine/install/ubuntu/).

- Open a terminal and clone the marsrover_2.0 repo into your Linux environment using `git clone https://github.com/BYUMarsRover/marsrover_2.0.git`.

- Run `cd marsrover_2.0 && bash compose.sh` to pull and launch the latest Docker image from DockerHub.

--

### Essential Tutorials

> **NOTE:** <mark>We would **strongly encourage** each year's team to take a couple of months at the beginning of the first semester and simply work together through these tutorials before diving into software development.</mark> It may not seem romantic, but I promise it'll be worth it.

**General:**

[Linux CLI Tutorial](https://linuxjourney.com/lesson/the-shell)

[GitHub Basics Tutorial](https://docs.github.com/en/get-started/start-your-journey/hello-world)

[Docker Concepts and Tutorials](https://docs.docker.com/get-started/introduction/whats-next/)

[ROS2 Concepts](https://docs.ros.org/en/humble/Concepts/Basic.html)

[ROS2 CLI Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)*

[ROS2 Code Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)*

[ROS2 Discovery Server Tutorial](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html)*

[Robotics in ROS2 Tutorial](https://github.com/henki-robotics/robotics_essentials_ros2/tree/main)

**Autonomy:**

[Nav2 Concepts](https://docs.nav2.org/concepts/index.html)

[Nav2 Tutorials (Gazebo Classic)](https://docs.nav2.org/setup_guides/index.html)*

[GPS Navigation w Nav2 Tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html)

**Servicing:**

[MoveIt2 Concepts](https://moveit.picknik.ai/humble/doc/concepts/concepts.html)

[MoveIt2 Tutorials](https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html)*

> **NOTE:** Although MoveIt2 is preinstalled, the standard MoveIt2 tutorials require removing the Docker container's MoveIt2 installation and rebuilding it from source using colcon.

**All of the dependencies for these tutorials are preinstalled in the Docker container, and we've mounted 'tutorial_ws' as a dedicated ROS2 tutorial workspace.*

--

### How-To Videos

[Running the Simulation](https://youtu.be/sQmkes66p2w)

![Running the Simulation](https://github.com/user-attachments/assets/d8187778-da11-4283-baff-5666b0f0cbd0)

[Localization Debugging](https://youtu.be/r0U6DLKrkSk)

![Localization Debugging](https://github.com/user-attachments/assets/1a3ea3f7-68d4-4e45-81bb-1a60fd6bbf9c)

--

### Software Diagrams

> **NOTE:** To make changes to a diagram, edit the code in the corresponding `diagrams/*/*.txt` file in the repository and visit [this website](https://www.mermaidchart.com/) to generate a new PNG file.

**Autonomy:**

[Software Overview](diagrams/autonomy/software_overview.png)

[Launch Structure](diagrams/autonomy/launch_structure.png)

[State Machine](diagrams/autonomy/state_machine.png)

[Sim Software Overview](diagrams/autonomy/simulation/sim_software_overview.png)

[Sim Launch Structure](diagrams/autonomy/simulation/sim_launch_structure.png)

> **TODO:** Add Servicing, Delivery, and Science software diagrams here.

--

### Contributing

- **Create a new branch.** This repository's main branch is protected, so you'll need to create a new branch. Name your branch with a combination of your name and the feature you are working on (e.g. nelson/repo-docs).

- **Make your changes.** Develop and debug your new feature or bug fix. Add good documentation.

  > **NOTE:** If you need to add dependencies, add them to the Dockerfile in your branch and test building the image locally. Once your pull request is merged into main, GitHub CI will automatically build and push the new Docker image to DockerHub.

- **Rebase your branch often.** Keep your branch up-to-date with main by rebasing. This will help prevent merge conflicts down the road.

- **Submit a pull request.** Once you have made and tested your changes, create a new pull request. Get another member of the team to review and approve it, and you can merge your new code into the main branch.

--

Created by Nelson Durrant, Feb 2025.
