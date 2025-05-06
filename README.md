> **TO THE 2026 MARS ROVER TEAM:** I put in 30+ hours of research on robotics development best practices at the beginning of the Winter 2025 semester and learned A TON about the open-source tools and software structure industry-grade robots use. After a short coding spree, this repository is the synthesis of our existing approach with some of those best practices, including:
> - A dockerized full-task simulator for rapid development and validation
> - Industry-standard Nav2 path planning and hazard avoidance
> - Tuned and tested EKF for fusing odometry sources with GPS data
> - A fully-defined map->odom->robot TF tree and rover URDF
> - Lots and lots of QOL and scripting updates
> 
> I've successfully tested and completed GPS, aruco, and object detection legs with the rover just behind the EB (even navigating between and around the trees and posts!) with a 100% success rate. I've completed hundreds more in simulation, testing outlier cases like blocked hex points. However, when I've tried to increase the rover speed on our physical hardware to as quick as we'd need to run it at competition, the CPU can't keep up with the depth cloud processing from the ZED camera for object avoidance and we don't recognize obstacles in time before crashing into them. Even with CPU cost-cutting measures, it's clear that we simply just don't have the processing power on the Jetson Orin to run Nav2 hazard avoidance at the speed we need to to compete, and forcing all the task teams to transition to a new computer so late into the semester to stay under budget isn't a feasible option. (Admittedly, I haven't had enough rover time to test using the LiDAR instead or map the point cloud to a laser scan to lower the costmap computation load, but I don't think we should be so close to the CPU limit regardless.)
>
> <mark>In short, I think there's clear 100-point potential in using this approach and these open-source tools. I'd encourage the 2026 team to mess around with the simulator to understand a bit better how it all works, run some tests on the rover, decide if it's worth purchasing a more powerful computer (laptop with a GPU? additional SBC?), and then revisit development on this code base.</mark>
>
> I'll be starting my Master's program at BYU in Fall 2025, but would be more than happy to help answer any questions or help you get started with it. One small step for a rover, one giant leap for roverkind!
>
> Nelson Durrant

--

[Get Started](https://github.com/BYUMarsRover/marsrover_2.0?tab=readme-ov-file#get-started)

[Essential Tutorials](https://github.com/BYUMarsRover/marsrover_2.0?tab=readme-ov-file#essential-tutorials)

[How-To Videos](https://github.com/BYUMarsRover/marsrover_2.0?tab=readme-ov-file#how-to-videos)

[Contributing](https://github.com/BYUMarsRover/marsrover_2.0?tab=readme-ov-file#contributing)

--

### Get Started

> **NOTE:** Newer Macs that use ARM64 architecture (M1, M2, etc) are not able to run the Gazebo simulation due to missing package support, and have not been extensively tested.

**Windows:**

- Install WSL2 on your Windows machine by following the instructions [here](https://docs.microsoft.com/en-us/windows/wsl/install).

- Install Docker Desktop on your Windows machine by following the instructions [here](https://docs.docker.com/desktop/), and enable the WSL 2 backend by following the instructions [here](https://docs.docker.com/desktop/windows/wsl/).

- Open a WSL terminal and clone the marsrover_2.0 repo into your WSL environment using `git clone https://github.com/BYUMarsRover/marsrover_2.0.git`.

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

[Nav2 Tutorials (Gazebo Classic)](https://docs.nav2.org/setup_guides/index.html)*

[GPS Navigation w Nav2 Tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html)

**All of the dependencies for these tutorials are pre-included in the Docker container, and we've mounted 'tutorial_ws' as a dedicated ROS2 tutorial workspace.*

--

### How-To Videos

[Running the Simulation](https://www.youtube.com/watch?v=uB8YDXrGd-g)

--

### Contributing

- **Create a new branch.** Our main code branches are protected, so you'll need to create a new branch. Name your branch with a combination of your name and the feature you are working on (i.e. nelson/repo-docs).

- **Make your changes.** Develop and debug your new feature or bug fix.

  > **NOTE:** If you need to add dependencies, add them to the Dockerfile in your branch and test building the image locally. Once your pull request is merged into main, GitHub CI will automatically build and push the new Docker image to DockerHub.

- **Rebase your branch often.** Keep your branch up-to-date with main by rebasing. This will help prevent merge conflicts down the road.

- **Submit a pull request.** Once you have made and tested your changes, create a new pull request. Get another member of the team to review and approve it, and you can merge your new code into the main branch.

--

Created by Nelson Durrant, Feb 2025.
