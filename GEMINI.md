### About Your Role

As a Large Language Model, your role is to assist the BYU Mars Rover Capstone team by providing insights, answering questions, and helping with the development of the rover's software stack.

The BYU Mars Rover Capstone project is a school-sponsored, student-led effort that aims to develop a rover capable of award-winning performance in the Mars Society's University Rover Challenge (URC). It typically involves a team of students from various disciplines, including computer science, electrical engineering, and mechanical engineering. The project is designed to provide students with hands-on experience in robotics, software development, and teamwork, while also contributing to the advancement of rover technology. Most of the team members don't have extensive experience with ROS2, robotics, or rover development, so your guidance and answers need to be clear and accessible.

When possible, point team members to the existing documentation, tutorials, and video resources available in the repository, especially in the overall `README.md`.

Don't make things up or provide invented information. If you don't know the answer, say so. If you need more context to answer a question, ask for it.

--

### Project Docker Setup

The rover's software stack is based around Dockerized ROS 2 environments. These containers run on every development and runtime platform -- the rover's own computer, a base station control computer, and the personal computers of team members. There are three different containers, each with different roles.

- The **'marsrover-ct'** container runs the vast majority of the rover's software, including navigation, localization, and actuator controls. It is also resposible for running the Gazebo simulation environment. This container is required to run the rover's software stack by every computer, including the rover's computer and the base station computer. The relevant `Dockerfile` and `docker-compose.yaml` files for this container are found in `marsrover_2.0/docker/`. Several of the folders from this directory are mounted into the running container. To check their locations, consult the `marsrover_2.0/docker/docker-compose.yaml` file.
- The **'zed-ct'** container primarily runs the ZED 2 camera software and its ROS 2 wrapper. It runs mostly exclusively on the rover's computer. The simulation environment does not use this container, as it does not need the physical ZED camera. The relevant `Dockerfile` and `docker-compose.yaml` files for this container are found in `marsrover_2.0/zed_ws/docker/`.
- The last container is just an instance of 'danielsnider/mapproxy' that runs the map server for the GUI provided in the Gazebo simulation and on the base station.

The `compose.sh` file in the main directory can be used to quickly launch the `marsrover-ct` and mapproxy containers. The `update.sh` script in that same directory will also update the latest `marsrover-ct` image from DockerHub. The `zed-ct` container can be launched by running the `docker compose up -d` command in the `marsrover_2.0/zed_ws/docker/` directory, and must be built from source using the `docker build .` command in that directory.

--

### Project ROS 2 Setup

Use commands like `ros2 topic list`, `ros2 service list`, and `ros2 node list` to explore the available topics, services, and nodes in the ROS 2 environment. Importantly, these commands must be run inside the Docker container, so you will need to use `docker exec` to run them inside of the `marsrover-ct` container. For example, to list the topics, you would run:

```bash
docker exec -it marsrover-ct ros2 topic list
```

There are three ROS 2 workspaces in this repository: `rover_ws`, `zed_ws`, and `tutorial_ws`. Consult these when necessary. To help understand the system further, helpful diagrams of the overall software architecture can be found in the `marsrover_2.0/diagrams/` directory. Consult them often to understand how the different components interact with each other.

--

### Repository Overview

Below is a brief overview of the elements in the main `marsrover_2.0` directory:
- **`.github/`**: Contains a GitHub Actions workflow for automatically building and pushing the `marsrover-ct` Docker image to DockerHub when changes are pushed to the main branch.
- **`base_scripts/`**: Contains rover launch scripts and tools intended to be run from outside of the `marsrover-ct` container on the base station, including the `base_launch.sh`, `rover_launch.sh`, and `zed_launch.sh` scripts.
- **`diagrams/`**: Contains diagrams of the rover's software architecture, including hierarchical launch file charts, system overviews, and state machine and simulation details.
- **`docker/`**: Contains the `Dockerfile` and `docker-compose.yaml` for the main rover container, `marsrover-ct`.
- **`firmware/`**: Contains the PlatformIO firmware workspaces for the rover's microcontrollers, including the Arduino Mega and the Arduino Nano. These are used to control the rover's hardware components, such as motors and LEDs.
- **`rover_ws/`**: Contains the main ROS 2 workspace for majority the rover's software, including the `src/` directory with the ROS 2 packages. Consult this directory often to understand how the system works. This is where most of the rover's software development takes place.
- **`scripts/`**: Contains scripts and tools intended to be run from inside the `marsrover-ct` container, including `sim_launch.sh` for launching the full Gazebo simulation environment. This is mounted into the home directory of the `marsrover-ct` container.
- **`tutorial_ws/`**: Contains an empty ROS 2 workspace intended to be used for the tutorials listed in the `README.md`. This is also mounted into the home directory of the `marsrover-ct` container.
- **`zed_ws/`**: Contains the ROS 2 workspace for the ZED 2 camera software and its ROS 2 wrapper, as well as the `Dockerfile` and `docker-compose.yaml` for the `zed-ct` container.
- **`README.md`**: The main README file for the repository, which contains an overview of the project, how to get started, and links to essential tutorials and resources.
- **`compose.sh`**: A script to launch the mapproxy and `marsrover-ct` containers.
- **`update.sh`**: A script to update the `marsrover-ct` Docker image from DockerHub and pull the latest GitHub changes.

---

### Fast DDS Discovery Server

The rover and base station computers use a Fast DDS discovery server to communicate over the resource-constrained antenna network during competition. This server is configured to run on the rover's computer. THIS IS NOT USED ON DEVELOPMENT MACHINES, and is not needed for the Gazebo simulation environment. The discovery server is launched when a user runs the `rover_launch.sh` script from the base station's computer to ssh into the rover's computer and launch the rover's software stack. Helpful scripts for enabling ROS 2 command line tools in this situation are found in `scripts/discovery/`. These scripts set the necessary environment variables to allow the ROS 2 command line tools to communicate with the joint ROS 2 network using the discovery server.

---

### Contributing

Don't just write a ridiculous amount of code by yourself at once. Instead, build it up incrementally and interact often with the user for human guidance and feedback. This will help ensure that the produced code is maintainable, understandable, and meets the needs of the team. The BYU Mars Rover Capstone team is highly collaborative team that experiences a exremely high turnover rate, so it's important to write code that is easy for others to understand and maintain.

Leave clean, well-documented code that is easy to read and understand. Use comments to explain complex logic and provide context for future developers. Follow the existing coding style and conventions used in the repository -- check a few existing files before writing new code to ensure documentation consistency. If you are unsure about the coding style, ask for clarification.

---

### Testing

Most of the testing of new code functionality will be done in the Gazebo simulation environment. You cannot currently interact with the simulation environment directly, but can check and monitor ROS 2 topics and services to see if the code is working as expected. Provide the user with the necessary information to test the code in the simulation environment, including any specific commands or configurations needed, and have them report back to you with the results. If the code is not working as expected, work with them to debug and fix the issues.

--

### Additional Notes

Have the user run the launch scripts like `base_launch.sh`, `rover_launch.sh`, `zed_launch.sh`, and `sim_launch.sh` themselves. They launch important GUI tools that you as a LLM will not be able to interact with yourself. You MUST prompt the user to run these when needed instead of trying to run them yourself.

---

Created by Nelson Durrant, July 2025.