# ROSCon 2026 Workshop: Motion Planning Fundamentals with Moveit®

## Prerequisites

1. A Linux-based OS is recommended (Ubuntu 22.04 or 24.04). No ROS setup is required locally — everything runs in a Docker container.
2. Docker Engine and the Docker Compose plugin. Installation instructions are on the linked pages.
3. VS Code with the Dev Containers extension. You'll use it to edit files inside the container.
4. About 20 GB of free disk space. The image is roughly 7 GB and Docker needs room to unpack it.

## Setup instructions

> Some commands below (`rosds`, `rosd`, and `cbnt`) are preinstalled into the Docker container and are part of the [RosTeamWorkspace](https://github.com/b-robotized/ros_team_workspace) package.

1. Download the compose file and pull the image.

   Do this as soon as you can, preferably before the conference. It's a large download and conference WiFi may not be kind to it. Pull again closer to the date to pick up any updates.

   ```
   wget https://tinyurl.com/roscon2026moveitWorkshop -O docker-compose.yaml
   docker compose pull
   ```

2. Start the environment.

    ```
    xhost +local:docker
    docker compose up -d
    docker compose exec workshop bash
    ```
    You are now inside the container. The remaining steps run there.

3. Get the latest exercise material.

      ```
      rosds && cd ./roscon2026_motion_planning_workshop
      git pull
      ```

4. Build the workspace.

    ```
    rosd
    cbnt
    ```
    
    `rosd` jumps to the workspace root and `cbnt` builds it. Everything is pre-built in the image, so this should finish in a few seconds.

5. Run a test simulation.

    ```
    ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
    ```
    
    A Gazebo window and an RViz window should open, both showing a UR5e arm. In the RViz MotionPlanning panel you can drag the interactive marker, click `Plan & Execute`, and watch the arm move in Gazebo.

6. Shut down.

    Press `Ctrl+D` in the launch terminal, then from your host:
    
    ```
    docker compose stop
    ```

## Notes

- To use the container again later, `docker compose start` and `docker compose exec workshop bash`. Your work inside the container is preserved between stop and start.

  `docker compose down` deletes the container and everything you changed inside it. Only use it if you want to start over from a clean image.

- To edit files, attach VS Code to the running container: open the Command Palette, choose `Dev Containers: Attach to Running Container`, and pick `roscon2026_motion_planning`.

  If these steps go smoothly, you are all set. If not, please raise an issue in this repository and we'll help you as soon as we can!
