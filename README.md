# Autonomous Systems Design - Lab

[![ROS](https://github.com/Braafisch/ASD-Labor/actions/workflows/ros.yml/badge.svg)](https://github.com/Braafisch/ASD-Labor/actions/workflows/ros.yml)

A lab exercise designed to explore autonomous systems design via vehicle simulation in Gazebo and ROS. This repository has been forked from [osrf/car_demo](https://github.com/osrf/car_demo).

## Getting Started

### Using Rocker

To start, you'll have to install a ROS/Gazebo environment on your local system. There are various ways to do this. Our recommended way of creating a neatly packaged installation is through [Docker](https://www.docker.com/) and the [osrf/rocker](https://github.com/osrf/rocker) helper tool. Assuming you have a **Linux environment** (either in a VM or native), follow these steps:

1. Install [Docker Engine](https://docs.docker.com/engine/install/), by following the steps listed for your distro.

2. _(Optional: owners of NVIDA graphics cards)_ Install [`nvidia-container-toolkit`](https://github.com/NVIDIA/nvidia-docker) by following [these](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installation-guide) steps.

3. Install [Rocker](https://github.com/osrf/rocker) by following [these](https://github.com/osrf/rocker#installation) steps.

4. We now have the tools needed to create Docker images and run ROS/Gazebo inside a container. It's time to prepare the image:

    1. Start with a pre-defined ROS image. If you installed `nvidia-container-toolkit`, you can use option `--nvidia` to include hardware acceleration capabilities.

        ```sh
        # If you do not use an NVIDA graphics card omit option '--nvidia'
        rocker \
            --pull \
            --nvidia \
            --name 'ros-asd-lab-prototype' \
            osrf/ros:noetic-desktop-full
        ```

        Without NVIDIA support:

        ```sh
        # For non-NVIDIA users
        rocker \
            --pull \
            --name 'ros-asd-lab-prototype' \
            osrf/ros:noetic-desktop-full
        ```

        This will download the `noetic-desktop-full` ROS image, which contains a complete base install of ROS Noetic Ninjemys, from which we can expand. After pulling, extracting and building the image, Docker will present you with a command prompt running _inside_ the new container. Continue from there with the next step.

    2. Inside the container install a few more libraries and tools:

        ```sh
        sudo apt update && sudo apt install -y \
            libignition-msgs-dev \
            python3-tk \
            ros-noetic-fake-localization \
            ros-noetic-joy \
            ros-noetic-map-server \
            ros-noetic-smach \
            ros-noetic-smach-msgs \
            ros-noetic-smach-ros \
            ros-noetic-smach-viewer

        # These are optional
        sudo apt install -y \
            git \
            vim \
            zsh

        # This will clean up the package cache and command history
        sudo apt clean
        history -c
        ```

        During installation, you may be prompted to provide some information about your keyboard layout. After installation completed **do not exit out of the container**.

    3. Open a second terminal (on your host Linux, not inside the container) and commit the container we commissioned just now.

        ```sh
        # This will solidify the changes made to container 'ros-asd-lab-prototype' in
        # form of a new image named 'osrf/ros:noetic-desktop-full-asd'
        docker commit 'ros-asd-lab-prototype' osrf/ros:noetic-desktop-full-asd
        ```

        This will _bake_ the tools we've installed into a neatly packaged image called `osrf/ros:noetic-desktop-full-asd`. Use this image to execute this project.

    4. You can now stop this container. In the future, simply use image `osrf/ros:noetic-desktop-full-asd`.

        ```sh
        # in container
        exit
        ```

5. We can now use the new image to start a fully commissioned container.

    ```sh
    # If you do not use an NVIDA graphics card omit option '--nvidia'
    rocker \
        --nvidia \
        --x11 \
        --git \
        --home \
        --user \
        --name 'ros-asd-lab' \
        osrf/ros:noetic-desktop-full-asd
    ```

    We use options `--x11` to forward the graphical applications inside the container to the Linux host. With `--git` we mount our git config inside the container. Same with `--home`, which will map the home directory of the current host user inside the container. Option `--user` will create a new non-root user, which will run our simulation.

6. Inside the container create a catkin workspace and clone this repo into it:

    ```sh
    mkdir -p ~/catkin_ws/src
    git clone https://github.com/Braafisch/ASD-Labor.git ~/catkin_ws/src/asd-lab
    ```

7. Build the catkin workspace:

    ```sh
    cd ~/catkin_ws
    catkin_make
    # where $SHELL should be either bash, zsh or sh
    source "devel/setup.$(basename $SHELL)"
    ```

8. Launch the demo:

    ```sh
    cd ~/catkin_ws
    roslaunch car_demo demo_keyboard.launch
    ```

9. You can attach additional terminals to the container using:

    ```sh
    # From host Linux
    docker exec -it 'ros-asd-lab' bash # or zsh
    ```

### Using Devcontainer

While less performant due to lack of hardware acceleration, the devcontainer may provide a better development experience. It only requires you to install [Docker Engine](https://docs.docker.com/engine/) and offers full language server support for vscode. To get up and running with the devcontainer follow these steps:

1. Install [Docker Engine](https://docs.docker.com/engine/install/), by following the steps listed for your distro.

2. In vscode install the [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.

3. After installation hit `Ctrl + Shift + P` and run `>Remote-Containers: Rebuild and Reopen in Container` from the command palette. This will instruct Docker to build the provided `Dockerfile`, inside of which, we will be able to develop and test our application. Installation may take a few minutes, but only has to be done once.

4. The dev-image comes with a light desktop environment (via VNC and noVNC), meaning we can run GUI applications _inside_ the container. To view them open a webbrowser of your choice, or use `>Simple Browser: Show` from the vscode command palette, and connect to `http://localhost:6080` (password: `vscode`). Alternatively, use a [VNC viewer](https://www.realvnc.com/en/connect/download/viewer/) to connect to port `5901`.

5. To build the application follow the usual steps:

    ```sh
    cd /workspace
    catkin_make
    # where $SHELL should be either bash, zsh or sh
    source "devel/setup.$(basename $SHELL)"
    ```

6. Launch the demo:

    ```sh
    cd /workspace
    roslaunch car_demo demo_keyboard.launch
    ```

    You should now see several GUI applications pop up on noVNC.

## Contributors

This project is a fork of [osrf/car_demo](https://github.com/osrf/car_demo) with its respective authors. The devcontainer environment is largely based on the template provided by [devrt/ros-devcontainer-vscode](https://github.com/devrt/ros-devcontainer-vscode). Light desktop environment script by [microsoft/vscode-dev-containers](https://github.com/microsoft/vscode-dev-containers).

Main contributors sorted by lastname:

-   Andreas Baulig
-   Wolfgang Bradfisch
-   Sungeeta Singh

Under supervision by:

-   Thao Dang
