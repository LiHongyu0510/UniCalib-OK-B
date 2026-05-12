<div style="text-align: center;">
    <img src="../img/logo.svg" style="width: 100vw; height: auto;">
</div>

---

<h3 align="center">Tutorial: Prepare iKalibr Environment Using Docker</h3>
<p align="center">
    <a href="https://github.com/Unsigned-Long"><strong>Author » Shuolong Chen</strong></a>
</p>


---



This tutorial will guide you through the process of using the `iKalibr` project in a Docker environment, including building and compiling it. Please follow the steps below.

## Steps

### 1. Install Docker

If you haven't installed Docker yet, please do so first. Choose the appropriate installation method for your operating system ([Docker Installation Guide](https://docs.docker.com/get-docker/)).

### 2. Pull Docker Image

Once Docker is installed, pull the latest `ulong2/ie_kalibr_image:latest` image from Docker Hub (see [here](https://hub.docker.com/repository/docker/ulong2/ie_kalibr_image/general)) by running the following command:

```bash
docker pull ulong2/ie_kalibr_image:latest
```

### 3. Build the Container

After pulling the image, build the Docker container. You can customize the container name by replacing `your_container_name` with your preferred name. For example:

```bash
# Allow local Docker connections to the X server.
# This command grants local Docker containers permission to access your host machine's X11 display.
xhost +local:docker

# Run the Docker container
docker run -it --device=/dev/dri:/dev/dri --env="XDG_RUNTIME_DIR=/tmp" --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --name your_container_name ulong2/ie_kalibr_image:latest
```

This command will start the container with your custom name (`your_container_name`) and open a terminal window inside it:

+ `--device=/dev/dri:/dev/dri`: Enables hardware acceleration via the Direct Rendering Interface (DRI).
+ `--env="DISPLAY"`: Passes the display environment variable to the container.
+ `--env="XDG_RUNTIME_DIR=/tmp"`: Ensures compatibility with some GUI apps using `xdg`.
+ `--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"`: Shares the X11 socket.
+ `--volume="$HOME/.Xauthority:/root/.Xauthority:rw"`: Shares X11 authentication credentials.
+ `--name`: Assigns a name to the running container.
+ `ulong2/ie_kalibr_image:latest`: Your Docker image.

### 4. Enter the Container and Use the iKalibr

Inside the container, navigate to the `/home/iKalibr/install` directory:

```bash
cd /home/iKalibr/install
```

Then source the `setup.py`:

```
source setup.py
```

Then you can use the binary programs of `iKalibr`:

```
roslaunch ikalibr ikalibr-learn.launch
```

