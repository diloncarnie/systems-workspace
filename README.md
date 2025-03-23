# ROS2 workspace for localization packages

## Production Setup Instructions

docker build -t my_image .devcontainer

1. Git Clone the repository in a suitable working directory

    ```bash
    git clone git@github.com:diloncarnie/systems-workspace.git
	```

1. Build the image:

    ```bash
    docker build -t systems_ws .devcontainer
	```

1. Run the image:

    ```bash
    docker run -it -u rosdev --network=host --ipc=host -v $PWD:/home/rosdev/ros2_ws systems_ws
	```