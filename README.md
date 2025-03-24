# ROS2 workspace for localization packages Test

## Production Setup Instructions

docker build -t my_image .devcontainer

1. First clone the repo into a suitable working directory

1. Build the image:

    ```bash
    docker build -t systems_ws .devcontainer
	```

1. Run the image:

    ```bash
    docker run -it -u rosdev --network=host --ipc=host -v $PWD:/home/rosdev/ros2_ws systems_ws
	```


docker run -it -u rosdev --network=host --ipc=host -v $PWD:/home/rosdev/ros2_ws --privileged --device=/dev/gpiochip0 --device=/dev/gpiomem0 --device=/dev/mem  systems_ws

docker run -it -u rosdev --network=host --ipc=host -v $PWD:/home/rosdev/ros2_ws -v /dev:/dev --device-cgroup-rule='b *:* rwm' --device-cgroup-rule='c *:* rwm' systems_ws