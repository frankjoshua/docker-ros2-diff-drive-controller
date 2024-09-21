# ROS2 diff drive controller in Docker [![](https://img.shields.io/docker/pulls/frankjoshua/ros2-diff-drive-controller)](https://hub.docker.com/r/frankjoshua/ros2-diff-drive-controller) [![CI](https://github.com/frankjoshua/docker-ros2-diff-drive-controller/workflows/CI/badge.svg)](https://github.com/frankjoshua/docker-ros2-diff-drive-controller/actions)

## Description

Reads from "/vel" topic Twist message.
Outputs transform from "odom" -> "base_link"

## Example

```
docker run -it \
    --network="host" \
    --ipc="host" \
    --pid="host" \
    frankjoshua/ros2-diff-drive-controller
```

## Building

Use [build.sh](build.sh) to build the docker containers.

<br>Local builds are as follows:

```
./build.sh -t frankjoshua/ros2-diff-drive-controller -l
```

## Template

This repo is a GitHub template. Just change the repo name in [.github/workflows/ci.yml](.github/workflows/ci.yml) and edit [Dockerfile](Dockerfile) and [README.md](README.md) to taste.

## Testing

Github Actions expects the DOCKERHUB_USERNAME and DOCKERHUB_TOKEN variables to be set in your environment.

## License

Apache 2.0

## Author Information

Joshua Frank [@frankjoshua77](https://www.twitter.com/@frankjoshua77)
<br>
[http://roboticsascode.com](http://roboticsascode.com)
