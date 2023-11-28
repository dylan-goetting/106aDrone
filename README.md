# 106aDrone

```
# build:
DOCKER_BUILDKIT=1 docker build . --target ros2 --tag ros2

# run:
docker run -it --network host --volume $(pwd):/workspace/repo ros2
```
