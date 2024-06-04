# Setup Docker

## Build Docker Image
In current path, run the following command to build the docker Image with the name `ubuntu20.04`.
```bash
docker build . -t ubuntu20.04
```

Or you can directly run:
```bash
bash build.bash
```

> **Note:**
> - You can change the name to whatever you like, but remember to update the new name in the bash file.

## Start Docker container
In current path, run the follwowing command to start the docker container with the name `grasp_control_container`:
```bash
bash ubuntu20.04.bash
```