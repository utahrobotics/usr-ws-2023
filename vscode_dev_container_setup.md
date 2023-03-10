### VSCode Dev Container Setup

Required Extentions:

- Dev Containers
- Remote - SSH
- Remove Development
- WSL (for windows users)

Setup:

1) Clone Repository.
2) Confirm Required Extentions are installed and DockerEngine (Or Docker Desktop) is running.
3) On the bottom left of the VSCode window, there should be a little icon that has two arrow icons (kinda like: ><), click that button.
4) After, a menu should pop up with loads of options. Click the options that says "Reopen In Container".
5) Visual Studio should restart and pull/build the docker image. This can take some time as it requires a decent internet connection.
6) Once the image is built, the new VSCode window should appear full inside the container.
