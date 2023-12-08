# Notes

- Using a first generation xbox kinect is pretty old now, and requires older versions of libraries and stuff so it makes sense to use docker

## Sanity Check

Make sure the kinect is working and it can be passed through to docker:

 - `xhost +local:docker`
 - `sudo docker run --device /dev/snd --env DISPLAY --interactive --net host --privileged --rm --tty zfields/kinect-opencv-face-detect`

## Building

It is spread across multiple images, yes this wastes disk space but OpenCV takes like 3 hours to compile so it takes ages to troubleshoot
otherwise. To build:

 - `docker build . -t reg.reaweb.uk/roscv -f=Dockerfile_opencv`
 - `docker build . -t reg.reaweb.uk/rosnect -f=Dockerfile_freenect`

## Using the registry

I have set up a docker registry so you don't have to compile it yourself:

 - `docker login reg.reaweb.uk` (Message Eden if you need a username/password)
 - `docker pull reg.reaweb.uk/roscv`