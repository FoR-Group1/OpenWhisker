# Notes

- Using a first generation xbox kinect is pretty old now, and requires older versions of libraries and stuff so it makes sense to use docker

## Sanity Check

Make sure the kinect is working and it can be passed through to docker:

 - `xhost +local:docker`
 - `sudo docker run --device /dev/snd --env DISPLAY --interactive --net host --privileged --rm --tty zfields/kinect-opencv-face-detect`

