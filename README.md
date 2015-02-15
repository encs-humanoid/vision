vision
======

vision system for the IEEE ENCS Humanoid Robot Project

example commands
================

Republish the compressed camera images in raw format.

```
rosrun image_transport republish compressed in:=/camera/image _image_transport:=compressed raw out:=/vision/image
```
Run the face detect node and display annotated video of detected faces.

```
rosrun vision face_detect_node.py _in:=/vision/image --show
```

Display the `/detected_face/image` stream which just contains images of cropped and normalized faces.

```
rosrun image_view image_view image:=/detected_face/image
```
