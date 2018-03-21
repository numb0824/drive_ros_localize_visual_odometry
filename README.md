# Visual Odometry
Calculates vehicle odometry based camera image. Easy, fast and simple.

## Basic functionality
Detects features in an image region (e.g. road surface). Features are tracked over two consecutive images. Feature points are transformed into the world coordinate system using a homography transformation. Movement of vehicle is being computed using SVD for transformed feature points.

## Limitations
Image region where features are detected should be on a plane surface (e.g. road). This region must contain some distinct features in order to calculate the odometry information.
