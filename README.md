# Visual Odometry
Calculates vehicle odometry based on camera image. Easy, fast and simple.

## Basic functionality
Detects features in an image region (e.g. road surface). Features are tracked over two consecutive images. Feature points are transformed into the world coordinate system using a homography transformation. Movement of vehicle is being computed using SVD for transformed feature points.

## Limitations
Image region where features are detected should be on a plain surface (e.g. road). This region must contain some distinct features in order to calculate the odometry information.

## Credits
Most parts are ported from [LMS visual odometry](https://github.com/lms-org/visual_odometry_from_road)
- [Phibedy](https://github.com/orgs/tum-phoenix/people/Phibedy)
- [fabolhak](https://github.com/orgs/tum-phoenix/people/fabolhak)
- [mherb](https://github.com/orgs/tum-phoenix/people/mherb)
