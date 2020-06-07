AR tags look like this:

![](http://wiki.ros.org/ar_track_alvar?action=AttachFile&do=get&target=artags.png)

URC uses them to mark posts and gates, so the rover needs to be able to determine their position and orientation.
Since they have predefined sizes and patterns, this can be done from an image.

Make sure to **calibrate your camera before doing AR tag detection**.
At least when using `usb_cam`, the default calibration is not an undistorted image like you might expect -- it's just a camera model with every parameter set to `0`.
This will cause AR tag detection to always fail even though the image looks fine visually.

*TODO: add more here*
