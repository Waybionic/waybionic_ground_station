# Orbbec Gemini 2 Camera and OpenCV Compatibility
> Not quite done.

OpenCV and the Orbbec cameras do work well together! There is some documentation on how to use the Orbbec camera with OpenCV, and it's very similar to our bridge as it currently stands.

Here is the documentation on the [Orbbec camera and OpenCV](https://docs.opencv.org/4.13.0/d1/d33/tutorial_orbbec_uvc.html)

## Limitations
- The maximum resolution would be 1280 x 800 and its maximum frame rate would be up to 30fps.

- The only thing we would have to change within the bridge would be the handling of the stereo camera. This can be found inside of the documentation/tutorial I found.

- A C++ implementation seems to have an extra feature that flattens colours. I don't think we need this, so we can continue the bridge in Python.
