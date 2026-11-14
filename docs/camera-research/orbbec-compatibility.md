# Orbbec Gemini 2 Camera and OpenCV Compatibility

OpenCV and the Orbbec cameras do work well together! There is some documentation on how to use the Orbbec camera with OpenCV, and it's very similar to our bridge as it currently stands.

Here is the documentation on the [Orbbec camera and OpenCV](https://docs.opencv.org/4.13.0/d1/d33/tutorial_orbbec_uvc.html)

> Here is the official documentation for the [Orbbec Gemini 2 camera](https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/07/Orbbec-Gemini-2-Series-DatasheetPublicV1.720240316.pdf?_gl=1*ffjzs5*_gcl_au*NzAxNTA3NDEzLjE3ODI1ODg5MzAuODcxNjc2MDcuMTc4MjU4ODk5OC4xNzgyNTg5MDAw)
> Here is the [Orbbec Gemini 2 ROS2 Wrapper Manual](https://doc.orbbec.com/documentation/Orbbec%20Gemini%20330%20Series%20Documentation/ROS2%20Wrapper%20User%20Manual)

## Limitations
- The maximum RGB resolution would be 1920 x 1080 and its maximum frame rate would be up to 30fps.  
  - It can also be set with a RBG resolution of 1280 x 720 and its maximum frame rate with that resolution would be 60fps.

- The Orbbec Gemini 2 has a depth capturing range of 0.15m - 10m with its ideal range being between 0.2m - 5m.

- The only thing we would have to change within the bridge would be the handling of the stereo camera. This can be found inside of the documentation/tutorial I found.

- A C++ implementation seems to have an extra feature that flattens colours. I don't think we need this, so we can continue the bridge in Python.

## Expected Formats
- Listed on the documentation sheet, the expected formats are MJPEG (JPEG images) or YUYV.
  - YUYV might be a bit more difficult to integrate, so I would say stick with MJPEG formatting.

## Bridge Modifications
- We would most likely have to tweak the camera bridge a bit in order to account for the stereo image capturing and the Orbbec camera in general
  
- There are a few resources out there covering how to properly use the 3D/Stereo option for the Orbbec Gemini 2, so we can always refer to those when setting up the camera with the Ground Station

## Unverified Hardware Questions
- Where will this be mounted?
- How is the proper stereo image going to be conveyed?
  - Usually, stereo images or video frames are viewed like a binocular (two eyes at once). What kind of mechanism are we going to implement for the doctor to properly view this?
  - Would we use VR headsets? 

