# Camera System Design (First Prototype Recommendations)
- [Confirmed Requirements](#confirmed-requirements)
- [Assumed Requirements](#assumed-requirements)
- [Benchtop Testing](#benchtop-testing)
- [ROS2 Details](#ros2-details)
- [Long Term Plan](#long-term-targets)
- [MVP Short Term Plan](#mvp-short-term-targets)
- [Component Shortlist](#component-shortlist)

## Confirmed MVP Short Term Requirements

### Camera (External)
| Specific | Requirement | Why? |
| --- | --- | --- |
| Resolution | 1080p | Minimum resolution needed for surgeries |
| Frame Rate | 30fps | Minimum FPS required for surgeries |
| Shutter | Rolling | Rolling shutters are acceptable for a first prototype since it is cheaper and still effective for now |
| FOV | 200° Horizontally and 135° Vertically. | It is most similar to the field of view of a human|
| Camera Type | Stereo | Gives us a good starting point on working with stereo cameras |
| Latency Needs | 0-200ms | 0-200ms is ideal for performing remote surgeries, however 300ms-700ms is also acceptable for performing surgeries|

### Camera Electrical Requirements
> Note: All assumptions are based off of the Gemini2 Camera by Orbbec

| Subject | Requirement |
| --- | --- |
| Power Input | **5V** via USB-C |
| Power Consumption | Typical Average: **<2.0W**, Max Average: <2.5W, Max Peak Average: <6.5W |
| Cable/Connector | **1 - 1.5m** in length |
| CPU needed | **qual-core, 2.9GHz** |
| RAM needed | **4GB** |

## Assumed Requirements

### Mechanical Requirements
> Note: Mechanical has yet to get back about specific requirements... I am going off my own judgement

| Requirement | External | Endoscope |
| --- | --- | --- |
| **Approximate Location** | Around the arm | On the arm, as an extra tool |
| **Working Distance** | 3 feet* | Close to the body/internally |
| **Camera/Lens Envelope** | TBD** | Within the arm |
| **Adjustment Needs** | May be adjusted by zooming in and out | Can be adjusted to look around internally |
| **Occlusion Risks** | View may be blocked or impacted by the tools of the arm. This needs to be considered in mounting position and arm movement. | Since the camera is so small, tools may have a bigger impact on the occlusion |
| **Cable Routing** | Similar to motors on the arm | Similar to the motors on the arm |
| **Lighting Conditions** | Must be able to handle very harsh light | Must work in darker areas (with light mounted?)|
| **Mounting** | A removeable mount may be helpful for maintenance purpose | Directly attatched to arm |

> \* for personnel who are NOT scrubbed in... not sure if this applies for our robot

> ** waiting for mechanical to get back for a more conclusive answer

## Benchtop Testing
Using the Gemini 2 by Orbbec, we can test the camera formatting of a stereo camera and can also test how it works in harsh, high contrast lighting. First, we can test simply via laptops to make sure the camera works. Then, we can try implementing it into our UI which can be made easier since we would have the actual camera with camera feed.

We should keep these in mind/log information during the first test:
- frame timestamp
- subscriber receive time
- frame number
- resolution
- FPS
- latency per frame

Then, we should note down:
- average latency
- minimum latency
- maximum latency
- jitter

## ROS2 Details

### Topic Names

| Stream | Proposed Topic Name | Purpose |
| --- | --- | --- |
| Left raw image | /doctor_view/left/image_raw | Unprocessed left camera feed |
| Right raw image | /doctor_view/right/image_raw | Unprocessed right camera feed|
| Left processed image | /doctor_view/left/image_proc | Calibrated/stabilized left camera feed |
| Right processed image | /doctor_view/right/image_proc | Calibrated/stabilized right camera feed |
| Left camera info | /doctor_view/left/camera_info| This is where calibration settings for the left camera live|
| Right camera info| /doctor_view/right/camera_info | This is where calibration setting for the right camera live |
| Operator preview | /doctor_view/preview/image_view | An overall view for the operator |

### Timestamps

| Requirement | Expectation |
| --- | --- |
| header.stamp | This will display when the frame was captured, not time displayed |
| Left and right stereo sync | The two cameras in the stereo setup should have matching timestamps |
| Frame ID | Using doctor_view_right or left_optical_frame gives the 3D data for the camera views |

### Latency Measurement Method
- We can manual measure latency by subtracting the *ROS_System_Time* - *Message_Header.stamp* as long as the ROS2 time is synchronized and the header.stamp for the camera feeds are working properly.
- There's a GitHub Repo that is able to calculate latency using OpenCV and ROS, though I don't know if it is helpful for us ([ros-can-latency](https://github.com/plusk01/ros-cam-latency))
- Another manual method is using a stopwatch to view how it changes on screen vs in real time

## Long Term Targets

### Long Term Camera Requirements

| Specific | Requirement | Notes |
| --- | --- | --- |
| Camera Type |  Stereo | Gives the doctor a good sense of depth perception |
| Resolution | 4k | Would give the doctor a clearer vision of what they are doing |
| Frame Rate | 60fps | Giving a higher frame rate would make surgeries feel closer to in-person surgeries |
| Shutter | Global | A global shutter would be best for moving tools and accurate positioning |
| FOV | 200° Horizontally and 135° Vertically. | It is most similar to the field of view of a human |

### Long Term Camera Architecture

For long term, a **GigE** camera would be best due to its ability to send camera feeds over long distances. It has low latency and is relatively easier to implement compared to the MIPI CSI cameras. ROS2 also has drivers that are compatible with many GigE cameras.

**Endoscopic Camera Option**: Endoscopic cameras would be very helpful for seeing closer internal footage. With this camera, we would need to decide how exaxtly we are going to implement this tool with the rest of the robotic arm. For a short-term target, it might not be possible to add this as endoscopic cameras can get expensive... but would be good to implement for a long-term project.

## MVP Short Term Targets

Camera Architecture: We should use a stereo USB3 camera because it is simple to set up and attatch to the arm. The stereo would give us a good idea of the depth perception for future cases.

Mounting: The external camera will be directly on the arm and the compute location would be around the base of the arm with everything else.*

Expected Cable Length: The Orbbec Gemini 2 cable length is 1-1.5m in length... we may be able to purchase a longer cable if necessary

Lighting: Lighting will be provided by surgical lights and can be tested in harsher conditions

Mechanical Requirements: Must provide an encasing/place for the camera to be mounted and where wires can be fed

Electrical Requirements: Must look over the electrical requirements and adapt that to the total electrical requirements of the arm

## Component Shortlist 
> Note: (TBD when mechanical gets back to us with more concrete materials needed)

### Camera Options 
> Note: I am leaning towards the Orbbec Gemini 2 Camera for the initial testing with decent quality... most assumptions are based off of that.

- Orbbec Gemini 2 Stereo Camera (cable included) -> ~332.15CAD
- Tara Stereo Camera (e-con Systems) -> ~325.05CAD
