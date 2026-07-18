# Camera System Design (First Prototype Recommendations)
- [Specific Camera Needs](#specific-camera-needs)
- [Electrial Requirements](#electrical-requirements)
- [Mechanical Requirements](#mechanical-requirements)
- [Benchtop Testing](#benchtop-testing)
- [ROS2 Topic Name Proposals](#ros2-topic-name-proposal)
- [Timestamps](#timestamps)
- [Latency](#latency-measurement-method)
- [Long Term Plan](#long-term-targets)
- [Component Shortlist](#component-shortlist)

## Specific External Camera Needs

| Specific | Requirements | Why? |
| --- | --- | --- |
| Resolution | 1080p | Minimum resolution needed for surgeries |
| Frame Rate | 30fps | Minimum FPS required for surgeries |
| Shutter | Rolling | The camera model I suggest uses a rolling shutter for the colour version of the camera. |
| Lighting | Dark with a light backing it | A surgery room is usually dark with an overhead light. I am going off of that visual. |
| Internal/Tool-Side View | Not required | Would be nice to have, but for a first prototype might not be as viable |
| External Room View | Required | I think that a birds-eye view for the initial prototype would be a good idea since it doesn't interfere with the mechanical aspects too much in case they don't get back with mechanical requirements |
| Camera Type | Stereo | Gives us a good starting point on working with stereo cameras |
| Camera Model | [Gemini 2 by Orbbec](https://www.orbbec.com/products/stereo-vision-camera/gemini-2/) | A relatively inexpensive stereo USB3.0 camera that will be good for initial benchtop testing. |
| Expected number of cameras | 1 or 2 | We can have a camera directly on the arm and/or one mounted elsewhere |
| Placement | On the robot or overhead | Gives the surgeon a more active view or a more natural view. |

## Electrical Requirements
> Note: All assumptions are based off of the Gemini2 Camera by Orbbec

### Camera Requirements
| Subject | Requirement |
| --- | --- |
| Power Input | **5V** via USB-C |
| Power Consumption | Typical Average: **<2.0W**, Max Average: <2.5W, Max Peak Average: <6.5W |
| Cable/Connector | **1 - 1.5m** in length |
| CPU needed | **qual-core, 2.9GHz** |
| RAM needed | **4GB** |

## Mechanical Requirements
> Note: Mechanical has yet to get back about specific requirements... I am going off my own judgement

| Requirement | External | Endoscope |
| --- | --- | --- |
| **Approximate Location** | Around the arm | On the arm, as an extra tool |
| **Working Distance** | 3 feet* | Close to the body/internally |
| **Field of View** | Birds eye view | Small(?) |
| **Camera/Lens Envelope** | TBD** | Within the arm |
| **Adjustment Needs** | May be adjusted by zooming in and out | Can be adjusted to look around internally |
| **Occlusion Risks** | View may be blocked or impacted by the tools of the arm. This needs to be considered in mounting position and arm movement. | Since the camera is so small, tools may have a bigger impact on the occlusion |
| **Cable Routing** | Similar to motors on the arm | Similar to the motors on the arm |
| **Lighting Conditions** | Must be able to handle very harsh light | Must work in darker areas (with light mounted?)|
| **Mounting** | A removeable mount may be helpful for maintenance purpose | Directly attatched to arm |

> \* for personnel who are NOT scrubbed in... not sure if this applies for our robot

> ** waiting for mechanical to get back for a more conclusive answer

## Benchtop Testing
Using the Gemini 2 by Orbbec, we can test the camera formatting of a stereo camera and can also test how it works in harsh, high contrast lighting. First, we can test simply via our laptops to make sure the camera works. Then, we can try implementing it into our UI which can be made easier since we would have the actual camera with camera feed.

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

## ROS2 Topic Name Proposal

| Stream | Proposed Topic Name | Purpose |
| --- | --- | --- |
| Left raw image | /doctor_view/left/image_raw | Unprocessed left camera feed |
| Right raw image | /doctor_view/right/image_raw | Unprocessed right camera feed|
| Left processed image | /doctor_view/left/image_proc | Calibrated/stabilized left camera feed |
| Right processed image | /doctor_view/right/image_proc | Calibrated/stabilized right camera feed |
| Left camera info | /doctor_view/left/camera_info| This is where calibration settings for the left camera live|
| Right camera info| /doctor_view/right/camera_info | This is where calibration setting for the right camera live |
| Operator preview | /doctor_view/preview/image_view | An overall view for the operator |

## Timestamps

| Requirement | Expectation |
| --- | --- |
| header.stamp | This will display when the frame was captured, not time displayed |
| Left and right stereo sync | The two cameras in the stereo setup should have matching timestamps |
| Frame ID | Using doctor_view_right or left_optical_frame gives the 3D data for the camera views |

## Latency Measurement Method
- We can manual measure latency by subtracting the *ROS_System_Time* - *Message_Header.stamp* as long as the ROS2 time is synchronized and the header.stamp for the camera feeds are working properly.
- There's a GitHub Repo that is able to calculate latency using OpenCV and ROS, though I don't know if it is helpful for us ([ros-can-latency](https://github.com/plusk01/ros-cam-latency))
- Another manual method is using a stopwatch to view how it changes on screen vs in real time

## Long-term Targets

For long term, a **GigE** camera would be best due to its ability to send camera feeds over long distances. It has low latency and is relatively easier to implement compared to the MIPI CSI cameras. ROS2 also has drivers that are compatible with many GigE cameras.

**Stereo Camera**: Stereo cameras on the market are generally very small and are automatically synchronized. With this type of camera, the placement of the camera only needs to be considered if we are using two seperate cameras to make a stereo camera (which is harder than buying an already stereoscopic camera)

**Endoscopic Camera**: Endoscopic cameras would be very helpful for seeing closer internal footage. With this camera, we would need to decide how exaxtly we are going to implement this tool with the rest of the robotic arm. For a short-term target, it might not be possible to add this as endoscopic cameras can get expensive... but would be good to implement for a long-term project.

## Component Shortlist 
> Note: (TBD when mechanical gets back to us with more concrete materials needed)

### Camera Options 
> Note: I am leaning towards the Orbbec Gemini 2 Camera for the initial testing with decent quality... most assumptions are based off of that.

- Orbbec Gemini 2 Stereo Camera (cable included) -> ~332.15CAD
- Tara Stereo Camera (e-con Systems) -> ~325.05CAD
