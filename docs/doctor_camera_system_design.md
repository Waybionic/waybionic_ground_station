# Camera System Design (First Prototype Recommendations)
- [Overview](#overview)
- [Specific Camera Needs](#specific-camera-needs)
- [Electrial Requirements](#electrical-requirements)
- [Mechanical Requirements](#mechanical-requirements)
- [Benchtop Testing](#benchtop-testing)
- [ROS2 Topic Name Proposals](#ros2-topic-name-proposal)
- [Timestamps](#timestamps)
- [Latency](#latency-measurement-method)
- [Component Shortlist](#component-shortlist)

## Overview

The camera should be a stereo camera with 1080p resolution, 30 frames per second at minimum with a 50mm FOV, and must work under harsh lighting (bright lights with high contrast).

The camera itself, I think, should be placed on a seperate mount from the arm as attatching the camera to the arm itself might lead to more issues with wiring and mechanical aspects.
It can be attatched to give a birds-eye view of the surgery with capabilities to zoom in and out while retaining resolution (perhaps an entire new arm dedicated to the camera?).

> Note: Much of these were assumptions based off of my own judgement. Mechanical will be consulted, though they are not present at the time of this document being made.

## Specific Camera Needs

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

| Subject | Requirement |
| --- | --- |
| Power Input | **5V** via USB-C |
| Power Consumption | Typical Average: **<2.0W**, Max Average: <2.5W, Max Peak Average: <6.5W |
| Cable/Connector | **1 - 1.5m** in length |
| CPU needed | qual-core, 2.9GHz |
| RAM needed | **4GB** |

## Mechanical Requirements
> Note: Mechanical has yet to get back about specific requirements

Endoscopic Camera Wiring: Similar to motor wiring

Mounting:

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

## Component Shortlist 
> Note: (TBD when mechanical gets back to us)
- Orbbec Gemini 2 Stereo Camera (cable included) -> ~332.15CAD 
