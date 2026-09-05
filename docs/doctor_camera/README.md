# Initial Research Documents

These google docs are still editable and may be changed
- [Initial Doctor View Research Document](https://docs.google.com/document/d/1OfJ6v8o2l7Od17A9XfhbKtnIbXPvU-SVXBNiBWnNtfE/edit?tab=t.0#heading=h.opf0zp86jmkf)
- [First Prototype Camera Requirements](https://docs.google.com/document/d/19sTK0wt7DqVyj-hby_aYes9H3HzxA_KSLALwobuXmB0/edit?tab=t.0#heading=h.34oim445jif)
- [Camera's Mechanical Requirements](https://docs.google.com/document/d/15i2DtYXTZA4_sDSY5-uei4Io7vbqmeA7kBANB8NLVCQ/edit?tab=t.0)

# Current Best Source
The [camera system design](https://github.com/Waybionic/waybionic_ground_station/blob/research/doctor-camera-pipeline-gianna/docs/doctor_camera/doctor_camera_system_design.md) is so far the most up to date document surrounding the camera, constraints, and everything that mechanical and electrical has told us.

# Current Recomendation (first testing)

MVP Camera Setup: A camera that lives above the arm, preferably either in the ceiling or attached to the surgery light above the patient.

Interface: Within the ground station interface like a plugin/add-on.

Expected ROS Topics: 
  - /doctor_view/left|right/image_raw - A topic for each of the raw camera feeds (left and/or right depends on stereoscopic camera capabilities)
  - /doctor_view/left|right/camera_info - A topic that contains the calibration settings for the left and/or right cameras
  - /doctor_view/preview/image_view - A topic that gives the overall view for the operator.

Latency Target: Should remain consistently >30-50ms.

Unresolved Hardware Questions: 
  - Mechanical envelope for the camera.
  - Working with electrical to make the camera work with the robot and the ground station

Test/Purchase decision: 
  - I personally would recommend the [Orbbec Gemini 2 Camera](https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/07/Orbbec-Gemini-2-Series-DatasheetPublicV1.720240316.pdf?_gl=1*ffjzs5*_gcl_au*NzAxNTA3NDEzLjE3ODI1ODg5MzAuODcxNjc2MDcuMTc4MjU4ODk5OC4xNzgyNTg5MDAw) as it is a stereo camera and a good fit for an initial test camera.
  - For now, a webcam would also be suitable for initial, universal testing of the latency monitor and/or doctor view interface.
