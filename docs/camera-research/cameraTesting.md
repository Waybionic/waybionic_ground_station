# Camera Testing

## Notes:
- Data was redone with a Logitech Brio 100 Full HD Webcam that shoots in a maximum resolution of 1080p.

## Five Minute Experiment Results
### Ran at 1280x720 at and average of 30fps
- Taking a picture of the timer and the doctor view, there is a **0.54ms** difference in the timer.
	- The monitor measures from when the bridge reads a frame from the camera to when the ground station receives it, while the stopwatch also includes the camera and the screen. The stopwatch test only covers visible delays, not internal.
- The minimum camera delay was **0.17ms**
- The maximum camera delay was **1031.21ms**
- The average camera delay was **153.53ms**
- There were 10833 frames captured, and over five minutes (+51.93s for set-up/take-down time), the average frame rate was **30.78fps**
- The average jitter was **16.51 ms for capture jitter** and **57.69 ms for receiving jitter**.

## Five Minute Experiment Results with Demo
### Ran at 1280x720 at an average of 31.59fps
- Taking a picture of the timer and the doctor view, there is a **0.56ms** difference in the timer.
	- The monitor measures from when the bridge reads a frame from the camera to when the ground station receives it, while the stopwatch also includes the camera and the screen. The stopwatch test only covers visible delays, not internal.
- The minimum camera delay was **0.13ms**
- The maximum camera delay was **1069.63ms**
- The average camera delay was **511.56ms**
- There were 10818 frames captured over 342.48 seconds (5 minutes + time for setup/takedown) and the average frame rate was **31.59fps**
- The average jitter was **16.80 ms for capture jitter** and **54.51 ms for receiving jitter**.

## Result Discussion
Overall, the results with the demo running in the background was not too different from the one without the demo.

- The overall average camera delay is significantly higher with the demo running in the background compared to the one without.
	- This could be due to the constant movement needing to be captured by the camera, thus creating more delay compared to a stopwatch moving. 
- The average jitter for capture and receiving weren't very different in the two experiments.
- The experiment with the demo seemed to capture more frames per second compared to the one without. Although, this difference in frame rate is not by much.
## Disconnect/Reconnect Behaviour
- Doctor view went black and stopped sending frames. There was a camera warning in the alert section.
	- The camera bridge in the terminal showed a message saying "Ground Station disconnected; retrying"
- On reconnect, the camera simply came back to the doctor view and resumed as if nothing happened.
	- The camera bridge in the terminal showed a message saying "Connected to ground"
 - The latency monitor picks up on negative frames, missing and stale frames and shows it in the alert section.

