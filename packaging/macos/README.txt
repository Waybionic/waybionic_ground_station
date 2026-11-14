WayBionic Ground Station $version for macOS
===========================================

This is the ground station's source with an installer. The Mac app runs ROS natively, so the
first start builds the workspace on your Mac; nothing here is prebuilt.

What you need
  - An Apple Silicon Mac on macOS 13 or newer. Intel Macs are untested.
  - About 10 GB of free disk space and an internet connection for the first start.
  - The arm's carrier board (Arduino UNO R4 WiFi) on USB, and a game controller.

Install it
  1. Unzip this folder.
  2. Double-click "Install WayBionic.command". If macOS says it cannot be opened because it
     is from an unidentified developer, right-click it and choose Open; on the newest macOS,
     use System Settings > Privacy & Security > Open Anyway.
     It copies this folder to ~/WayBionic and puts "WayBionic Ground Station" in
     ~/Applications.
  3. Open "WayBionic Ground Station" from ~/Applications. The first time, say yes when it
     offers to set up the environment: 30 to 60 minutes and a few GB, once.

Use it
  Plug in the carrier and the controller, then open the app. It says what to do if
  something is missing. When RViz opens, follow its Arm setup panel: put the arm in its zero
  pose, press "Zero the arm", then press Start (Menu) on the controller. Close RViz to stop.
  Its log is in ~/Library/Logs/WayBionic.

About this download
  Version $version. Check the file you downloaded against SHA256SUMS on the release page,
  or with
    gh attestation verify <file> --repo Waybionic/waybionic_ground_station
  docs/operator_guide.md in this folder says more.
