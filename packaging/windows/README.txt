WayBionic Ground Station $version for Windows
=============================================

This starts the ground station for the real arm. Python is included; you only need Docker.

What you need
  - Windows 11 with WSL 2 version 2.1.5 or newer ("wsl --version" shows yours; if WSL is
    missing, run "wsl --install --no-distribution" in an administrator PowerShell and restart).
  - Docker Desktop with the WSL 2 backend and Linux containers, installed.
  - The arm's carrier board (Arduino UNO R4 WiFi) on USB, and the Xbox controller.

Start it
  1. Unzip this folder somewhere permanent, for example C:\WayBionic. Do not run it from
     inside the zip.
  2. Start Docker Desktop and wait until it says it is running.
  3. Plug in the carrier and the controller.
  4. Double-click "Start WayBionic Arm.cmd".
     The first start downloads the ground station image, several GB, which can take a while.
     Later starts are quick.
  5. When RViz opens, follow its Arm setup panel: put the arm in its zero pose, press
     "Zero the arm", then press Start (Menu) on the controller. Close RViz to stop.

If something is wrong
  - Double-click "Check WayBionic Setup.cmd". It checks Docker, Python and the bridges, and
    starts nothing.
  - "The carrier bridge is not ready": plug in only the carrier, or name its port. In a
    Command Prompt in this folder: "Start WayBionic Arm.cmd" COM5
  - "unauthorized" while downloading the image: the image is not public yet. Ask a software
    lead, or log in once with: docker login ghcr.io  (a GitHub user and a token with
    read:packages).
  - RViz does not open: restart Docker Desktop, then run "wsl --update".

About this download
  Version $version. It runs this exact, tested image:
    $image
  Check the file you downloaded against SHA256SUMS on the release page, or with
    gh attestation verify <file> --repo Waybionic/waybionic_ground_station
  The release notes and docs/operator_guide.md in the repository say more.
