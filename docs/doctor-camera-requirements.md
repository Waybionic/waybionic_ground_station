# Doctor Camera Requirements

## 1. Summary

The doctor-view camera should prioritize **depth, clarity, low latency, and reliability**.

| Area | Requirement |
|---|---|
| Camera type | **Stereo** for depth perception |
| Resolution | **1080p minimum**, **4K target** (as long as latency stays acceptable) |
| Frame rate | **30 fps minimum**, **60 fps preferred** |
| Sensor | **CMOS** as it is the default sensor |
| Shutter | **Global shutter** if budget allows |
| Lighting | Assume surgical/auxiliary illumination. Camera should still be good in low-light just to be sure |
| Latency | Target should stay low enough for real-time surgeon control, around **>30ms - 50ms** |

## 2. Core Requirements

| Requirement | Priority | Notes |
|---|---|---|
| Stereo view | High | Gives the surgeon depth perception. Monocular is simpler and cheaper but lacks true depth, making it difficult for surgeries. |
| 1080p-4K video | High | Do not go below 1080p. Use 4K only if frame rate and latency remain acceptable. |
| 30-60 fps | High | 30 fps is the minimum. 60 fps is preferred for smoother motion. |
| Global shutter | Medium/High | Best for moving tools and accurate positioning. Rolling shutter is acceptable only if testing confirms low distortion. |
| FOV/zoom | Medium | Start with a natural, eye-like view. Allow zoom or alternate lenses if possible. |
| Low-light handling | Medium | Stereo and high frame rate may need more light. Add light if needed. |
| CMOS sensor | Low | Since it is the default for modern cameras, finding a camera with CMOS will not be difficult. |

## 3. Camera Type Decision

### Preferred: Stereo

Stereo cameras use two viewpoints to provide depth perception. This is the preferred approach for a doctor/surgeon view since they would need a sense of depth when performing surgery remotely.

### Fallback: Monocular

Monocular cameras are cheaper and simpler, but they only provide 2D video/imagine. Depth would have to be estimated in software, which is less reliable for precise surgical viewing.

## 4. Image Quality Targets

| Setting | Target |
|---|---|
| Minimum resolution | 1080p |
| Preferred resolution | 4K if latency allows |
| Minimum frame rate | 30 fps |
| Preferred frame rate | 60 fps |
| FOV | Natural/human-like starting point (around 50mm). Could make this adjustable if possible |
| Lighting | Test with expected surgical lighting and optional auxiliary light |

## 5. Shutter

### Preferred: Global Shutter

Use a **global shutter** if budget allows. It captures the frame all at once, which reduces motion distortion and helps with moving tools, synchronization, and accurate positioning.

### Fallback: Rolling Shutter

Use a **rolling shutter** as a cost fallback. It must be tested for tool-motion distortion, flicker under artificial lighting, and synchronization issues.

## 6. Camera Interface Comparison

| Interface | Main Strength | Main Limitation |
|---|---|---|
| **USB3** | Easiest and fastest to test locally | Short cables. Not ideal as a final space architecture |
| **GigE / 10GigE** | Better for longer routed cables and multi-camera setups | More setup/network tuning |
| **MIPI CSI** | Very low latency to robot-side compute | Very short cables and board-specific setup |

