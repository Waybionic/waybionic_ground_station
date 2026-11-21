# McGill conference poster — software section

Khuzaymah / software team. Conference week of 3 November 2026. This folder is the software column only: how the ROS 2 ground station controls the arm, and how that software has been tested.

**Git branch:** `docs/poster-software-khuzaymah` (from `origin/main` at `cf3e887`).
**Scope:** documentation and figures under `docs/poster/` only. No production code or tests were changed.

---

## Authoritative poster files

The finalized software-section layout is the PowerPoint. Use these three files for review and print:

| File | Role |
| --- | --- |
| [figures/waybionic_software_section.pptx](figures/waybionic_software_section.pptx) | Editable final design |
| [figures/waybionic_software_section.pdf](figures/waybionic_software_section.pdf) | Print/share PDF (one page, 12 × 19.6 in) |
| [figures/waybionic_software_section.png](figures/waybionic_software_section.png) | High-resolution preview (3600 × 5880, 300 dpi) |

How to use them:

1. Edit the PowerPoint if the conference template or column width changes.
2. Re-export PDF/PNG from PowerPoint after any edit (`File → Export → PDF`, or slide export as PNG).
3. Place the PDF (or PNG) into the full conference poster when the shared template is ready.

The official WayBionic logo in the PowerPoint is the same file as [assets/waybionic-logo.png](assets/waybionic-logo.png) (website blob `7239fa956e8a44b97ccb8227bcda6f9e8d632b1d`). It is not a redraw.

Doctor View is a labeled **pending** slot. Do not fill it with a generated image.

---

## Standalone architecture figure

These match the architecture inset used on the poster and can be reused elsewhere:

| File | Role |
| --- | --- |
| [figures/waybionic_software_architecture.svg](figures/waybionic_software_architecture.svg) | Editable vector diagram |
| [figures/waybionic_software_architecture.png](figures/waybionic_software_architecture.png) | High-resolution render (5200 × 2100) |
| [diagram_spec.md](diagram_spec.md) | Accuracy brief for that figure |

---

## Evidence and copy (for reviewers)

| File | Use |
| --- | --- |
| [architecture_research.md](architecture_research.md) | Implemented ROS 2 path vs `main`, sim vs hardware |
| [software_content.md](software_content.md) | Four technical paragraphs (teleop/incision, collisions, CAN, doctor view + diagnostics) |
| [verified_results.md](verified_results.md) | Traceable numbers, sources, and results that must not be printed as headlines |
| [screenshots_guide.md](screenshots_guide.md) | How the existing shots were taken, and what a lead still needs for Doctor View |
| [screenshots/rviz_simulated_arm.png](screenshots/rviz_simulated_arm.png) | Real RViz capture, simulated drives |
| [screenshots/diagnostics_panel.png](screenshots/diagnostics_panel.png) | Real Engineering Monitor, live `/diagnostics` |
| [screenshots/rviz_ground_station_overview.png](screenshots/rviz_ground_station_overview.png) | Full-window reference; not placed on the poster |
| [assets/LOGO_SOURCE.txt](assets/LOGO_SOURCE.txt) | Provenance of the official logo |

Poster-safe numbered results (software / virtual / test-video only):

1. **81,421** MKS frame cross-checks, 0 mismatches — PR #40, software only.
2. **19 + 11** virtual-CAN tests passed — PR #45, virtual CAN only.
3. **30 fps** test-video stream; **2 ms** capture-to-arrival delay at 1280×720 — PR #36. Not a final camera, and not end-to-end display latency.

Held off the poster, documented in `verified_results.md`: 1,305 tests (later `main` CI failure on one simulated health test); 25 mm incision kinematics (simulation only).

---

## October 24 checklist

| Requirement | Status |
| --- | --- |
| One system architecture figure | Present on the poster and as a standalone SVG/PNG |
| 3–4 sentence explanations: teleop/incision, collision/limits, CAN, doctor view | Present on the poster; longer wording in `software_content.md` |
| 3–5 numbered results from merged PRs | Three printed, each sourced; qualifications on the cards |
| Screenshots: RViz, diagnostics, doctor view | RViz and diagnostics are real. Doctor View is outstanding |
| Software vs powered hardware | Stated on the poster and in `verified_results.md` |
| Production code untouched | Yes |

---

## Outstanding

- Doctor View screenshot: a team lead needs to supply a real RViz frame or an approved test-video capture.
- Team-lead review of wording and placement in the full conference poster.
- Conference poster template / final printed dimensions, if they differ from this 12 × 19.6 in software column.
- No merged powered five-drive numerical result exists.

Earlier SVG layout drafts (`figures/software_section.svg`, `software_section_standalone.svg`, `software_section.png`) were working files for the layout. The PowerPoint supersedes them.
