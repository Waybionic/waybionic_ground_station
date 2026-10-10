- Only add ROS-display mesh assets to this folder
- No CAD dumps or mechanical source files in this folder

The six `tool/*.stl` meshes were supplied by Biomedical for the clamp preview.
The STL format has no unit declaration. Their numerical bounds look plausible
in millimetres (the base is 80 mm wide), so RViz uses scale 0.001. Confirm the
units against the corrected CAD export before relying on dimensions. The
relative positions and claw motion in the preview are provisional because the
assembled model and measured pivots have not yet been supplied.
