- Only add ROS-display mesh assets to this folder
- No CAD dumps or mechanical source files in this folder

The original six `tool/*.stl` meshes were supplied by Biomedical. Julie later
sent `Julie - Assembly.stl`. The three `assembly_*.stl` files were split from
that export by `scripts/cad/split_biomedical_assembly.py`: one fixed assembly
mesh and two moving claw meshes. This preserves Julie's relative part positions.

STL has no unit declaration. The assembled base measures 80 units wide and RViz
currently treats one unit as one millimetre. The base is centred on `tool_link`
with its bottom at the flange. Biomedical still needs to confirm the file units,
tool mounting transform, claw opening direction, pivot axis and measured travel.
