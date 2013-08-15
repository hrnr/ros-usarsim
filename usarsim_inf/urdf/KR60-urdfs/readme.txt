This directory contains different versions of the KR60 URDF file, used for testing IK performance under various conditions.

"jt" refers to using transformed joint orientation frames in the URDF file so that the joint axis of rotation always points along +z.
"nojt" means using the same orientation frame for every joint and changing the axis of rotation.
"box" means using a 0.05x0.05 box, with a length 80% that of the actual arm link, as the collision model for the link.
"real" means using the collision models built in Blender from the arm visual models.
"none" means that both the collision models and the visual models were removed from the URDF file.
 "perp" means that the joint axes of rotation on the URDF model were adjusted to be perpendicular and axis-aligned. For instance, axes pointing along (0.002, -1.000, 0.000) were changed to (0.000, -1.000, 0.000).
 "ikfast" is shorthand for "nojt-real-perp"
