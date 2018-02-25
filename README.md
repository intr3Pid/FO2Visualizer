#FO2Visualizer

Primary purpose of this program is to visualise the AABB tree of the track_cdb2.gen files from FlatOut 2.
It can also visualize the track_bvh.gen files.

Usage:  Grab a FlatOut 2 map, make one mesh (merge all objects to one) and export it to Collada file format,
but make sure you keep the absolute coordinates in space, for this don't move the mesh around.
Grab the track_cdb2.gen file of this map. Now as the file loading in not implemented very good, but sufficient for 
a debug tool, it's a bit inconvinient to load the files, you have to do the following;
rename the track_cdb2 or track_bvh file to the same file name as the Collada Geometry file, and put as file ending
"cdb2" or "bvh". Now put both files (the geometry and the cdb2 or bvh )in the same directory as the FO2Visualizer.exe,
and start the application by providing the the Collada geometry file as argument to it, for example by dragging the file on 
the program.
Pressing V will read the tree and visualize it, now you can use the arrow keys to traverse the tree.



based on http://462cmu.github.io/asst3_pathtracer/