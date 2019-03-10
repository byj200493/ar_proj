after compiling the librosbagreadpluginexe.so and putting it into /lib
load up this project into unity and click on file->build settings
make sure that the architecture is the same architecture that you built the library with (mine is x86_64)
and then click on build

to run it:
1. run the built file normally
2. run roscore on a different terminal
3. do rosbag play --clock rgbdepth.bag on a different terminal
4. press 'h' to toggle between heatmap and depth map

note : use the rgbdepth.bag dataset for now, if you want to use the test3_depth.bag dataset, go back to the plugin and change the rgb subscribe topic to the one that test3 dataset has /cam0_remap/image_raw for rgbdepth.bag it is /cam0/image_raw and don't forget to change the size of "planeobj" object in unity because the test3 dataset would look better with a bigger planeobj compared to the rgbdepth dataset (currently the size is good for rgbdepth dataset)
