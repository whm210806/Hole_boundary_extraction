For 3D camera extraction:
1. Change the file name in camera_calibration.cpp
2. Run the 'pointcloud_processing_node' node
3. Use camera_calibration.py to view the segmented point cloud
4. Change the point cloud file name in the circle_boundary_extraction.cpp
5. Run the 'circle_hole_extraction_node' --> Cirle boundary point cloud will be saved in the .pcd file.

For extrinsic parameters:
Change the file name in param_calibration.py to the saved circle boundary cloud.

For internal transformation (Half developed):
intergrate_calibration.py is for one position, can repeat for 2 positions to get the optimum result.
