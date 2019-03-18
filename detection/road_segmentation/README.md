# ROAD SEGMENTATION

## Input
RGB Frame via topic /camera/rgb/image_raw (can be set in roslaunch file)

## Output
Binary image which is mask of road on topic /road_segmentation

## TODO
For now, just use HSV color threshold to detect the road segmentation
[] Using ENet for semantic segmentation