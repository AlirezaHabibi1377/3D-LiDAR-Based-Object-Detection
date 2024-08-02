# 3D LiDAR Based Object Detection

This repository provides a comprehensive set of tools for processing and analyzing 3D point cloud data, particularly for object detection and segmentation tasks. Leveraging powerful libraries like Open3D, NumPy, and Pandas, the code demonstrates various techniques for enhancing 3D object detection using LiDAR point clouds.

## Overview

The main function, `Detection_Main.py`, processes a 3D point cloud file through the following stages:

1. **Downsampling**: Reduces the density of the point cloud for efficient processing.
2. **Segmentation**: Uses RANSAC to identify inliers and outliers, fitting a geometric model to the data.
3. **Clustering**: Applies DBSCAN to the outliers to group them into clusters.
4. **Bounding Box Detection**: Computes axis-aligned bounding boxes for each cluster.

## Usage

To run the pipeline, execute the script with the following structure:

```python
import time
import numpy as np
import open3d as o3d
import Downsampling as ds
import Segmentation as sg
import Clustering as cl
import BoundingBoxes as bb

def detection_pipeline(pcl_file, downsample_factor=0.25, iterations=100, tolerance=0.3, eps=0.4, min_points=5, debug=True):
    # Load the point cloud
    pcd = o3d.io.read_point_cloud(pcl_file)
    if debug:
        print(len(np.asarray(pcd.points)))
        o3d.visualization.draw_geometries([pcd])
    
    # Downsample
    t = time.time()
    downsample_pcd = ds.downsample(pcd, downsample_factor)
    if debug:
        print("Downsample Time", time.time() - t)
        o3d.visualization.draw_geometries([downsample_pcd])
    
    # Segment
    t = time.time()
    inlier_pts, outlier_pts = sg.ransac(downsample_pcd, iterations=iterations, tolerance=tolerance)
    if debug:
        print("Segmentation Time", time.time() - t)
        o3d.visualization.draw_geometries([outlier_pts, inlier_pts])
    
    # Cluster
    t = time.time()
    outlier_pts, labels = cl.dbscan(outlier_pts, eps=eps, min_points=min_points, print_progress=False, debug=debug)
    if debug:
        print("Clustering Time", time.time() - t)
        o3d.visualization.draw_geometries([outlier_pts, inlier_pts])
    
    # Bounding Boxes
    t = time.time()
    bboxes = bb.oriented_bbox(outlier_pts, labels)
    outlier_with_bboxes = [outlier_pts]
    outlier_with_bboxes.extend(bboxes)
    outlier_with_bboxes.append(inlier_pts)
    if debug:
        print("Bounding Boxes Time", time.time() - t)
        o3d.visualization.draw_geometries(outlier_with_bboxes)

    return outlier_with_bboxes

if __name__ == "__main__":
    pcl_file = "Data/test_files/UDACITY/0000000008.pcd"
    a = detection_pipeline(pcl_file, debug=True)
```

## Requirements

- **Python 3.x**
- **numpy**
- **open3d**
- **pandas**
- **matplotlib**

Install the dependencies using pip:
```python
pip install numpy open3d pandas matplotlib
```

## Outputs
- **Load the point cloud**
![Output Image](Outputs\0_loaded_pcl.png)

- **Downsample the point cloud**
![Output Image](Outputs\1_downsample_pcl.png)

- **Segment the point clouds**
![Output Image](Outputs\2_segment_pcl.png)

- **Clustering inliers to identify separate objects**
![Output Image](Outputs\3_clustering_pcl.png)

- **Determine 3D bounding boxes for all clusters**
![Output Image](Outputs\4_bbox_pcl.png)

- **Video**
![Output Image](5_visualization.gif)
