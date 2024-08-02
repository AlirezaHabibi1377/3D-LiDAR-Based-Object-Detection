import time
import numpy as np
import open3d as o3d
import Downsampling as ds
import Segmentation as sg
import Clustering as cl
import BoundingBoxes as bb

# Main function for the point cloud processing
def detection_pipeline(pcl_file, downsample_factor=0.25, iterations=100, tolerance=0.3, eps=0.4, min_points=5, debug=True):
    
    # Load the point cloud from the specified file
    pcd = o3d.io.read_point_cloud(pcl_file)
    
    # Debug: Print the number of points and visualize the original point cloud
    if debug:
        print(len(np.asarray(pcd.points)))
        o3d.visualization.draw_geometries([pcd])
    
    # Timing the downsampling process
    t = time.time()
    downsample_pcd = ds.downsample(pcd, downsample_factor)
    
    # Debug: Print time taken for downsampling and visualize the downsampled point cloud
    if debug:
        print("Downsample Time", time.time() - t)
        o3d.visualization.draw_geometries([downsample_pcd])
    
    # Timing the segmentation process using RANSAC
    t = time.time()
    inlier_pts, outlier_pts = sg.ransac(downsample_pcd, iterations=iterations, tolerance=tolerance)
    if debug:
        print("Segmentation Time", time.time() - t)
        o3d.visualization.draw_geometries([outlier_pts, inlier_pts])
    
    # Debug: Print time taken for segmentation and visualize inliers and outliers
    t = time.time()
    outlier_pts, labels = cl.dbscan(outlier_pts, eps=eps, min_points=min_points, print_progress=False, debug=debug)
    if debug:
        print("Clustering Time", time.time() - t)
        o3d.visualization.draw_geometries([outlier_pts, inlier_pts])
    
    # Timing the DBSCAN clustering process
    t = time.time()
    bboxes = bb.oriented_bbox(outlier_pts, labels)

    # Prepare the output with bounding boxes added to the point cloud
    outlier_with_bboxes = [outlier_pts]
    outlier_with_bboxes.extend(bboxes)
    outlier_with_bboxes.append(inlier_pts)
    
    # Debug: Print time taken for bounding box calculation and visualize the final output
    if debug:
        print("Bounding Boxes Time", time.time() - t)
        o3d.visualization.draw_geometries(outlier_with_bboxes)

    # Return the final result including outliers with bounding boxes and inliers
    return outlier_with_bboxes

# Main execution: Set the file path and run the detection pipeline
if __name__ == "__main__":
    pcl_file = "Data/test_files/UDACITY/0000000008.pcd"
    a = detection_pipeline(pcl_file, debug=True)
