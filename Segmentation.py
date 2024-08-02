
def ransac(pcd, iterations=10, tolerance=0.25):
    """
    Apply RANSAC to fit a plane to the point cloud and separate inliers from outliers.

    Parameters:
    - pcd (o3d.geometry.PointCloud): Input point cloud.
    - iterations (int): The number of iterations for the RANSAC algorithm.
    - tolerance (float): The distance threshold to determine if a point is an inlier.

    Returns:
    - inlier_cloud (o3d.geometry.PointCloud): Point cloud containing points that fit the plane model.
    - outlier_cloud (o3d.geometry.PointCloud): Point cloud containing points that do not fit the plane model.
    """

    # Perform RANSAC to fit a plane model to the point cloud
    plane_model, inliers = pcd.segment_plane(distance_threshold=tolerance, ransac_n=3, num_iterations=iterations)

    # Extract the inlier points (points that fit the plane model)
    inlier_cloud = pcd.select_by_index(inliers)

    # Extract the outlier points (points that do not fit the plane model)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)

    # Paint inliers blue and outliers red for visualization
    outlier_cloud.paint_uniform_color([1, 0, 0])  # R, G, B - Red color for outliers
    inlier_cloud.paint_uniform_color([0, 0, 1])  # R, G, B - Blue color for inliers

    return inlier_cloud, outlier_cloud
