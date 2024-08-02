import pandas as pd


def oriented_bbox(pcd, labels, min_points=30, max_points=400):
    """
    Compute oriented bounding boxes (OBBs) for clusters in a point cloud.

    Parameters:
    - pcd (o3d.geometry.PointCloud): Input point cloud.
    - labels (np.ndarray): Array of cluster labels corresponding to each point in the point cloud.
    - min_points (int): Minimum number of points required in a cluster to compute a bounding box.
    - max_points (int): Maximum number of points allowed in a cluster to compute a bounding box.

    Returns:
    - obbs (list): List of axis-aligned bounding boxes for each valid cluster.
    """

    obbs = []

    # Create a list of indices for each cluster
    indices = pd.Series(range(len(labels))).groupby(labels, sort=False).apply(list).tolist()

    # Iterate over each cluster
    for i in range(len(indices)):

        # Select points belonging to the current cluster
        num_points = len(pcd.select_by_index(indices[i]).points)

        # Check if the number of points in the cluster is within the specified range
        if min_points < num_points < max_points:

            # Extract the subset of the point cloud for the current cluster
            sub_cloud = pcd.select_by_index(indices[i])

            # Compute the axis-aligned bounding box for the subset
            obb = sub_cloud.get_axis_aligned_bounding_box()
            obb.color = (0, 0, 0) # Set the color of the bounding box
            obbs.append(obb)

    # Print the number of bounding boxes calculated
    print(f"Number of Bounding Boxes calculated {len(obbs)}")

    return obbs
