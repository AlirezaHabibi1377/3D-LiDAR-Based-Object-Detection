import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


def dbscan(pcd, eps=0.45, min_points=7, print_progress=False, debug=False):
    """
    Apply DBSCAN clustering to a point cloud.

    Parameters:
    - pcd (o3d.geometry.PointCloud): Input point cloud.
    - eps (float): The maximum distance between two points for them to be considered as in the same neighborhood.
    - min_points (int): The number of points required to form a dense region.
    - print_progress (bool): Whether to print the progress of the clustering.
    - debug (bool): If True, sets the verbosity level to Debug to show detailed logs.

    Returns:
    - pcd (o3d.geometry.PointCloud): Point cloud with colors assigned to each cluster.
    - labels (np.ndarray): Cluster labels for each point.
    """

    # Set the verbosity level for Open3D logging
    verbosityLevel = o3d.utility.VerbosityLevel.Warning
    if debug:
        verbosityLevel = o3d.utility.VerbosityLevel.Debug
    with o3d.utility.VerbosityContextManager(verbosityLevel) as cm:
        
        # Perform DBSCAN clustering on the point cloud
        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=print_progress))

    # Determine the number of clusters
    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")

    # Generate colors for each cluster using a colormap
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0 # Assign black color to noise points
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3]) # Set colors in the point cloud

    return pcd, labels


def hdbscan(pcd):
    """
    Placeholder function for HDBSCAN clustering.

    Parameters:
    - pcd (o3d.geometry.PointCloud): Input point cloud.

    Returns:
    - None: Function to be implemented.
    """
    
    # TODO - implement this
    pass


