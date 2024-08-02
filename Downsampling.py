
def downsample(pcd, factor=0.2):
    """
    Downsample the point cloud using voxel grid filtering.

    Parameters:
    - pcd (o3d.geometry.PointCloud): Input point cloud to be downsampled.
    - factor (float): The voxel size for downsampling. Smaller values result in higher resolution.

    Returns:
    - downsample_pcd (o3d.geometry.PointCloud): The downsampled point cloud.
    """
    # Apply voxel grid filtering to downsample the point cloud
    downsample_pcd = pcd.voxel_down_sample(voxel_size=factor)
    return downsample_pcd