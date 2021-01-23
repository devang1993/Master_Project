# Master_Project
Detection of rotational deviation between two objects represented by point clouds
by means of implementing the Iterative Closest Point (ICP) Algorithm. The 3D imaging
device used here is the Intel RealSense Depth Camera D415.

# Theory:
Point clouds are an efficient way of representing the depth data, and optional colour
data of images captured by a 3D camera/scanner. Alternatively a point cloud can be defined
as a collection of data points within an image defined by a given coordinate system.
The Point Cloud Library (PCL), a open-source library for 2D/3D image processing,
is extensively used within this code, in order to facilitate several algorithm key
points, such as point cloud I/O, post-processing, point cloud registration, etc.
The key algorithm used herein is called Iterative Closest Point (ICP).
ICP is more commonly implemented in scenarios where point cloud images captured of
different orientations of the same environment are needed to be 'stitched' or merged
into a single seamless 3D image. This requires every set of point cloud datasets
acquired from different views to be aligned into a single point cloud model,
corresponding to a global coordinate framework. Hence, ICP aims to find the
transformation between a point cloud and some reference point cloud, by minimizing
the square errors between the corresponding entities => find optimal the rotation and
translation matrices to transform source point cloud P2 to target point cloud P1.
The ICP provides output as a 4x4 homogenous transformation matrix between the original
'reference' point cloud and the test 'deviated' point cloud. The angular deviation in
Euler angles as well as the overall translational difference can be retrieved
from this matrix.
