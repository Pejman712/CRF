# 3dMapper classes explanation {#dmapper_class}
> There are two main classes inherited from IMapper3d, each one intended to be used in a specific use case
>
> The main difference in between both classes is the type of ICP algorithm used
>
> Several configuration files have been created to work as a guide. Activate debug logs to be able to fine tune the parameters to adapt it to your environment. It is also easier to debug with the steps visualization on.


## Mapper3d
> * Uses standard ICP algorithm. It compares the current pointcloud with the previous one in order to minimize the distance in between them.
>
> * This class should be used when the camera position is previously known with a decent precition because it accumulates the error from each map update if the camera pose is not precise enough.
>
> * ICP algorithm could be disabled and operate it as a mapper without any missaligment checks.


## HandHeldMapper3d
> * Uses Global ICP algorithm. It compares the current pointcloud with the hole previously mapped environment.
>
> * This class could be used without a precise camera pose estimation because the error will not accumulate in the same way as the Mapper3d.
>
> * It could be used to extract the camera pose estimation from it after pointcloud insertion as a Visual Odometer (return value of updateMap function). It has not been tested with a wheeled robot yet.
>
> * ICP algorithm could not be disable in this class.

## Useful links
* For a better understanding of each ICP and filtering parameter visit pcl documentation <http://pointclouds.org/documentation/>
* For a better understanding of each Octree parameter visit octomap documentation <https://octomap.github.io/octomap/doc/index.html>
