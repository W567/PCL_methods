-- What to do?
Check if plates can be placd on the destination assigned by user.
In other words, check if any objects occupied the assigned place.

-- Input argument
Location of the pcd file
ex. ~$./check pcd/sample.pcd

-- Methods used
1. Remove the plane of the table surface.
2. Remove the points outside of the area of destination.
3. Extract clusters left in the area
4. Check the number of points in each cluster to estimate if any objects exist.
