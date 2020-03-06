------------------------------------------------------------------------------------------------
Method:

Use SAC-IA to find the plate target and transform it to the origin of the coordinate system.

Remove the plane from the transformed target and calculate the centroid of the leftint part.

Evaluate the Target according to:
1. the distance from Centroid of the lefting part to (0,0,z(ommited)). Since the position and oriention of the models are adjusted by PCA, the centroid of the edge should be close to (0,0).

2. the normal of the plane that has been extracted, which should be around (0,0,1) or (0,0,-1).

3. the amount of the points in the lefting part.

------------------------------------------------------------------------------------------------
Advantage:
stable.


------------------------------------------------------------------------------------------------
Disadvantage: Cost too much time for each aligning.

