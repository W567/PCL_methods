*******************************************************************************************************************************************
Method:

First, extract clusters from the original cloud according to the euclidean distance;

Second, delete the cluster which has the longest width on the direction of x axis;

Third, apply sac-ia to all the other clusters and get the valid targets which satisfied the alignment_score condition from each cluster; 


*******************************************************************************************************************************************
Time to process the plate_both.ply : 38 s  for 2 plates with the literator of 500. 


*******************************************************************************************************************************************
Shortcoming : sometimes can't extract the plate cluster out because of other stuff.
              and the execution time is base on the amount of clusters it extracted, while some of the clusters may not have the plate target inside.

*******************************************************************************************************************************************
Advantage : Since the target for aligning is smaller then aligning directly, it can save some time for each aligning. And the final result can get a better score (average distance).
