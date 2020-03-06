-- Input arguments
Location of template files, location of scene pcd file
~$ ./plate_align_nobase data_nobase/object_templates.txt pcd/sample.pcd


------------------------------------------------------------------------------------------------
Method:

Extract the plane first.

Change the plate models to plates withoud the bottom (extract the plane out while builing the model).

Still use SAC-IA to align the models with the targets.

-------------------------------------------------------------------------------------------------
Advantage:

Because both the amount of points in model and target are less than aligning the whole plate model with the target, the speed of aligning can be quicker.

Since the models only include points from the edge of those plates, put some food in the plate won't influence the final result of alignment.

------------------------------------------------------------------------------------------------
Disadvantage:

without the bottom of the plates, and also because of the MAX Correspondance Distance, it is much harder for the algorithm to find the best transform matrix to fit the model with the target accurately. 

And since the thickness of the plates is alway very thin, which means the feature got from the edge part of the plate is not such distinct, it is stil with big possibility to align the model of the edge of the plate to the plane(desktop).


