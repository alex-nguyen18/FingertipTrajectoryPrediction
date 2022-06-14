# FingertipTrajectoryPrediction

BVH_to_Positions.m is the main script that would convert motion capture data into 3D coordinates. I have commented lots of pieces of code that each section did some different transformation.

PredictedElbow.m does forward kinematics from data in "predicted" matrix

inverse???.m are the functions that I call to do inverse kinematics for the bvh or swivel angle models

L5P.m is the model we introduced for learning (learn 5 parameters).

In general, the process was transform BVH data into 3D coordinates of shoulder, elbow, and wrist. From there, you can use inverse kinematics (depending on the 7R model you are using) and geometry to calcuate various measurements. We then fitted the paths to the 5PL models, trained a regression model, predicted the paths based on start/end positions of the wrist, and then perform inverse kinematics to produce joint angles that would be used by an arm.

The transformations might not be exact, and there are cases where you will find discontinuities and things. I tried my best to produce smooth curves, but it was not perfect obviously.
