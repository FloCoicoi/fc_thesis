# Dynamic Obstacle Layer

This package intends to provide a dynamic obstacle layer plugin to work with the costmap_2d package. The general idea is to end up with a layer that keeps the probability each cell is free or occupied. It can use a PointCloud in a smarter way: when the bottom of an obstacle is occluded but the obstacle is tall enough so that we can see it (for example the wall behind a small box), it may be seen on the piontcloud, but if it shows only points above the robot's height, this information would be ignore completely by the original obstacle layer. In the dynamic obstacle layer such information is used to increase the probability of the hidden cells when we don't know for sure if the obstacle reaches down tot he robot's height. This helps for global planning.

Other ideas not yet implemented are to have some decay applied to every cell so that the probabilities of cells being occupied or free decrease when unobserved. Thus the robot can have some knwoledge about what it really known of the environment. This could be use to avoid making blind wild turns into space we have no fresh information of.