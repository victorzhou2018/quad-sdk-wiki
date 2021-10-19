QuadKD is our kinematics and dynamics library to aid in performing those types of calculations. Some methods are written by hand, others implement RBDL as a backend (mostly for Jacobian and inverse dynamics computation).

The general naming convention for coordinate transformations is
```
QuadKD::proximallinkToDistallinkFK/IKCoordFrame();
```
So for example `QuadKD::worldToFootFKWorldFrame();` would give the coordinates of the foot frame origin relative to the world frame origin expressed the world frame for a given set of body and joint positions , while `QuadKD::legbaseToFootIKLegbaseFrame();` would give the joint positions corresponding to a given foot location specified in the legbase frame.