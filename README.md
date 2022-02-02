## Slope

A tiny physics engine focused on rigid body simulation. Work in progress, a lot still needs to be done.

[![IMAGE ALT TEXT](https://i.imgur.com/9EqwEdo.png)](https://youtu.be/6_ipRQVZeME "Slope playground")

### Supported OS
Tested on MacOS and Windows.

### Project structure
* **slope**: the engine itself including math, collision detection, joints, constraint solver
* **app**: application framework to build internal tools and samples
* **tools**: directory for asset converters and other utilities
* **samples**: demo applications
* **deps**: external dependencies

### Building
```
git submodule init
git submodule update
mkdir .release
cd .release
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --config Release -j 8
```
You'll find various samples in "playground" app.

### Collision detection
CD works with convex shapes. Concave geometries should be modeled via convex composition.

Broadphase is currently done by sweep and prune algorithm.

Narrowphase is broken into two stages: finding an axis of minimal penetration and contact manifold generation.
The Gilbert-Johnson-Keerthi algorithm (along with the Expanding Polytope algorithm) is used to find the axis for complex convexes.
The Separating Axis test is used for smaller polyhedra.
After the axis is found, contact points are generated using polygon clipping.

Supported shapes: arbitrary convex polyhedron, box, capsule, sphere.

|-             |Polyhedron|Box|Capsule|Sphere|
|---           |---       |---|---    |---   |
|**Polyhedron**|GJK|GJK|GJK|GJK|
|**Box**|GJK|SAT|GJK|GJK|
|**Capsule**|GJK|GJK|Custom|Custom|
|**Sphere**|GJK|GJK|Custom|Custom|

### Dynamic simulation
At the core of the simulation is a constraint solver 
based on a standard Linear Complementarity formulation for rigid body dynamics.
Method: projected Gauss-Seidel.

Three contact friction models are supported:
* Axis-aligned pyramid
* Velocity-aligned pyramid
* Implicit cone

Gyroscopic forces are supported through a stable implicit scheme.

Joints are modeled using reaction constraints in spatial coordinates.
Currently, only spherical (ball-socket) joint is supported. Other types will be added in the future. User can easily implement custom joints.
