## Slope

A tiny physics engine focused on rigid body simulation. 
Work in progress, a lot still needs to be done (see ToDo section).

[![Playground](https://i.imgur.com/9EqwEdo.png)](https://youtu.be/Co2f-l-V3jM "Slope Playground")

### Supported OS
Tested on MacOS and Windows.

### Project structure
* **slope**: the engine itself including math, collision detection, joints, constraint solver
* **app**: application framework to build internal tools and samples
* **tools**: directory for asset converters and other utilities
* **samples**: demo applications
* **deps**: external dependencies

### Building
```bash
git submodule init
git submodule update
mkdir .build
cd .build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --config Release -j 8
```
You'll find various demos in samples/playground app.

### Minimal example
From samples/minimal_example:
```c++
    // create an executor with single worker thread
    ParallelExecutor executor(1);

    DynamicsWorldConfig config;
    // 30 Hz update interval
    config.solver_config.time_interval = 1.f / 30.f;
    // standard gravity
    config.gravity = {0.f, -9.81f, 0.f};

    // create dynamics world and assign the executor
    DynamicsWorld world(config);
    world.setup_executor(executor);

    // create dynamic actor with box shape
    auto* actor = world.create_dynamic_actor();
    world.assign_shape(actor, BoxShape({1.f, 1.f, 1.f}));
    actor->set_transform(mat44::translate({0.f, 3.f, 0.f}));

    // create floor
    auto* floor = world.create_kinematic_actor();
    world.assign_shape(floor, BoxShape({100.f, 1.f, 100.f}));
    floor->set_transform(mat44::translate({0.f, -0.5f, 0.f}));

    // simulate 30 frames
    for (int i = 0; i < 30; i++) {
        executor.run().wait();

        auto& pos = actor->transform().translation();
        log::info("Actor position: ({}  {}  {})", pos.x, pos.y, pos.z);
    }
```

### Collision detection
CD works with convex shapes. Concave geometries should be modeled via convex composition.

Broadphase is currently done using the sweep and prune algorithm.

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
Projected Gauss-Seidel method is used. Position stabilization is done using the Baumgarte technique. 

Slope utilizes temporal coherence to improve PGS convergence. 
Constraint information is cached for use in the next frame. 
Note that this is not a full-fledged "Persistent Contact Manifold", 
so stacking is not very stable for complex shapes.

Three contact friction models are supported:
* Axis-aligned pyramid
* Velocity-aligned pyramid
* Implicit cone

Gyroscopic forces are supported through a stable implicit scheme.

### Joints

Joints are modeled using reaction constraints in spatial coordinates.
Currently, only spherical (ball-socket) joint is supported. 
Other types will be added in the future. User can easily implement custom joints.

### Multithreading
Slope is designed to run in a multi-threaded environment.
Its execution flow can be effectively parallelized with a provided task runner.

Slope comes with 2 task runners out of the box:
- SequentialExecutor: a naive single-threaded implementation
- ParallelExecutor: multi-threaded runner backed by [Taskflow](https://github.com/taskflow/taskflow)

The engine uses an abstract task runner interface,
so user can fully embed the simulation into their own execution flow by implementing the interface.

Note that not all scenes can be parallelized equally well.
PGS is sequential in its very nature, so it's not easy to make it parallel for a connected system.
Although it's possible, any such implementation will come with stability trade-offs.
Slope can effectively parallelize collision detection and constraint setup. 
Slope also tries to divide the scene into disconnected islands which can then be solved in parallel.

The "1K Boxes" scene from samples/playground 
is an example of a fully connected system with a lot of constraints:

![1K Boxes](https://i.imgur.com/fCWtpvb.png)

On the contrary, the "10K Boxes" scene 
can be perfectly divided into small islands:

![1K Boxes](https://i.imgur.com/xaI5Nob.png)

### Profiling

Slope comes with embedded [Tracy](https://github.com/wolfpld/tracy) 
client version [v0.7.8](https://github.com/wolfpld/tracy/releases/tag/v0.7.8). 
To enable tracy use cmake option:
```bash
-DSlopeTracyEnable=ON
```

### ToDo
- Deactivation
- Better position stabilization technique (e.g. position-based stabilization)
- Rolling friction
- More joint types
- Better broadphase (e.g. BVH)
- Collision filters and callbacks
- Raycasts
- Compound shape
- Trimesh shape
- Cylinder shape
- Convex decomposition
- Persistent contact manifold
- Resource system
