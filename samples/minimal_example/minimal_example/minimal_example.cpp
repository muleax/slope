#include "slope/dynamics/dynamics_world.hpp"
#include "slope/collision/shape/box_shape.hpp"
#include "slope/flow/parallel_executor.hpp"
#include "slope/debug/log.hpp"

using namespace slope;

int main()
{
    // create an executor with single worker thread
    ParallelExecutor executor{1};

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
    world.set_shape(actor, BoxShape({1.f, 1.f, 1.f}));
    actor->set_transform(mat44::translate({0.f, 3.f, 0.f}));

    // create floor
    auto* floor = world.create_static_actor();
    world.set_shape(floor, BoxShape({100.f, 1.f, 100.f}));
    floor->set_transform(mat44::translate({0.f, -0.5f, 0.f}));

    // simulate 30 frames
    for (int i = 0; i < 30; i++) {
        executor.run().wait();

        auto& pos = actor->transform().translation();
        log::info("Actor position: ({}  {}  {})", pos.x, pos.y, pos.z);
    }

    return 0;
}
