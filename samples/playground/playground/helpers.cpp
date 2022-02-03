#include "playground/helpers.hpp"

std::shared_ptr<Mesh> MeshFactory::from_polyhedron(const std::shared_ptr<ConvexPolyhedron>& geom)
{
    auto trimesh = m_trimesh_factory.from_polyhedron(*geom);

    Vector<uint32_t> indices;
    Vector<Mesh::Vertex> vertices;
    for (auto& tri: trimesh->triangles()) {
        for (auto vid: tri.vert_ids) {
            indices.push_back(static_cast<uint32_t>(vertices.size()));
            auto& v = vertices.emplace_back();
            v.position = trimesh->vertices()[vid];
            v.normal = trimesh->normals()[tri.normal_id];
            v.color = Vec4{1.f, 1.f, 1.f, 0.f};
            v.tex_coords = Vec2{0.f, 0.f};
        }
    }

    return std::make_shared<Mesh>(vertices, indices);
}

std::shared_ptr<Mesh> MeshFactory::from_sphere(float radius)
{
    constexpr int YN = 30;
    constexpr int PN = 15;

    auto up = Vec3{0.f, 1.f, 0.f};
    auto side = Vec3{1.f, 0.f, 0.f};

    Vector<uint32_t> indices;
    Vector<Mesh::Vertex> vertices;

    float prev_pitch = -slope::PI * 0.5f;

    for (int j = 1; j <= PN; j++) {
        float pitch = j * slope::PI / PN - slope::PI * 0.5f;

        float prev_yaw = 0.f;
        for (int i = 1; i <= YN; i++) {
            float yaw = i * 2.f * slope::PI / YN;

            auto ur = static_cast<uint32_t>(vertices.size());
            auto ul = ur + 1;
            auto dr = ur + 2;
            auto dl = ur + 3;

            Vec4 color = pitch > 0 ? Vec4{0.6, 0.8, 0.2, 0.f} : Vec4{0.8, 0.6, 0.2, 0.f};
            if (yaw > slope::PI)
                color.z = 0.7;

            auto& vur = vertices.emplace_back();
            vur.position = (Mat44::rotation(side, pitch) * Mat44::rotation(up, yaw)).apply_normal(
                {0.f, 0.f, radius});
            vur.normal = vur.position.normalized();
            vur.color = color;
            vur.tex_coords = Vec2{0.f, 0.f};

            auto& vul = vertices.emplace_back();
            vul.position = (Mat44::rotation(side, pitch) * Mat44::rotation(up, prev_yaw)).apply_normal(
                {0.f, 0.f, radius});
            vul.normal = vul.position.normalized();
            vul.color = color;
            vul.tex_coords = Vec2{0.f, 0.f};

            auto& vdr = vertices.emplace_back();
            vdr.position = (Mat44::rotation(side, prev_pitch) * Mat44::rotation(up, yaw)).apply_normal(
                {0.f, 0.f, radius});
            vdr.normal = vdr.position.normalized();
            vdr.color = color;
            vdr.tex_coords = Vec2{0.f, 0.f};

            auto& vdl = vertices.emplace_back();
            vdl.position = (Mat44::rotation(side, prev_pitch) * Mat44::rotation(up, prev_yaw)).apply_normal(
                {0.f, 0.f, radius});
            vdl.normal = vdl.position.normalized();
            vdl.color = color;
            vdl.tex_coords = Vec2{0.f, 0.f};

            indices.push_back(ur);
            indices.push_back(dr);
            indices.push_back(ul);

            indices.push_back(dr);
            indices.push_back(ul);
            indices.push_back(dl);

            prev_yaw = yaw;
        }

        prev_pitch = pitch;
    }

    return std::make_shared<Mesh>(vertices, indices);
}

std::shared_ptr<Mesh> MeshFactory::from_capsule(float radius, float axis_length)
{
    constexpr int YN = 30;
    constexpr int PN = 16;

    auto up = Vec3{0.f, 1.f, 0.f};
    auto side = Vec3{1.f, 0.f, 0.f};

    Vector<uint32_t> indices;
    Vector<Mesh::Vertex> vertices;

    float prev_pitch = -slope::PI * 0.5f;

    for (int j = 1; j <= PN; j++) {
        float pitch = j * slope::PI / PN - slope::PI * 0.5f;

        Vec3 hemisphere_offset = {0.f, axis_length * (j <= PN / 2 ? 0.5f : -0.5f), 0.f};

        float prev_yaw = 0.f;
        for (int i = 1; i <= YN; i++) {
            float yaw = i * 2.f * slope::PI / YN;

            auto ur = static_cast<uint32_t>(vertices.size());
            auto ul = ur + 1;
            auto dr = ur + 2;
            auto dl = ur + 3;

            Vec4 color = (yaw > slope::PI) ? Vec4{0.6, 0.8, 0.2, 0.f} : Vec4{0.8, 0.6, 0.2, 0.f};

            auto& vur = vertices.emplace_back();
            vur.position = (Mat44::rotation(side, pitch) * Mat44::rotation(up, yaw)).apply_normal(
                {0.f, 0.f, radius});
            vur.normal = vur.position.normalized();
            vur.position += hemisphere_offset;
            vur.color = color;
            vur.tex_coords = Vec2{0.f, 0.f};

            auto& vul = vertices.emplace_back();
            vul.position = (Mat44::rotation(side, pitch) * Mat44::rotation(up, prev_yaw)).apply_normal(
                {0.f, 0.f, radius});
            vul.normal = vul.position.normalized();
            vul.position += hemisphere_offset;
            vul.color = color;
            vul.tex_coords = Vec2{0.f, 0.f};

            auto& vdr = vertices.emplace_back();
            vdr.position = (Mat44::rotation(side, prev_pitch) * Mat44::rotation(up, yaw)).apply_normal(
                {0.f, 0.f, radius});
            vdr.normal = vdr.position.normalized();
            vdr.position += hemisphere_offset;
            vdr.color = color;
            vdr.tex_coords = Vec2{0.f, 0.f};

            auto& vdl = vertices.emplace_back();
            vdl.position = (Mat44::rotation(side, prev_pitch) * Mat44::rotation(up, prev_yaw)).apply_normal(
                {0.f, 0.f, radius});
            vdl.normal = vdl.position.normalized();
            vdl.position += hemisphere_offset;
            vdl.color = color;
            vdl.tex_coords = Vec2{0.f, 0.f};

            indices.push_back(ur);
            indices.push_back(dr);
            indices.push_back(ul);

            indices.push_back(dr);
            indices.push_back(ul);
            indices.push_back(dl);

            prev_yaw = yaw;
        }

        prev_pitch = pitch;
    }

    float prev_yaw = 0.f;
    for (int i = 1; i <= YN; i++) {
        float yaw = i * 2.f * slope::PI / YN;

        Vec3 hemisphere_offset = {0.f, axis_length * 0.5f, 0.f};

        auto ur = static_cast<uint32_t>(vertices.size());
        auto ul = ur + 1;
        auto dr = ur + 2;
        auto dl = ur + 3;

        Vec4 color = (yaw > slope::PI) ? Vec4{0.6, 0.8, 0.2, 0.f} : Vec4{0.8, 0.6, 0.2, 0.f};

        auto& vur = vertices.emplace_back();
        vur.position = Mat44::rotation(up, yaw).apply_normal({0.f, 0.f, radius}) + hemisphere_offset;
        vur.normal = Vec3{vur.position.x, 0.f, vur.position.z}.normalized();
        vur.color = color;
        vur.tex_coords = Vec2{0.f, 0.f};

        auto& vul = vertices.emplace_back();
        vul.position = Mat44::rotation(up, prev_yaw).apply_normal({0.f, 0.f, radius}) + hemisphere_offset;
        vul.normal = Vec3{vul.position.x, 0.f, vul.position.z}.normalized();
        vul.color = color;
        vul.tex_coords = Vec2{0.f, 0.f};

        auto& vdr = vertices.emplace_back();
        vdr.position = Mat44::rotation(up, yaw).apply_normal({0.f, 0.f, radius}) - hemisphere_offset;
        vdr.normal = Vec3{vdr.position.x, 0.f, vdr.position.z}.normalized();
        vdr.color = color;
        vdr.tex_coords = Vec2{0.f, 0.f};

        auto& vdl = vertices.emplace_back();
        vdl.position = Mat44::rotation(up, prev_yaw).apply_normal({0.f, 0.f, radius}) - hemisphere_offset;
        vdl.normal = Vec3{vdl.position.x, 0.f, vdl.position.z}.normalized();
        vdl.color = color;
        vdl.tex_coords = Vec2{0.f, 0.f};

        indices.push_back(ur);
        indices.push_back(dr);
        indices.push_back(ul);

        indices.push_back(dr);
        indices.push_back(ul);
        indices.push_back(dl);

        prev_yaw = yaw;
    }


    return std::make_shared<Mesh>(vertices, indices);
}

DynamicActor* BodySpawner::spawn_box(const Mat44& tr, const Vec3& velocity, float mass, const Vec3& size)
{
    if (!m_box_material) {
        m_box_material = std::make_shared<Material>(DefaultShaders::mesh_shader());
        m_box_material->set_ambient_strength(0.2f);
        m_box_material->set_color({0.9, 0.75, 0.0});
    }

    auto box_ctor = [this, &size]() { return m_mesh_factory.from_polyhedron(m_poly_factory.box(size, {})); };

    auto e = m_world->create_entity();
    auto* rc = m_world->create<RenderComponent>(e);

    rc->mesh = get_mesh(m_box_meshes, size, box_ctor);
    rc->material = m_box_material;

    auto* tc = m_world->create<TransformComponent>(e);
    tc->transform = tr;

    auto* pc = m_world->create<PhysicsComponent>(e);

    auto* actor = m_dynamics_world->create_actor<DynamicActor>();

    Vec3 collision_size = {size.x + COLLISION_BLOAT, size.y + COLLISION_BLOAT, size.z + COLLISION_BLOAT};
    actor->set_shape<BoxShape>(collision_size);

    actor->set_transform(tc->transform);
    actor->body().set_velocity(velocity);
    actor->body().set_mass(mass);
    float inertia = mass / 6.f;
    actor->body().set_local_inertia({inertia, inertia, inertia});

    actor->set_friction(0.5f);

    actor->shape().set_transform(actor->body().transform());

    pc->actor = actor;
    return actor;
}

DynamicActor* BodySpawner::spawn_sphere(const Mat44& tr, const Vec3& velocity, float mass, float radius)
{
    if (!m_sphere_material) {
        m_sphere_material = std::make_shared<Material>(DefaultShaders::mesh_shader());
        m_sphere_material->set_ambient_strength(0.2f);
    }

    auto sphere_ctor = [this, radius]() { return m_mesh_factory.from_sphere(radius); };

    auto e = m_world->create_entity();
    auto* rc = m_world->create<RenderComponent>(e);
    rc->mesh = get_mesh(m_sphere_meshes, radius, sphere_ctor);
    rc->material = m_sphere_material;

    auto* tc = m_world->create<TransformComponent>(e);
    tc->transform = tr;

    auto* pc = m_world->create<PhysicsComponent>(e);

    float collision_radius = radius + COLLISION_BLOAT;

    auto* actor = m_dynamics_world->create_actor<DynamicActor>();
    actor->set_shape<SphereShape>(collision_radius);
    actor->set_transform(tc->transform);
    actor->body().set_velocity(velocity);
    actor->body().set_mass(mass);
    float inertia = 0.4f * mass * radius * radius;
    actor->body().set_local_inertia({inertia, inertia, inertia});

    actor->set_friction(0.5f);

    pc->actor = actor;
    return actor;
}

DynamicActor* BodySpawner::spawn_capsule(
    const Mat44& tr, const Vec3& velocity, const Vec3& ang_velocity, float mass, float radius, float height)
{
    if (!m_capsule_material) {
        m_capsule_material = std::make_shared<Material>(DefaultShaders::mesh_shader());
        m_capsule_material->set_ambient_strength(0.2f);
    }

    auto capsule_ctor = [this, radius, height]() { return m_mesh_factory.from_capsule(radius, height); };

    auto e = m_world->create_entity();
    auto* rc = m_world->create<RenderComponent>(e);
    rc->mesh = get_mesh(m_capsule_meshes, Vec2{radius, height}, capsule_ctor);
    rc->material = m_capsule_material;

    auto* tc = m_world->create<TransformComponent>(e);
    tc->transform = tr;

    auto* pc = m_world->create<PhysicsComponent>(e);

    float collision_radius = radius + COLLISION_BLOAT;

    auto* actor = m_dynamics_world->create_actor<DynamicActor>();
    actor->set_shape<CapsuleShape>(collision_radius, height);
    actor->set_transform(tc->transform);
    actor->body().set_velocity(velocity);
    actor->body().set_ang_velocity(ang_velocity);
    actor->body().set_mass(mass);

    float h = height + radius * 0.5f;
    float xz_inertia = mass * (3.f * radius * radius + h * h) / 12.f;
    float y_inertia = mass * radius * radius / 2.f;
    Vec3 localInertia = {xz_inertia, y_inertia, xz_inertia};
    //localInertia = {1.666666f, 0.666667f, 1.666666f};
    //localInertia = {1.666666f, 0.666667f, 5.666666f};
    localInertia = {1.666666f, 0.666667f, 1.666666f};
    actor->body().set_local_inertia(localInertia);

    actor->set_friction(0.5f);

    pc->actor = actor;
    return actor;
}
