#pragma once

#include "slope/dynamics/constraint_solver.hpp"
#include "slope/dynamics/rigid_body.hpp"
#include "slope/containers/array.hpp"
#include <optional>

namespace slope {

class BaseJoint {
public:
    explicit BaseJoint(RigidBody* b1, RigidBody* b2 = nullptr) : m_body1(b1), m_body2(b2) {}
    virtual ~BaseJoint() = default;

    void            set_warmstarting_ratio(float ratio) { m_warmstarting_ratio = ratio; }

    virtual void    apply_constraints(ConstraintSolver* solver) = 0;
    virtual void    cache_lambdas(ConstraintSolver* solver) = 0;

    RigidBody*      body1() { return m_body1; }
    RigidBody*      body2() { return m_body2; }

protected:
    RigidBody* m_body1 = nullptr;
    RigidBody* m_body2 = nullptr;

    float m_warmstarting_ratio = 0.8f;
};

template <int CACHE_SIZE>
class CachedJoint : public BaseJoint {
public:
    using BaseJoint::BaseJoint;

    void cache_lambdas(ConstraintSolver* solver) final;

protected:
    struct LambdaCache {
        std::optional<ConstraintId> constraint_id;
        float lambda = 0.f;
    };

    Array<LambdaCache, CACHE_SIZE> m_cache;
};

class SphericalJoint : public CachedJoint<4> {
public:
    using CachedJoint::CachedJoint;

    void        set_damping(const float value) { m_damping = value; }
    void        set_anchor1(const vec3& value) { m_anchor1 = value; }
    void        set_anchor2(const vec3& value) { m_anchor2 = value; }

    const vec3& anchor1() const { return m_anchor1; }
    const vec3& anchor2() const { return m_anchor2; }

    void        apply_constraints(ConstraintSolver* solver) final;

private:
    vec3 m_anchor1;
    vec3 m_anchor2;

    float m_damping = 0.f;
};

template <int CACHE_SIZE>
void CachedJoint<CACHE_SIZE>::cache_lambdas(ConstraintSolver* solver)
{
    for (auto& cache: m_cache) {
        if (cache.constraint_id.has_value()) {
            cache.lambda = solver->get_lambda(*cache.constraint_id);
            cache.constraint_id.reset();
        } else {
            cache.lambda = 0.f;
        }
    }
}

} // slope
