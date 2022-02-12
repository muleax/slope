#pragma once
#include "slope/dynamics/constraint_solver.hpp"
#include "slope/dynamics/rigid_body.hpp"
#include "slope/core/array.hpp"
#include <optional>

namespace slope {

class BaseJoint {
public:
    explicit BaseJoint(RigidBody* b1, RigidBody* b2 = nullptr) : m_body1(b1), m_body2(b2) {}
    virtual ~BaseJoint() = default;

    void            set_warmstarting_ratio(float ratio) { m_warmstarting_ratio = ratio; }
    void            set_erp(float erp) { m_erp = erp; }

    virtual void    apply_constraints(ConstraintSolver* solver) = 0;
    virtual void    cache_lambdas(ConstraintSolver* solver) = 0;

    RigidBody*      body1() { return m_body1; }
    RigidBody*      body2() { return m_body2; }

protected:
    RigidBody* m_body1 = nullptr;
    RigidBody* m_body2 = nullptr;

    float m_warmstarting_ratio = 0.8f;
    float m_erp = 0.2f;
};

template <int CACHE_SIZE>
class WarmStartJoint : public BaseJoint {
public:
    using BaseJoint::BaseJoint;

    void cache_lambdas(ConstraintSolver* solver) final;

protected:
    struct LambdaCache {
        std::optional<ConstraintId> constraint_id;
        float lambda = 0.f;
    };

    ConstraintId add_constraint_warm(int cache_id, ConstraintSolver* solver, Constraint& conf);

    Array<LambdaCache, CACHE_SIZE> m_cache;
};

template <int CACHE_SIZE>
void WarmStartJoint<CACHE_SIZE>::cache_lambdas(ConstraintSolver* solver)
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

template <int CACHE_SIZE>
ConstraintId WarmStartJoint<CACHE_SIZE>::add_constraint_warm(int cache_id, ConstraintSolver* solver, Constraint& conf)
{
    SL_ASSERT(cache_id < CACHE_SIZE);
    auto& cache = m_cache[cache_id];
    conf.init_lambda = cache.lambda * m_warmstarting_ratio;
    cache.constraint_id = solver->add_constraint(conf);
    return *cache.constraint_id;
}

} // slope
