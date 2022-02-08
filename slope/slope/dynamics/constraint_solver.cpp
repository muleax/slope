#include "slope/dynamics/constraint_solver.hpp"
#include "slope/debug/assert.hpp"
#include "slope/debug/log.hpp"
#include <immintrin.h>

// #define SLOPE_DISABLE_SIMD

// TODO: make portable
#ifdef _MSC_VER
#define SLOPE_FORCEINLINE __forceinline
#else
#define SLOPE_FORCEINLINE __attribute__((always_inline))
#endif

namespace slope {

struct ConstraintHelper {
    using ConstraintData = ConstraintSolver::ConstraintData;
    using BodyData = ConstraintSolver::BodyData;

    Vector<BodyData>& bodies;

    void solve_constraint_ref(ConstraintData& c, float& lambda, float min_bound, float max_bound)
    {
        if (c.body2_idx >= 0) {
            auto& b1 = bodies[c.body1_idx];
            auto& b2 = bodies[c.body2_idx];

            float cur_lambda = lambda;

            float dot = cur_lambda * c.cfm_inv_dt;
            dot += c.jacobian11.dot(b1.inv_m_f1) + c.jacobian12.dot(b1.inv_m_f2);
            dot += c.jacobian21.dot(b2.inv_m_f1) + c.jacobian22.dot(b2.inv_m_f2);

            float delta = (c.rhs - dot) * c.inv_diag;
            lambda = clamp(min_bound, cur_lambda + delta, max_bound);
            delta = lambda - cur_lambda;

            b1.inv_m_f1 += c.inv_m_j11 * delta;
            b1.inv_m_f2 += c.inv_m_j12 * delta;
            b2.inv_m_f1 += c.inv_m_j21 * delta;
            b2.inv_m_f2 += c.inv_m_j22 * delta;

        } else {
            auto& b1 = bodies[c.body1_idx];

            float cur_lambda = lambda;

            float dot = cur_lambda * c.cfm_inv_dt;
            dot += c.jacobian11.dot(b1.inv_m_f1) + c.jacobian12.dot(b1.inv_m_f2);

            float delta = (c.rhs - dot) * c.inv_diag;
            lambda = clamp(min_bound, cur_lambda + delta, max_bound);
            delta = lambda - cur_lambda;

            b1.inv_m_f1 += c.inv_m_j11 * delta;
            b1.inv_m_f2 += c.inv_m_j12 * delta;
        }
    }

    SLOPE_FORCEINLINE
    float compute_lambda_unbounded_single(ConstraintData& c, BodyData& b1, float lambda)
    {
        float dot = lambda * c.cfm_inv_dt;
        dot += c.jacobian11.dot(b1.inv_m_f1) + c.jacobian12.dot(b1.inv_m_f2);
        float delta = (c.rhs - dot) * c.inv_diag;

        return lambda + delta;
    }

    SLOPE_FORCEINLINE
    float compute_lambda_unbounded_pair(ConstraintData& c, BodyData& b1, BodyData& b2, float lambda)
    {
        float dot = lambda * c.cfm_inv_dt;
        dot += c.jacobian11.dot(b1.inv_m_f1) + c.jacobian12.dot(b1.inv_m_f2);
        dot += c.jacobian21.dot(b2.inv_m_f1) + c.jacobian22.dot(b2.inv_m_f2);
        float delta = (c.rhs - dot) * c.inv_diag;

        return lambda + delta;
    }

    SLOPE_FORCEINLINE
    void apply_delta_single(ConstraintData& c, BodyData& b1, float delta)
    {
        b1.inv_m_f1 += c.inv_m_j11 * delta;
        b1.inv_m_f2 += c.inv_m_j12 * delta;
    }

    SLOPE_FORCEINLINE
    void apply_delta_pair(ConstraintData& c, BodyData& b1, BodyData& b2, float delta)
    {
        b1.inv_m_f1 += c.inv_m_j11 * delta;
        b1.inv_m_f2 += c.inv_m_j12 * delta;
        b2.inv_m_f1 += c.inv_m_j21 * delta;
        b2.inv_m_f2 += c.inv_m_j22 * delta;
    }

    SLOPE_FORCEINLINE
    void solve_bounded_single(ConstraintData& c, BodyData& b1, float& lambda, float min_bound, float max_bound)
    {
        float new_lambda = compute_lambda_unbounded_single(c, b1, lambda);

        new_lambda = clamp(min_bound, new_lambda, max_bound);
        float delta = new_lambda - lambda;
        lambda = new_lambda;

        apply_delta_single(c, b1, delta);
    }

    SLOPE_FORCEINLINE
    void solve_bounded_pair(ConstraintData& c, BodyData& b1, BodyData& b2, float& lambda, float min_bound, float max_bound)
    {
        float new_lambda = compute_lambda_unbounded_pair(c, b1, b2, lambda);

        new_lambda = clamp(min_bound, new_lambda, max_bound);
        float delta = new_lambda - lambda;
        lambda = new_lambda;

        apply_delta_pair(c, b1, b2, delta);
    }

    SLOPE_FORCEINLINE
    void scale_lambda_to_cone(float& lambda1, float& lambda2, float bound1, float bound2)
    {
        double divisor = ((double)lambda1 * lambda1) * ((double)bound2 * bound2) + ((double)lambda2 * lambda2) * ((double)bound1 * bound1);
        double product = (double)bound1 * bound2;

        if (divisor > 1e-8 && divisor > product * product) {
            // scale down and preserve angle
            double t = product / sqrt(divisor);
            lambda1 = float(t * lambda1);
            lambda2 = float(t * lambda2);

            // float angle = std::atan2f(new_lambda1, new_lambda2);
            // new_lambda1 = bound1 * std::sinf(angle);
            // new_lambda2 = bound2 * std::cosf(angle);

        } else {
            lambda1 = clamp(-bound1, lambda1, bound1);
            lambda2 = clamp(-bound2, lambda2, bound2);
        }
    }

    void solve_constraint(ConstraintData& c, float& lambda, float min_bound, float max_bound)
    {
        if (c.body2_idx >= 0) {
            auto& b1 = bodies[c.body1_idx];
            auto& b2 = bodies[c.body2_idx];
            solve_bounded_pair(c, b1, b2, lambda, min_bound, max_bound);

        } else {
            auto& b1 = bodies[c.body1_idx];
            solve_bounded_single(c, b1, lambda, min_bound, max_bound);
        }
    }

    void solve_constraint_friction_1d(ConstraintData& c, float& lambda, float normal_lambda)
    {
        float bound = normal_lambda * c.friction_ratio;

        if (c.body2_idx >= 0) {
            auto& b1 = bodies[c.body1_idx];
            auto& b2 = bodies[c.body2_idx];
            solve_bounded_pair(c, b1, b2, lambda, -bound, bound);

        } else {
            auto& b1 = bodies[c.body1_idx];
            solve_bounded_single(c, b1, lambda, -bound, bound);
        }
    }

    void solve_constraint_friction_2d(
        ConstraintData& c1, ConstraintData& c2, float& lambda1, float& lambda2, float normal_lambda)
    {
        float bound1 = normal_lambda * c1.friction_ratio;
        float bound2 = normal_lambda * c2.friction_ratio;

        if (c1.body2_idx >= 0) {
            auto& b1 = bodies[c1.body1_idx];
            auto& b2 = bodies[c1.body2_idx];
            solve_bounded_pair(c1, b1, b2, lambda1, -bound1, bound1);
            solve_bounded_pair(c2, b1, b2, lambda2, -bound2, bound2);

        } else {
            auto& b1 = bodies[c1.body1_idx];
            solve_bounded_single(c1, b1, lambda1, -bound1, bound1);
            solve_bounded_single(c2, b1, lambda2, -bound2, bound2);
        }
    }

    void solve_constraint_friction_cone(
        ConstraintData& c1, ConstraintData& c2, float& lambda1, float& lambda2, float normal_lambda)
    {
        // TODO: SIMD version
        float new_lambda1 = 0.f;
        float new_lambda2 = 0.f;

        float bound1 = normal_lambda * c1.friction_ratio;
        float bound2 = normal_lambda * c2.friction_ratio;

        if (c1.body2_idx >= 0) {
            auto& b1 = bodies[c1.body1_idx];
            auto& b2 = bodies[c1.body2_idx];

            if (normal_lambda > 0.f) {
                new_lambda1 = compute_lambda_unbounded_pair(c1, b1, b2, lambda1);
                new_lambda2 = compute_lambda_unbounded_pair(c2, b1, b2, lambda2);
                scale_lambda_to_cone(new_lambda1, new_lambda2, bound1, bound2);
            }

            apply_delta_pair(c1, b1, b2, new_lambda1 - lambda1);
            apply_delta_pair(c2, b1, b2, new_lambda2 - lambda2);
        }
        else
        {
            auto& b1 = bodies[c1.body1_idx];

            if (normal_lambda > 0.f) {
                new_lambda1 = compute_lambda_unbounded_single(c1, b1, lambda1);
                new_lambda2 = compute_lambda_unbounded_single(c2, b1, lambda2);
                scale_lambda_to_cone(new_lambda1, new_lambda2, bound1, bound2);
            }

            apply_delta_single(c1, b1, new_lambda1 - lambda1);
            apply_delta_single(c2, b1, new_lambda2 - lambda2);
        }

        lambda1 = new_lambda1;
        lambda2 = new_lambda2;
    }

#ifndef SLOPE_DISABLE_SIMD

    SLOPE_FORCEINLINE
    __m128 clamp_simd(__m128 value, __m128 min_bound, __m128 max_bound)
    {
        value = _mm_min_ps(value, max_bound);
        return _mm_max_ps(value, min_bound);
    }

    SLOPE_FORCEINLINE
    __m128 clamp_simd(__m128 value, __m128 bound)
    {
        value = _mm_min_ps(value, bound);
        return _mm_max_ps(value, _mm_xor_ps(bound, _mm_set1_ps(-0.0)));
    }

    SLOPE_FORCEINLINE
    void load_min_max(float bound_s, __m128& min_bound, __m128& max_bound)
    {
        max_bound = _mm_load1_ps(&bound_s);
        min_bound = _mm_xor_ps(max_bound, _mm_set1_ps(-0.0));
    }

    SLOPE_FORCEINLINE
    void load_inv_m_f_single(BodyData& b1, __m128& inv_m_f0, __m128& inv_m_f1)
    {
        inv_m_f0 = _mm_load_ps(b1.inv_m_f1.data);
        inv_m_f1 = _mm_load_ps(b1.inv_m_f1.data + 4);
    }

    SLOPE_FORCEINLINE
    void load_inv_m_f_pair(BodyData& b1, BodyData& b2, __m128& inv_m_f0, __m128& inv_m_f1, __m128& inv_m_f2)
    {
        inv_m_f0 = _mm_load_ps(b1.inv_m_f1.data);

        auto inv_m_f01 = _mm_load_ps(b1.inv_m_f1.data + 4);
        auto inv_m_f10 = _mm_load_ps(b2.inv_m_f1.data);
        auto inv_m_f11 = _mm_load_ps(b2.inv_m_f1.data + 4);

        inv_m_f1 = _mm_shuffle_ps(inv_m_f01, inv_m_f10, _MM_SHUFFLE(1, 0, 1, 0));
        inv_m_f2 = _mm_shuffle_ps(inv_m_f10, inv_m_f11, _MM_SHUFFLE(1, 0, 3, 2));
    }

    SLOPE_FORCEINLINE
    void store_inv_m_f_single(BodyData& b1, __m128 inv_m_f0, __m128 inv_m_f1)
    {
        _mm_store_ps(b1.inv_m_f1.data, inv_m_f0);
        _mm_store_ps(b1.inv_m_f1.data + 4, inv_m_f1);
    }

    SLOPE_FORCEINLINE
    void store_inv_m_f_pair(BodyData& b1, BodyData& b2, __m128 inv_m_f0, __m128 inv_m_f1, __m128 inv_m_f2)
    {
        _mm_store_ps(b1.inv_m_f1.data, inv_m_f0);
        _mm_store_ps(b1.inv_m_f1.data + 4, inv_m_f1);

        auto tmp1 = _mm_shuffle_ps(inv_m_f1, inv_m_f2, _MM_SHUFFLE(1, 0, 3, 2));
        _mm_store_ps(b2.inv_m_f1.data, tmp1);

        auto tmp2 = _mm_shuffle_ps(inv_m_f2, inv_m_f2, _MM_SHUFFLE(3, 2, 3, 2));
        _mm_store_ps(b2.inv_m_f1.data + 4, tmp2);
    }

    SLOPE_FORCEINLINE
    __m128 dot_product(__m128 a, __m128 b)
    {
        auto m = _mm_mul_ps(a, b);
        m = _mm_add_ps(m, _mm_shuffle_ps(m, m, _MM_SHUFFLE(1, 0, 3, 2)));
        return _mm_add_ps(m, _mm_shuffle_ps(m, m, _MM_SHUFFLE(2, 3, 0, 1)));

        //return _mm_dp_ps(a, b, 255);
    }

    SLOPE_FORCEINLINE
    __m128 compute_lambda_unbounded_single_simd(ConstraintData& c, __m128 cur_lambda, __m128 inv_m_f0, __m128 inv_m_f1)
    {
        auto dot_base = _mm_mul_ps(cur_lambda, _mm_load1_ps(&c.cfm_inv_dt));
        auto delta = _mm_sub_ps(_mm_load1_ps(&c.rhs), dot_base);

        auto j0 = _mm_load_ps(c.jacobian11.data);
        auto j1 = _mm_load_ps(c.jacobian11.data + 4);

        auto dot0 = dot_product(j0, inv_m_f0);
        auto dot1 = dot_product(j1, inv_m_f1);

        delta = _mm_sub_ps(delta, dot0);
        delta = _mm_sub_ps(delta, dot1);

        delta = _mm_mul_ps(delta, _mm_load1_ps(&c.inv_diag));

        return _mm_add_ps(cur_lambda, delta);
    }

    SLOPE_FORCEINLINE
    __m128 compute_lambda_unbounded_pair_simd(ConstraintData& c, __m128 cur_lambda, __m128 inv_m_f0, __m128 inv_m_f1, __m128 inv_m_f2)
    {
        auto dot_base = _mm_mul_ps(cur_lambda, _mm_load1_ps(&c.cfm_inv_dt));
        auto delta = _mm_sub_ps(_mm_load1_ps(&c.rhs), dot_base);

        auto j0 = _mm_load_ps(c.jacobian11.data);
        auto j1 = _mm_load_ps(c.jacobian11.data + 4);
        auto j2 = _mm_load_ps(c.jacobian11.data + 8);

        auto dot0 = dot_product(j0, inv_m_f0);
        auto dot1 = dot_product(j1, inv_m_f1);
        auto dot2 = dot_product(j2, inv_m_f2);

        auto d0d1 = _mm_add_ps(dot0, dot1);
        delta = _mm_sub_ps(delta, d0d1);
        delta = _mm_sub_ps(delta, dot2);

        delta = _mm_mul_ps(delta, _mm_load1_ps(&c.inv_diag));

        return _mm_add_ps(cur_lambda, delta);
    }

    SLOPE_FORCEINLINE
    void apply_delta_single_simd(ConstraintData& c, __m128 delta, __m128& inv_m_f0, __m128& inv_m_f1)
    {
        auto inv_m_j0 = _mm_load_ps(c.inv_m_j11.data);
        auto inv_m_j1 = _mm_load_ps(c.inv_m_j11.data + 4);

        inv_m_f0 = _mm_add_ps(inv_m_f0, _mm_mul_ps(inv_m_j0, delta));
        inv_m_f1 = _mm_add_ps(inv_m_f1, _mm_mul_ps(inv_m_j1, delta));
    }

    SLOPE_FORCEINLINE
    void apply_delta_pair_simd(ConstraintData& c, __m128 delta, __m128& inv_m_f0, __m128& inv_m_f1, __m128& inv_m_f2)
    {
        auto inv_m_j0 = _mm_load_ps(c.inv_m_j11.data);
        auto inv_m_j1 = _mm_load_ps(c.inv_m_j11.data + 4);
        auto inv_m_j2 = _mm_load_ps(c.inv_m_j11.data + 8);

        inv_m_f0 = _mm_add_ps(inv_m_f0, _mm_mul_ps(inv_m_j0, delta));
        inv_m_f1 = _mm_add_ps(inv_m_f1, _mm_mul_ps(inv_m_j1, delta));
        inv_m_f2 = _mm_add_ps(inv_m_f2, _mm_mul_ps(inv_m_j2, delta));
    }

    SLOPE_FORCEINLINE
    void solve_bounded_pair_simd(
        ConstraintData& c, __m128 min_bound, __m128 max_bound,
        __m128& lambda, __m128& inv_m_f0, __m128& inv_m_f1, __m128& inv_m_f2)
    {
        auto new_lambda = compute_lambda_unbounded_pair_simd(c, lambda, inv_m_f0, inv_m_f1, inv_m_f2);
        new_lambda = clamp_simd(new_lambda, min_bound, max_bound);
        auto delta = _mm_sub_ps(new_lambda, lambda);
        lambda = new_lambda;

        apply_delta_pair_simd(c, delta, inv_m_f0, inv_m_f1, inv_m_f2);
    }

    SLOPE_FORCEINLINE
    void solve_bounded_single_simd(ConstraintData& c, __m128 min_bound, __m128 max_bound, __m128& lambda, __m128& inv_m_f0, __m128& inv_m_f1)
    {
        auto new_lambda = compute_lambda_unbounded_single_simd(c, lambda, inv_m_f0, inv_m_f1);
        new_lambda = clamp_simd(new_lambda, min_bound, max_bound);
        auto delta = _mm_sub_ps(new_lambda, lambda);
        lambda = new_lambda;

        apply_delta_single_simd(c, delta, inv_m_f0, inv_m_f1);
    }

    void solve_constraint_simd(ConstraintData& c, float& lambda_s, float min_bound_s, float max_bound_s)
    {
        __m128 min_bound = _mm_load1_ps(&min_bound_s);
        __m128 max_bound = _mm_load1_ps(&max_bound_s);

        __m128 lambda = _mm_load1_ps(&lambda_s);

        if (c.body2_idx >= 0) {
            auto& b1 = bodies[c.body1_idx];
            auto& b2 = bodies[c.body2_idx];

            __m128 inv_m_f0;
            __m128 inv_m_f1;
            __m128 inv_m_f2;
            load_inv_m_f_pair(b1, b2, inv_m_f0, inv_m_f1, inv_m_f2);
            solve_bounded_pair_simd(c, min_bound, max_bound, lambda, inv_m_f0, inv_m_f1, inv_m_f2);
            store_inv_m_f_pair(b1, b2, inv_m_f0, inv_m_f1, inv_m_f2);

        } else {
            auto& b1 = bodies[c.body1_idx];

            __m128 inv_m_f0;
            __m128 inv_m_f1;
            load_inv_m_f_single(b1, inv_m_f0, inv_m_f1);
            solve_bounded_single_simd(c, min_bound, max_bound, lambda, inv_m_f0, inv_m_f1);
            store_inv_m_f_single(b1, inv_m_f0, inv_m_f1);
        }

        lambda_s = _mm_cvtss_f32(lambda);
    }

    void solve_constraint_friction_1d_simd(ConstraintData& c, float& lambda_s, float normal_lambda_s)
    {
        __m128 min_bound;
        __m128 max_bound;
        load_min_max(normal_lambda_s * c.friction_ratio, min_bound, max_bound);

        __m128 lambda = _mm_load1_ps(&lambda_s);

        if (c.body2_idx >= 0) {
            auto& b1 = bodies[c.body1_idx];
            auto& b2 = bodies[c.body2_idx];

            __m128 inv_m_f0;
            __m128 inv_m_f1;
            __m128 inv_m_f2;
            load_inv_m_f_pair(b1, b2, inv_m_f0, inv_m_f1, inv_m_f2);
            solve_bounded_pair_simd(c, min_bound, max_bound, lambda, inv_m_f0, inv_m_f1, inv_m_f2);
            store_inv_m_f_pair(b1, b2, inv_m_f0, inv_m_f1, inv_m_f2);

        } else {
            auto& b1 = bodies[c.body1_idx];

            __m128 inv_m_f0;
            __m128 inv_m_f1;
            load_inv_m_f_single(b1, inv_m_f0, inv_m_f1);
            solve_bounded_single_simd(c, min_bound, max_bound, lambda, inv_m_f0, inv_m_f1);
            store_inv_m_f_single(b1, inv_m_f0, inv_m_f1);
        }

        lambda_s = _mm_cvtss_f32(lambda);
    }

    void solve_constraint_friction_2d_simd(
        ConstraintData& c1, ConstraintData& c2, float& lambda1_s, float& lambda2_s, float normal_lambda_s)
    {
        __m128 min_bound1;
        __m128 max_bound1;
        load_min_max(normal_lambda_s * c1.friction_ratio, min_bound1, max_bound1);

        __m128 min_bound2;
        __m128 max_bound2;
        load_min_max(normal_lambda_s * c2.friction_ratio, min_bound2, max_bound2);

        __m128 lambda1 = _mm_load1_ps(&lambda1_s);
        __m128 lambda2 = _mm_load1_ps(&lambda2_s);

        if (c1.body2_idx >= 0) {
            auto& b1 = bodies[c1.body1_idx];
            auto& b2 = bodies[c1.body2_idx];

            __m128 inv_m_f0;
            __m128 inv_m_f1;
            __m128 inv_m_f2;
            load_inv_m_f_pair(b1, b2, inv_m_f0, inv_m_f1, inv_m_f2);

            solve_bounded_pair_simd(c1, min_bound1, max_bound1, lambda1, inv_m_f0, inv_m_f1, inv_m_f2);
            solve_bounded_pair_simd(c2, min_bound2, max_bound2, lambda2, inv_m_f0, inv_m_f1, inv_m_f2);

            store_inv_m_f_pair(b1, b2, inv_m_f0, inv_m_f1, inv_m_f2);

        } else {
            auto& b1 = bodies[c1.body1_idx];

            __m128 inv_m_f0;
            __m128 inv_m_f1;
            load_inv_m_f_single(b1, inv_m_f0, inv_m_f1);

            solve_bounded_single_simd(c1, min_bound1, max_bound1, lambda1, inv_m_f0, inv_m_f1);
            solve_bounded_single_simd(c2, min_bound2, max_bound2, lambda2, inv_m_f0, inv_m_f1);

            store_inv_m_f_single(b1, inv_m_f0, inv_m_f1);
        }

        lambda1_s = _mm_cvtss_f32(lambda1);
        lambda2_s = _mm_cvtss_f32(lambda2);
    }

#else // SLOPE_DISABLE_SIMD

    SLOPE_FORCEINLINE
    void solve_constraint_simd(ConstraintData& c, float& lambda, float min_bound, float max_bound)
    {
        solve_constraint(c, lambda, min_bound, max_bound);
    }

    SLOPE_FORCEINLINE
    void solve_constraint_friction_1d_simd(ConstraintData& c, float& lambda, float normal_lambda)
    {
        solve_constraint_friction_1d(c, lambda, normal_lambda);
    }

    SLOPE_FORCEINLINE
    void solve_constraint_friction_2d_simd(
        ConstraintData& c1, ConstraintData& c2, float& lambda1, float& lambda2, float normal_lambda)
    {
        solve_constraint_friction_2d(c1, c2, lambda1, lambda2, normal_lambda);
    }

#endif // SLOPE_DISABLE_SIMD

};

Constraint Constraint::generic(
        RigidBody* body1, RigidBody* body2, const ConstraintGeom& geom,
        float pos_error, float min_bound, float max_bound)
{
    Constraint c;
    c.body1 = body1;

    vec3 r1 = geom.p1 - body1->transform().translation();
    c.jacobian1[0] = -geom.axis;
    c.jacobian1[1] = -r1.cross(geom.axis);

    if (body2) {
        c.body2 = body2;

        vec3 r2 = geom.p2 - body2->transform().translation();
        c.jacobian2[0] = geom.axis;
        c.jacobian2[1] = r2.cross(geom.axis);
    }

    c.pos_error = pos_error;
    c.min_bound = min_bound;
    c.max_bound = max_bound;

    return c;
}

ConstraintSolver::ConstraintSolver()
{
    set_concurrency(1);
}

void ConstraintSolver::register_body(RigidBody* body)
{
    body->set_in_solver_index(static_cast<int>(m_bodies.size()));
    m_bodies.emplace_back();
    m_bodies_extra.emplace_back().body = body;
}

ConstraintId ConstraintSolver::allocate(ConstraintGroup group, int count)
{
    auto& group_data = m_groups[(int)group];
    ConstraintId begin_id = { group, group_data.size };

    group_data.size += count;

    if (group_data.constraints.size() < group_data.size) {
        group_data.constraints.resize(group_data.size);
        group_data.lambda.resize(group_data.size);
    }

    return begin_id;
}

void ConstraintSolver::setup_constraint(ConstraintId constr_id, const Constraint& c)
{
    SL_ASSERT(constr_id.group() == ConstraintGroup::General);

    auto& data = basic_setup(constr_id, c);
    data.min_bound = c.min_bound;
    data.max_bound = c.max_bound;
    data.cfm_inv_dt = c.cfm * m_inv_dt;

    // TODO: reconsider
    if (c.min_bound == 0.f) {
        // unilateral case
        if (c.pos_error > c.unilateral_penetration)
            data.bg_error = (c.pos_error - c.unilateral_penetration) * c.erp;
        else
            data.bg_error = c.pos_error - c.unilateral_penetration;
    } else {
        // bilateral case
        data.bg_error = c.pos_error * c.erp;
    }
}

void ConstraintSolver::setup_friction_1d(ConstraintId constr_id, const Constraint& c, float friction_ratio, ConstraintId normal_constr_id)
{
    SL_ASSERT(constr_id.group() == ConstraintGroup::Friction1D);

    auto& data = basic_setup(constr_id, c);

    data.cfm_inv_dt = c.cfm * m_inv_dt;
    data.bg_error = 0.f;

    data.friction_ratio = friction_ratio;
    data.normal_constr_idx = normal_constr_id.index();
}

void ConstraintSolver::setup_friction_2d(ConstraintIds ids, const Constraint& c1, const Constraint& c2, vec2 ratio, ConstraintId normal_id)
{
    SL_ASSERT(constr_id.group() == ConstraintGroup::Friction2D);

    auto setup_constraint = [this, normal_id](ConstraintData& data, const Constraint& c, float ratio) {
        data.cfm_inv_dt = c.cfm * m_inv_dt;
        data.bg_error = 0.f;

        data.friction_ratio = ratio;
        data.normal_constr_idx = normal_id.index();
    };

    auto& data1 = basic_setup(ids.first, c1);
    setup_constraint(data1, c1, ratio.x);

    auto& data2 = basic_setup(ids.second, c2);
    setup_constraint(data2, c2, ratio.y);
}

void ConstraintSolver::setup_friction_cone(ConstraintIds ids, const Constraint& c1, const Constraint& c2, vec2 ratio, ConstraintId normal_id)
{
    SL_ASSERT(constr_id.group() == ConstraintGroup::FrictionCone);

    auto setup_constraint = [this, normal_id](ConstraintData& data, const Constraint& c, float ratio) {
        data.cfm_inv_dt = c.cfm * m_inv_dt;
        data.bg_error = 0.f;

        data.friction_ratio = ratio;
        data.normal_constr_idx = normal_id.index();
    };

    auto& data1 = basic_setup(ids.first, c1);
    setup_constraint(data1, c1, ratio.x);

    auto& data2 = basic_setup(ids.second, c2);
    setup_constraint(data2, c2, ratio.y);
}

ConstraintSolver::ConstraintData& ConstraintSolver::basic_setup(ConstraintId id, const Constraint& c)
{
    SL_ASSERT(c.body1->in_solver_index() != -1);

    auto& group = m_groups[(int)id.group()];
    auto& data = group.constraints[id.index()];

    data.body1_idx = c.body1->in_solver_index();
    data.jacobian11 = c.jacobian1[0];
    data.jacobian12 = c.jacobian1[1];

    if (c.body2) {
        SL_ASSERT(c.body2->in_solver_index() != -1);

        data.body2_idx = c.body2->in_solver_index();
        data.jacobian21 = c.jacobian2[0];
        data.jacobian22 = c.jacobian2[1];
    } else {
        data.body2_idx = -1;
        data.jacobian21.set_zero();
        data.jacobian22.set_zero();
    }

    data.normal_constr_idx = -1;

    group.lambda[id.index()] = c.init_lambda;

    return data;
}

void ConstraintSolver::clear()
{
    for (auto& data : m_bodies_extra)
        data.body->set_in_solver_index(-1);

    m_bodies.clear();
    m_bodies_extra.clear();

    for (auto& container : m_groups) {
        //container.constraints.clear();
        //container.lambda.clear();
        container.size = 0;
    }
}

void ConstraintSolver::solve_pass0()
{
    // V_delta = V / dt + M_inv * F
    auto b_extra = m_bodies_extra.begin();
    for (auto b = m_bodies.begin(); b != m_bodies.end(); ++b, ++b_extra) {
        auto* body = b_extra->body;
        b_extra->v_delta1 = body->velocity() * m_inv_dt + body->force() * body->inv_mass();
        b_extra->v_delta2 = body->ang_velocity() * m_inv_dt + body->inv_inertia().apply_normal(body->torque());
    }
}

void ConstraintSolver::solve_pass1(int worker_id)
{
    static constexpr float INV_DIAG_EPSILON = 1e-8f;

    auto& bodies = m_worker_ctx[worker_id].bodies;
    bodies.clear();
    bodies.resize(m_bodies.size());

    // J * M_inv * J_t * lambda = pos_error / dt^2 - J * V_delta
    // inv_diag = 1 / (cfm / dt + J * M_inv * J_t)
    // M_inv * force = M_inv * J_t * lambda
    for (auto& group: m_groups) {
        auto chunk_size = group.size / concurrency();
        auto chunk_beg = group.constraints.begin() + (worker_id * chunk_size);
        auto chunk_end = (worker_id == concurrency() - 1)
                         ? group.constraints.begin() + group.size
                         : (chunk_beg + chunk_size);

        auto* lambda = group.lambda.data() + (worker_id * chunk_size);
        for (auto c = chunk_beg; c != chunk_end; ++c, ++lambda) {
            auto& b1_extra = m_bodies_extra[c->body1_idx];

            float j_v_delta = c->jacobian11.dot(b1_extra.v_delta1) + c->jacobian12.dot(b1_extra.v_delta2);

            c->inv_m_j11 = c->jacobian11 * b1_extra.body->inv_mass();
            c->inv_m_j12 = b1_extra.body->inv_inertia().apply_normal(c->jacobian12);

            float j_inv_m_j = c->jacobian11.dot(c->inv_m_j11) + c->jacobian12.dot(c->inv_m_j12);

            auto& b1 = bodies[c->body1_idx];
            b1.inv_m_f1 += c->inv_m_j11 * *lambda;
            b1.inv_m_f2 += c->inv_m_j12 * *lambda;

            if (c->body2_idx >= 0) {
                auto& b2_extra = m_bodies_extra[c->body2_idx];

                j_v_delta += c->jacobian21.dot(b2_extra.v_delta1) + c->jacobian22.dot(b2_extra.v_delta2);

                c->inv_m_j21 = c->jacobian21 * b2_extra.body->inv_mass();
                c->inv_m_j22 = b2_extra.body->inv_inertia().apply_normal(c->jacobian22);

                j_inv_m_j += c->jacobian21.dot(c->inv_m_j21) + c->jacobian22.dot(c->inv_m_j22);

                auto& b2 = bodies[c->body2_idx];
                b2.inv_m_f1 += c->inv_m_j21 * *lambda;
                b2.inv_m_f2 += c->inv_m_j22 * *lambda;
            }

            c->rhs = m_inv_dt * m_inv_dt * c->bg_error - j_v_delta;

            float rcp = c->cfm_inv_dt + j_inv_m_j;
            c->inv_diag = rcp * rcp > INV_DIAG_EPSILON ? m_config.sor / rcp : 0.f;
        }
    }
}

void ConstraintSolver::solve_pass2()
{
    for (size_t i = 0; i < m_bodies.size(); i++) {
        auto& b = m_bodies[i];
        b.inv_m_f1.set_zero();
        b.inv_m_f2.set_zero();

        for (auto& ctx : m_worker_ctx) {
            auto& wb = ctx.bodies[i];
            b.inv_m_f1 += wb.inv_m_f1;
            b.inv_m_f2 += wb.inv_m_f2;
        }
    }

    if (m_config.use_simd)
        solve_iterations<true>();
    else
        solve_iterations<false>();

    apply_impulses();
}

template<bool UseSIMD>
void ConstraintSolver::solve_iterations()
{
    ConstraintHelper helper{m_bodies};

    for(uint32_t iter = 0; iter < m_config.iteration_count; ++iter) {

        auto& general_group = m_groups[(int) ConstraintGroup::General];
        auto& general_lambda = general_group.lambda;

        {
            auto c          = general_group.constraints.begin();
            auto c_end      = general_group.constraints.begin() + general_group.size;
            auto c_lambda   = general_lambda.begin();
            for (; c != c_end; ++c, ++c_lambda) {
                if constexpr (UseSIMD)
                    helper.solve_constraint_simd(*c, *c_lambda, c->min_bound, c->max_bound);
                else
                    helper.solve_constraint(*c, *c_lambda, c->min_bound, c->max_bound);
            }
        }

        {
            auto& friction_1d_group = m_groups[(int) ConstraintGroup::Friction1D];

            auto c          = friction_1d_group.constraints.begin();
            auto c_end      = friction_1d_group.constraints.begin() + friction_1d_group.size;
            auto c_lambda   = friction_1d_group.lambda.data();
            for (; c != c_end; ++c, ++c_lambda) {
                float normal_lambda = c->friction_ratio * fabs(general_lambda[c->normal_constr_idx]);

                if constexpr (UseSIMD)
                    helper.solve_constraint_friction_1d_simd(*c, *c_lambda, normal_lambda);
                else
                    helper.solve_constraint_friction_1d(*c, *c_lambda, normal_lambda);
            }
        }

        {
            auto& friction_2d_group = m_groups[(int) ConstraintGroup::Friction2D];

            auto c1         = friction_2d_group.constraints.begin();
            auto c1_end     = friction_2d_group.constraints.begin() + friction_2d_group.size;
            auto c1_lambda  = friction_2d_group.lambda.data();
            for (; c1 != c1_end; c1 += 2, c1_lambda += 2) {
                auto c2 = c1 + 1;
                auto c2_lambda = c1_lambda + 1;
                float normal_lambda = fabs(general_lambda[c1->normal_constr_idx]);

                if constexpr (UseSIMD)
                    helper.solve_constraint_friction_2d_simd(*c1, *c2, *c1_lambda, *c2_lambda, normal_lambda);
                else
                    helper.solve_constraint_friction_2d(*c1, *c2, *c1_lambda, *c2_lambda, normal_lambda);
            }
        }

        {
            auto& friction_cone_group = m_groups[(int) ConstraintGroup::FrictionCone];

            auto c1         = friction_cone_group.constraints.begin();
            auto c1_end     = friction_cone_group.constraints.begin() + friction_cone_group.size;
            auto c1_lambda  = friction_cone_group.lambda.data();
            for (; c1 != c1_end; c1 += 2, c1_lambda += 2) {
                auto c2 = c1 + 1;
                auto c2_lambda = c1_lambda + 1;
                float normal_lambda = fabs(general_lambda[c1->normal_constr_idx]);

                helper.solve_constraint_friction_cone(*c1, *c2, *c1_lambda, *c2_lambda, normal_lambda);
            }
        }
    }
}

void ConstraintSolver::apply_impulses()
{
    auto b_extra = m_bodies_extra.begin();
    for (auto b = m_bodies.begin(); b != m_bodies.end(); ++b, ++b_extra) {
        auto* body = b_extra->body;
        body->set_velocity(body->velocity() + b->inv_m_f1 * m_dt);
        body->set_ang_velocity(body->ang_velocity() + b->inv_m_f2 * m_dt);
    }
}

} // slope
