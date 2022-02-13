#include "slope/collision/epa.hpp"
#include <tuple>
#include <algorithm>

namespace slope {

namespace {

inline uint64_t encode_edge(uint32_t a, uint32_t b)
{
    if (a > b)
        std::swap(a, b);
    return a | static_cast<uint64_t>(b) << 32;
}

inline std::pair<uint32_t, uint32_t> decode_edge(uint64_t code)
{
    return {static_cast<uint32_t>(code & 0xffffffff), static_cast<uint32_t>(code >> 32)};
}

} // unnamed

void EPAStats::reset()
{
    cum_test_count = 0;
    cum_iterations_count = 0;
    max_iteration_count = 0;
}

void EPAStats::merge(const EPAStats& other)
{
    total_fail_count +=     other.total_fail_count;
    cum_test_count +=       other.cum_test_count;
    cum_iterations_count += other.cum_iterations_count;
    max_iteration_count =   std::max(max_iteration_count, other.max_iteration_count);
}

void EPASolver::collect_stats(uint32_t iteration_count, bool fail)
{
    m_stats.cum_test_count++;
    m_stats.cum_iterations_count += iteration_count;
    m_stats.max_iteration_count = std::max(m_stats.max_iteration_count, iteration_count);
    m_stats.total_fail_count += (int)fail;
}

void EPASolver::add_face(uint32_t a_idx, uint32_t b_idx, uint32_t c_idx)
{
    constexpr float NORMAL_EPSILON = 1e-8f;

    auto& a = m_points[a_idx];
    auto& b = m_points[b_idx];
    auto& c = m_points[c_idx];

    vec3 ab = b - a;
    vec3 ac = c - a;

    vec3 normal_dir = ab.cross(ac);
    float normal_dir_len = normal_dir.length();
    if (normal_dir_len < NORMAL_EPSILON)
        return;

    vec3 normal = normal_dir / normal_dir_len;

    if (normal.dot(m_inner_point - a) > 0.f)
        normal = -normal;

    float dist = normal.dot(a);

    m_heap.push_back({a_idx, b_idx, c_idx, normal, dist});
    std::push_heap(m_heap.begin(), m_heap.end());
}

std::optional<vec3> EPASolver::find_penetration_axis(
    const CollisionShape* shape1, const CollisionShape* shape2, const GJKSolver::Simplex& simplex)
{
    m_points.clear();
    m_heap.clear();
    m_edges.clear();

    m_inner_point.set_zero();

    for (auto& p: simplex) {
        m_points.push_back(p);
        m_inner_point += p;
    }

    m_inner_point *= 0.25f;

    add_face(0, 1, 2);
    add_face(0, 1, 3);
    add_face(0, 2, 3);
    add_face(1, 2, 3);

    for (int iter = 1; true; iter++) {
        SL_VERIFY(!m_heap.empty());
        while (m_heap.front().obsolete) {
            std::pop_heap(m_heap.begin(), m_heap.end());
            m_heap.pop_back();
        }

        auto face = m_heap.front();
        std::pop_heap(m_heap.begin(), m_heap.end());
        m_heap.pop_back();

        auto new_pt = shape1->support_diff(shape2, face.normal, config().support_bloat, true);

        float proximity = face.normal.dot(new_pt - m_points[face.a]);

        if (proximity < config().early_threshold) {
            collect_stats(iter, false);
            return face.normal;
        }

        if (iter == config().max_iteration_count) {
            if (proximity < config().final_threshold) {
                collect_stats(iter, false);
                return face.normal;
            }

            break;
        }

        int idx = static_cast<int>(m_points.size());
        m_points.push_back(new_pt);

        m_edges.clear();
        m_edges.push_back(encode_edge(face.a, face.b));
        m_edges.push_back(encode_edge(face.a, face.c));
        m_edges.push_back(encode_edge(face.b, face.c));


        for (auto& f: m_heap) {
            if (!f.obsolete && f.normal.dot(new_pt - m_points[f.a]) > 0.f) {
                f.obsolete = true;

                m_edges.push_back(encode_edge(f.a, f.b));
                m_edges.push_back(encode_edge(f.a, f.c));
                m_edges.push_back(encode_edge(f.b, f.c));
            }
        }

        std::sort(m_edges.begin(), m_edges.end());
        m_edges.push_back(0);

        uint64_t prev = 0;
        uint64_t prev_prev = 0;

        for (uint64_t code: m_edges) {
            if (prev != code && prev != prev_prev) {
                auto[a, b] = decode_edge(prev);
                add_face(a, b, idx);
            }

            prev_prev = prev;
            prev = code;
        }
    }

    collect_stats(config().max_iteration_count, true);
    return std::nullopt;
}

} // slope
