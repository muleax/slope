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
    return {code & 0xffffffff, code >> 32};
}

} // unnamed

void EPASolver::reset_stats()
{
    m_stats.cum_test_count = 0;
    m_stats.cum_iterations_count = 0;
    m_stats.max_iteration_count = 0;
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

    Vec3 ab = b - a;
    Vec3 ac = c - a;

    Vec3 normal_dir = ab.cross(ac);
    float normal_dir_len = normal_dir.length();
    if (normal_dir_len < NORMAL_EPSILON)
        return;

    Vec3 normal = normal_dir / normal_dir_len;

    if (normal.dot(m_inner_point - a) > 0.f)
        normal = -normal;

    float dist = normal.dot(a);

    m_heap.push_back({a_idx, b_idx, c_idx, normal, dist});
    std::push_heap(m_heap.begin(), m_heap.end());

    // records.back().push_back({a, b, c, normal, Status::New});
}

std::optional<Vec3> EPASolver::find_penetration_axis(
    const CollisionShape* shape1, const CollisionShape* shape2, const GJKSolver::Simplex& simplex)
{
    //records.clear();

    m_points.clear();
    m_heap.clear();
    m_edges.clear();

    // Vec3 best_axis;

    m_inner_point.set_zero();

    for (auto& p: simplex) {
        m_points.push_back(p);
        m_inner_point += p;
    }

    m_inner_point *= 0.25f;

    //records.push_back({});

    add_face(0, 1, 2);
    add_face(0, 1, 3);
    add_face(0, 2, 3);
    add_face(1, 2, 3);

    for (int iter = 1; true; iter++) {
        //records.push_back({});

        SL_VERIFY(!m_heap.empty());
        while (m_heap.front().obsolete) {
            std::pop_heap(m_heap.begin(), m_heap.end());
            m_heap.pop_back();
        }

        auto face = m_heap.front();
        std::pop_heap(m_heap.begin(), m_heap.end());
        m_heap.pop_back();

        auto new_pt = shape1->support_diff(shape2, face.normal, m_config.support_bloat, true);

        float proximity = face.normal.dot(new_pt - m_points[face.a]);

        if (proximity < m_config.early_threshold) {
            collect_stats(iter, false);
            return face.normal;
        }

        if (iter == m_config.max_iteration_count) {
            if (proximity < m_config.final_threshold) {
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

    collect_stats(m_config.max_iteration_count, true);
    return std::nullopt;
}

/*
struct EPADebugRecord {
     enum Status {
        Old,
        New,
        Removed
    };

    Vec3 a;
    Vec3 b;
    Vec3 c;
    Vec3 n;
    Status status;
};

void epa_debug_draw()
{
    float scale = 1.f;
    Vec3 offs = {5.f, 5.f, 5.f};
    Vec3 iter_offs = {4.f, 0.f, 0.f};
    for (auto& iter: m_collider.m_epa.records) {
        for (auto& tri: iter) {
            Vec3 color = tri.status == Status::New
                         ? Vec3{0.f, 1.f, 0.f}
                         : (tri.status == Status::Old ? Vec3{0.5f, 0.5f, 1.f} : Vec3{1.f, 0.f, 0.f});

            Vec3 moffs = tri.status == Status::New
                         ? Vec3{0.02f, 0.f, 0.f}
                         : (tri.status == Status::Old ? Vec3{0.0f, 0.f, 0.f} : Vec3{-0.02f, 0.f, 0.f});

            m_debug_drawer->draw_line(tri.a * scale + offs + moffs, tri.b * scale + offs + moffs, color);
            m_debug_drawer->draw_line(tri.b * scale + offs + moffs, tri.c * scale + offs + moffs, color);
            m_debug_drawer->draw_line(tri.c * scale + offs + moffs, tri.a * scale + offs + moffs, color);

            if (tri.status != Status::Removed) {

                auto mid = (tri.a + tri.b + tri.c) / 3.f;

                auto n_end = mid * scale + offs + tri.n * 0.4f;
                m_debug_drawer->draw_line(mid * scale + offs, n_end, {0.f, 0.8f, 0.8f});

                float no = 0.05f;
                m_debug_drawer->draw_line(n_end + Vec3{0.f, no, 0.f}, n_end + Vec3{0.f, -no, 0.f},
                                          {0.f, 0.8f, 0.8f});
                m_debug_drawer->draw_line(n_end + Vec3{no, 0.f, 0.f}, n_end + Vec3{-no, 0.f, 0.f},
                                          {0.f, 0.8f, 0.8f});
                m_debug_drawer->draw_line(n_end + Vec3{0.f, 0.f, no}, n_end + Vec3{0.f, 0.f, -no},
                                          {0.f, 0.8f, 0.8f});
            }
        }

        m_debug_drawer->draw_line(Vec3{0.f, 0.1f, 0.f} + offs, Vec3{0.f, -0.1f, 0.f} + offs, {1.f, 1.f, 1.f});
        m_debug_drawer->draw_line(Vec3{0.1f, 0.f, 0.f} + offs, Vec3{-0.1f, 0.f, 0.f} + offs, {1.f, 1.f, 1.f});
        m_debug_drawer->draw_line(Vec3{0.f, 0.f, 0.1f} + offs, Vec3{0.f, -0.f, -0.1f} + offs, {1.f, 1.f, 1.f});

        offs += iter_offs;
    }
}
*/

} // slope
