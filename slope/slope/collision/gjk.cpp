#include "slope/collision/gjk.hpp"
#include "slope/collision/primitives.hpp"
#include "slope/debug/log.hpp"
#include <tuple>

namespace slope {

namespace {

inline Vec3 support_diff(
    const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2, const Vec3& axis)
{
    return shape1->support_point(axis) - shape2->support_point(-axis);
}

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

Vec3 GJKCollider::update_point(Vec3 a)
{
    m_simplex_size = 1;
    m_simplex[0] = a;
    return -a;
}

Vec3 GJKCollider::update_line(Vec3 b, Vec3 a)
{
    m_simplex_size = 2;
    m_simplex[0] = b;
    m_simplex[1] = a;

    Vec3 ab = b - a;
    Vec3 ao = -a;

    return ab.cross(ao).cross(ab);
}

Vec3 GJKCollider::update_triangle(Vec3 c, Vec3 b, Vec3 a)
{
    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 ao = -a;
    Vec3 n = ab.cross(ac);

    if (ab.cross(n).dot(ao) >= 0.f) {
        return update_line(b, a);
    }

    if (n.cross(ac).dot(ao) >= 0.f) {
        return update_line(c, a);
    }

    m_simplex_size = 3;
    m_simplex[0] = c;
    m_simplex[1] = b;
    m_simplex[2] = a;

    return n.dot(ao) >= 0.f ? n : -n;
}

std::optional<Vec3> GJKCollider::update_tetrahedron(Vec3 d, Vec3 c, Vec3 b, Vec3 a)
{
    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 ad = d - a;
    Vec3 ao = -a;

    Vec3 abc = ab.cross(ac);
    if (abc.dot(ad) * abc.dot(ao) <= 0.f) {
        return update_triangle(c, b, a);
    }

    Vec3 abd = ab.cross(ad);
    if (abd.dot(ac) * abd.dot(ao) <= 0.f) {
        return update_triangle(d, b, a);
    }

    Vec3 acd = ac.cross(ad);
    if (acd.dot(ab) * acd.dot(ao) <= 0.f) {
        return update_triangle(d, c, a);
    }

    return std::nullopt;
}

bool GJKCollider::collide(const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2)
{
    constexpr float SUPPORT_EPSILON = 1e-12f;

    Vec3 init_axis = shape1->transform().translation() - shape2->transform().translation();
    m_simplex[0] = support_diff(shape1, shape2, init_axis);
    m_simplex_size = 1;

    for (int iter = 0; iter < 50; iter++) {
        std::optional<Vec3> axis;

        switch (m_simplex_size) {
        case 1: {
            axis = update_point(m_simplex[0]);
            break;
        }
        case 2: {
            axis = update_line(m_simplex[0], m_simplex[1]);
            break;
        }
        case 3: {
            axis = update_triangle(m_simplex[0], m_simplex[1], m_simplex[2]);
            break;
        }
        case 4: {
            axis = update_tetrahedron(m_simplex[0], m_simplex[1], m_simplex[2], m_simplex[3]);
            break;
        }
        default: {
            SL_VERIFY(false);
            break;
        }
        }

        if (!axis) {
            //slope::log::info("GJK Collision iter {}", iter);
            SL_ASSERT(m_simplex_size == 4);
            return true;
        }

        Vec3 new_pt = support_diff(shape1, shape2, *axis);

        if (axis->dot(new_pt) < SUPPORT_EPSILON)
            return false;

        m_simplex[m_simplex_size++] = new_pt;
    }

    return false;
}

void EPA::add_face(uint32_t a_idx, uint32_t b_idx, uint32_t c_idx)
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

Vec3 EPA::find_penetration_axis(
    const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2, const GJKCollider::Simplex& simplex)
{
    constexpr float PROXIMITY_EPSILON = 1e-5f;

    //records.clear();

    m_points.clear();
    m_heap.clear();
    m_edges_linear.clear();

    Vec3 best_axis;

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

    for (int iter = 0; iter < 50; iter++) {
        SL_VERIFY(!m_heap.empty());

        //records.push_back({});

        while (m_heap.front().obsolete) {
            std::pop_heap(m_heap.begin(), m_heap.end());
            m_heap.pop_back();
        }

        auto face = m_heap.front();
        std::pop_heap(m_heap.begin(), m_heap.end());
        m_heap.pop_back();

        best_axis = face.normal;

        auto new_pt = support_diff(shape1, shape2, face.normal);

        float dot = face.normal.dot(new_pt - m_points[face.a]);
        if (dot < PROXIMITY_EPSILON) {
            break;
        }

        int idx = static_cast<int>(m_points.size());
        m_points.push_back(new_pt);

        m_edges_linear.clear();
        m_edges_linear.push_back(encode_edge(face.a, face.b));
        m_edges_linear.push_back(encode_edge(face.a, face.c));
        m_edges_linear.push_back(encode_edge(face.b, face.c));


        for (auto& f: m_heap) {
            if (!f.obsolete && f.normal.dot(new_pt - m_points[f.a]) > 0.f) {
                f.obsolete = true;

                m_edges_linear.push_back(encode_edge(f.a, f.b));
                m_edges_linear.push_back(encode_edge(f.a, f.c));
                m_edges_linear.push_back(encode_edge(f.b, f.c));
            }
        }

        std::sort(m_edges_linear.begin(), m_edges_linear.end());
        m_edges_linear.push_back(0);

        uint64_t prev = 0;
        uint64_t prev_prev = 0;

        for (uint64_t code: m_edges_linear) {
            if (prev != code && prev != prev_prev) {
                auto[a, b] = decode_edge(prev);
                add_face(a, b, idx);
            }

            prev_prev = prev;
            prev = code;
        }
    }

    return best_axis;
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
