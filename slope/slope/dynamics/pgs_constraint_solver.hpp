#pragma once
#include "slope/dynamics/constraint_solver.hpp"

namespace slope {

// Projected Gauss-Seidel
class PGSConstraintSolver : public ConstraintSolver {
private:
    void solve_constraint(ConstraintData& c, float min_bound, float max_bound);
    void solve_impl() final;
};

} // slope
