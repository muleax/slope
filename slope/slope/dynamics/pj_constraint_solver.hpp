#pragma once
#include "slope/dynamics/constraint_solver.hpp"

namespace slope {

// Projected Jacobi
class PJConstraintSolver : public ConstraintSolver {
private:
    void solve_constraint(ConstraintData& c);
    void solve_impl() final;
};

} // slope
