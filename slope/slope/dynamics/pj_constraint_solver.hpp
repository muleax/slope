#pragma once
#include "slope/dynamics/constraint_solver.hpp"

namespace slope {

// Experimental Projected Jacobi constraint solver
class PJConstraintSolver : public ConstraintSolver {
private:
    void solve_constraint(ConstraintData& c);
    void solve_iterations() final;
};

} // slope
