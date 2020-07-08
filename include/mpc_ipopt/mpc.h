//
// Created by suraj on 7/5/20.
//

#ifndef MPC_IPOPT_MPC_H
#define MPC_IPOPT_MPC_H

#include <cppad/cppad.hpp>
#include <eigen3/Eigen/Core>

#include "helpers.h"


namespace mpc_ipopt {
    using Dvector = CppAD::vector<double>;


    // Stores a pair of values low and high
    template<typename T>
    struct LH {
        T low, high;
    };


    struct Params {
        struct Forward {
            double frequency;
            size_t steps;
        } forward;

        struct Limits {
            LH<double> vel, acel;
        } limits;
    };

    struct State {
        double x, y, theta, v_r, v_l, a_r, a_l;
    };

    // Stores indices of variables in constraints
    class ConsIndices {
        const size_t _v_r, _v_l;
    public:
        Range v_r(size_t offset = 0) const { return {0 + offset, _v_r}; }

        Range v_l(size_t offset = 0) const { return {_v_r + offset, _v_l}; }

        explicit ConsIndices(const size_t N) : _v_r{N - 1}, _v_l{_v_r + N - 1} {}
    };

    class VarIndices {
        const size_t _a_r, _a_l;
    public:
        Range a_r(size_t offset = 0) const { return {0 + offset, _a_r}; }

        Range a_l(size_t offset = 0) const { return {_a_r + offset, _a_l}; }

        explicit VarIndices(const size_t N) : _a_r{N - 2}, _a_l{_a_r + N - 2} {}
    };

    class MPC {
        Params params;
        VarIndices varIndices;
        ConsIndices cons_indices;

        const double dt;
        const size_t steps; // reference?

//        /*const*/ size_t num_vars, num_constraints{};

        Dvector _vars;
        LH<Dvector> vars_b, cons_b;

    public:
        using ADvector = CppAD::vector<CppAD::AD<double>>;

        State state;

        explicit MPC(Params p);

        void run();

        // This sets the cost function and calculates constraints from variables
        void operator()(ADvector &outputs, ADvector &vars);
    };
}

#endif //MPC_IPOPT_MPC_H
