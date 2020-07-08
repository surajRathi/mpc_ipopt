//
// Created by suraj on 7/5/20.
//

#ifndef MPC_IPOPT_MPC_H
#define MPC_IPOPT_MPC_H

#include <cppad/cppad.hpp>
#include <eigen3/Eigen/Core>


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


    class Range {
        const size_t last;
        size_t cur;
    public:
        Range(size_t start, size_t end) : cur(start), last(end) {}

        // Iterable functions
        const Range &begin() const { return *this; }

        const Range &end() const { return *this; }

        // Iterator functions
        bool operator!=(const Range &) const { return cur < last; }

        void operator++() { ++cur; }

        size_t operator*() const { return cur; }
    };

    // Stores indices of variables in constraints
    struct Indices {
        const size_t _v_r, _v_l;

        Range v_r() const { return {0, _v_r}; }

        Range v_l() const { return {_v_r, _v_l}; }

        explicit Indices(const size_t N) : _v_r{N - 1}, _v_l{_v_r + N - 1} {}
    }

    class MPC {
        Params params;
        Indices _indices;

        const double dt;
        const size_t steps; // reference?

        /*const*/ size_t num_vars, num_constraints{};

        Dvector vars;
        LH<Dvector> vars_b, cons_b;

    public:
        using ADvector = CppAD::vector<CppAD::AD<double>>;

        explicit MPC(Params p);

        void run();

        // This sets the cost function and calculates constraints from variables
        void operator()(ADvector &outputs, ADvector &vars);
    };
}

#endif //MPC_IPOPT_MPC_H
