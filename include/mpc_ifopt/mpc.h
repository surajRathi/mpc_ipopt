//
// Created by suraj on 7/5/20.
//

#ifndef MPC_IPOPT_MPC_H
#define MPC_IPOPT_MPC_H

#include <cppad/cppad.hpp>
#include <eigen3/Eigen/Core>

#include <functional>


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


    class Range {
        size_t cur;
        const size_t last;
    public:
        Range(size_t start, size_t end) : cur(start), last(end) {
        }

        // Iterable functions
        const Range &begin() const { return *this; }

        const Range &end() const { return *this; }

        // Iterator functions
        bool operator!=(const Range &r) const {
            return cur < r.last;
        }

        void operator++() {
            ++cur;
        }

        size_t operator*() const { return cur; }

        size_t length() const { return last - cur; }

        friend Range operator+(const Range &a, const Range &b) {
            assert(a.last == b.cur);
            return {a.cur, b.last};
        }
    };

//    template<typename... T>
//    class zip {
//        std::tuple<T...> containers;
//    public:
//        zip(T &&... containers) : containers(std::make_tuple(containers...)) {}
//
//        auto begin() { return std::make_tuple(std::apply([](auto x) { return std::begin(x); }, containers)); }
//
//        auto end() { return std::make_tuple(std::apply([](auto x) { return std::end(x); }, containers)); }
//
//    };

    // Stores indices of variables in constraints
    class Indices {
        const size_t _v_r, _v_l;
    public:
        Range v_r(size_t offset = 0) const { return {0 + offset, _v_r}; }

        Range v_l(size_t offset = 0) const { return {_v_r + offset, _v_l}; }

        explicit Indices(const size_t N) : _v_r{N - 1}, _v_l{_v_r + N - 1} {}
    };

    class MPC {
        Params params;
        Indices _indices;

        const double dt;
        const size_t steps; // reference?

        /*const*/ size_t num_vars, num_constraints{};

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
