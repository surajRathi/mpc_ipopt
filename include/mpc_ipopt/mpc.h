//
// Created by suraj on 7/5/20.
//

#ifndef MPC_IPOPT_MPC_H
#define MPC_IPOPT_MPC_H

#include <cppad/cppad.hpp>
#include <eigen3/Eigen/Core>

#include "helpers.h"


namespace mpc_ipopt {

    // Vector of doubles
    // Can also use std::vector or std::valarray
    using Dvector = CppAD::vector<double>;


    // Stores a pair of values low and high
    template<typename T>
    struct LH {
        T low, high;
    };


    struct Params {
        struct Forward {
            double frequency;   // Hz
            size_t steps;       // Seconds
        } forward;

        struct Limits {
            LH<double> vel, acc;
        } limits;

        /*unsigned*/ double wheel_dist; // meters
    };

    struct State {
        double x, y, theta, v_r, v_l;
    };

    // Stores indices of variables in constraints


    class MPC {
        Params params;

        class Indices {
            const size_t _a_r, _a_l;
            const size_t _v_r, _v_l;
        public:
            Range a_r(size_t offset = 0) const { return {0 + offset, _a_r}; }

            Range a_l(size_t offset = 0) const { return {_a_r + offset, _a_l}; }

            Range v_r(size_t offset = 0) const { return {0 + offset, _v_r}; }

            Range v_l(size_t offset = 0) const { return {_v_r + offset, _v_l}; }

            explicit Indices(const size_t N) : _a_r{N - 1}, _a_l{_a_r + N - 1}, // Variables
                                               _v_r{N - 1}, _v_l{_v_r + N - 1} // Constraints
            {}
        } indices;

        const double dt;
        const size_t steps; // reference?

        Dvector _vars;
        LH<Dvector> vars_b, cons_b;

    public:
        // Diffrentiable vector of doubles
        // Can also use std::vector or std::valarray
        using ADvector = CppAD::vector<CppAD::AD<double>>;

        class ConsWrapper {
            ADvector &_outputs;
        public:
            explicit ConsWrapper(ADvector &outputs) : _outputs(outputs) {}

            ADvector::value_type &operator[](size_t index) { return _outputs[1 + index]; }

            // const ADvector::value_type &operator[](size_t index) const { return _outputs[1 + index]; }
        };

        State state;
        Dvector global_plan;

        explicit MPC(Params p);

        void solve(const State &s, const Dvector &plan);

        void solve();

        // This sets the cost function and calculates constraints from variables
        void operator()(ADvector &outputs, ADvector &vars);
    };

    // Finds f(x) where f = coeffs[0] + coeffs[1] * x + coeffs[2] * x^2 ...
    template<typename Tc, typename Tx>
    Tx polyeval(const Tx &x, const Tc &coeffs) {
        Tx ret = 0, pow = 1;
        for (decltype(coeffs.size()) i = 0; i < coeffs.size(); i++, pow *= x) {
            ret += coeffs[i] * pow;
        }
        return ret;
    }

    /*// Finds f(x) where f = coeffs[0] + coeffs[1] * x + coeffs[2] * x^2 ...
    template<typename Tc>
    typename Tc::value_type polyeval(typename Tc::value_type x, Tc coeffs) {
        typename Tc::value_type ret = 0, pow = 1;
        for (decltype(coeffs.size()) i = 0; i < coeffs.size(); i++, pow *= x) {
            ret += coeffs[i] * pow;
        }
        return ret;
    }*/
}

#endif //MPC_IPOPT_MPC_H
