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

    template<typename T>
    struct State_ {
        T x, y, theta, v_r, v_l;
    };

    using State = State_<Dvector::value_type>;


    // Stores a pair of values low and high
    template<typename T>
    struct LH {
        T low, high;
    };
    struct Params {
        struct Forward {
            double frequency;   // Hz
            size_t steps;       // time steps
        } forward;

        struct Limits {
            LH<double> vel, acc;
        } limits;

        struct Weights {
            double acc, vel, cte;
        } wt;
        double v_ref;
        /*unsigned*/ double wheel_dist; // meters
    };

    // Stores indices of variables in constraints


    class MPC {
        Params params;

        class Indices {
        private:
            // Variables
            const size_t _a_r, _a_l;
        public:
            const size_t vars_length;

            [[nodiscard]] Range a_r(size_t offset = 0) const { return {0 + offset, _a_r}; }

            [[nodiscard]] Range a_l(size_t offset = 0) const { return {_a_r + offset, _a_l}; }

        private:
            // Constraints
            const size_t _v_r, _v_l;
        public:
            const size_t cons_length;

            [[nodiscard]] Range v_r(size_t offset = 0) const { return {0 + offset, _v_r}; }

            [[nodiscard]] Range v_l(size_t offset = 0) const { return {_v_r + offset, _v_l}; }

            // Constructor
        public:
            explicit Indices(const size_t N) :
            // Variables
                    _a_r{N}, _a_l{_a_r + N},
                    vars_length{_a_l},
                    // Constraints
                    _v_r{N}, _v_l{_v_r + N},
                    cons_length{_v_l} {}
        } indices;

        const double dt;
        const size_t steps; // reference?

        Dvector _vars;
        LH<Dvector> vars_b, cons_b;

        std::vector<State> get_states(const Dvector &vars, const Dvector &cons, const State &initial);

    public:
        // Diffrentiable vector of doubles
        // Can also use std::vector or std::valarray
        using ADvector = CppAD::vector<CppAD::AD<double>>;
        using ADState = State_<ADvector::value_type &>;

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

//        void solve(const State &s, const Dvector &plan);

        bool solve(std::pair<double, double> &acc);

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
