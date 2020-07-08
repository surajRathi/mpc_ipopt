#include <mpc_ifopt/mpc.h>
#include <iostream>

int main() {
    std::cout << "a" << std::endl;

    mpc_ipopt::Params p;
    p.forward.steps = 5;
    p.forward.frequency = 1;
    p.limits.vel = {-1, 1};
    p.limits.acel = {-0.1, 0.1};

    mpc_ipopt::MPC mpc(p);
    mpc.run();
    return 0;

}