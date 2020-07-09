#include <mpc_ipopt/mpc.h>
#include <iostream>

int main() {
    mpc_ipopt::Params p;
    p.forward.steps = 5;
    p.forward.frequency = 1;
    p.limits.vel = {-1, 1};
    p.limits.acc = {-0.1, 0.1};
    p.wheel_dist = 0.65; //meters

    mpc_ipopt::MPC mpc(p);
    mpc.run();
    return 0;

}