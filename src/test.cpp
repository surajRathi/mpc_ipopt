#include <mpc_ifopt/mpc.h>
#include <iostream>

int main() {
    std::cout << "a" << std::endl;

    mpc_ipopt::MPC mpc;
    std::cout << mpc.temp << std::endl;
    mpc.run();
    return 0;

}