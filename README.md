# mpc_ipopt
Uses Model Predictive Control for path planning of a diffrential drive vehicle. Uses IPOPT 

Moved to project-TARIV/mpc_ipopt . Based on project-TARIV/mpc_lib


## Dependencties
- **catkin** OPTIONAL  
  Project is made to be compiled with catkin for the ROS framework, but does work with just cmake.

- **cmake**: Build System


- Install **cppad** from your package manager  
For Ubuntu:
```bash
sudo apt-get install cppad
```

- Installing **Ipopt**
Currently installs to /usr/local. Editing the script can change that.  
For Ubuntu:
```bash
sudo apt-get install gfortran
sudo apt-get install unzip
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip && unzip Ipopt-3.12.7.zip && rm Ipopt-3.12.7.zip
sudo bash install_ipopt.sh ./Ipopt-3.12.7/
```

## Working
We solve an optimisation problem for acceleration in each motor.

- A simple kinematic model has been selected  
If v_r and v_l are velocities in either frame, and a_r and a_l are the respective constraints.  
x_i = x_{i-1} + (v_r + v_l) * dt * cos(theta_{i-1}) / 2 
y_i = y_{i-1} + (v_r + v_l) * dt * sin(theta_{i-1}) / 2
theta_i = theta_{i-1} + (v_r - v_l) * dt / distance_between_wheels


- Constraints
The path to follow has been defined as a polynomial f(x).

The objective function is of the form \sum{w_i*(a_i)^2}

The different terms (a_i)
v_r + v_l - 2 * vref  
v_r - v_l  
a_r + a_l  
a_r - a_l  
f(x) - y  

It is easy to expand to this list.



## References

[Based on project-TARIV/mpc_lib](https://github.com/project-TARIV/mpc_lib)  
Am a member of that organisation.
