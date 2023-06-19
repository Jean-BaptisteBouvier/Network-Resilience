# Network Resilience
Repository for the paper "Resilient Stabilizability of Linear Networks"



## IEEE 39-bus system

Picture of the [IEEE 39-bus system](https://icseg.iti.illinois.edu/ieee-39-bus-system/) from [1].

![IEEE 39-bus system](pictures/IEEE_39.PNG "IEEE 39-bus system")

We obtain the linearized network equation from [2].
After the loss of control authority over generator bus 39, we split the network state between $x_q = \big( \delta_{39}, \dot \delta_{39} \big)$ and $\chi$ for the remaining states.

![Evolution of state $\chi$ and its analytical bound](pictures/IEEE_X.png "Evolution of state $\chi$ and its analytical bound")


## File Structure

- `main_IEEE_net.m` runs the simulation and compute bounds on the network states.
- `Pnorm.m` calculates the $P$-norm of a vector $x$ as $\\|x\\|_P = \sqrt{x^\top P x}$, where $P$ is a positive definite matrix.








## Citation
```
@article{bouvier2023networks,  
  title = {},   
  author = {Jean-Baptiste Bouvier and Sai Pushpak Nandanoori and Melkior Ornik},    
  journal = {},    
  year = {2023},   
  volume = {},
  pages = {},
  doi = {}
}
```


## Contributors

- [Jean-Baptiste Bouvier](https://jean-baptistebouvier.github.io/)
- [Sai Pushpak Nandanoori](https://sites.google.com/view/saipushpakn)
- [Melkior Ornik](https://mornik.web.illinois.edu/)



## References

[1] T. Athay, R. Podmore, and S. Virmani, [“A practical method for the direct analysis of transient stability,”](https://ieeexplore.ieee.org/abstract/document/4113518) IEEE Transactions on Power Apparatus and Systems, vol. PAS-98, no. 2, pp. 573 – 584, 1979.

[2] S. P. Nandanoori, S. Kundu, J. Lian, U. Vaidya, D. Vrabie, and K. Kalsi, [“Sparse control synthesis for uncertain responsive loads with stochastic stability guarantees,”]( https://ieeexplore.ieee.org/abstract/document/9489331) IEEE Transactions on Power Systems, vol. 37, no. 1, pp. 167 – 178, 2022.
