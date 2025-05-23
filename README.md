# Resilience of Linear Networks

Repository for the paper "Losing Control of your Network? Try Resilience Theory", which is available on [ArXiv](https://arxiv.org/abs/2306.16588) and published in the journal [IEEE Transactions on Control of Network Systems](https://ieeexplore.ieee.org/abstract/document/10605041).

Here are all the codes and data necessary to reproduce the simulations of the paper.
We study three different scenarios:
- a fully actuated 3-component network losing control over one of its actuators;
- an underactuated 3-component network losing control over one of its actuators;
- the IEEE 39-bus system losing control over one of its generator buses.


## Fully actuated 3-component network

This is an academic example aimed at illustrating the resilience theory for fully actuated systems.
The dynamics of the network are:
```math
\dot \chi(t) = \begin{bmatrix} -1 & 0.3 \\ 0.3 & -1 \end{bmatrix} \chi(t) + \begin{bmatrix} 2 & 0 \\ 0 & 2 \end{bmatrix} \hat{u}(t) + \begin{bmatrix} 0.3 \\ 0.3 \end{bmatrix} x_q(t),
```
```math
\dot x_q(t) = -x_q(t) + u_q(t) + 2w_q(t) + \begin{array} 0.3 & 0.3 \end{array} \chi(t),
```
with
```math
\chi(0) = \begin{bmatrix} 1 \\ 1 \end{bmatrix}, \quad x_q(0) = 0, \quad \hat{u}(t) = \begin{bmatrix} \hat{u}_1(t) \\ \hat{u}_1(t) \end{bmatrix} \in \hat{\mathcal{U}} = [-1, 1]^2,
```
$u_q(t) \in \mathcal{U}_q = [-1, 1]$ and $w_q(t) \in \mathcal{W}_q = [-1, 1]$.
After a cyber-attack we lose control authority over the actuator producing $w_q$
Notice that control input $u_q$ cannot overcome all undesirable inputs $w_q$.
Then, we want to bound how far away from the origin can $x_q$ be driven by $w_q$ despite our best effort with $u_q$.
To do so, we first solve Lyapunov equations $A_q^\top P_q + P_q A_q = -Q_q$ and $(\hat{A}+\hat{D})^\top \hat{P} + \hat{P} (\hat{A}+\hat{D}) = -\hat{Q}$ with the function `lyap` on MATLAB:
```math
Q_q = 1, \quad P_q = 0.5, \quad \hat{Q} = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix},\quad \text{and}\quad \hat{P} = \begin{bmatrix} 0.23 & 0.05 \\ 0.05 & 0.5 \end{bmatrix}.
```
Then, the resilient stabilizability conditions established in this paper are satisfied: $\gamma \gamma_q = 0.25 < \alpha \alpha_q = 0.7$ and $\gamma z_{max}^{P_q} = 0.5 < \alpha_q b_{min}^{\hat{P}} = 2$.
As shown below, $\chi$ is indeed resiliently stabilizable in finite time by $\hat{B}\hat{u} = \frac{-\chi(t)}{ \\| \chi(t) \\| b_{min}^{\hat{P}}}$.


![Bounding network state](pictures/academic_full_X.png "Bounding network state")

The numbers in parenthesis in the legend of the figures refer to equation numbers used in the [paper](https://arxiv.org/abs/2306.16588).
The state $x_q$ of the malfunctioning system can also be bounded even in the worst-case scenario as shown below.

![Bounding malfunctioning network state](pictures/academic_full_x_q.png "Bounding malfunctioning network state")

These simulations are performed with `test_full_actuation.m`.





## Underactuated 3-component network


We now modify $\hat{B}$ not to be full rank anymore, while maintinging controllability of the pair $(\hat{A} + \hat{D}, \hat{B})$.
The updated network dynamics for $\chi$ are
```math
\dot \chi(t) = \begin{bmatrix} -1 & 0.3 \\ 0.3 & -1 \end{bmatrix} \chi(t) + \begin{bmatrix} 2 \\ 0 \end{bmatrix} \hat{u}(t) + \begin{bmatrix} 0.3 \\ 0.3 \end{bmatrix} x_q(t),
```
and the dynamics of $x_q$ are left unchanged:
```math
\dot x_q(t) = -x_q(t) + u_q(t) + 2w_q(t) + \begin{array} 0.3 & 0.3 \end{array} \chi(t),
```
with
```math
\chi(0) = \begin{bmatrix} 1 \\ 1 \end{bmatrix}, \quad x_q(0) = 0, \quad \hat{u}(t) = \begin{bmatrix} \hat{u}_1(t) \\ \hat{u}_1(t) \end{bmatrix} \in \hat{\mathcal{U}} = [-1, 1]^2,
```
$u_q(t) \in \mathcal{U}_q = [-1, 1]$ and $w_q(t) \in \mathcal{W}_q = [-1, 1]$.
As previously, we assume a loss of control authority over $w_q$ that control input $u_q$ is still unable to counteract.
Because of the underactuation of dynamics $\chi$, we cannot use the same controler as above.
Instead, we determine a gain matrix $K$ and positive definite matrices $\hat{P}$ and $\hat{Q}$ with MATLAB functions `lqr` and `lyap`:
```math
K = \begin{bmatrix} 0.64 & 0.15 \end{bmatrix}, \quad \hat{P} = \begin{bmatrix} 0.22 & 0.04 \\ 0.04 & 0.5 \end{bmatrix},\quad \text{and} \quad \hat{Q} = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}.
```
Then, the linear feedback control $\hat{u}(t) := -K\chi(t) \in [-1, 1]$ is admissible as shown below. 

![Admissible control input](pictures/academic_KX.png "Admissible control input")

With this controller the resilience condition $\gamma \gamma_q = 0.24 < \alpha \alpha_q = 0.98$ holds.
Thus, network state $\chi$ can be resiliently bounded as illustrated below.

![Bounding network state](pictures/academic_X.png "Bounding network state")

The state $x_q$ of the malfunctioning system can also be bounded even in the worst-case scenario as shown below.

![Bounding malfunctioning network state](pictures/academic_x_q.png "Bounding malfunctioning network state")

These simulations are performed with `test_underactuation.m`.





## IEEE 39-bus system

We now study the resilience of the IEEE 39-bus system.
This network is composed of 29 load buses numbered 1 to 29 on the figure below, and 10 generator buses numbered 30 to 39.
The picture of the [IEEE 39-bus system](https://icseg.iti.illinois.edu/ieee-39-bus-system/) is taken from [1].

![IEEE 39-bus system](pictures/IEEE_39.PNG "IEEE 39-bus system")

We obtain the linearized network equation from [2].
After the loss of control authority over generator bus 39, we split the network state between $x_q = \big( \delta_{39}, \dot \delta_{39} \big)$ and $\chi$ for the remaining states.
The malfunctioning dynamics are
```math
    \begin{bmatrix} \dot \delta_{39}(t) \\ \dot \omega_{39}(t) \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ -18.6 & -11.2 \end{bmatrix}  \begin{bmatrix} \delta_{39}(t) \\ \omega_{39}(t) \end{bmatrix} + \begin{bmatrix} 0 \\ 0.22 \end{bmatrix} w_q(t) + D_{q,\_} \chi(t).
```
We choose initial states $\chi(0) = \mathbf{1}$, $\delta_{39}(0) = 0$ rad and $\omega_{39}(0) = 0$ Hz.
Since pair $(\hat{A} + \hat{D}, \hat{B})$ is controllable, we can find a stabilizing gain matrix $K$ for the network dynamics. 
However, the resilient stability condition $\gamma \gamma_q < \alpha \alpha_q$ is not satisfied.
Indeed, $\gamma \gamma_q = 6.3 \times 10^4$, while $\alpha \alpha_q = 5.7 \times 10^{-3}$. 
This magnitude difference leads to the exponential divergence of bounds (15) and (16), as seen below.

![Evolution of the network state and its analytical bound](pictures/IEEE_X.png "Evolution of the network state and its analytical bound")

![Evolution of the malfunctioing state and its analytical bounds](pictures/IEEE_x_q.png "Evolution of the malfunctioing state and its analytical bounds")

Bound (7) remains a reasonable bound for malfunctioning state $x_q$ over a much longer time horizon as illustrated below.

![Evolution of the malfunctioning state and its analytical bound](pictures/IEEE_x_q_long.png "Evolution of the malfunctioning state and its analytical bound")

The choice of $K$ ensures admissibility of controller $\hat{u} = -K\chi$ by guaranteeing $\underset{i, t}{\max} |K\chi_i(t)| \leq 1$ as shown below.

![Admissible feedback control](pictures/IEEE_KX.png "Admissible feedback control")

Despite having $\gamma \gamma_q\gg \alpha \alpha_q$, the coupling does not destabilize states $x_q$ and $\chi$, which are both bounded, as shown on this last figure.

![Evolution of the network state and the malfunctioning state](pictures/IEEE_X_x_q.png "Evolution of the network state and the malfunctioning state")

These simulations are performed with `main_IEEE_net.m`.





## File Structure

- `main_IEEE_net.m` runs the simulation of the IEEE 39-bus network and compute all discussed bounds on the states $\chi$ and $x_q$.
- `test_full_actuation.m` runs the simulation of the fully actuated 3-component network.
- `test_underactuation.m` runs the simulation of the underactuated 3-component network.
- `Pnorm.m` calculates the $P$-norm of a vector $x$ as $\\|x\\|_P = \sqrt{x^\top P x}$, where $P$ is a positive definite matrix.







## Citation
```
@article{bouvier2023networks,  
  title = {Losing Control of your Linear Network? Try Resilience Theory},   
  author = {Jean-Baptiste Bouvier and Sai Pushpak Nandanoori and Melkior Ornik},    
  journal = {IEEE Transactions on Control of Network Systems},    
  year = {2025},   
  volume = {12},
  number = {1},
  pages = {980-992},
  doi = {10.1109/TCNS.2024.3431409}
}
```


## Contributors

- [Jean-Baptiste Bouvier](https://jean-baptistebouvier.github.io/)
- [Sai Pushpak Nandanoori](https://sites.google.com/view/saipushpakn)
- [Melkior Ornik](https://mornik.web.illinois.edu/)



## References

[1] T. Athay, R. Podmore, and S. Virmani, [“A practical method for the direct analysis of transient stability,”](https://ieeexplore.ieee.org/abstract/document/4113518) IEEE Transactions on Power Apparatus and Systems, vol. PAS-98, no. 2, pp. 573 – 584, 1979.

[2] S. P. Nandanoori, S. Kundu, J. Lian, U. Vaidya, D. Vrabie, and K. Kalsi, [“Sparse control synthesis for uncertain responsive loads with stochastic stability guarantees,”]( https://ieeexplore.ieee.org/abstract/document/9489331) IEEE Transactions on Power Systems, vol. 37, no. 1, pp. 167 – 178, 2022.
