%%%% IEEE 39-bus network
% after using generator 1 as a reference

% Only the generator buses are actuated.

clearvars
clc



%%%%%%%%%%%%%%%%%%%% Inputs %%%%%%%%%%%%%%%%

failure_id = 39; % id of the failing generator bus in [31, 39]
video_name = 'IEEE_test_video';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


assert(failure_id >= 31, 'The failing bus must be a generator, i.e., of id >= 31.')
assert(failure_id <= 39, 'The failing bus must be a generator, i.e., of id <= 39.')

load('openloop_sys_matrices_June16.mat')
% using generator bus 31 as a reference
failure_id = failure_id -1;

%%% Initial convention
% States 1 to 9 are the phase angles of generator buses 31 to 39.
% States 10 to 19 are the frequencies of the generator buses.
% States 20 to 48 are the phase angles of the load buses.
%%% Figure convention
% States 1 to 29 are the phase angles of the load buses.
% States 30 to 39 are the phase angles of the generator buses.
% States 40 to 49 are the frequencies of the generator buses.


B = B_o_hat(2:end,11:20);
A = A_o;
affine = D_o_hat(2:end);
% State equation  dx/dt = Ax + Bu + affine
e = pinv(A)*affine;
assert( norm(A*e - affine) < 1e-10 ) % verify that state can be translated to the origin
% New state: y = x+e
% New state equation dy/dt = Ay + Bu

%%% Reorganizing states into the figure convention
Agg = A(1:19, 1:19); All = A(20:end, 20:end);
Agl = A(1:19, 20:end); Alg = A(20:end, 1:19);
A = [All, Alg; Agl, Agg];
Bg = B(1:19,:); Bl = B(20:end,:);
B = [Bl; Bg];


nb_bad_generators = length(failure_id); % number of failing generator buses
id_bad_states = [failure_id, failure_id + 10]; % id of the states to isolate
nb_bad_states = length(id_bad_states); % number of states to isolate




%%% Separating the states of the network based on their subsystem
A_q = A(id_bad_states, id_bad_states);
A_hat = A; A_hat(id_bad_states, :) = [];
A_hat(:, id_bad_states) = [];
n = length(A_hat(1,:));
D_q_col = A(:, id_bad_states); D_q_col(id_bad_states,:) = [];
D_q_row = A(id_bad_states, :); D_q_row(:,id_bad_states) = [];

disp('max Re(lambda(A_q)) = ' + string(max(real(eig(A_q)))) + '  ||D_q_row|| = ' + string(norm(D_q_row)))

%%% Separating the actuators of the network based on their subsystem
B_hat = B;  B_hat(id_bad_states, :) = [];
B_hat(:, failure_id-28) = []; % columns are in [1, 10] while failure_id is in [30, 39]
m = length(B_hat(1,:));
B_q = B(id_bad_states, failure_id-28);

%%% Initial states
x_q_0 = zeros(nb_bad_states,1);
X0 = ones(n,1);

assert( max(real(eig(A_q))) < 0, 'Subsystem '+string(id_bad_states)+' is not Hurwitz.')

%%% MATLAB function ctrb(A_hat, B_hat) does not work because the full
%%% controllability matrix is too large: 47 x 423
ctrb_mat = B_hat; % partial controllability matrix
scaling = 40; % scaling down for controllability matrix to be computable
power = 0; % highest power of the A_hat matrix in ctrb_mat 
while power < n && rank(ctrb_mat) < n
    ctrb_mat = [B_hat, A_hat*ctrb_mat/scaling];
    power = power + 1;
%     disp(rank(ctrb_mat))
end
assert(rank(ctrb_mat) == n, 'Functioning network is not controllable.')


%%% Bounding the malfunctioning subsystem (Proposition 6)
Q_q = eye(nb_bad_states);
% P_q = lyap(A_q', Q_q);
P_q = [1.17571126026256,0.0268424708161375;0.0268424708161375,0.0469463587855964];
assert( norm( A_q'*P_q + P_q*A_q + Q_q) < 1e-12, 'Lyapunov equation is not holding with P_N')
alpha_q = min(eig(Q_q))/(2*max(eig(P_q)));
z = Pnorm(B_q*ones(nb_bad_generators,1), P_q); % single actuator loss


%%% Stabilizing gain for the controlled network (Proposition 9)
K = lqr(A_hat, B_hat, 0.2*eye(n), eye(m)); 
% K = ;
Q_hat = eye(n);
P_hat = lyap( (A_hat - B_hat*K)', Q_hat);
% P_hat = ;
assert( norm( (A_hat - B_hat*K)'*P_hat + P_hat*(A_hat - B_hat*K) + Q_hat) < 2e-10, 'Lyapunov equation is not holding with P_hat')

alpha = min(eig(Q_hat))/(2*max(eig(P_hat)));
gamma = sqrt(max(eig(D_q_col'*P_hat*D_q_col))/min(eig(P_q)));
gamma_q = sqrt(max(eig(D_q_row'*P_q*D_q_row))/min(eig(P_hat)));

disp('alpha prod = '+string(alpha*alpha_q)+ '   gamma prod = '+string(gamma*gamma_q))
if alpha*alpha_q < gamma*gamma_q
    disp('Functioning network is not stabilizable for failure '+ string(failure_id))
end



%%% ODE constants
p = gamma*z/(alpha*alpha_q - gamma*gamma_q);
r_plus = 0.5*(alpha_q - alpha + sqrt( (alpha - alpha_q)^2 + 4*gamma*gamma_q));
r_minus = 0.5*(alpha_q - alpha - sqrt( (alpha - alpha_q)^2 + 4*gamma*gamma_q));
h_minus =  ((alpha_q - alpha - r_plus)*Pnorm(X0, P_hat) + gamma*Pnorm(x_q_0, P_q) + (r_plus - alpha_q)*p)/(r_minus-r_plus);
h_plus = ((alpha_q - alpha - r_minus)*Pnorm(X0, P_hat) + gamma*Pnorm(x_q_0, P_q) + (r_minus - alpha_q)*p)/(r_plus-r_minus);



%%% Verification of the ODE solution
t_f = 1e-2;
dt = t_f*1e-2;
times = 0:dt:t_f;
nb_steps = length(times);


%%% Simulation test
A_net = [A_hat - B_hat*K, D_q_col; D_q_row, A_q]; % overall network A matrix
C_net = [zeros(n, nb_bad_generators); B_q]; % overall network C matrix
w_net = eye(nb_bad_generators); % constant undesirable input in [-1, 1]
[~, states] = ode45(@(t, X_net) A_net*X_net + C_net*w_net, times, [X0; x_q_0]);

norm_X = zeros(1, nb_steps);
norm_x_q = zeros(1, nb_steps);
max_KX = zeros(1, nb_steps);
for i = 1:nb_steps
    X = states(i, 1:n)';
    norm_X(i) = Pnorm( X, P_hat);
    norm_x_q(i) = Pnorm( states(i, n+1:end)', P_q);
    max_KX(i) = max(abs(K*X));
end


%%% Predicted upper bounds
up_bd_x_q1 = zeros(1, nb_steps);
up_bd_x_q1(1) = Pnorm(x_q_0, P_q);
intgral = 0;
for i = 1:nb_steps-1
    t = i*dt;
    intgral = intgral + dt*exp(alpha_q*t)*(z + Pnorm(D_q_row*states(i+1,1:n)', P_q));
    up_bd_x_q1(i+1) = exp(-alpha_q*t)*( Pnorm(x_q_0, P_q) + intgral); % Proposition 6
end

up_bd_X = p + h_plus*exp((r_plus-alpha_q)*times) + h_minus*exp((r_minus-alpha_q)*times);
up_bd_X = max(up_bd_X, zeros(1, nb_steps)); % Proposition 9

up_bd_x_q2 = zeros(1, nb_steps);
converged_id = find(norm_X < 1e-4, 1, 'first'); % index at which X has converged
if isempty(converged_id)
    converged_id = nb_steps;
end
pre = times(1:converged_id); 
up_bd_x_q2(1:converged_id) = alpha*z*(1-exp(-alpha_q*pre))/(alpha*alpha_q - gamma*gamma_q) + exp(-alpha_q*pre).*(Pnorm(x_q_0, P_q) + gamma_q*h_plus*(exp(r_plus*pre)-1)/r_plus + gamma_q*h_minus*(exp(r_minus*pre)-1)/r_minus);   
post = times(converged_id+1:end);
up_bd_x_q2(converged_id+1:end) = z/alpha_q + (Pnorm(x_q_0, P_q) - z/alpha_q)*exp(-alpha_q*post);
up_bd_x_q2 = max(up_bd_x_q2, zeros(1, nb_steps)); % Proposition 8



%%% Plots
figure
hold on; grid on
plot(times, norm_X, 'LineWidth', 2)
plot(times, up_bd_X, ':', 'LineWidth', 2)
legend('$\|\chi(t)\|_{\hat{P}}$', 'bound (15)', 'Interpreter','latex')
% title('Network state')
xlabel('t (s)')
set(gca, 'Fontsize', 16)

figure
hold on; grid on
plot(times, norm_x_q, 'LineWidth', 2)
plot(times, up_bd_x_q1, ':', 'LineWidth', 2)
plot(times, up_bd_x_q2, '--', 'LineWidth', 2)
legend('$\|x_q(t)\|_{P_q}$', 'bound (7)', 'bound (16)','Interpreter','latex')
% title('Malfunctioning state')
xlabel('t (s)')
set(gca, 'Fontsize', 16)




%%% Redoing longer state propagation to show x_q close to its bound
dt = 0.1;
times = 0:dt:10;
[~, states] = ode45(@(t, X_net) A_net*X_net + C_net*w_net, times, [X0; x_q_0]);

nb_steps = length(times);

norm_x_q = zeros(1, nb_steps);
for i = 1:nb_steps
    norm_x_q(i) = Pnorm( states(i, n+1:end)', P_q);
end

up_bd_x_q1 = zeros(1, nb_steps);
up_bd_x_q1(1) = Pnorm(x_q_0, P_q);
intgral = 0;
for i = 1:nb_steps-1
    t = i*dt;
    intgral = intgral + dt*exp(alpha_q*t)*(z + Pnorm(D_q_row*states(i+1,1:n)', P_q));
    up_bd_x_q1(i+1) = exp(-alpha_q*t)*( Pnorm(x_q_0, P_q) + intgral); % Proposition 6
end

figure
hold on; grid on
plot(times, norm_x_q, 'LineWidth', 2)
plot(times, up_bd_x_q1, ':', 'LineWidth', 2)
legend('$\|x_q(t)\|_{P_q}$', 'bound (7)','Interpreter','latex')
xlabel('t (s)')
set(gca, 'Fontsize', 16)





%%% Even longer state propagation to show both states are bounded despite
%%% bounds diverging
dt = 0.1;
times = 0:dt:40;
[~, states] = ode45(@(t, X_net) A_net*X_net + C_net*w_net, times, [X0; x_q_0]);

nb_steps = length(times);

norm_X = zeros(1, nb_steps);
norm_x_q = zeros(1, nb_steps);
max_KX = zeros(1, nb_steps);
for i = 1:nb_steps
    X = states(i, 1:n)';
    norm_X(i) = Pnorm( X, P_hat);
    norm_x_q(i) = Pnorm( states(i, n+1:end)', P_q);
    max_KX(i) = max(abs(K*X));
end

up_bd_X = p + h_plus*exp((r_plus-alpha_q)*times) + h_minus*exp((r_minus-alpha_q)*times);
up_bd_X = max(up_bd_X, zeros(1, nb_steps)); % Proposition 9

up_bd_x_q1 = zeros(1, nb_steps);
up_bd_x_q1(1) = Pnorm(x_q_0, P_q);
intgral = 0;
for i = 1:nb_steps-1
    t = i*dt;
    intgral = intgral + dt*exp(alpha_q*t)*(z + Pnorm(D_q_row*states(i+1,1:n)', P_q));
    up_bd_x_q1(i+1) = exp(-alpha_q*t)*( Pnorm(x_q_0, P_q) + intgral); % Proposition 6
end

figure
hold on; grid on
plot(times, norm_X, 'LineWidth', 2)
plot(times, norm_x_q, ':', 'LineWidth', 2)
legend('$\|\chi(t)\|_{\hat{P}}$', '$\|x_q(t)\|_{P_q}$', 'Interpreter','latex')
ylim([0 1.1*max(norm_x_q)])
xlabel('t (s)')
set(gca, 'Fontsize', 16)

figure
hold on; grid on
plot(times, max_KX, 'LineWidth', 2)
legend('$\max_i \{ |\hat{u}_i(t)| = |K \chi_i(t)|\}$','Interpreter','latex')
xlabel('t (s)')
ylim([0 1])
set(gca, 'Fontsize', 16)








%%% Video
% 
% states = [states(:,1:id_bad_states(1)-1), states(:,end-1), states(:,id_bad_states(1):end)];
% states = [states(:,1:id_bad_states(2)-1), states(:,end), states(:,id_bad_states(2):end)];
% states(:,end-1:end) = [];
% 
% video_state_evolution(times, states, id_bad_states, video_name)






