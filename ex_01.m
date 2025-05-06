% H_2 Exercise
% Robust control Seminar
% Dr. Julio A García-Rodríguez

clc, clear all, close all

% -----------------------------
% Set random seed for reproducibility
% -----------------------------
rng(5, 'twister');

% -----------------------------
% Generate a random state-space system and transpose it. 
% This command creates a 4-output, 5-input stable model and then takes its Hermitian
% conjugate. This operation yields a 5-output, 4-input unstable model.
% -----------------------------
P = rss(3, 3, 3)'; % 3 states, 5 inputs, 4 outputs → transposed: 4 inputs, 5 outputs
pole(P)            % Confirm that P is unstable. All the poles are in the right half-plane.

% -----------------------------
% Display state-space matrices
% -----------------------------
A = P.A;
B = P.B;
C = P.C;
D = P.D;

disp('Matrix A:'); disp(A);
disp('Matrix B:'); disp(B);
disp('Matrix C:'); disp(C);
disp('Matrix D:'); disp(D);


% -----------------------------
% Controllability and observability analysis
% -----------------------------
ctrbRank = rank(ctrb(A, B))
obsvRank = rank(obsv(A, C))


% -----------------------------
% Step response
% -----------------------------
figure
step(P)
title('Step Response of P');
grid on

% -----------------------------
% Impulse response
% -----------------------------
figure
impulse(P)
title('Impulse Response of P');
grid on

% -----------------------------
% Frequency response (Bode plot)
% -----------------------------
figure
bode(P)
title('Bode Plot of P');
grid on

% -----------------------------
% Pole-zero map
% -----------------------------
figure
pzmap(P)
title('Pole-Zero Map of P');
grid on


% -----------------------------
% Controller synthesis
% -----------------------------
[K,CL,GAM] = h2syn(P,1,1);

size(K)

Ak = K.A;
Bk = K.B;
Ck = K.C;
Dk = K.D;

disp('Controller A matrix:'); disp(Ak);
disp('Controller B matrix:'); disp(Bk);
disp('Controller C matrix:'); disp(Ck);
disp('Controller D matrix:'); disp(Dk);

% -----------------------------
% Impulse response 
% -----------------------------
figure
impulse(CL)
title('Impulse Response of CL');
grid on

% -----------------------------
% Step response of closed-loop system
% -----------------------------

figure
step(CL)
title('Step Response of Closed-Loop System (CL)');
grid on

bodemag(K)






