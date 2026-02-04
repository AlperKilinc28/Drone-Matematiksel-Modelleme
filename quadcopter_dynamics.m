function dXdt = quadcopter_dynamics(t, X, constants)

% X = [x, y, z, phi, theta, psi, dx, dy, dz, p, q, r]

% 1. Durum Vektörünün Ayrılması
x   = X(1);  y   = X(2);  z   = X(3);
phi = X(4);  theta = X(5);  psi = X(6);
dx  = X(7);  dy  = X(8);  dz  = X(9);
p   = X(10); q   = X(11); r   = X(12);

% 2. Sabitlerin Ayrılması
m = constants(1); g = constants(2); l = constants(3); k = constants(4); b = constants(5);
Ixx = constants(6); Iyy = constants(7); Izz = constants(8); 
Ax = constants(9); Ay = constants(10); Az = constants(11); Ar = constants(14);

% 3. Trigonometrik Değişkenler
Cphi = cos(phi); Sphi = sin(phi);
Ctheta = cos(theta); Stheta = sin(theta);
Cpsi = cos(psi); Spsi = sin(psi);

% 4. Kontrol Girişlerini Al (Rotor Hızlarının Karesi)
omega_sq = quadcopter_inputs(t, constants);
omega1_sq = omega_sq(1);
omega2_sq = omega_sq(2);
omega3_sq = omega_sq(3);
omega4_sq = omega_sq(4);

% 5. Toplam İtme (T) ve Torkları (Tau) Hesapla
T_total = k * sum(omega_sq);
tau_phi = l * k * (omega4_sq - omega2_sq);      % Roll Torku
tau_theta = l * k * (omega3_sq - omega1_sq);    % Pitch Torku
tau_psi = b * (omega1_sq - omega2_sq + omega3_sq - omega4_sq); % Yaw Torku

% 6. Dinamik Sürtünme Kuvveti (Lineer Model: F_drag = -A * v)
F_drag_x = -Ax * dx; 
F_drag_y = -Ay * dy; 
F_drag_z = -Az * dz;


%% --- DENKLEMLERİN UYGULANMASI (dX/dt) ---
% dXdt = [dx, dy, dz, dphi, dtheta, dpsi, ddx, ddy, ddz, dp, dq, dr]

% A. Konum ve Açıların 1. Türevleri (Hızlar)
dXdt(1:6) = [dx; dy; dz; p; q; r]; % Basitçe hızlar ivmedir.


% B. Doğrusal İvmeler (ddot_x, ddot_y, ddot_z)
% Sadece Duruş ve İtme ile kontrol edilir.
ddot_x = (T_total/m) * (Cpsi*Stheta*Cphi + Spsi*Sphi) + (F_drag_x / m);
ddot_y = (T_total/m) * (Spsi*Stheta*Cphi - Cpsi*Sphi) + (F_drag_y / m);
ddot_z = (T_total/m) * (Ctheta*Cphi) - g + (F_drag_z / m); 

dXdt(7) = ddot_x;
dXdt(8) = ddot_y;
dXdt(9) = ddot_z;


% C. Açısal İvmeler (dot_p, dot_q, dot_r)
% Sadece Kontrol Torkları ile kontrol edilir (Sadeleştirilmiş Model)
r = X(12);
Ar = 0.55;
dot_p = tau_phi / Ixx;  % Roll İvmesi
dot_q = tau_theta / Iyy; % Pitch İvmesi
dr = (tau_psi - Ar * r + (Ixx - Iyy) * p * q) / Izz;

dXdt(10) = dot_p;
dXdt(11) = dot_q;
dXdt(12) = dr;


% Vektör Boyutunu Düzenle (ode45 formatı için gerekli)
dXdt = dXdt';

end