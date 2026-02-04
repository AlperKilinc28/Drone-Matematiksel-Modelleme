function draw_quad(X_current, constants)
% X_current: [x, y, z, phi, theta, psi, dx, dy, dz, p, q, r]
% constants: [m, g, l, k, b, Ixx, Iyy, Izz, Ax, Ay, Az, omega_hover_sq]

% Durum ve Sabitlerin Ayrılması
x = X_current(1); y = X_current(2); z = X_current(3);
phi = X_current(4); theta = X_current(5); psi = X_current(6);
l = constants(3); % Rotor kol uzunluğu (0.225m)

% --- GÖRSEL AYARLAR ---
arm_width = 0.02; % Kol kalınlığı
arm_height = 0.01; % Kol yüksekliği
body_size = 0.1; % Gövde merkezi küp boyutu

% 1. DÖNÜŞ MATRİSİ (R) HESAPLANMASI (Aynı kalır)
Cphi = cos(phi); Sphi = sin(phi);
Ctheta = cos(theta); Stheta = sin(theta);
Cpsi = cos(psi); Spsi = sin(psi);

R = [Cpsi*Ctheta, Cpsi*Stheta*Sphi - Spsi*Cphi, Cpsi*Stheta*Cphi + Spsi*Sphi;
     Spsi*Ctheta, Spsi*Stheta*Sphi + Cpsi*Cphi, Spsi*Stheta*Cphi - Cpsi*Sphi;
     -Stheta, Ctheta*Sphi, Ctheta*Cphi];

% 2. ROTOR KOLLARININ VE GÖVDENİN KÖŞE NOKTALARINI HESAPLAYAN ALT FONKSİYON
% Bu fonksiyon basit 3D kutu çizmek için köşe noktalarını (vertices) döndürür.
[P_quad_body, F_quad_body] = create_quad_geometry(l, arm_width, arm_height, body_size);


% 3. DÖNÜŞÜM ve ÖTELEME
% P_world = R * P_body + [x; y; z]
P_world = R * P_quad_body;
P_world(1, :) = P_world(1, :) + x; % X öteleme
P_world(2, :) = P_world(2, :) + y; % Y öteleme
P_world(3, :) = P_world(3, :) + z; % Z öteleme


% 4. 3D ÇİZİM (patch komutu ile katı cisimler çizimi)
clf; % Önceki çizimi temizle
hold on; 

% Tüm yüzeyleri tek seferde çizmek için P_world'ün transpozesini (P_world') kullan
patch('Vertices', P_world', 'Faces', F_quad_body.body, 'FaceColor', [0.8 0.1 0.1], 'EdgeColor', 'k'); % Gövde (Kırmızı)
patch('Vertices', P_world', 'Faces', F_quad_body.arm1, 'FaceColor', [0.1 0.1 0.8], 'FaceAlpha', 0.8); % Kol 1 (Mavi)
patch('Vertices', P_world', 'Faces', F_quad_body.arm2, 'FaceColor', [0.1 0.1 0.8], 'FaceAlpha', 0.8); % Kol 2
patch('Vertices', P_world', 'Faces', F_quad_body.arm3, 'FaceColor', [0.1 0.1 0.8], 'FaceAlpha', 0.8); % Kol 3
patch('Vertices', P_world', 'Faces', F_quad_body.arm4, 'FaceColor', [0.1 0.1 0.8], 'FaceAlpha', 0.8); % Kol 4

% Ayarlar
xlabel('X Konumu (m)'); ylabel('Y Konumu (m)'); zlabel('Z Konumu (m)');
title('Gelişmiş Quadcopter Uçuş Animasyonu');
grid on;

% Görüş Alanı ve Oran Ayarı (Mevcut uçuş menziline göre ayarlanır)
axis([x-1 x+x+10 y-5 y+5 -20 105]); % X ekseninde hareket ettiğiniz için alanı genişlettik
pbaspect([1 1 0.5]); 
view(3); 
hold off;

end