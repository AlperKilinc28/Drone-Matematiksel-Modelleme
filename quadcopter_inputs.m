function omega_sq = quadcopter_inputs(t, constants)

% Constants
m = constants(1);
g = constants(2);
k = constants(4);

% Hover Hızı (Radyan/saniye'nin karesi)
omega_hover_sq = (m * g) / (4 * k);

% --- KONTROL GİRİŞİ BELİRLEME ---
% Başlangıçta tüm motorlar hover hızında dönsün (Stabil kalmak için)
% Torkları kontrol etmek için hızları zamanla değiştirebiliriz.

omega1_sq =1.1*omega_hover_sq;
omega2_sq =1.1*omega_hover_sq;
omega3_sq =1.1*omega_hover_sq;
omega4_sq =1.1*omega_hover_sq;

omega_sq = [omega1_sq; omega2_sq; omega3_sq; omega4_sq];

end