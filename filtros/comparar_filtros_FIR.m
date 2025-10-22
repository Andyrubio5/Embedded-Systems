% =========================================================================
% Autor: Andrea Zarahi Rubio Quezada - A01645257
% Actividad: P2 — Comparación de filtros FIR pasa-bajas con distintos órdenes
% =========================================================================

clear; close all; clc;

%% ===========================================================
% 1. GENERACIÓN DE LA SEÑAL DE PRUEBA
% ============================================================
fs = 1000;              % Frecuencia de muestreo [Hz]
Tsig = 1.0;             % Duración total de la señal [s]
t = 0:1/fs:Tsig-1/fs;   % Vector de tiempo
Ns = length(t);         % Número total de muestras

% Frecuencias de interés
f_signal = 60;          % Frecuencia principal (Hz)
f_noise  = 280;         % Frecuencia interferente (Hz)
A_sig = 1.0;            % Amplitud de la señal principal
A_noise = 0.8;          % Amplitud de la interferencia

% Señal ideal (limpia)
x_ideal = A_sig * sin(2*pi*f_signal*t);
% Señal con interferencia + ruido blanco
x_mix   = x_ideal + A_noise*sin(2*pi*f_noise*t) + 0.05*randn(1,Ns);

%% ===========================================================
% 2. LISTA DE ÓRDENES A EVALUAR
% ============================================================
ordenes = [16 32 64 128 256];   % Diferentes órdenes FIR
fcorte  = 120;                  % Frecuencia de corte [Hz]
w_norm  = fcorte / (fs/2);      % Frecuencia normalizada (Nyquist)

%% ===========================================================
% 3. EVALUACIÓN DE LOS FILTROS FIR
% ============================================================
NFFT = 4096;                                     % Tamaño de la FFT
[frecs, Xmag] = mag_fft(x_mix, fs, NFFT);        % FFT de la señal mixta
Ymag_all = zeros(numel(frecs), numel(ordenes));  % Para guardar espectros filtrados
y_filtradas = zeros(numel(ordenes), Ns);         % Señales filtradas en el tiempo

for i = 1:numel(ordenes)
    N = ordenes(i);                                  % Orden actual
    h = fir1(N, w_norm, 'low', hamming(N+1));        % Coeficientes FIR
    y = filtfilt(h,1,x_mix);                         % Filtrado sin retardo
    y_filtradas(i,:) = y;                            % Guardar resultado
    [~, Ymag_all(:,i)] = mag_fft(y, fs, NFFT);       % Magnitud FFT
end

%% ===========================================================
% 4. RESPUESTAS EN FRECUENCIA DE LOS FILTROS
% ============================================================
figure('Name','P2: Respuesta en frecuencia FIR');
hold on;
colores = [1.0 0.7 0.8; 0.8 0.6 0.9; 0.6 0.8 1.0; 0.7 0.9 0.8; 0.9 0.8 0.7]; % paleta pastel

for i = 1:numel(ordenes)
    N = ordenes(i);
    h = fir1(N, w_norm, 'low', hamming(N+1));
    [H, F] = freqz(h,1,2001,fs);
    plot(F, abs(H), 'Color', colores(i,:), 'LineWidth',1.3);
end
xlabel('Frecuencia [Hz]'); ylabel('|H(f)|');
xlim([0 fs/2]); grid on;
legend(arrayfun(@(n) sprintf('N=%d',n), ordenes, 'UniformOutput',false), ...
       'Location','SouthWest');
title('Respuesta en frecuencia FIR — efecto del orden (colores pastel)');
box on;

%% ===========================================================
% 5. ESPECTROS DE MAGNITUD: SEÑAL ORIGINAL Y FILTRADAS
% ============================================================
figure('Name','P2: Espectros de magnitud');
plot(frecs, Xmag, 'Color',[0.85 0.5 0.6],'LineWidth',1.5); hold on; % tono rosado
plot(frecs, Ymag_all, 'LineWidth',1.1);
xlabel('Frecuencia [Hz]'); ylabel('Magnitud');
xlim([0 fs/2]); grid on;
legend(['|X|', arrayfun(@(n) sprintf('N=%d',n), ordenes, 'UniformOutput',false)], ...
       'Location','best');
title('Espectros de salida según el orden del filtro FIR');

%% ===========================================================
% 6. ZOOM TEMPORAL PARA OBSERVAR CAMBIOS EN EL TIEMPO
% ============================================================
figure('Name','P2: Zoom temporal (comparación)');
rango = t >= 0.3 & t <= 0.36;      % Segmento a observar
plot(t(rango), x_mix(rango), 'Color',[0.7 0.6 0.9], 'LineWidth',1.3); hold on; % lila
for i = 1:numel(ordenes)
    plot(t(rango), y_filtradas(i,rango), 'Color', colores(i,:), 'LineWidth',1.2);
end
xlabel('Tiempo [s]'); ylabel('Amplitud');
legend(['x(t)', arrayfun(@(n) sprintf('N=%d',n), ordenes, 'UniformOutput',false)], ...
       'Location','best');
grid on; box on;
title('Comparación temporal (0.3–0.36 s)');

%% ===========================================================
% 7. MÉTRICAS DE DESEMPEÑO SEGÚN EL ORDEN DEL FILTRO
% ============================================================
% Se mide el residuo de interferencia a 280 Hz y el error RMS
[~, bin_int] = min(abs(frecs - f_noise));          % Índice de 280 Hz
residuo_280 = Ymag_all(bin_int,:).';               % Magnitud residual a 280 Hz
rms_error = sqrt(mean((y_filtradas - x_ideal).^2,2)); % Error RMS respecto a señal limpia

figure('Name','P2: Métricas vs Orden del FIR');
subplot(2,1,1)
plot(ordenes, residuo_280, '-o', 'Color',[0.9 0.6 0.7], 'LineWidth',1.4);
grid on;
xlabel('Orden N'); ylabel(sprintf('|Y| a %.0f Hz',f_noise));
title('Residuo de interferencia (280 Hz)');

subplot(2,1,2)
plot(ordenes, rms_error, '-o', 'Color',[0.6 0.8 0.9], 'LineWidth',1.4);
grid on;
xlabel('Orden N'); ylabel('RMS error vs señal ideal');
title('Fidelidad respecto a la señal de 60 Hz');

%% ===========================================================
% 8. COMENTARIOS Y CONCLUSIONES
% ===========================================================
%{
• Aumentar el orden del FIR mejora la atenuación de la interferencia (280 Hz)
  y hace la transición de la banda más estrecha.
• Sin embargo, después de N≈128 las mejoras son mínimas frente al costo computacional.
• La ventana Hamming brinda buen equilibrio entre atenuación y suavidad.
• Conclusión: el orden ideal depende de los requisitos de diseño, no solo de "más grande = mejor".
%}

%% ===========================================================
% 9. FUNCIÓN AUXILIAR PARA FFT DE MAGNITUD
% ===========================================================
function [f, Xmag] = mag_fft(x, fs, NFFT)
    % Calcula la magnitud de la FFT de una señal
    x = x(:).';
    L = length(x);
    X = fft(x, NFFT)/L;
    f = fs*(0:NFFT-1)/NFFT;
    idx = 1:(floor(NFFT/2)+1);
    f = f(idx);
    X = X(idx);
    Xmag = 2*abs(X);
    Xmag(1) = abs(X(1)); % Corrige la componente DC
end
