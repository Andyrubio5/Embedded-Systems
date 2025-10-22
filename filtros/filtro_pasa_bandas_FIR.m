% =========================================================================
% Autor: Andrea Zarahi Rubio Quezada - A01645257
% Actividad: P4 — Implementación de un filtro FIR pasa-banda y análisis de su efecto
% =========================================================================

clear; close all; clc;

%% ============================================================
% 1. CONFIGURACIÓN DE LA SEÑAL DE ENTRADA
% =============================================================
fs = 2000;                    % Frecuencia de muestreo [Hz]
Tsig = 0.5;                   % Duración total de la señal [s]
t = 0:1/fs:Tsig-1/fs;         % Vector de tiempo
Ns = length(t);               % Número total de muestras

% La señal está compuesta por:
% - Un tono de baja frecuencia (50 Hz)
% - Un tono central de 300 Hz (el que queremos conservar)
% - Un tono de alta frecuencia (700 Hz)
% - Ruido blanco
x_in = 0.8*sin(2*pi*50*t) + 1.0*sin(2*pi*300*t) + ...
       0.7*sin(2*pi*700*t) + 0.1*randn(1,Ns);

%% ============================================================
% 2. DISEÑO DEL FILTRO FIR PASA-BANDA
% =============================================================
f_low  = 250;                 % Límite inferior de banda [Hz]
f_high = 400;                 % Límite superior de banda [Hz]
Wnorm  = [f_low f_high] / (fs/2);   % Normalización (0–1)
orden  = 128;                        % Orden del filtro FIR

% Diseño del filtro FIR pasa-banda con ventana Hamming
coefBPF = fir1(orden, Wnorm, hamming(orden+1));

% Cálculo de la respuesta en frecuencia
[H, fvec] = freqz(coefBPF, 1, 2001, fs);

%% ============================================================
% 3. APLICACIÓN DEL FILTRO
% =============================================================
% Se usa filtfilt() para eliminar el desfase (fase cero)
y_out = filtfilt(coefBPF, 1, x_in);

%% ============================================================
% 4. ANÁLISIS EN FRECUENCIA
% =============================================================
NFFT = 4096;
[freqs, Xmag] = mag_fft(x_in, fs, NFFT);
[~, Ymag] = mag_fft(y_out, fs, NFFT);

%% ============================================================
% 5. GRÁFICAS EN EL DOMINIO DEL TIEMPO
% =============================================================
figure('Name','P4: Señales en el tiempo');

% Señal original
subplot(2,1,1);
plot(t, x_in, 'Color',[1.0 0.7 0.8], 'LineWidth', 1.4); % Rosa pastel
xlabel('Tiempo [s]'); ylabel('x(t)');
title('Entrada: 50 Hz + 300 Hz + 700 Hz + ruido');
grid on;

% Señal filtrada
subplot(2,1,2);
plot(t, y_out, 'Color',[0.6 0.7 1.0], 'LineWidth', 1.4); % Azul lavanda
xlabel('Tiempo [s]'); ylabel('y(t)');
title('Salida filtrada (pasa-banda 250–400 Hz, fase cero)');
grid on;

%% ============================================================
% 6. ESPECTROS DE MAGNITUD
% =============================================================
figure('Name','P4: Espectros de magnitud');

plot(freqs, Xmag, 'Color',[0.9 0.5 0.6], 'LineWidth', 1.4); hold on; % Rosa coral
plot(freqs, Ymag, '--', 'Color',[0.5 0.9 0.8], 'LineWidth', 1.3);   % Verde menta
xlabel('Frecuencia [Hz]'); ylabel('Magnitud');
legend('|X| (Entrada)','|Y| (Salida)','Location','best');
xlim([0 fs/2]); grid on; box on;
title('Comparación espectral: antes y después del filtrado');

%% ============================================================
% 7. RESPUESTA EN FRECUENCIA DEL FILTRO
% =============================================================
figure('Name','P4: |H(f)| y fase');

% Magnitud
subplot(2,1,1)
plot(fvec, abs(H), 'Color',[0.8 0.6 1.0], 'LineWidth', 1.4); % Violeta suave
xlabel('Frecuencia [Hz]'); ylabel('|H(f)|');
title('Respuesta en magnitud del filtro pasa-banda');
xlim([0 fs/2]); grid on;

% Fase
subplot(2,1,2)
plot(fvec, unwrap(angle(H)), 'Color',[0.5 0.6 1.0], 'LineWidth', 1.4); % Azul cielo
xlabel('Frecuencia [Hz]'); ylabel('\theta(f) [rad]');
title('Respuesta en fase');
xlim([0 fs/2]); grid on;

%% ============================================================
% 8. COMENTARIOS Y CONCLUSIONES
% ============================================================
%{
• El filtro FIR pasa-banda (250–400 Hz) conserva el tono de 300 Hz y
  elimina las componentes de 50 y 700 Hz con buena selectividad.
• Los límites de banda (f_low, f_high) y el orden N determinan la nitidez
  de la transición y el nivel de fuga espectral.
• La ventana Hamming reduce los lóbulos laterales (menor ripple) a costa de
  una transición un poco más ancha.
• En bandas estrechas se necesita un orden mayor para lograr selectividad.
• Conclusión: fir1() con ventana Hamming permite diseñar filtros FIR de fase
  lineal estables y fácilmente controlables.
%}

%% ============================================================
% 9. FUNCIÓN AUXILIAR PARA FFT DE MAGNITUD
% ============================================================
function [f, Xmag] = mag_fft(x, fs, NFFT)
    % Calcula la magnitud de la FFT (solo mitad positiva del espectro)
    x = x(:).';
    L = numel(x);
    X = fft(x, NFFT)/L;
    f = fs*(0:NFFT-1)/NFFT;
    idx = 1:(floor(NFFT/2)+1);
    f = f(idx); X = X(idx);
    Xmag = 2*abs(X);
    Xmag(1) = abs(X(1)); % Corrige componente DC
end
