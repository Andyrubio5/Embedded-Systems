% =========================================================================
% Autor: Andrea Zarahi Rubio Quezada - A01645257
% Actividad: P3 — Implementación y análisis de un filtro FIR pasa-altas
% =========================================================================

clear; close all; clc;
    
%% ============================================================
% 1. PARÁMETROS DE LA SEÑAL DE PRUEBA
% =============================================================
fs = 2000;                   % Frecuencia de muestreo [Hz]
Ttotal = 0.5;                % Duración total [s]
t = 0:1/fs:Ttotal-1/fs;      % Vector de tiempo
Ns = length(t);              % Número total de muestras

% La señal contiene:
% - Una componente baja de 10 Hz
% - Una componente alta de 300 Hz
% - Ruido blanco para simular interferencia
x_in = 0.5*sin(2*pi*10*t) + 1.0*sin(2*pi*300*t + pi/6) + 0.05*randn(1,Ns);

%% ============================================================
% 2. DISEÑO DEL FILTRO FIR PASA-ALTAS
% =============================================================
fcorte = 100;                     % Frecuencia de corte [Hz]
orden  = 128;                     % Orden del filtro (controla nitidez y retardo)
wnorm  = fcorte / (fs/2);         % Frecuencia normalizada (0 a 1)
coefHP = fir1(orden, wnorm, 'high', hamming(orden+1));  % Filtro FIR pasa-altas

% Respuesta en frecuencia teórica del filtro
[H, fvec] = freqz(coefHP, 1, 2001, fs);

%% ============================================================
% 3. APLICACIÓN DEL FILTRO
% =============================================================
% filter(): aplica el filtro en una sola dirección (causal) → introduce retardo
y_causal = filter(coefHP, 1, x_in);

% filtfilt(): aplica el filtro hacia adelante y hacia atrás → elimina el retardo
y_cerofase = filtfilt(coefHP, 1, x_in);

%% ============================================================
% 4. ANÁLISIS EN FRECUENCIA (ESPECTROS)
% =============================================================
NFFT = 4096;  % Número de puntos para la FFT (mayor = mejor resolución)
[frecs, Xmag]  = calc_fft_mag(x_in, fs, NFFT);       % Espectro de la señal original
[~, Ymag_dir]  = calc_fft_mag(y_causal, fs, NFFT);   % Espectro filtrado (causal)
[~, Ymag_zero] = calc_fft_mag(y_cerofase, fs, NFFT); % Espectro filtrado (sin fase)

%% ============================================================
% 5. GRÁFICAS EN EL DOMINIO DEL TIEMPO
% =============================================================
figure('Name','P3: Señal en el tiempo');

% --- Señal original ---
subplot(2,1,1)
plot(t, x_in, 'Color',[1.0 0.6 0.7], 'LineWidth', 1.4); % Rosa pastel
xlabel('Tiempo [s]'); ylabel('x(t)');
title('Entrada: 10 Hz + 300 Hz + ruido blanco');
grid on;

% --- Señales filtradas ---
subplot(2,1,2)
plot(t, y_causal, 'Color',[0.6 0.5 0.9], 'LineWidth', 1.3); hold on;  % Lila
plot(t, y_cerofase, '--', 'Color',[0.5 0.8 1.0], 'LineWidth', 1.3);   % Azul cielo
xlabel('Tiempo [s]'); ylabel('y(t)');
legend('filter()','filtfilt()','Location','best');
grid on;
title('Comparación entre filter() y filtfilt()');

%% ============================================================
% 6. GRÁFICAS EN FRECUENCIA
% =============================================================
figure('Name','P3: Espectros de magnitud');

% Señal original en rosa, y salidas en tonos fríos
plot(frecs, Xmag, 'Color',[0.9 0.4 0.6], 'LineWidth', 1.4); hold on; % Rosa medio
plot(frecs, Ymag_dir, '--', 'Color',[0.6 0.7 1.0], 'LineWidth', 1.2); % Azul lavanda
plot(frecs, Ymag_zero, '-.', 'Color',[0.5 0.9 0.8], 'LineWidth', 1.2); % Verde menta
xlabel('Frecuencia [Hz]'); ylabel('Magnitud');
legend('|X|','|Y_{filter}|','|Y_{filtfilt}|','Location','best');
xlim([0 fs/2]);
grid on;
title('Espectro de la señal antes y después del filtrado');

%% ============================================================
% 7. RESPUESTA DEL FILTRO (MAGNITUD Y FASE)
% =============================================================
figure('Name','P3: |H(f)| y fase');

% Magnitud de la respuesta en frecuencia
subplot(2,1,1)
plot(fvec, abs(H), 'Color',[0.8 0.6 1.0], 'LineWidth', 1.4); % Violeta suave
xlabel('Frecuencia [Hz]');
ylabel('|H(f)|');
title('Respuesta en magnitud del filtro pasa-altas');
xlim([0 fs/2]);
grid on;

% Fase del filtro
subplot(2,1,2)
plot(fvec, unwrap(angle(H)), 'Color',[0.4 0.5 0.9], 'LineWidth', 1.4); % Azul lavanda
xlabel('Frecuencia [Hz]');
ylabel('\theta(f) [rad]');
title('Respuesta en fase del filtro');
xlim([0 fs/2]);
grid on;

%% ============================================================
% 8. CONCLUSIONES
% ============================================================
%{
• El filtro FIR pasa-altas elimina eficazmente la componente de baja frecuencia (10 Hz)
  y preserva la de 300 Hz con mínima distorsión.
• El filtrado causal (filter) introduce un retardo de N/2 muestras,
  mientras que filtfilt corrige ese desfase temporal.
• En el espectro se observa una fuerte atenuación por debajo de 100 Hz.
• Aumentar la frecuencia de corte (fc) elimina más bajas frecuencias,
  pero puede afectar las cercanas al límite de corte.
• El parámetro N determina la nitidez de la transición y el retardo asociado.
%}

%% ============================================================
% 9. FUNCIÓN AUXILIAR PARA CALCULAR LA MAGNITUD DE LA FFT
% ============================================================
function [f, Xmag] = calc_fft_mag(x, fs, NFFT)
    % Calcula la magnitud de la FFT (espectro de un solo lado)
    x = x(:).';
    L = numel(x);
    X = fft(x, NFFT)/L;
    f = fs*(0:NFFT-1)/NFFT;          % Vector de frecuencias
    idx = 1:(floor(NFFT/2)+1);       % Se toma solo la mitad positiva
    X = X(idx); f = f(idx);
    Xmag = 2*abs(X);                 % Doble magnitud (espectro de un solo lado)
    Xmag(1) = abs(X(1));             % Corrección de la componente DC
end
