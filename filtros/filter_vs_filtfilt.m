% =========================================================================
% Autor:  Andrea Zarahi Rubio Quezada - A01645257
% Actividad: P1 — Comparación entre filter() y filtfilt() usando una señal senoidal
% =========================================================================

clear; close all; clc;

%% ============================================================
% PARÁMETROS DE LA SEÑAL
% =============================================================
fs = 1000;                     % Frecuencia de muestreo [Hz]
Ttotal = 1.0;                  % Duración de la señal en segundos
t = 0:1/fs:Ttotal-1/fs;        % Vector de tiempo
Nsamples = length(t);          % Número total de muestras (L)

freq_sine = 50;                % Frecuencia de la señal senoidal (Hz)
amplitude = 1;                 % Amplitud de la señal
x_sig = amplitude * sin(2*pi*freq_sine*t);  % Señal pura (sin ruido)

%% ============================================================
% DISEÑO DEL FILTRO FIR PASA-BAJAS
% =============================================================
% Nota: filtfilt requiere que la longitud de la señal sea > 3*N
% Por eso se usa un orden N ≈ L/3 para evitar errores

orden_fir = 2*floor(Nsamples/6);    % Orden del filtro (ajustado a L/3)
fcorte = 100;                       % Frecuencia de corte [Hz]
wnorm = fcorte / (fs/2);            % Frecuencia normalizada (0 a 1)
coef = fir1(orden_fir, wnorm, 'low', hamming(orden_fir+1)); % Diseño del filtro FIR

%% ============================================================
% APLICAR FILTROS
% =============================================================
% filter() aplica el filtro de forma causal (introduce retardo)
y_causal = filter(coef, 1, x_sig);

% filtfilt() aplica el filtro hacia adelante y hacia atrás (sin retardo)
y_nocausal = filtfilt(coef, 1, x_sig);

%% ============================================================
% COMPENSACIÓN DE RETARDO DEL FILTRO CAUSAL
% =============================================================
retardo = orden_fir/2;                     % Retardo teórico (muestras)
y_causal_corr = [y_causal(1+retardo:end), zeros(1, retardo)];  % Alineación

%% ============================================================
% CÁLCULO DE ERRORES RMS ENTRE LA ENTRADA Y LAS SALIDAS
% =============================================================
rms_filter   = sqrt(mean((y_causal_corr - x_sig).^2));  % Error RMS de filter()
rms_filtfilt = sqrt(mean((y_nocausal - x_sig).^2));     % Error RMS de filtfilt()

%% ============================================================
% GRÁFICAS EN EL DOMINIO DEL TIEMPO
% =============================================================
figure('Name','P1: Dominio temporal');

% --- Subplot 1: Señal original ---
subplot(3,1,1)
plot(t, x_sig, 'Color',[1.0 0.6 0.7], 'LineWidth',1.5);  % Rosa pastel
grid on
xlabel('Tiempo [s]');
ylabel('x(t)');
title(sprintf('Señal senoidal de %.0f Hz', freq_sine));

% --- Subplot 2: Señal filtrada (causal) ---
subplot(3,1,2)
plot(t, y_causal, 'Color',[0.6 0.5 0.8], 'LineWidth',1.5); % Lila suave
grid on
xlabel('Tiempo [s]');
ylabel('y_{filter}(t)');
title(sprintf('filter() — retardo = %d muestras (%.3f s)', retardo, retardo/fs));

% --- Subplot 3: Señal original vs filter alineado ---
subplot(3,1,3)
plot(t, x_sig, 'Color',[0.8 0.7 1.0], 'LineWidth',1.3); hold on;  % Lavanda claro
plot(t, y_causal_corr, '--', 'Color',[0.5 0.3 0.8], 'LineWidth',1.3); % Morado
grid on
legend('x(t)', 'filter alineado', 'Location','best');
xlabel('Tiempo [s]');
ylabel('Amplitud');
title('Comparación: señal original vs filter alineado');

%% ============================================================
% COMPARACIÓN filter() vs filtfilt()
% =============================================================
figure('Name','P1: Fase cero vs entrada');
plot(t, x_sig, 'Color',[1.0 0.7 0.8], 'LineWidth',1.5); hold on;  % Rosa suave
plot(t, y_nocausal, '--', 'Color',[0.5 0.8 1.0], 'LineWidth',1.5); % Azul cielo
grid on
xlabel('Tiempo [s]');
ylabel('Amplitud');
legend('x(t)', 'filtfilt (cero fase)', 'Location', 'best');
title('Comparación entre filter() y filtfilt()');

%% ============================================================
% ANÁLISIS EN FRECUENCIA (FFT)
% =============================================================
NFFT = 4096; % Puntos para la FFT (alta resolución)
[freq_vec, Xmag]  = calc_fft_mag(x_sig, fs, NFFT);
[~, Ymag_filter]  = calc_fft_mag(y_causal, fs, NFFT);
[~, Ymag_filtfilt]= calc_fft_mag(y_nocausal, fs, NFFT);

% --- Gráfica espectral ---
figure('Name','P1: Espectro de magnitud');
plot(freq_vec, Xmag, 'Color',[1.0 0.5 0.6],'LineWidth',1.5); hold on;  % Rosa coral
plot(freq_vec, Ymag_filter, '--','Color',[0.5 0.7 0.9],'LineWidth',1.3); % Celeste
plot(freq_vec, Ymag_filtfilt, '-.','Color',[0.6 0.9 0.7],'LineWidth',1.3); % Verde menta
xlabel('Frecuencia [Hz]');
ylabel('Magnitud');
legend('|X|','|Y_{filter}|','|Y_{filtfilt}|','Location','best');
xlim([0 fs/2]);
grid on;
title('Espectros de entrada y señales filtradas');

%% ============================================================
% RESULTADOS EN CONSOLA
% =============================================================
fprintf('\n=== RESULTADOS FILTRO FIR ===\n');
fprintf('Orden FIR: N = %d (cumple L > 3*N)\n', orden_fir);
fprintf('Retardo teórico: %d muestras = %.6f s\n', retardo, retardo/fs);
fprintf('Error RMS (filter alineado): %.6g\n', rms_filter);
fprintf('Error RMS (filtfilt): %.6g\n', rms_filtfilt);

fprintf('\n=== INTERPRETACIÓN ===\n');
fprintf('• filter(): introduce retardo de N/2 muestras, pero es causal.\n');
fprintf('• filtfilt(): elimina el retardo y mantiene fase cero (no causal).\n');
fprintf('• En tiempo real se usa filter(); en análisis offline, filtfilt().\n');

%% ============================================================
% FUNCIÓN AUXILIAR: FFT DE MAGNITUD
% =============================================================
function [f, Xmag] = calc_fft_mag(x, fs, NFFT)
    % Convierte la señal en vector fila y calcula la FFT normalizada
    x = x(:).';
    L = numel(x);
    X = fft(x, NFFT)/L;
    f = fs*(0:NFFT-1)/NFFT;   % Vector de frecuencias
    idx = 1:(floor(NFFT/2)+1); % Solo la mitad positiva
    X = X(idx); f = f(idx);
    Xmag = 2*abs(X);           % Magnitud doble para espectro de un solo lado
    Xmag(1) = abs(X(1));       % Corrige la DC
end
