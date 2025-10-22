
clearvars; close all; clc

%% SEÑAL DE PRUEBA
fs = 1000; T = 1.0; t = 0:1/fs:T-1/fs; L = numel(t);
f_sig = 60;  A_sig = 1.0;         
f_int = 280; A_int = 0.8;         
x_clean = A_sig*sin(2*pi*f_sig*t);
x       = x_clean + A_int*sin(2*pi*f_int*t) + 0.05*randn(1,L);

%% ÓRDENES A COMPARAR
orders = [16 32 64 128 256];     
fc     = 120;                     
Wn     = fc/(fs/2);

%% EVALUACIÓN
NFFT = 4096; [ff, Xmag] = local_fft_mag(x, fs, NFFT);
Ymag_all = zeros(numel(ff), numel(orders));
y_all    = zeros(numel(orders), L);

for k = 1:numel(orders)
    N = orders(k);
    b = fir1(N, Wn, 'low', hamming(N+1));
    y = filtfilt(b,1,x);                 
    y_all(k,:) = y;
    [~, Ymag_all(:,k)] = local_fft_mag(y, fs, NFFT);
end

%% RESPUESTAS EN FRECUENCIA
figure('Name','Q2: |H(f)| vs orden'); hold on
for k = 1:numel(orders)
    N = orders(k);
    b = fir1(N, Wn, 'low', hamming(N+1));
    [H, F] = freqz(b,1,2001,fs);
    plot(F, abs(H), 'LineWidth',1.0);
end
grid on; xlim([0 fs/2]); xlabel('Frecuencia [Hz]'); ylabel('|H(f)|');
legend(arrayfun(@(n) sprintf('N=%d',n), orders, 'UniformOutput',false),'Location','SouthWest');
title('FIR LP: mayor orden => transición más estrecha');

%% ESPECTROS ANTES/DESPUÉS
figure('Name','Q2: Espectros');
plot(ff, Xmag, 'k','LineWidth',1.2); hold on
plot(ff, Ymag_all, 'LineWidth',1.0); grid on
legend(['|X|', arrayfun(@(n) sprintf('N=%d',n), orders, 'UniformOutput',false)],'Location','best');
xlabel('Frecuencia [Hz]'); ylabel('Magnitud'); xlim([0 fs/2]);

%% ZOOM EN TIEMPO
figure('Name','Q2: Zoom temporal');
idx = t>=0.3 & t<=0.36;
plot(t(idx), x(idx), 'k','LineWidth',1.0); hold on
for k = 1:numel(orders)
    plot(t(idx), y_all(k,idx), 'LineWidth',1.0);
end
legend(['x', arrayfun(@(n) sprintf('N=%d',n), orders, 'UniformOutput',false)],'Location','best');
xlabel('Tiempo [s]'); ylabel('Amplitud'); grid on

%% MÉTRICAS VS ORDEN
[~, bin_int] = min(abs(ff - f_int));                
residual_at_interf = Ymag_all(bin_int,:).';
rms_err = sqrt(mean((y_all - x_clean).^2, 2));      

figure('Name','Q2: Métricas vs orden');
subplot(2,1,1); plot(orders, residual_at_interf,'-o','LineWidth',1.2); grid on
xlabel('Orden N'); ylabel(sprintf('|Y| a %.0f Hz',f_int)); title('Residuo de interferencia');
subplot(2,1,2); plot(orders, rms_err,'-o','LineWidth',1.2); grid on
xlabel('Orden N'); ylabel('RMS error vs x_{clean}'); title('Fidelidad a 60 Hz');

%% DISCUSIÓN / OBSERVACIONES / CONCLUSIONES
%{
• A mayor orden, mejor atenuación en 280 Hz y transición más estrecha.
• Rendimientos decrecientes: después de ~N=128, la mejora marginal baja
  vs costo computacional. El error RMS contra la seno ideal se estabiliza.
• La ventana importa: Hamming da lóbulos laterales bajos a costo de
  anchura de lóbulo principal; otra ventana cambiará el compromiso.
• Lección: elegir N por especificación (ancho de transición/atenuación),
  no "mientras más grande mejor". En tiempo real, recuerda retardo N/2.
%}

function [f, Xmag] = local_fft_mag(x, fs, NFFT)
    x = x(:).';  L = numel(x);
    X = fft(x, NFFT)/L;
    f = fs*(0:NFFT-1)/NFFT;
    half = floor(NFFT/2)+1;
    X = X(1:half);  f = f(1:half);
    Xmag = 2*abs(X); Xmag(1) = abs(X(1));
end
