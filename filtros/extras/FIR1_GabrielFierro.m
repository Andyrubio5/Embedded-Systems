%Gabriel Azael Fierro Acosta A01639999
clearvars; close all; clc

%% PARÁMETROS
fs = 1000;          
T  = 2.000;         
t  = 0:1/fs:T-1/fs; 
L  = numel(t);     
f0 = 50;            
A0 = 1;             
x  = A0*sin(2*pi*f0*t);   

%% DISEÑO FIR (PASA-BAJAS)
N  = 2*floor((L/3)/2);          
fc = 50;                         
Wn = fc/(fs/2);                   
b  = fir1(N, Wn, 'low', hamming(N+1)); 

%% FILTER vs FILTFILT
y_filt     = filter(b, 1, x);     
y_filtfilt = filtfilt(b, 1, x);   

%% ALINEAR SALIDA CAUSAL (COMPENSAR RETRASO)
delay_samp = N/2;
y_filt_aligned = [y_filt(1+delay_samp:end), zeros(1,delay_samp)];

%% MÉTRICAS
err_rms_filt     = sqrt(mean((y_filt_aligned - x).^2));
err_rms_filtfilt = sqrt(mean((y_filtfilt     - x).^2));

%% GRÁFICAS (TIEMPO)
figure('Name','Q1: Dominio del tiempo');
subplot(3,1,1); plot(t,x,'LineWidth',1.2); grid on
xlabel('Tiempo [s]'); ylabel('x(t)');
title(sprintf('Seno f_0=%d Hz (sin ruido)',f0));
subplot(3,1,2); plot(t,y_filt,'LineWidth',1.2); grid on
xlabel('Tiempo [s]'); ylabel('y_{filter}(t)');
title(sprintf('filter( ) — retardo N/2 = %d muestras (%.3f s)',delay_samp,delay_samp/fs));
subplot(3,1,3); plot(t,x,'k','LineWidth',1.2); hold on
plot(t,y_filt_aligned,'r--','LineWidth',1.2); grid on
legend('x(t)','filter alineado'); xlabel('Tiempo [s]'); ylabel('Amplitud');

figure('Name','Q1: Cero fase vs entrada');
plot(t,x,'k','LineWidth',1.2); hold on
plot(t,y_filtfilt,'g--','LineWidth',1.2); grid on
legend('x(t)','filtfilt (cero fase)'); xlabel('Tiempo [s]'); ylabel('Amplitud');

%% GRÁFICAS (FRECUENCIA)
NFFT = 4096;
[Xf, Xmag]  = local_fft_mag(x, fs, NFFT);
[~,  Ymag1] = local_fft_mag(y_filt, fs, NFFT);
[ff, Ymag2] = local_fft_mag(y_filtfilt, fs, NFFT);

figure('Name','Q1: Espectros de magnitud');
plot(Xf, Xmag,'k','LineWidth',1.1); hold on
plot(ff, Ymag1,'b--','LineWidth',1.1);
plot(ff, Ymag2,'g-.','LineWidth',1.1); grid on
legend('|X|','|Y_{filter}|','|Y_{filtfilt}|','Location','best');
xlabel('Frecuencia [Hz]'); ylabel('Magnitud'); xlim([0 fs/2]);
fprintf('\nQ1 metrics\n');
fprintf(' — Orden FIR N = %d (retardo = N/2 = %d muestras = %.6f s)\n',N,delay_samp,delay_samp/fs);
fprintf(' — RMS error vs x (filter alineado): %.6g\n',err_rms_filt);
fprintf(' — RMS error vs x (filtfilt):        %.6g\n',err_rms_filtfilt);

%% DISCUSIÓN / OBSERVACIONES / CONCLUSIONES 
%{
• filter() con FIR de fase lineal desplaza la señal un retardo constante N/2;
  la forma se conserva pero los picos llegan tarde. Si compenso el retardo,
  se sobrepone casi perfecto a la entrada.
• filtfilt() elimina el retardo (filtrado ida/vuelta) y mantiene la alineación
  temporal, ideal para análisis fuera de línea; pero no es causal y puede
  introducir transitorios de borde.
• Lección: En tiempo real uso filter() y acepto retardo; para análisis off-line,
  filtfilt() me da cero fase. No conviene crecer N sin criterio (más retardo y costo).
%}


% Helper
function [f, Xmag] = local_fft_mag(x, fs, NFFT)
    x = x(:).';  L = numel(x);
    X = fft(x, NFFT)/L;
    f = fs*(0:NFFT-1)/NFFT;
    half = floor(NFFT/2)+1;
    X = X(1:half);  f = f(1:half);
    Xmag = 2*abs(X); Xmag(1) = abs(X(1));  
end
