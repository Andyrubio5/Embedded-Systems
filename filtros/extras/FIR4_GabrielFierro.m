%Gabriel Azael Fierro Acosta 

clearvars; close all; clc

%% SEÑAL
fs = 2000; T = 0.5; t = 0:1/fs:T-1/fs; L = numel(t);
x = 0.8*sin(2*pi*50*t) + 1.0*sin(2*pi*300*t) + 0.7*sin(2*pi*700*t) + 0.1*randn(1,L);

%% DISEÑO BPF — aislar ~300 Hz (250–400 Hz)
f1 = 250; f2 = 400;
Wn = [f1 f2]/(fs/2);
N  = 128;
b  = fir1(N, Wn, hamming(N+1));    
[H,F] = freqz(b,1,2001,fs);

%% FILTRADO (cero fase para claridad)
y = filtfilt(b,1,x);

%% ESPECTROS
NFFT = 4096; [ff, Xmag] = local_fft_mag(x, fs, NFFT);
[~,  Ymag]  = local_fft_mag(y, fs, NFFT);

%% GRÁFICAS
figure('Name','Q4: Tiempo');
subplot(2,1,1); plot(t,x,'k','LineWidth',1.0); grid on
xlabel('Tiempo [s]'); ylabel('x(t)');
subplot(2,1,2); plot(t,y,'r','LineWidth',1.0); grid on
xlabel('Tiempo [s]'); ylabel('y(t)');

figure('Name','Q4: Espectros');
plot(ff,Xmag,'k','LineWidth',1.2); hold on
plot(ff,Ymag,'r--','LineWidth',1.2); grid on
xlabel('Frecuencia [Hz]'); ylabel('Magnitud'); legend('|X|','|Y|'); xlim([0 fs/2]);

figure('Name','Q4: |H(f)| y fase');
subplot(2,1,1); plot(F,abs(H),'LineWidth',1.2); grid on; xlim([0 fs/2]);
xlabel('Frecuencia [Hz]'); ylabel('|H(f)|'); title('Respuesta pasa-banda');
subplot(2,1,2); plot(F,unwrap(angle(H)),'LineWidth',1.2); grid on; xlim([0 fs/2]);
xlabel('Frecuencia [Hz]'); ylabel('\theta(f) [rad]');

%% DISCUSIÓN / OBSERVACIONES / CONCLUSIONES 
%{
• El BPF aísla el componente de 300 Hz y suprime los tonos de 50 y 700 Hz.
• Los bordes de banda y el orden N gobiernan fuga/rizado y necesidad de N alto
  en bandas estrechas.
• Lección: FIR con fir1+ventana (sinc-ventaneado) ofrece fase lineal y control
  sencillo; para bandas angostas, suele requerirse mayor N.
%}
function [f, Xmag] = local_fft_mag(x, fs, NFFT)
    x = x(:).';  L = numel(x);
    X = fft(x, NFFT)/L;
    f = fs*(0:NFFT-1)/NFFT;
    half = floor(NFFT/2)+1;
    X = X(1:half);  f = f(1:half);
    Xmag = 2*abs(X); Xmag(1) = abs(X(1));
end
