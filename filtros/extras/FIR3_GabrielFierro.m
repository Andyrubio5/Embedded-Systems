%Gabriel Azael Fierro Acosta 
clearvars; close all; clc

%% SEÑAL
fs = 2000; T = 0.5; t = 0:1/fs:T-1/fs; L = numel(t);
x = 0.5*sin(2*pi*10*t) + 1.0*sin(2*pi*300*t + pi/6) + 0.05*randn(1,L); 

%% DISEÑO HPF
fc_hp = 100;                 
N     = 128;                
Wn    = fc_hp/(fs/2);
b     = fir1(N, Wn, 'high', hamming(N+1));
[H,F] = freqz(b,1,2001,fs);

%% FILTRADO
y_causal = filter(b,1,x);
y_zero   = filtfilt(b,1,x);   

%% ESPECTROS
NFFT = 4096; [ff, Xmag] = local_fft_mag(x, fs, NFFT);
[~,  YmagC ] = local_fft_mag(y_causal, fs, NFFT);
[~,  YmagZ ] = local_fft_mag(y_zero,   fs, NFFT);

%% GRÁFICAS
figure('Name','Q3: Tiempo');
subplot(2,1,1); plot(t,x,'LineWidth',1.0); grid on
xlabel('Tiempo [s]'); ylabel('x(t)'); title('Entrada: 10 Hz + 300 Hz + ruido')
subplot(2,1,2); plot(t,y_causal,'b','LineWidth',1.0); hold on
plot(t,y_zero,'g--','LineWidth',1.0); grid on
xlabel('Tiempo [s]'); ylabel('y(t)'); legend('filter','filtfilt');

figure('Name','Q3: Espectros');
plot(ff,Xmag,'k','LineWidth',1.0); hold on
plot(ff,YmagC,'b--','LineWidth',1.0);
plot(ff,YmagZ,'g-.','LineWidth',1.0); grid on
xlabel('Frecuencia [Hz]'); ylabel('Magnitud');
legend('|X|','|Y_{filter}|','|Y_{filtfilt}|'); xlim([0 fs/2]);

figure('Name','Q3: |H(f)| y fase');
subplot(2,1,1); plot(F,abs(H),'LineWidth',1.2); grid on; xlim([0 fs/2]);
xlabel('Frecuencia [Hz]'); ylabel('|H(f)|'); title('Respuesta pasa-altas');
subplot(2,1,2); plot(F,unwrap(angle(H)),'LineWidth',1.2); grid on; xlim([0 fs/2]);
xlabel('Frecuencia [Hz]'); ylabel('\theta(f) [rad]');

%% DISCUSIÓN / OBSERVACIONES / CONCLUSIONES 
%{
• El HPF elimina claramente la deriva de 10 Hz y conserva el tono de 300 Hz.
• El filtro causal muestra retardo N/2; filtfilt alinea en tiempo. Ambos
  suprimen fuerte <100 Hz en el espectro.
• Subir fc remueve más LF pero arriesga recortar señal útil cercana.
  N controla la nitidez de transición vs costo/retardo.
%}
function [f, Xmag] = local_fft_mag(x, fs, NFFT)
    x = x(:).';  L = numel(x);
    X = fft(x, NFFT)/L;
    f = fs*(0:NFFT-1)/NFFT;
    half = floor(NFFT/2)+1;
    X = X(1:half);  f = f(1:half);
    Xmag = 2*abs(X); Xmag(1) = abs(X(1));
end
