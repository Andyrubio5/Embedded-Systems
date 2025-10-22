%% INITIALIZE
clearvars
close all
clc

%% ========================================================================
% Crear frecuencia de muestreo
fs = 1000; 
% Q: Que es la frecuencia de muestreo?
% Q: Que es la frecuencia de Nyquist?

% Crear vector tiempo
t  = 0:1/fs:0.4;

% Number of samples or length of signal
L  = length(t);                     

%% ========================================================================
% Crear señal
x  = 20*cos(2*pi*50*t+2*pi/5) + 20*cos(2*pi*214*t-pi/4);

% Agregar ruido gausiano a la señal
x  = x + 20*randn(L,1)';
% Q: Cambiar la amplitud del ruido a 10, 20 y 50. Para cada caso, graficar y 
% responder, que pasa cuando incrementa la amplitud del ruido?

%% ========================================================================
% Transformada de Fourier de la señal
NFFT     = 4096; %or 2^nextpow2(L); % Next power of 2 from length of y
Xfft     = fft(x,NFFT)/L;
f        = fs*(0:NFFT-1)/NFFT;

% Nos quedamos con la mitad ¡
Xfft     = Xfft(1:NFFT/2+1);
f        = f(1:NFFT/2+1);

Xfft_mag = 2*abs(Xfft); % *2
Xfft_fas = angle(Xfft);

% Grafica de resultados
subplot(3,1,1)
plot(t,x)
xlabel('Time (s)'), ylabel('Amplitude (V)')

subplot(3,1,2)
plot(f,Xfft_mag)
xlabel('Frequency (Hz)'), ylabel('Amplitude (V)')

subplot(3,1,3)
plot(f,Xfft_fas)
xlabel('Frequency (Hz)'), ylabel('Radians (rad)')










