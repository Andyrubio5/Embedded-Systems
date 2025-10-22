%% INITIALIZE
clearvars
close all
clc



%% CREATE VECTOR DE TIEMPO

% Crear frecuencia de muestreo
fs = 1000;
% Q: Que es la frecuencia de muestreo?
% Q: Que es la frecuencia de Nyquist?

% Periodo de muestreo
dt = 1/fs;
% Q: Que es el periodo de muestreo?

% Create time vector
if (1)
    % Crear vector tiempo
    t  = -0.0:dt:0.2; %
    % Number of samples or length of signal
    L  = length(t);
else
    % Number of samples or length of signal
    L  = 178;
    % Crear vector tiempo
    t  = (0:L-1)*dt;
end



%% CREAR SEÑAL 

x  = 6*sin(2*pi*50*t+pi/5) + 6*sin(2*pi*158*t);
% Q: Sumar dos mas funciones sinusoidales, de amplitud 12 y frecuencia 80Hz, y de amplitud 4 y frecuencia 150Hz, 

% Agregar ruido gausiano a la señal
x  = x + 5*randn(L,1)';
% Q: Cambiar la amplitud del ruido a 0, 1, 5 y 10. Para cada caso, graficar y 
% responder, que pasa en x cuando incrementa la amplitud del ruido?

% Grafica de la señal
figure
plot(t,x)
% xlabel('Time (s)')

% Q: Usar las intrucciones "xlable", "ylabel" y "title" para agregar las 
% unidades de los ejes. Suponer que las unidades de la señal x es voltaje


