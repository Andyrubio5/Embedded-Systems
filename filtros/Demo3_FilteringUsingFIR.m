%% INITIALIZE
clearvars
close all
clc



%% ACTIVITY
%
%  INDETIFY THE MAIN FREQUENCY COMPONENTS IN A SIGNAL AND DESING A FIR
%  FILTER TO ATTENUATE THE NOISY COMPONENTS
%
%  COMPUTE TIME DELAY INTRODUCED BY THE FIR FILTER
% 
%  USE THE FUNCTIONS "FILTER" AND "FILTFILT"
% 



%% ACTIVITY PART 1: TIME-DOMAIN AND FREQUENCY DOMAIN SIGNAL

% ==================================
% 1) LOAD SIGNAL

% Load signal
%x        = readmatrix('s1.txt');
%fs       = 2000;

% Number of samples 
%L        = length(x);

% Time vector
%n        = 0:1:L-1;
%t        = n/fs;

% ==================================
% 1) CREATE SIGNAL
% Sampling frequency
fs    = 2000;

% Signal duration
Tend  = 0.4;

% Time vector
t     = 0:1/fs:Tend-(1/fs);

% Number of samples or length of signal
L  = length(t);                     

% Create signal
x  = 10*sin(2*pi*100*t + 1*2*pi/5) + 10*sin(2*pi*350*t + 1*2*pi/5);

% Add noise
x  = x + 10*randn(L,1)';

% ==================================
% 2) FREQUENCY ANALYSIS

% Compute fft
NFFT     = 1*fs;  % or 2^nextpow2(L);
X        = fft(x,NFFT)/L;
Xf       = (fs) * ( (0:NFFT-1) )/NFFT;

% Get single-side fft 
X        = 2*X (1:(NFFT)/2);
Xf       = Xf(1:(NFFT)/2);

% Compute magnitude and phase
Xmag     = abs(X);
Xpha     = angle(X);


% ==================================
% 3) PLOT SIGNAL

figure, hold on

subplot(3,1,1)
plot(t,x,'LineWidth',2)
xlabel('Time (s)'), ylabel('x(t)')
box on, grid on, set(gca,'FontSize',12)

subplot(3,1,2)
plot(Xf,Xmag,'LineWidth',2)
xlabel('Frequency (Hz)'), ylabel('|X(f)|')
box on, grid on, set(gca,'FontSize',12)
set(gca,'Xlim',[0 fs/2])

subplot(3,1,3)
plot(Xf,Xpha,'LineWidth',2)
xlabel('Frequency (Hz)'), ylabel('\Theta(f)')
box on, grid on, set(gca,'FontSize',12)
set(gca,'Xlim',[0 fs/2])

% return

%% ACTIVITY PART 2: FIR FILTER DESING

% ==================================
% FILTER BANDPASS

% Cut-off frequency of the band pass (Hz)
fc       = 200;

% Check for Nyquist
if fc>=fs/2
    error('PILAS: Nyquist impone que fc no puede ser mayor fs/2')
end

% Define cut-off frequency
% pi ---> fs/2
% wo --->  fc
wo       = pi*fc/(fs/2); % Frequencies in rad/samples i.e. From 0 to pi

% Define normalized edge frequency
% 1  ---> fs/2
% wn --->  fc
wn       = fc/(fs/2); 
% wn       = wo/pi; % Matlab uses normalized frequencies i.e. From 0 to 1


% ==================================
% 1) FIR FILTER DESIGN
M        = 1000;
b        = fir1(M,wn,'low');
%b        = fir1(M-1,wn,'low',hamming(M));
%b        = fir1(M-1,[100 200]/(fs/2),hamming(M));


% ==================================
% 2) FREQUENCY RESPONSE
[H,Hf]   = freqz(b,1,2001,fs);
Hmag     = abs(H);
% Hmag     = 20*log10(abs(H));


% ==================================
% 3) PLOT FREQUENCY RESPONSE

figure, hold on
subplot(2,1,1), hold on,
plot(Hf,Hmag,'LineWidth',2)
box on, grid on, xlabel('Frequency (Hz)'), ylabel('|H(f)|')
set(gca,'Xlim',[0 fs/2]),
title('Frequency response')
set(gca,'FontSize',12)

subplot(2,1,2), hold on, 
plot(Hf,unwrap(angle(H)),'LineWidth',2)
box on, grid on, xlabel('Frequency (Hz)'), ylabel('\Theta(f)')
set(gca,'Xlim',[0 fs/2]),
set(gca,'FontSize',12)



%% ACTIVITY PART 3: COMPUTE FILTERED SIGNAL


% ==================================
% 1) SIGNAL FILTERING
y     = filter(b,1,x);
%y    = filtfilt(b,1,x);

% ==================================
% 2) FREQUENCY ANALYSIS

% Compute fft
NFFT     = fs;  % or 2^nextpow2(L);
Y        = fft(y,NFFT)/L;
Yf       = (fs) * ( (0:NFFT-1) )/NFFT;

% Get single-side 
Y        = 2*Y (1:(NFFT)/2);
Yf       = Yf(1:(NFFT)/2);

% Compute magnitude and phase
Ymag     = abs(Y);
Ypha     = angle(Y);


% ==================================
% 3) PLOT SIGNAL

figure, hold on

subplot(3,1,1)
plot(t,y,'LineWidth',2)
xlabel('Time (s)'), ylabel('x(t)')
box on, grid on, set(gca,'FontSize',12)

subplot(3,1,2)
plot(Yf,Ymag,'LineWidth',2)
xlabel('Frequency (Hz)'), ylabel('|Y(f)|')
box on, grid on, set(gca,'FontSize',12)
set(gca,'Xlim',[0 fs/2])

subplot(3,1,3)
plot(Yf,Ypha,'LineWidth',2)
xlabel('Frequency (Hz)'), ylabel('\Theta(f)')
box on, grid on, set(gca,'FontSize',12)
set(gca,'Xlim',[0 fs/2])

% return

%% ACTIVITY PART 4: PLOT ALL AND COMPUTE DELAY

% ==================================
% PLOT ALL
figure, hold on

subplot(3,2,1)
plot(t,x,'LineWidth',2)
xlabel('Time (s)'), ylabel('x(t)')
box on, grid on, set(gca,'FontSize',12)

subplot(3,2,2)
plot(Xf,Xmag,'LineWidth',2)
xlabel('Frequency (Hz)'), ylabel('|X(f)|')
box on, grid on, set(gca,'FontSize',12)
set(gca,'Xlim',[0 fs/2])


subplot(3,2,4), hold on,
plot(Hf,Hmag,'LineWidth',2)
box on, grid on, xlabel('Frequency (Hz)'), ylabel('|H(f)|')
set(gca,'Xlim',[0 fs/2]),
title('Frequency response')
set(gca,'FontSize',12)


subplot(3,2,5)
plot(t,y,'LineWidth',2)
xlabel('Time (s)'), ylabel('x(t)')
box on, grid on, set(gca,'FontSize',12)

subplot(3,2,6)
plot(Yf,Ymag,'LineWidth',2)
xlabel('Frequency (Hz)'), ylabel('|Y(f)|')
box on, grid on, set(gca,'FontSize',12)
set(gca,'Xlim',[0 fs/2])



% ==================================
% PLOT INPUT AND FILTERED SIGNAL

figure, hold on
plot(t,x,'LineWidth',2)
plot(t,y,'LineWidth',2)
xlabel('Time (s)'), ylabel('x(t)')
box on, grid on, set(gca,'FontSize',12)
legend('Input','Output')


% DELAY VISUAL: 0.00675 SECONDS
% DELAY THEORICO: ((M-1)/2)/fs

% return



%% ACTIVITY

% For each of the following questions, create script to explore and undertand the questions. 
% Recall
% you can AI to generate code, but you need to undertand all details of the
% code. Th homework will be graded only face-to-face through questions to
% the student

% At the end of each script, include discussions, observations, conclussion
% and lessons learned. It is essention that this writing is from your own
% mental process

1) Use noiseless input signal of one sinusoinf only and udnerstand the differences between "filter" and "filtfilt"

2) Compare 5 low-pass FIR filter of different order and draw conclusions

3) Implement a high-pass FIR filter (can be fully based on Demo3.m)

4) Implement a band-pass FIR filter (can be fully based on Demo3.m)


