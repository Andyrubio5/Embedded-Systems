%% IIR FILTERS

clearvars
close all
clc



%% CUT-OFF FREQUENCY

% Sampling frequency (Hz)
fs       = 4000;

% Cut-off frequency of the band pass (Hz)
fc       = 850;

% Check for Nyquist
if fc>=fs/2
    error('PILAS: Nyquist impone que fc no puede ser mayor fs/2')
end

% Define cut-off frequency
% pi ---> fs/2
% wo --->  fc
% wo       = pi*fc/(fs/2); % Frequencies in rad/samples i.e. From 0 to pi

% Define normalized edge frequency
% 1  ---> fs/2
% wn --->  fc
wn       = fc/(fs/2);  % Matlab uses normalized frequencies i.e. From 0 to 1


%% FREQUENCY RESPONSE

% Filter desing
Nord       = 3;
[b,a]      = butter(Nord,wn,'Low');

% Compute H
[H, w]     = freqz(b,a,2001);
w          = (w/pi)*(fs/2);
% w=w/pi;

% Compute magnitude anfd phase
Hmag       = abs(H);
% Hmag       = 20*log(Hmag);

Hang       = angle(H); 
% Hang       = (180*(Hang)/pi); % unwrap

% Plot frequency response
figure 

subplot(2,1,1)
plot(w,Hmag,'LineWidth',2)
xlabel('w')
ylabel('|H(f)|')
grid on

subplot(2,1,2)
plot(w,Hang,'LineWidth',2)
xlabel('w')
ylabel('\angle H(w)')
grid on



%% FVTOOL: FILTER VISUALIZATION TOOL

% Filter desing
fcl      = 500;
fch      = 1000;
wnl      = fcl/(fs/2);
wnh      = fch/(fs/2);

[b,a]    = butter(6,[wnl wnh]);

% FVTool
fvtool(b,a)



%% ATENUATION AT FC

% Filter desing
Nord       = 8;
[b,a]      = butter(Nord,wn,'Low');

% Compute H
[H, w]     = freqz(b,a,fs/2);
w          = (w/pi)*(fs/2);

% Compute magnitude anfd phase
Hmag       = abs(H);
% Hmag       = 20*log(abs(H));

Hang       = unwrap(angle(H)); 
%H_ang       = 180*unwrap(angle(H))/pi;

% Plot frequency response
figure

subplot(2,1,1)
plot(w,Hmag,'LineWidth',2)
xline(fc,'Color','m','LineWidth',4)
xlabel('w')
ylabel('|H(f)|')
grid on

subplot(2,1,2)
plot(w,Hang,'LineWidth',2)
xline(fc,'Color','m','LineWidth',4)
xlabel('w')
ylabel('\angle H(w)')
grid on

% Analisis
[~, indfc] = min(abs(w-fc));
att        = Hmag(indfc);

fprintf('  Order     Attenuation \n')
fprintf('   %i          %.4f \n',Nord,att)



%% EFFECT OF ORDER

% Filter desing
[b1,a1]    = butter(1,wn,'low');
[b2,a2]    = butter(2,wn,'low');
[b3,a3]    = butter(3,wn,'low');
[b4,a4]    = butter(4,wn,'low');

% Compute H
[H1, ~]    = freqz(b1,a1,fs/2);
[H2, ~]    = freqz(b2,a2,fs/2);
[H3, ~]    = freqz(b3,a3,fs/2);
[H4, w]    = freqz(b4,a4,fs/2);
w          = (w/pi)*(fs/2);

% Compute magnitude anfd phase
H1mag      = abs(H1);
H2mag      = abs(H2);
H3mag      = abs(H3);
H4mag      = abs(H4);
% H1mag      = 20*log(abs(H1));
% H2mag      = 20*log(abs(H2));
% H3mag      = 20*log(abs(H3));
% H4mag      = 20*log(abs(H4));

H1ang      = unwrap(angle(H1));
H2ang      = unwrap(angle(H2));
H3ang      = unwrap(angle(H3));
H4ang      = unwrap(angle(H4));

% Plot frequency response
figure

subplot(2,1,1), hold on
plot(w,H1mag,'LineWidth',2)
plot(w,H2mag,'LineWidth',2)
plot(w,H3mag,'LineWidth',2)
plot(w,H4mag,'LineWidth',2)
xline(fc,'Color','m')
xlabel('w')
ylabel('|H(f)|')
grid on, box on


subplot(2,1,2), hold on
plot(w,H1ang,'LineWidth',2)
plot(w,H2ang,'LineWidth',2)
plot(w,H3ang,'LineWidth',2)
plot(w,H4ang,'LineWidth',2)
xline(fc,'Color','m')
xlabel('w')
ylabel('\angle H(w)')
grid on, box on



%% COMPARE IIR FILTERS 

% Filter desing
Nord       = 2;
[b1,a1]    = butter(Nord,wn,'low');
[b2,a2]    = cheby1(Nord,2,wn,'low');
[b3,a3]    = cheby2(Nord,20,wn,'low');
[b4,a4]    = ellip(Nord,2,20,wn,'low');

% Compute H
[H1, ~]    = freqz(b1,a1,fs/2);
[H2, ~]    = freqz(b2,a2,fs/2);
[H3, ~]    = freqz(b3,a3,fs/2);
[H4, w]    = freqz(b4,a4,fs/2);
w          = (w/pi)*(fs/2);

% Compute magnitude anfd phase
H1mag      = abs(H1);
H2mag      = abs(H2);
H3mag      = abs(H3);
H4mag      = abs(H4);
% H1mag      = 20*log(H1mag);
% H2mag      = 20*log(H1mag);
% H3mag      = 20*log(H1mag);
% H4mag      = 20*log(H1mag);

H1ang      = unwrap(angle(H1));
H2ang      = unwrap(angle(H2));
H3ang      = unwrap(angle(H3));
H4ang      = unwrap(angle(H4));

% Plot frequency response
figure

subplot(2,1,1), hold on
plot(w,H1mag,'LineWidth',2)
plot(w,H2mag,'LineWidth',2)
plot(w,H3mag,'LineWidth',2)
plot(w,H4mag,'LineWidth',2)
xline(fc,'Color','m')
xlabel('w')
ylabel('|H(f)|')
grid on, box on


subplot(2,1,2), hold on
plot(w,H1ang,'LineWidth',2)
plot(w,H2ang,'LineWidth',2)
plot(w,H3ang,'LineWidth',2)
plot(w,H4ang,'LineWidth',2)
xline(fc,'Color','m')
xlabel('w')
ylabel('\angle H(w)')
grid on, box on
legend('butter','cheby1','cheby2','ellip')

