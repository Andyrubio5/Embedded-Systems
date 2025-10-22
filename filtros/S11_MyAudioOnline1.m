%% INITIALIZE
clearvars
close
clc

%% DEFINITIONS

% Sampling frequency
fs           = 8000;

% Define window
WinDura      = 0.01;
WinSamp      = ceil(WinDura*fs);
WinTime      = (0:1:WinSamp-1)/fs;
WinData      = zeros(WinSamp,1);

% Define audio device reader that can read audio from your microphone
MyADR        = audioDeviceReader('SampleRate',fs,'SamplesPerFrame',WinSamp);

% Initialize a buffer of length enought to process
BufDura      = 0.05; 
BufSamp      = ceil(BufDura*fs);
t            = (0:1:BufSamp-1)/fs;
BufData      = zeros(BufSamp,1);
BF           = dsp.AsyncBuffer(BufSamp);

% Fill the buffer completelly with zeros
write(BF, BufData);

% Get the data from the buffer
BufData  = peek(BF);

% Redefine WinTime
WinTime = WinTime+(BufDura-WinDura);

% % Plot Win and buffer
% figure(1), clf, hold on
% plot(WinTime,WinData,'LineWidth',8)
% plot(BufTime,BufData,':','LineWidth',4)
% legend('WinData','BufData')
% return

%% GET AND PLOT AUDIO DATA

% Define a maximum acquisition time
Tmax      = 1000*BufDura;

% Star data acquisition
tic
while toc < Tmax
    
    % Get WinData form the audio device (Duration of 'WinDura')
    WinData  = MyADR();
    
    % Write data into the buffer
    write(BF, WinData);
    
    % Get the data from the buffer (Duration of 'BufDura')
    x  = peek(BF);
    
    % Plot WinData and BufData
    % {
    figure(1), clf, hold on
    plot(t,x,'LineWidth',2)
    plot(WinTime,WinData,'LineWidth',1)
    xlabel('Time (s)'), ylabel('x(t)'), box on
    set(gca,'YLim',[-.2 .2])
    % }

end
