% Define the Q15 conversion functions
toQ15 = @(x) int16(x * 32768);  % Convert from float to Q15
fromQ15 = @(x) double(x) / 32768;  % Convert from Q15 to float

fs = 2000; % Set the sampling frequency to 2000 Hz
% 2000 samples --> 2000 coefficients
t = 0:1/fs:1; % Create a time vector from 0 to 1 second with a sampling interval of 1/fs
y = sin(220 * t); % Generate a sine wave with a frequency of 220 Hz
% f=w/2*pi = 35
%plot(t, y) % Plot the sine wave in the time domain
%title('Original Sine Wave'); % Add title to the plot
%xlabel('Time (s)'); % Label the x-axis
%ylabel('Amplitude'); % Label the y-axis

N = length(y); % Get the number of samples in the signal y
fy = fft(y); % Compute the Fast Fourier Transform of the signal y
m = abs(fy); % Compute the magnitude of the FFT coefficients
m = m(1:N/2); % Keep only the first half of the magnitude spectrum
f = (0:N/2-1) * fs / N; % Create a frequency vector corresponding to the first half of the FFT
%figure; % Open a new figure window
%plot(f, m) % Plot the magnitude spectrum of the FFT
%title('Magnitude Spectrum of the Sine Wave'); % Add title to the plot
%xlabel('Frequency (Hz)'); % Label the x-axis
%ylabel('Magnitude'); % Label the y-axis

%n = randn(1, length(y)); % Generate a noise signal with the same length as y
%yn = y + n % Add the noise to the original sine wave to create a noisy signal
%figure; % Open a new figure window
%plot(t, yn); % Plot the noisy signal in the time domain
%title('Noisy Sine Wave'); % Add title to the plot
%xlabel('Time (s)'); % Label the x-axis
%ylabel('Amplitude'); % Label the y-axis

% we want to delete the noise --> filter bandpass
n = 100; % Set the filter order to 100
fh = 33 / (fs / 2); % Define the high cut-off frequency normalized to the Nyquist frequency
fl = 37 / (fs / 2); % Define the low cut-off frequency normalized to the Nyquist frequency
w = window(@hamming, n + 1); % Create a Hamming window of length n+1 for the filter
b = fir1(n, [fh fl], "bandpass", w); % Design a bandpass FIR filter with the specified cut-off frequencies and window

yn = yn/5;
% Convert to Q15
b_q15 = toQ15(b);
yn_q15 = toQ15(yn);

% Convert Q15 values back to float
yn_float = fromQ15(yn_q15); 
b_float = fromQ15(b_q15); 

%yn_float = yn_float / 5;

figure; % Open a new figure window
freqz(b) % Plot the frequency response of the designed filter
title('Frequency Response of the Bandpass Filter'); % Add title to the plot

a = 1; % Set the filter denominator to 1 (FIR filter)
y_filtered = filter(b_float, a, yn_float); % Apply the bandpass filter to the noisy signal yn
y_filtered = y_filtered * 5;
figure; % Open a new figure window
plot(t, y) % Plot the filtered signal in the time domain
title('Filtered Signal'); % Add title to the plot
xlabel('Time (s)'); % Label the x-axis
ylabel('Amplitude'); % Label the y-axis
hold off;

figure;

plot(t, y) % Plot the sine wave in the time domain
hold on;
plot(t, y_filtered) % Plot the filtered signal in the time domain
title('Original Sine Wave vs Filtered Signal'); % Add title to the plot
xlabel('Time (s)'); % Label the x-axis
ylabel('Amplitude'); % Label the y-axis