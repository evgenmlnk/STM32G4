% Given parameters
Fs = 2000000; % Sampling frequency in Hz
T = 1/Fs;   % Sampling period in seconds
Lenght = 4096*2;  % Validation of 2 frames x4096 samples
% Time vector
t = (0:Lenght-1)*T;

% Generate the input signal: 10 kHz and 150 kHz sine waves
Input_signal = 0.5*sin(2*pi*10000*t) + 0.25*sin(2*pi*150000*t);

% Define filter cut-off frequency (30 kHz)
Cutoff_Freq = 12000;

% Nyquist frequency
Nyq_Freq = Fs/2;
cutoff_norm = Cutoff_Freq / Nyq_Freq;

%FIR filter order
order = 40;

% Creat low-pass FIR-Filter
FIR_Coeff = fir1(order, cutoff_norm, hann(order+1));

% Filter the input signal
Filtered_signal = filter(FIR_Coeff, 1, Input_signal);

%fvtool(FIR_Coeff, 'Fs', Fs);

% Convert to Q15
FIR_Coeff_q15 = int16(FIR_Coeff * 32768);
Input_signal_q15 = int16(Input_signal * 32768);
%Filtered_signal_q15 = int16(Filtered_signal * 32768);

% Write to a CSV file
writematrix(FIR_Coeff_q15, 'FIR_Coeff_q15.csv');
writematrix(Input_signal_q15, 'Input_signal_q15.csv');

% Convert from q15 to double
stm32_cmsis_fir_out =  stm32_cmsis_fir_out_q15/32768;

%Plot time-domain signals
figure;
plot(t, Filtered_signal);
hold on;
plot(t, stm32_cmsis_fir_out);
title('Time Domain Comparison');
xlabel('Time (seconds)');
ylabel('Amplitude');
legend('Filtered Signal','STM32 CMSIS FIR Output');

% Frequency-domain analysis
N = length(Filtered_signal);           % Number of samples
Y = fft(Filtered_signal);              % Compute the FFT
P2 = abs(Y/N);                      % Two-sided spectrum
P1 = P2(1:N/2+1);                   % Single-sided spectrum
P1(2:end-1) = 2*P1(2:end-1);        % Adjust for single-sided spectrum

f = Fs*(0:(N/2))/N;                 % Frequency vector


N_stm32_cmsis_fir = length(stm32_cmsis_fir_out);
Y_stm32_cmsis_fir = fft(stm32_cmsis_fir_out);
P2_stm32_cmsis_fir = abs(Y_stm32_cmsis_fir/N_stm32_cmsis_fir);
P1_stm32_cmsis_fir = P2_stm32_cmsis_fir(1:N_stm32_cmsis_fir/2+1);
P1_stm32_cmsis_fir(2:end-1) = 2*P1_stm32_cmsis_fir(2:end-1);
f_stm32_cmsis_fir = Fs*(0:(N_stm32_cmsis_fir/2))/N_stm32_cmsis_fir;

% Plot error
error_signal = Filtered_signal - stm32_cmsis_fir_out;

figure;
plot(t, error_signal);
title('Error Signal (Filtered Signal - STM32 CMSIS FIR Output)');
xlabel('Time (seconds)');
ylabel('Error Amplitude');

% Plot the single-sided magnitude spectrum
figure;
plot(f, P1);
hold on;
plot(f_stm32_cmsis_fir , P1_stm32_cmsis_fir );
title('Single-Sided Magnitude Spectrum of Signal');
xlabel('Frequency (Hz)');
ylabel('|P1(f)|');

legend('Filtered Signal', 'STM32 CMSIS FIR Output');

