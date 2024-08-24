
clc
%Samples per Second
Fs=2000000;
T=1/Fs;
%Number of Samples in the Signal
Lenght=4000;
t=(0:Lenght-1)*T;
%Generate the input signal
y=0.5*sin(2*pi*10000*t)+0.25*sin(2*pi*150000*t);
plot(y)
%Define filter cut_off frequency (60 KHz)
cutoff_Freq=60000;
%Nyquist frequency
NYQ_Freq=Fs/2;
cutoff_norm=cutoff_Freq/NYQ_Freq;
%FIR filter order
order=41;
%Create low-pass FIR filter
FIR_Coeff=fir1(order, cutoff_norm);
%Filter the input signal (y) with the FIR filter
Filterd_signal=filter(FIR_Coeff, 1, y);
%Using Filter Visualization Tool
fvtool(FIR_Coeff,"Fs",Fs)


