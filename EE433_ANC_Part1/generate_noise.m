%% ANC Part 1 - Noise Generation, Visualization, and Saving
clear; clc; close all;

%% Output folder
outDir = 'anc_noise_outputs';
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

%% Common parameters
fs = 44100;          % sample rate for phone playback
dur = 10;            % seconds
N = fs * dur;
t = (0:N-1)' / fs;

%% Amplitude target
targetPeak = 0.95;   % avoid clipping

%% -------- 1) Single-tone noise --------
f_tone = 500; % Hz
x_tone = sin(2*pi*f_tone*t);
x_tone = targetPeak * x_tone / max(abs(x_tone));

save_and_plot_signal(x_tone, fs, outDir, ...
    'noise_single_tone_500Hz', ...
    'Single-Tone Noise (500 Hz)');

%% -------- 2) Two-tone noise --------
f1 = 300;
f2 = 900;
x_twotone = 0.8*sin(2*pi*f1*t) + 0.6*sin(2*pi*f2*t);
x_twotone = targetPeak * x_twotone / max(abs(x_twotone));

save_and_plot_signal(x_twotone, fs, outDir, ...
    'noise_two_tone_300_900Hz', ...
    'Two-Tone Noise (300 Hz + 900 Hz)');

%% -------- 3) White noise --------
rng(1); % reproducible
x_white = randn(N,1);
x_white = targetPeak * x_white / max(abs(x_white));

save_and_plot_signal(x_white, fs, outDir, ...
    'noise_white', ...
    'White Noise');

%% -------- 4) Low-frequency engine-like colored noise --------
rng(2);
x_engine = filter(1, [1 -0.98], randn(N,1));  % low-frequency emphasis
x_engine = x_engine - mean(x_engine);
x_engine = targetPeak * x_engine / max(abs(x_engine));

save_and_plot_signal(x_engine, fs, outDir, ...
    'noise_engine_like', ...
    'Engine-Like Low-Frequency Noise');

%% -------- 5) Band-limited noise --------
rng(3);
x_band = randn(N,1);

% Bandpass roughly from 400 Hz to 2000 Hz
bpFilt = designfilt('bandpassiir', ...
    'FilterOrder', 8, ...
    'HalfPowerFrequency1', 400, ...
    'HalfPowerFrequency2', 2000, ...
    'SampleRate', fs);

x_band = filtfilt(bpFilt, x_band);
x_band = x_band - mean(x_band);
x_band = targetPeak * x_band / max(abs(x_band));

save_and_plot_signal(x_band, fs, outDir, ...
    'noise_band_limited_400_2000Hz', ...
    'Band-Limited Noise (400-2000 Hz)');

%% -------- 6) Amplitude-modulated hum-like noise --------
f_carrier = 150;
f_mod = 2;
x_hum = (1 + 0.5*sin(2*pi*f_mod*t)) .* sin(2*pi*f_carrier*t);
x_hum = targetPeak * x_hum / max(abs(x_hum));

save_and_plot_signal(x_hum, fs, outDir, ...
    'noise_am_hum_150Hz', ...
    'Amplitude-Modulated Hum-Like Noise');

disp('Done. Audio and figures were saved in the folder:');
disp(outDir);

