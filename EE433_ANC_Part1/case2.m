clear; clc; close all;

%% =========================================================
% CASE 2 OFFLINE ANC WITH CALIBRATION - FAIR COMPARISON
%
% Steps:
% 1) Define true unknown secondary path S(z)
% 2) Generate calibration pairs u(n), m(n)
% 3) Estimate Shat(z) from calibration
% 4) Run Case 2 FxNLMS using estimated Shat
% 5) Run NLMS baseline with Shat = identity in the update,
%    but with the SAME true ear physics using real S(z)
% 6) Compare residual ear errors fairly
%
% NOTE:
% Frequency conditioning / bandpass preprocessing has been removed.
%% =========================================================

%% =========================
% USER SETTINGS
%% =========================
noiseFile = 'anc_noise_outputs/noise_band_limited_400_2000Hz.wav';
songFile  = 'song.mp3';

outDir = fullfile('case2_calibrated_compare_fair');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

fs_target  = 44100;
dur_sec    = 10;
trainSec   = 8;        % unchanged

% Controller
Lw         = 1024;      % unchanged
mu_fx      = 0.05;     % unchanged
mu_nlms    = 0.05;     % unchanged
delta      = 1e-6;

% Calibration
Ls_true    = 200;      % unchanged
Ls_est     = 200;      % unchanged
mu_sid     = 0.10;     % unchanged
calib_sec  = 6;        % unchanged

noiseGain  = 0.9;
songGain   = 0.3;

rng(7);

%% =========================
% LOAD MAIN ANC SIGNALS
%% =========================
[x_noise, fs1] = audioread(noiseFile);
[song, fs2]    = audioread(songFile);

if size(x_noise,2) > 1, x_noise = mean(x_noise,2); end
if size(song,2) > 1,    song    = mean(song,2);    end

if fs1 ~= fs_target, x_noise = resample(x_noise, fs_target, fs1); end
if fs2 ~= fs_target, song    = resample(song,    fs_target, fs2); end

fs = fs_target;

N = min([length(x_noise), length(song), dur_sec*fs]);
x_noise = x_noise(1:N);
song    = song(1:N);

x_noise = x_noise / (max(abs(x_noise)) + 1e-12);
song    = song    / (max(abs(song))    + 1e-12);

t = (0:N-1)'/fs;

%% =========================
% TRUE PRIMARY PATH P(z)
% noise source -> ear / error mic
%% =========================
c = 343;
d_ref = 0.20;
d_err = 0.80;

delayP_sec = (d_err - d_ref)/c;
delayP = round(delayP_sec * fs);

P = zeros(200,1);
P(1)  = 0.90;
P(18) = 0.25;
P(43) = 0.12;
P(77) = 0.06;

v0 = filter(P, 1, x_noise);
if delayP >= N
    error('Primary path delay too large.');
end
v = [zeros(delayP,1); v0(1:end-delayP)];
v = v / (max(abs(v)) + 1e-12);
v = noiseGain * v;

%% =========================
% TRUE SECONDARY PATH S(z)
% Real/unknown to the controller
%% =========================
S = zeros(Ls_true,1);
S(1)   = 0.70;
S(10)  = 0.20;
S(26)  = -0.15;
S(48)  = 0.09;
S(85)  = 0.05;
S(130) = 0.03;

%% =========================
% CALIBRATION STAGE
%% =========================
Ncal = min(calib_sec*fs, N);

u = randn(Ncal,1);
u = u / (max(abs(u)) + 1e-12);

m = filter(S, 1, u);
m = m / (max(abs(m)) + 1e-12);

Shat = zeros(Ls_est,1);
m_hat = zeros(Ncal,1);
e_sid = zeros(Ncal,1);

for n = Ls_est:Ncal
    uvec = u(n:-1:n-Ls_est+1);
    m_hat(n) = Shat.' * uvec;
    e_sid(n) = m(n) - m_hat(n);
    normFactor = delta + uvec.'*uvec;
    Shat = Shat + (mu_sid / normFactor) * e_sid(n) * uvec;
end

% identity estimate for NLMS baseline
Shat_identity = 1;   % x_f(n) = x(n)

%% =========================
% PRIMARY SIGNAL AT EAR
%% =========================
s = songGain * song;
trainN = min(round(trainSec * fs), N);
s(1:trainN) = 0;

d = s + v;
mx = max(abs(d));
if mx > 0.99
    d = 0.99 * d / mx;
end

%% =========================
% FILTERED REFERENCES
%% =========================
xf_cal = filter(Shat, 1, x_noise);          % calibrated estimate
xf_id  = filter(Shat_identity, 1, x_noise); % identity => raw x

%% =========================
% METHOD 1: CALIBRATED FxNLMS
%% =========================
w_fx   = zeros(Lw,1);
y_fx   = zeros(N,1);
ys_fx  = zeros(N,1);
e_fx   = zeros(N,1);

for n = Lw:N
    xvec  = x_noise(n:-1:n-Lw+1);
    xfvec = xf_cal(n:-1:n-Lw+1);

    y_fx(n) = w_fx.' * xvec;

    sLen = min(length(S), n);
    ys_fx(n) = S(1:sLen).' * y_fx(n:-1:n-sLen+1);

    e_fx(n) = d(n) - ys_fx(n);

    normFactor = delta + xfvec.' * xfvec;
    w_fx = w_fx + (mu_fx / normFactor) * e_fx(n) * xfvec;
end

%% =========================
% METHOD 2: NLMS BASELINE WITH Shat = I
% Fair comparison:
% - update uses raw x (because identity Shat)
% - ear physics still uses the SAME true S(z)
%% =========================
w_nl   = zeros(Lw,1);
y_nl   = zeros(N,1);
ys_nl  = zeros(N,1);
e_nl   = zeros(N,1);

for n = Lw:N
    xvec  = x_noise(n:-1:n-Lw+1);
    xfvec = xf_id(n:-1:n-Lw+1);   % raw x because Shat_identity = 1

    y_nl(n) = w_nl.' * xvec;

    sLen = min(length(S), n);
    ys_nl(n) = S(1:sLen).' * y_nl(n:-1:n-sLen+1);

    e_nl(n) = d(n) - ys_nl(n);

    normFactor = delta + xfvec.' * xfvec;
    w_nl = w_nl + (mu_nlms / normFactor) * e_nl(n) * xfvec;
end

%% =========================
% METRICS AT THE EAR
%% =========================
inputNoise  = d - s;
resNoise_fx = e_fx - s;
resNoise_nl = e_nl - s;

P_before   = mean(inputNoise.^2);
P_after_fx = mean(resNoise_fx.^2);
P_after_nl = mean(resNoise_nl.^2);

supp_fx = 10*log10(P_before / P_after_fx);
supp_nl = 10*log10(P_before / P_after_nl);

inputSNR     = 10*log10(mean(s.^2) / mean(inputNoise.^2));
outputSNR_fx = 10*log10(mean(s.^2) / mean(resNoise_fx.^2));
outputSNR_nl = 10*log10(mean(s.^2) / mean(resNoise_nl.^2));

snrImp_fx = outputSNR_fx - inputSNR;
snrImp_nl = outputSNR_nl - inputSNR;

MSE_sid   = mean(e_sid(Ls_est:end).^2);
MSE_taps  = mean((S/(max(abs(S))+1e-12) - Shat/(max(abs(Shat))+1e-12)).^2);

fprintf('=== CALIBRATION ===\n');
fprintf('System-ID MSE                 : %.6e\n', MSE_sid);
fprintf('Tap comparison MSE            : %.6e\n\n', MSE_taps);

fprintf('=== CASE 2 EAR-ERROR COMPARISON ===\n');
fprintf('Input SNR                     : %.2f dB\n\n', inputSNR);

fprintf('Calibrated FxNLMS:\n');
fprintf('  Ear suppression             : %.2f dB\n', supp_fx);
fprintf('  Ear output SNR              : %.2f dB\n', outputSNR_fx);
fprintf('  Ear SNR improvement         : %.2f dB\n\n', snrImp_fx);

fprintf('NLMS with identity Shat:\n');
fprintf('  Ear suppression             : %.2f dB\n', supp_nl);
fprintf('  Ear output SNR              : %.2f dB\n', outputSNR_nl);
fprintf('  Ear SNR improvement         : %.2f dB\n', snrImp_nl);

%% =========================
% SAVE AUDIO
%% =========================
normaudio = @(x) x/(max(abs(x))+1e-12);

audiowrite(fullfile(outDir,'calibration_u.wav'), normaudio(u), fs);
audiowrite(fullfile(outDir,'calibration_m.wav'), normaudio(m), fs);
audiowrite(fullfile(outDir,'calibration_m_hat.wav'), normaudio(m_hat), fs);

audiowrite(fullfile(outDir,'primary_ear_signal.wav'), normaudio(d), fs);
audiowrite(fullfile(outDir,'fxnlms_output.wav'), normaudio(e_fx), fs);
audiowrite(fullfile(outDir,'nlms_identity_output.wav'), normaudio(e_nl), fs);

%% =========================
% PLOTS
%% =========================
figure;
stem(0:Ls_true-1, S/(max(abs(S))+1e-12), 'filled'); hold on;
stem(0:Ls_est-1, Shat/(max(abs(Shat))+1e-12), 'r');
grid on;
xlabel('Tap index');
ylabel('Normalized amplitude');
legend('True S(z)', 'Estimated S_hat(z)');
title('Secondary Path Calibration');
saveas(gcf, fullfile(outDir,'secondary_path_taps.png'));

figure;
plot((0:Ncal-1)'/fs, m, 'LineWidth', 1); hold on;
plot((0:Ncal-1)'/fs, m_hat, 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
legend('True m(n)', 'Predicted m_hat(n)');
title('Calibration Input-Output Fit');
saveas(gcf, fullfile(outDir,'calibration_fit.png'));

figure;
subplot(4,1,1);
plot(t, d); grid on; xline(trainSec,'--r','Training End');
title('Primary Signal at Ear d(n)');
xlabel('Time (s)'); ylabel('Amp');

subplot(4,1,2);
plot(t, e_nl); grid on; xline(trainSec,'--r','Training End');
title('Ear Residual - NLMS with Identity S_hat');
xlabel('Time (s)'); ylabel('Amp');

subplot(4,1,3);
plot(t, e_fx); grid on; xline(trainSec,'--r','Training End');
title('Ear Residual - Calibrated FxNLMS');
xlabel('Time (s)'); ylabel('Amp');

subplot(4,1,4);
plot(t, s); grid on; xline(trainSec,'--r','Training End');
title('Desired Signal s(n)');
xlabel('Time (s)'); ylabel('Amp');

saveas(gcf, fullfile(outDir,'ear_time_comparison.png'));

Nfft = 2^nextpow2(N);
f = (0:Nfft/2-1)'*fs/Nfft;

VN   = fft(inputNoise, Nfft);
VNL  = fft(resNoise_nl, Nfft);
VFX  = fft(resNoise_fx, Nfft);

figure;
plot(f, 20*log10(abs(VN(1:Nfft/2))+1e-12), 'LineWidth', 1); hold on;
plot(f, 20*log10(abs(VNL(1:Nfft/2))+1e-12), 'LineWidth', 1);
plot(f, 20*log10(abs(VFX(1:Nfft/2))+1e-12), 'LineWidth', 1);
grid on; xlim([0 5000]);
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
legend('Noise Before ANC', 'Residual Noise NLMS-Identity', 'Residual Noise FxNLMS-Calibrated');
title('Residual Noise at Ear');
saveas(gcf, fullfile(outDir,'ear_noise_spectrum.png'));

figure;
bar([supp_nl, supp_fx]);
grid on;
set(gca,'XTickLabel',{'NLMS identity S_hat','FxNLMS calibrated S_hat'});
ylabel('Suppression (dB)');
title('Ear Suppression Comparison');
saveas(gcf, fullfile(outDir,'bar_suppression.png'));

%% =========================
% SAVE SUMMARY
%% =========================
fid = fopen(fullfile(outDir,'results_summary.txt'),'w');
fprintf(fid, 'CASE 2 WITH CALIBRATION\n\n');
fprintf(fid, 'True secondary path was unknown to controller.\n');
fprintf(fid, 'Calibration used u(n), m(n) pairs to estimate S_hat.\n\n');
fprintf(fid, 'Calibration MSE              : %.6e\n', MSE_sid);
fprintf(fid, 'Tap comparison MSE           : %.6e\n\n', MSE_taps);

fprintf(fid, 'Input SNR                    : %.2f dB\n\n', inputSNR);

fprintf(fid, 'NLMS with identity S_hat:\n');
fprintf(fid, 'Suppression at ear           : %.2f dB\n', supp_nl);
fprintf(fid, 'Output SNR at ear            : %.2f dB\n', outputSNR_nl);
fprintf(fid, 'SNR improvement at ear       : %.2f dB\n\n', snrImp_nl);

fprintf(fid, 'Calibrated FxNLMS:\n');
fprintf(fid, 'Suppression at ear           : %.2f dB\n', supp_fx);
fprintf(fid, 'Output SNR at ear            : %.2f dB\n', outputSNR_fx);
fprintf(fid, 'SNR improvement at ear       : %.2f dB\n', snrImp_fx);
fclose(fid);

disp('All results saved.');