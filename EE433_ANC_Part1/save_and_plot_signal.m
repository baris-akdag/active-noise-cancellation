function save_and_plot_signal(x, fs, outDir, baseName, plotTitle)

    % Save WAV
    wavPath = fullfile(outDir, [baseName '.wav']);
    audiowrite(wavPath, x, fs);

    % Save short time-domain plot
    figure('Visible', 'off');
    nShow = min(length(x), 0.03*fs); % first 30 ms
    plot((0:nShow-1)/fs, x(1:nShow), 'LineWidth', 1);
    grid on;
    xlabel('Time (s)');
    ylabel('Amplitude');
    title([plotTitle ' - Time Domain']);
    saveas(gcf, fullfile(outDir, [baseName '_time.png']));
    close(gcf);

    % Frequency-domain plot using FFT
    Nfft = 2^nextpow2(length(x));
    X = fft(x, Nfft);
    f = (0:Nfft/2-1)' * fs / Nfft;
    mag = abs(X(1:Nfft/2));
    mag_dB = 20*log10(mag + 1e-12);

    figure('Visible', 'off');
    plot(f, mag_dB, 'LineWidth', 1);
    grid on;
    xlabel('Frequency (Hz)');
    ylabel('Magnitude (dB)');
    title([plotTitle ' - Frequency Spectrum']);
    xlim([0 fs/2]);
    saveas(gcf, fullfile(outDir, [baseName '_spectrum.png']));
    close(gcf);

    % Also save spectrogram for report
    figure('Visible', 'off');
    spectrogram(x, 1024, 768, 1024, fs, 'yaxis');
    title([plotTitle ' - Spectrogram']);
    saveas(gcf, fullfile(outDir, [baseName '_spectrogram.png']));
    close(gcf);
end