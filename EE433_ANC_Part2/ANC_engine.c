#include <stdlib.h>
#include <windows.h>
#include <mmsystem.h>
#include "Dynamic-Link Library.h" 

// Tell the linker to include the Windows Multimedia library
#pragma comment(lib, "winmm.lib")

#define FILTER_ORDER 64
#define PRIMARY_DELAY 16   // ~0.12m / 343 m/s at 44100 Hz (JBL C100SI In-line Mic)
#define S_HAT_LEN 128
#define X_HISTORY_LEN 256

// --- ANC DSP Memory Structures ---
static double w_weights[FILTER_ORDER]        = {0.0};
static double x_history[X_HISTORY_LEN]       = {0.0};
static double x_filt_history[FILTER_ORDER]   = {0.0};
static double y_history[S_HAT_LEN]           = {0.0};
static double primary_history[PRIMARY_DELAY] = {0.0};

static double s_hat[S_HAT_LEN] = {
    -0.539527,  0.103073,  -0.085869,  0.041707,  -0.036471,  -0.017989,  -0.046357,  -0.062188,  -0.041408,  0.001144,  0.056746,  0.050334,  0.070396,  0.026260,  -0.013002,  -0.032491,  
    0.045912,  0.083134,  0.064953,  0.017451,  -0.028055,  -0.081927,  -0.064446,  0.019772,  0.061294,  0.023655,  0.008050,  -0.000247,  -0.020501,  -0.004550,  0.022577,  -0.000226,  
    -0.036727,  -0.032348,  -0.014170,  0.003504,  0.018576,  0.018335,  -0.003477,  -0.011775,  -0.007013,  0.010510,  0.007916,  -0.002871,  -0.017287,  -0.012419,  0.001001,  0.011641,  
    0.017172,  0.007885,  -0.008855,  -0.008653,  0.001077,  0.013083,  0.011263,  0.007195,  0.003606,  -0.003192,  0.003925,  0.006348,  0.004424,  0.003187,  0.001083,  -0.000774,  
    0.007062,  0.002574,  0.000961,  0.000686,  -0.004450,  -0.003796,  0.000982,  0.002666,  0.003605,  0.001054,  -0.000756,  -0.001275,  0.000789,  0.002338,  0.001109,  0.000289,  
    0.000454,  0.002579,  -0.006472,  -0.000721,  -0.002325,  -0.001583,  -0.001127,  0.001342,  0.002501,  0.004297,  0.000481,  -0.002161,  -0.000929,  -0.000562,  0.001826,  -0.01428,  
    0.003343,  -0.000018,  0.001229,  -0.001417,  -0.001191,  0.003212,  0.001508,  0.000694,  -0.000118,  -0.000378,  0.000492,  0.002244,  -0.002341,  0.000169,  -0.000358,  0.001342,  
    0.001264,  0.002333,  0.000884,  -0.003596,  -0.005888,  -0.000992,  -0.000312,  0.003354,  -0.001791,  0.001387,  -0.000638,  0.001150,  0.002141,  0.000012,  0.000641
};

// --- Low-Level Windows Audio Globals ---
static HWAVEIN hWaveIn = NULL;
static WAVEHDR waveHdrA;
static WAVEHDR waveHdrB;
static short* rawBufferA = NULL;
static short* rawBufferB = NULL;
static int currentBufferToggle = 0; // 0 = Buffer A, 1 = Buffer B

// --- Function 1: Initialize Audio Hardware and Double Buffers ---
// Call this ONCE outside and before your LabVIEW While Loop begins.
__declspec(dllexport) int initCAPIHardware(int frameSize, int sampleRate) {
    WAVEFORMATEX wfx;
    wfx.wFormatTag = WAVE_FORMAT_PCM;
    wfx.nChannels = 1;                  // Mono Microphone
    wfx.nSamplesPerSec = sampleRate;    // 44100 Hz
    wfx.wBitsPerSample = 16;            // Standard 16-bit PCM Audio
    wfx.nBlockAlign = (wfx.nChannels * wfx.wBitsPerSample) / 8;
    wfx.nAvgBytesPerSec = wfx.nSamplesPerSec * wfx.nBlockAlign;
    wfx.cbSize = 0;

    // Open Default Audio Recording Device
    if (waveInOpen(&hWaveIn, WAVE_MAPPER, &wfx, 0, 0, CALLBACK_NULL) != MMSYSERR_NOERROR) {
        return -1; 
    }

    // Allocate memory blocks for the Ping-Pong buffers
    rawBufferA = (short*)malloc(frameSize * sizeof(short));
    rawBufferB = (short*)malloc(frameSize * sizeof(short));

    // Clear memory segments
    memset(rawBufferA, 0, frameSize * sizeof(short));
    memset(rawBufferB, 0, frameSize * sizeof(short));

    // Configure Hardware Buffer Header A
    waveHdrA.lpData = (LPSTR)rawBufferA;
    waveHdrA.dwBufferLength = frameSize * sizeof(short);
    waveHdrA.dwBytesRecorded = 0;
    waveHdrA.dwUser = 0;
    waveHdrA.dwFlags = 0;
    waveHdrA.dwLoops = 0;
    waveInPrepareHeader(hWaveIn, &waveHdrA, sizeof(WAVEHDR));

    // Configure Hardware Buffer Header B
    waveHdrB.lpData = (LPSTR)rawBufferB;
    waveHdrB.dwBufferLength = frameSize * sizeof(short);
    waveHdrB.dwBytesRecorded = 0;
    waveHdrB.dwUser = 0;
    waveHdrB.dwFlags = 0;
    waveHdrB.dwLoops = 0;
    waveInPrepareHeader(hWaveIn, &waveHdrB, sizeof(WAVEHDR));

    // Load both buffers into the queue so the device streams smoothly from one to the next
    waveInAddBuffer(hWaveIn, &waveHdrA, sizeof(WAVEHDR));
    waveInAddBuffer(hWaveIn, &waveHdrB, sizeof(WAVEHDR));

    // Start streaming microphone data into memory
    waveInStart(hWaveIn);
    currentBufferToggle = 0;

    // Seed DSP weights
    w_weights[0] = 0.001;

    return 0; // Success
}

// --- Function 2: Core Processing Block ---
// Put this inside your LabVIEW While Loop.
// LabVIEW no longer supplies an input array! C reads the hardware directly.
void CVIFUNC processANC(double* mic_display, double* clean_error, double* output, int frameSize, double mu, int caseSelection) {
    
    short* activeProcessingBuffer = NULL;
    WAVEHDR* currentHdr = NULL;

    // Determine which buffer pool is currently filling vs which one we process
    if (currentBufferToggle == 0) {
        currentHdr = &waveHdrA;
        activeProcessingBuffer = rawBufferA;
    } else {
        currentHdr = &waveHdrB;
        activeProcessingBuffer = rawBufferB;
    }

    // Block thread execution until the current buffer chunk is 100% full of fresh audio
    while (!(currentHdr->dwFlags & WHDR_DONE)) {
        Sleep(1); // Yield execution frame control (~1ms) to keep CPU overhead at 0%
    }

    // Unset driver flag immediately so hardware can reuse this container later
    currentHdr->dwFlags &= ~WHDR_DONE;

    // --- Main DSP Loop Execution ---
    for (int i = 0; i < frameSize; i++) {

        // Normalize raw 16-bit signed integer hardware sample [-32768, 32767] to double [-1.0, 1.0]
        double mic_sample = (double)activeProcessingBuffer[i] / 32768.0;
        
        // Pass out to LabVIEW wire so front-panel UI graph still updates
        mic_display[i] = mic_sample; 

        // 1. Shift and update raw input history
        for (int j = X_HISTORY_LEN - 1; j > 0; j--) {
            x_history[j] = x_history[j - 1];
        }
        x_history[0] = mic_sample;

        // 2. Calculate filtered-X reference sample (convolve input with s_hat)
        double x_filt = 0.0;
        for (int j = 0; j < S_HAT_LEN; j++) {
            x_filt += s_hat[j] * x_history[j];
        }

        // 3. Shift and update filtered reference history 
        for (int j = FILTER_ORDER - 1; j > 0; j--) {
            x_filt_history[j] = x_filt_history[j - 1];
        }
        x_filt_history[0] = x_filt;

        // 4. Compute filter output (Anti-Noise)
        double y = 0.0;
        for (int j = 0; j < FILTER_ORDER; j++) {
            y += w_weights[j] * x_history[j];
        }
        output[i] = y; 

        // 5. Shift and update anti-noise output history
        for (int j = S_HAT_LEN - 1; j > 0; j--) {
            y_history[j] = y_history[j - 1];
        }
        y_history[0] = y;

        // 6. Simulate Primary Path d(n)
        double d_sim = primary_history[PRIMARY_DELAY - 1];
        for (int j = PRIMARY_DELAY - 1; j > 0; j--) {
            primary_history[j] = primary_history[j - 1];
        }
        primary_history[0] = mic_sample;

        // 7. Simulate Secondary Path output y_sec(n)
        double y_sec = 0.0;
        for (int j = 0; j < S_HAT_LEN; j++) {
            y_sec += s_hat[j] * y_history[j];
        }

        // 8. Simulate Earcup Error Signal
        double error = d_sim - y_sec;
        clean_error[i] = error;
        
        // 9. Adaptive Weight Updates based on Selection
        if (caseSelection == 1) {
            double norm = 1e-6;
            for (int j = 0; j < FILTER_ORDER; j++) {
                norm += x_history[j] * x_history[j];
            }
            for (int j = 0; j < FILTER_ORDER; j++) {
                w_weights[j] = (0.9999 * w_weights[j]) + (mu / norm) * error * x_history[j];
            }
        }
        else if (caseSelection == 2) {
            double norm_fx = 1e-1;
            for (int j = 0; j < FILTER_ORDER; j++) {
                norm_fx += x_filt_history[j] * x_filt_history[j];
            }
            for (int j = 0; j < FILTER_ORDER; j++) {
                w_weights[j] = (0.9999 * w_weights[j]) + (mu / norm_fx) * error * x_filt_history[j];
            }
        }
    }

    // Immediately recycle the processed block back to the hardware device queue
    waveInAddBuffer(hWaveIn, currentHdr, sizeof(WAVEHDR));

    // Flip toggle pointer to process the companion background buffer on the next loop cycle
    currentBufferToggle = (currentBufferToggle == 0) ? 1 : 0;
}

// --- Function 3: Hardware Close Routine ---
// Call this ONCE outside and to the right of your LabVIEW While Loop when execution stops.
__declspec(dllexport) void closeCAPIHardware(void) {
    if (hWaveIn) {
        waveInReset(hWaveIn);
        waveInUnprepareHeader(hWaveIn, &waveHdrA, sizeof(WAVEHDR));
        waveInUnprepareHeader(hWaveIn, &waveHdrB, sizeof(WAVEHDR));
        waveInClose(hWaveIn);
        hWaveIn = NULL;
    }
    if (rawBufferA) { free(rawBufferA); rawBufferA = NULL; }
    if (rawBufferB) { free(rawBufferB); rawBufferB = NULL; }
}