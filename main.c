#define ALSA_IMPLEMENTATION
#include "nanoalsa.h"
#include "filter_coefficients.h"
#include <rtl-sdr.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <stdio.h>
#include <ncurses.h>

#define M_PI 3.14159265358979323846

#define SDR_SAMPLE_RATE 1536000
#define SDR_BUFFER_SIZE 32768 // Samples not bytes
#define AUDIO_SAMPLE_RATE 48000
#define DECIMATION_FACTOR (SDR_SAMPLE_RATE / AUDIO_SAMPLE_RATE)
#define AUDIO_OUTPUT_LENGTH (SDR_BUFFER_SIZE / DECIMATION_FACTOR)
#define FM_DEVIATION 75000

rtlsdr_dev_t* dev = NULL; // Used to read samples from the SDR device
alsa_t ctx; // Used to play audio

int msleep(long msec);
double elapsed_ms(struct timespec start, struct timespec end);

// FLOW: Samples -> Rescaler -> FM Filter -> Polar Discriminator -> Audio Filter -> Decimator -> Audio Output
unsigned char raw_signal[2 * SDR_BUFFER_SIZE]; // Complex SDR samples range 0-255
double scaled_signal[2 * SDR_BUFFER_SIZE];
double fm_filter_output[2 * SDR_BUFFER_SIZE]; // Complex output of 100 KHz LPF
double polar_discriminator_output[SDR_BUFFER_SIZE]; // Real samples of polar discriminator
double audio_filter_output[SDR_BUFFER_SIZE]; // Real samples from 20 KHz LPF
float audio_output[AUDIO_OUTPUT_LENGTH] = {}; // Decimated output in float format ready for sound card

// Control variables
uint32_t radio_station_frequency;
volatile bool use_overlap_add = false;
volatile bool use_fm_filter = false;
volatile bool use_audio_filter = false;
volatile bool use_sleep = true;

// Read SDR_BUFFER_SIZE samples range 0-255 from SDR device
void read_samples() {
    int n_read;
    rtlsdr_read_sync(dev, raw_signal, 2 * SDR_BUFFER_SIZE, &n_read);
}

// Scale IQ samples to range -1.0 to 1.0
void scale() {
    for(int i = 0;i < 2 * SDR_BUFFER_SIZE;i++) {
        scaled_signal[i] = ((double)raw_signal[i] - 127.5) / 127.5;
    }
}

// LPF 100 KHz
double fm_filter_tail[2 * (FM_FILTER_LENGTH - 1)] = {};
void fm_filter() {
    if(!use_fm_filter) {
        for(int i = 0;i < 2 * SDR_BUFFER_SIZE;i++) {
            fm_filter_output[i] = scaled_signal[i];
        }
        return;
    }
    uint32_t output_length = SDR_BUFFER_SIZE + FM_FILTER_LENGTH - 1;
    for(int n = 0;n < output_length;n++) {
        int h_idx = 0, x_idx = 2 * n;
        double acc_i = 0, acc_q = 0;
        while(x_idx >= 0 && h_idx < FM_FILTER_LENGTH) {
            if(x_idx + 1 < 2 *SDR_BUFFER_SIZE) {
                acc_i += scaled_signal[x_idx] * fm_filter_coefficients[h_idx];
                acc_q += scaled_signal[x_idx + 1] * fm_filter_coefficients[h_idx];
            }
            x_idx -= 2;
            h_idx++;
        }
        if(n < FM_FILTER_LENGTH - 1 && use_overlap_add) {
            fm_filter_output[2 * n] = acc_i + fm_filter_tail[2 * n];
            fm_filter_output[2 * n + 1] = acc_q + fm_filter_tail[2 * n + 1];
        }
        else if(n < SDR_BUFFER_SIZE){
            fm_filter_output[2 * n] = acc_i;
            fm_filter_output[2 * n + 1] = acc_q;
        }
        else {
            fm_filter_tail[2 * (n - SDR_BUFFER_SIZE)] = acc_i;
            fm_filter_tail[2 * (n - SDR_BUFFER_SIZE) + 1] = acc_q;
        }
    }
}

// Estimate the instantaneous frequency at each sample point
double previous_sample_i = 0, previous_sample_q = 0;
void polar_interpolator() {
    for(int i = 0;i < 2 * SDR_BUFFER_SIZE;i+=2){
        double current_sample_i = fm_filter_output[i];
        double current_sample_q = fm_filter_output[i + 1];

        // Conjugate
        previous_sample_q *= -1.0;

        // Estimate phase difference
        double real = current_sample_i * previous_sample_i - current_sample_q * previous_sample_q;
        double imag = current_sample_i * previous_sample_q + current_sample_q * previous_sample_i;
        double arg = atan2f(imag, real);

        // Update
        previous_sample_i = current_sample_i;
        previous_sample_q = current_sample_q;
        polar_discriminator_output[i / 2] = arg * (1.0 * SDR_SAMPLE_RATE / (2.0 * M_PI * FM_DEVIATION));
    }
}

// 20 KHz LPF
double audio_filter_tail[AUDIO_FILTER_LENGTH - 1] = {};
void audio_filter() {
    if(!use_audio_filter) {
        for(int i = 0;i < SDR_BUFFER_SIZE;i++) {
            audio_filter_output[i] = polar_discriminator_output[i];
        }
        return;
    }
    uint32_t output_length = SDR_BUFFER_SIZE + AUDIO_FILTER_LENGTH - 1;
    for(int n = 0;n < output_length;n++) {
        int h_idx = 0, x_idx = n;
        double acc = 0;
        while(x_idx >= 0 && h_idx < AUDIO_FILTER_LENGTH) {
            if(x_idx < SDR_BUFFER_SIZE) {
             acc += polar_discriminator_output[x_idx] * audio_filter_coefficients[h_idx];
            }
            x_idx--;
            h_idx++;
        }
        if(n < AUDIO_FILTER_LENGTH - 1 && use_overlap_add) {
            audio_filter_output[n] = acc + audio_filter_tail[n];
        }
        else if(n < SDR_BUFFER_SIZE){
            audio_filter_output[n] = acc;
        }
        else {
            audio_filter_tail[n - SDR_BUFFER_SIZE] = acc;
        }
    }
}

void decimate() {
    for(int i = 0;i < AUDIO_OUTPUT_LENGTH;i++){
        audio_output[i] = (float)(audio_filter_output[i * DECIMATION_FACTOR]);
    }
}

volatile int go_a = 0;
volatile int go_b = 0;
volatile int go_c = 0;
volatile bool finished = false;
volatile bool paused = false;
void* worker_a_func(void* arg) {
    while(1){
        if(finished) return NULL;
        while(!go_a || paused){
            // Wait for signal
        }
        go_a = 0;
        struct timespec start, end, sample_start, sample_end;
        // Sample
        clock_gettime(CLOCK_MONOTONIC, &sample_start);
        read_samples();
        clock_gettime(CLOCK_MONOTONIC, &sample_end);
        go_b = 1;
        // Process
        clock_gettime(CLOCK_MONOTONIC, &start);
        scale();
        fm_filter();
        polar_interpolator();
        audio_filter();
        decimate();
        clock_gettime(CLOCK_MONOTONIC, &end);
        // Yield spare CPU time
        double elapsed_milliseconds = elapsed_ms(start, end);
        double sampling_time = elapsed_ms(sample_start, sample_end);
        double target = 85;
        double tts = target - elapsed_milliseconds;
        //printf("%lf %lf %lf %ld %lf\n", elapsed_milliseconds, target, tts, (long)tts, sampling_time);
        if(use_sleep) msleep((long)tts);
    }
}
volatile unsigned long iter = 0;
void* worker_b_func(void* arg) {
    while(1) {
        if(finished) return NULL;
        while(!go_b || paused){
            // Wait for signal
        }
        go_b = 0;
        struct timespec start, end, sample_start, sample_end;
        // Sample
        clock_gettime(CLOCK_MONOTONIC, &sample_start);
        read_samples();
        clock_gettime(CLOCK_MONOTONIC, &sample_end);
        go_a = 1;
        // Process
        clock_gettime(CLOCK_MONOTONIC, &start);
        scale();
        fm_filter();
        polar_interpolator();
        audio_filter();
        decimate();
        clock_gettime(CLOCK_MONOTONIC, &end);
        // Yield spare CPU time
        double elapsed_milliseconds = elapsed_ms(start, end);
        double sampling_time = elapsed_ms(sample_start, sample_end);
        double target = 85;
        double tts = target - elapsed_milliseconds;
        if(use_sleep) msleep((long)tts);
        //printf("%lf %lf %lf %ld %lf\n", elapsed_milliseconds, target, tts, (long)tts, sampling_time);
    }
}

void* worker_c_func(void* arg) {
    while(1) {
        if(finished) {
            return NULL;
        }
        int sound_result = alsa_write(&ctx, audio_output, AUDIO_OUTPUT_LENGTH);
        if(sound_result < 0) {
            printf("ioctl failed with errno: %d\n", errno);
            finished = true;
        }
    }
}

void change_station(uint32_t new_station) {
    radio_station_frequency = new_station;
    move(0, 0);
    printw("Listening to: %.1lf", radio_station_frequency / 1000000.0);
    rtlsdr_set_center_freq(dev, new_station);
    rtlsdr_reset_buffer(dev);
    paused = true;
    for(int i = 0;i < FM_FILTER_LENGTH - 1;i++) {
        fm_filter_tail[i] = 0;
    }
    for(int i = 0;i < AUDIO_FILTER_LENGTH - 1;i++) {
        audio_filter_tail[i] = 0;
    }
    paused = false;
}

void toggle_overlap_add() {
    use_overlap_add = !use_overlap_add;
    move(2, 0);
    clrtoeol();
    printw("1. Overlap Add: %s", use_overlap_add ? "Enabled" : "Disabled");
}

void toggle_fm_filter() {
    use_fm_filter = !use_fm_filter;
    move(3, 0);
    clrtoeol();
    printw("2. FM Filter: %s", use_fm_filter ? "Enabled" : "Disabled");
}

void toggle_audio_filter() {
    use_audio_filter = !use_audio_filter;
    move(4, 0);
    clrtoeol();
    printw("3. Audio Filter: %s", use_audio_filter ? "Enabled" : "Disabled");
}

void toggle_sleep() {
    use_sleep = !use_sleep;
    move(5, 0);
    clrtoeol();
    printw("4. Sleep: %s", use_sleep ? "Enabled" : "Disabled");
}


void* worker_d_func(void* arg) {
    while(true) {
        int ch = getch();
        switch(ch) {
            case KEY_LEFT:
                change_station(radio_station_frequency - 200000);
                break;
            case KEY_RIGHT:
                change_station(radio_station_frequency + 200000);
                break;
            case 49:
                toggle_overlap_add();
                break;
            case 50:
                toggle_fm_filter();
                break;
            case 51:
                toggle_audio_filter();
                break;
            case 52:
                toggle_sleep();
                break;
            default:
                finished = true;
                return NULL;
        }
        refresh();
    }
}

int main(int argc, char *argv[]) {
    // Init SDR
    int r = rtlsdr_open(&dev, 0);
    if (r < 0 || !dev) {
        fprintf(stderr, "Error opening RTL-SDR device.\n");
        return 1;
    }
    rtlsdr_set_sample_rate(dev, SDR_SAMPLE_RATE);
    rtlsdr_set_tuner_gain_mode(dev, 0);  // auto gain

    // Init audio
    alsa_open(&ctx, 0, 0, SNDRV_PCM_FORMAT_FLOAT_LE, AUDIO_SAMPLE_RATE, 1);

    // Init controls
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    change_station(96500000);
    toggle_overlap_add();
    toggle_fm_filter();
    toggle_audio_filter();
    toggle_sleep();

    // Init threads
    pthread_t worker_a, worker_b, worker_c, worker_d;
    pthread_create(&worker_a, NULL, worker_a_func, NULL); // DSP thread 1
    pthread_create(&worker_b, NULL, worker_b_func, NULL); // DSP thread 2
    pthread_create(&worker_c, NULL, worker_c_func, NULL); // Sound thread
    pthread_create(&worker_d, NULL, worker_d_func, NULL); // Input thread
    go_a = 1;
    pthread_join(worker_a, NULL);
    pthread_join(worker_b, NULL);
    pthread_join(worker_c, NULL);
    pthread_join(worker_d, NULL);

    endwin();
    alsa_close(&ctx);
    rtlsdr_close(dev);
}

int msleep(long msec)
{
    struct timespec ts;
    int res;

    if (msec < 0)
    {
        errno = EINVAL;
        return -1;
    }

    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (msec % 1000) * 1000000;

    do {
        res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);

    return res;
}

double elapsed_ms(struct timespec start, struct timespec end) {
    return (end.tv_sec - start.tv_sec) * 1000.0 +
    (end.tv_nsec - start.tv_nsec) / 1e6;
}
