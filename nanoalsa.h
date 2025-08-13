/*
 * nanoalsa.h
 *
 * Copyright (C) 2023 bzt (bztsrc@gitlab) MIT license
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * @brief Simple and suckless PCM ALSA player library (replaces that buggy bloated asoundlib)
 * https://gitlab.com/bztsrc/nanoalsa
 */

#ifndef _NANOALSA_H
#define _NANOALSA_H

#ifdef  __cplusplus
extern "C" {
    #endif

    /* we don't need asoundlib, we just use its header for the enums and structs, that's all */
    #include <sound/asound.h>

    /* error codes */
    enum {
        ALSA_SUCCESS,
        ALSA_ERR_INP,
        ALSA_ERR_DEV,
        ALSA_ERR_HWPAR,
        ALSA_ERR_SWPAR,
        ALSA_ERR_MMAP,
        ALSA_ERR_PREP,
        ALSA_ERR_THREAD
    };

    /* callback prototype */
    typedef void (*alsa_callback_t)(void *buf, int samplesize, int channels, int frames, void *data);

    /* main ALSA context */
    typedef struct {
        int fd, fmt, freq, chan;
        #ifndef ALSA_NORESAMPLE
        unsigned int reslen, needresample;
        short *resample;
        #endif
        unsigned int period_size, period_count, sample_size, boundary;
        struct snd_pcm_mmap_status *mmap_status;
        struct snd_pcm_mmap_control *mmap_control;
        #ifdef _PTHREAD_H
        alsa_callback_t cb;
        pthread_t th;
        void *abuf, *data;
        #endif
    } alsa_t;

    /* public API */
    int alsa_open(alsa_t *ctx, int card, int device, int fmt, int freq, int chan);
    int alsa_write(alsa_t *ctx, void *buf, unsigned int numframes);
    void alsa_close(alsa_t *ctx);

    #ifdef _PTHREAD_H
    int alsa_start(alsa_t *ctx, alsa_callback_t callback, void *data);
    int alsa_stop(alsa_t *ctx);
    #endif

    #ifdef ALSA_IMPLEMENTATION

    #ifndef ALSA_BUFFER_SIZE
    #define ALSA_BUFFER_SIZE 4096
    #endif
    #ifndef ALSA_PERIOD_SIZE
    #define ALSA_PERIOD_SIZE 1024
    #endif

    #include <stdio.h>
    #include <string.h>
    #include <fcntl.h>
    #include <unistd.h>
    #include <endian.h>
    #include <sys/mman.h>
    #include <sys/ioctl.h>
    #include <linux/ioctl.h>

    /* get host native signed 16 bit */
    #ifndef SNDRV_PCM_FORMAT_S16
    # if __BYTE_ORDER == __LITTLE_ENDIAN
    #  define SNDRV_PCM_FORMAT_S16 SNDRV_PCM_FORMAT_S16_LE
    # else
    #  define SNDRV_PCM_FORMAT_S16 SNDRV_PCM_FORMAT_S16_BE
    # endif
    #endif

    /* helpers for hw_params */
    static struct snd_interval *param_to_interval(struct snd_pcm_hw_params *p, int n)
    {
        return &(p->intervals[n - SNDRV_PCM_HW_PARAM_FIRST_INTERVAL]);
    }

    static struct snd_mask *param_to_mask(struct snd_pcm_hw_params *p, int n)
    {
        return &(p->masks[n - SNDRV_PCM_HW_PARAM_FIRST_MASK]);
    }

    static void param_set_mask(struct snd_pcm_hw_params *p, int n, unsigned int bit)
    {
        struct snd_mask *m = param_to_mask(p, n);
        if (bit >= SNDRV_MASK_MAX) return;
        m->bits[0] = 0; m->bits[1] = 0; m->bits[bit >> 5] |= (1 << (bit & 31));
    }

    static void param_set_min(struct snd_pcm_hw_params *p, int n, unsigned int val)
    {
        struct snd_interval *i = param_to_interval(p, n);
        i->min = val;
    }

    static void param_set_int(struct snd_pcm_hw_params *p, int n, unsigned int val)
    {
        struct snd_interval *i = param_to_interval(p, n);
        i->min = val; i->max = val; i->integer = 1;
    }

    static unsigned int param_get_int(struct snd_pcm_hw_params *p, int n)
    {
        struct snd_interval *i = param_to_interval(p, n);
        return (i->integer) ? i->max : 0;
    }

    static void param_init(struct snd_pcm_hw_params *p)
    {
        struct snd_mask *m;
        struct snd_interval *i;
        int n;

        memset(p, 0, sizeof(struct snd_pcm_hw_params));
        for (n = SNDRV_PCM_HW_PARAM_FIRST_MASK; n <= SNDRV_PCM_HW_PARAM_LAST_MASK; n++) {
            m = param_to_mask(p, n); m->bits[0] = ~0; m->bits[1] = ~0;
        }
        for (n = SNDRV_PCM_HW_PARAM_FIRST_INTERVAL; n <= SNDRV_PCM_HW_PARAM_LAST_INTERVAL; n++) {
            i = param_to_interval(p, n); i->min = 0; i->max = ~0;
        }
        p->rmask = ~0U; p->cmask = 0; p->info = ~0U;
    }


    /**
     * Open the ALSA context
     */
    int alsa_open(alsa_t *ctx, int card, int device, int fmt, int freq, int chan)
    {
        char dev[64];
        struct snd_pcm_hw_params params;
        struct snd_pcm_sw_params spar;

        if(!ctx || card < 0 || device < 0 || freq < 1 || chan < 1) return ALSA_ERR_INP;
        memset(ctx, 0, sizeof(alsa_t));
        sprintf(dev, "/dev/snd/pcmC%uD%up", card, device);
        ctx->fd = open(dev, O_RDWR);
        if(ctx->fd < 1) { ctx->fd = -1; return ALSA_ERR_DEV; }
        ctx->fmt = fmt; ctx->chan = chan; ctx->freq = freq;
        param_init(&params);
        param_set_mask(&params, SNDRV_PCM_HW_PARAM_ACCESS, SNDRV_PCM_ACCESS_RW_INTERLEAVED);
        param_set_mask(&params, SNDRV_PCM_HW_PARAM_FORMAT, fmt);
        param_set_min(&params, SNDRV_PCM_HW_PARAM_BUFFER_SIZE, ALSA_BUFFER_SIZE);
        param_set_min(&params, SNDRV_PCM_HW_PARAM_PERIOD_SIZE, ALSA_PERIOD_SIZE);
        param_set_int(&params, SNDRV_PCM_HW_PARAM_PERIODS, ALSA_BUFFER_SIZE / ALSA_PERIOD_SIZE);
        param_set_int(&params, SNDRV_PCM_HW_PARAM_RATE, freq);
        param_set_int(&params, SNDRV_PCM_HW_PARAM_CHANNELS, chan);
        switch(fmt) {
            case SNDRV_PCM_FORMAT_S8: case SNDRV_PCM_FORMAT_U8: case SNDRV_PCM_FORMAT_DSD_U8:
            case SNDRV_PCM_FORMAT_MU_LAW: case SNDRV_PCM_FORMAT_A_LAW: ctx->sample_size = 1; break;
            case SNDRV_PCM_FORMAT_S16_LE: case SNDRV_PCM_FORMAT_S16_BE:
            case SNDRV_PCM_FORMAT_U16_LE: case SNDRV_PCM_FORMAT_U16_BE:
            case SNDRV_PCM_FORMAT_DSD_U16_LE: case SNDRV_PCM_FORMAT_DSD_U16_BE: ctx->sample_size = 2; break;
            case SNDRV_PCM_FORMAT_S18_3LE: case SNDRV_PCM_FORMAT_S18_3BE:
            case SNDRV_PCM_FORMAT_U18_3LE: case SNDRV_PCM_FORMAT_U18_3BE:
            case SNDRV_PCM_FORMAT_S20_3LE: case SNDRV_PCM_FORMAT_S20_3BE:
            case SNDRV_PCM_FORMAT_U20_3LE: case SNDRV_PCM_FORMAT_U20_3BE:
            case SNDRV_PCM_FORMAT_S24_3LE: case SNDRV_PCM_FORMAT_S24_3BE:
            case SNDRV_PCM_FORMAT_U24_3LE: case SNDRV_PCM_FORMAT_U24_3BE: ctx->sample_size = 3; break;
            case SNDRV_PCM_FORMAT_S20_LE: case SNDRV_PCM_FORMAT_S20_BE:
            case SNDRV_PCM_FORMAT_U20_LE: case SNDRV_PCM_FORMAT_U20_BE:
            case SNDRV_PCM_FORMAT_S24_LE: case SNDRV_PCM_FORMAT_S24_BE:
            case SNDRV_PCM_FORMAT_U24_LE: case SNDRV_PCM_FORMAT_U24_BE:
            case SNDRV_PCM_FORMAT_S32_LE: case SNDRV_PCM_FORMAT_S32_BE:
            case SNDRV_PCM_FORMAT_U32_LE: case SNDRV_PCM_FORMAT_U32_BE:
            case SNDRV_PCM_FORMAT_FLOAT_LE: case SNDRV_PCM_FORMAT_FLOAT_BE:
            case SNDRV_PCM_FORMAT_IEC958_SUBFRAME_LE: case SNDRV_PCM_FORMAT_IEC958_SUBFRAME_BE:
            case SNDRV_PCM_FORMAT_DSD_U32_LE: case SNDRV_PCM_FORMAT_DSD_U32_BE: ctx->sample_size = 4; break;
            case SNDRV_PCM_FORMAT_FLOAT64_LE: case SNDRV_PCM_FORMAT_FLOAT64_BE: ctx->sample_size = 8; break;
        }
        if(!ctx->sample_size) { close(ctx->fd); ctx->fd = -1; return ALSA_ERR_HWPAR; }
        /* let's see if the sound card supports the requested format */
        if(ioctl(ctx->fd, SNDRV_PCM_IOCTL_HW_PARAMS, &params)) {
            #ifndef ALSA_NORESAMPLE
            /* if not, fallback to int16_t stereo (most commonly supported by hardware) and convert samples from software */
            if(fmt != SNDRV_PCM_FORMAT_S16 || chan != 2) {
                fmt = SNDRV_PCM_FORMAT_S16; param_set_mask(&params, SNDRV_PCM_HW_PARAM_FORMAT, fmt);
                chan = 2; param_set_int(&params, SNDRV_PCM_HW_PARAM_CHANNELS, chan);
                if(!ioctl(ctx->fd, SNDRV_PCM_IOCTL_HW_PARAMS, &params)) { ctx->needresample = 1; goto ok; }
            }
            #endif
            close(ctx->fd); ctx->fd = -1; return ALSA_ERR_HWPAR;
        }
        #ifndef ALSA_NORESAMPLE
        ok:
        #endif
        ctx->period_size = param_get_int(&params, SNDRV_PCM_HW_PARAM_PERIOD_SIZE);
        ctx->period_count = param_get_int(&params, SNDRV_PCM_HW_PARAM_PERIODS);
        memset(&spar, 0, sizeof(spar));
        spar.tstamp_mode = SNDRV_PCM_TSTAMP_ENABLE;
        spar.period_step = 1;
        spar.avail_min = ctx->period_size;
        spar.start_threshold = ALSA_BUFFER_SIZE - ctx->period_size;
        spar.stop_threshold = ALSA_BUFFER_SIZE;
        spar.xfer_align = ctx->period_size / 2; /* for old kernels */
        if(ioctl(ctx->fd, SNDRV_PCM_IOCTL_SW_PARAMS, &spar)) { close(ctx->fd); ctx->fd = -1; return ALSA_ERR_SWPAR; }
        ctx->boundary = spar.boundary;
        ctx->mmap_status = mmap(NULL, 4096, PROT_READ, MAP_SHARED, ctx->fd, SNDRV_PCM_MMAP_OFFSET_STATUS);
        if(!ctx->mmap_status || ctx->mmap_status == MAP_FAILED) {
            ctx->mmap_status = NULL; close(ctx->fd); ctx->fd = -1; return ALSA_ERR_MMAP; }
            ctx->mmap_control = mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, ctx->fd, SNDRV_PCM_MMAP_OFFSET_CONTROL);
            if(!ctx->mmap_control || ctx->mmap_control == MAP_FAILED) {
                munmap(ctx->mmap_status, 4096); ctx->mmap_status = NULL; ctx->mmap_control = NULL; close(ctx->fd); ctx->fd = -1; return ALSA_ERR_MMAP; }
                if(ioctl(ctx->fd, SNDRV_PCM_IOCTL_PREPARE) < 0) {
                    munmap(ctx->mmap_status, 4096); munmap(ctx->mmap_control, 4096); ctx->mmap_status = NULL; ctx->mmap_control = NULL;
                    close(ctx->fd); ctx->fd = -1; return ALSA_ERR_PREP;
                }
                #ifdef _PTHREAD_H
                ctx->abuf = (void*)malloc(ctx->period_size * ctx->sample_size * ctx->chan);
                if(!ctx->abuf) {
                    munmap(ctx->mmap_status, 4096); munmap(ctx->mmap_control, 4096); ctx->mmap_status = NULL; ctx->mmap_control = NULL;
                    close(ctx->fd); ctx->fd = -1; return ALSA_ERR_THREAD;
                }
                #endif
                return ALSA_SUCCESS;
    }

    #ifndef ALSA_NORESAMPLE
    /**
     * Convert a sample in specified format to int16_t
     */
    static int alsa_conv(int fmt, unsigned char *src)
    {
        float f;
        int *i = (int*)&f;
        /* TODO: not all formats are handled yet */
        switch(fmt) {
            case SNDRV_PCM_FORMAT_S8: return (int)(char)src[0] * 256;
            case SNDRV_PCM_FORMAT_U8: return ((int)src[0] - 127) * 256;
            case SNDRV_PCM_FORMAT_S16_LE: return ((int)(char)src[1] << 8) | (int)src[0];
            case SNDRV_PCM_FORMAT_S16_BE: return ((int)(char)src[0] << 8) | (int)src[1];
            case SNDRV_PCM_FORMAT_U16_LE: return (((int)src[1] << 8) | (int)src[0]) - 32768;
            case SNDRV_PCM_FORMAT_U16_BE: return (((int)src[0] << 8) | (int)src[1]) - 32768;
            case SNDRV_PCM_FORMAT_S24_LE: return ((int)(char)src[2] << 8) | (int)src[1];
            case SNDRV_PCM_FORMAT_S24_BE: return ((int)(char)src[0] << 8) | (int)src[1];
            case SNDRV_PCM_FORMAT_U24_LE: return ((((int)src[2] << 16) | ((int)src[1] << 8) | (int)src[0]) - 8388608) / 256;
            case SNDRV_PCM_FORMAT_U24_BE: return ((((int)src[2] << 16) | ((int)src[1] << 8) | (int)src[0]) - 8388608) / 256;
            case SNDRV_PCM_FORMAT_S32_LE: return ((int)(char)src[3] << 8) | (int)src[2];
            case SNDRV_PCM_FORMAT_S32_BE: return ((int)(char)src[0] << 8) | (int)src[1];
            case SNDRV_PCM_FORMAT_U32_LE: return ((((int)src[3] << 24) | ((int)src[2] << 16) | ((int)src[1] << 8) | (int)src[0]) - 2147483648) / 32768;
            case SNDRV_PCM_FORMAT_U32_BE: return ((((int)src[0] << 24) | ((int)src[1] << 16) | ((int)src[2] << 8) | (int)src[3]) - 2147483648) / 32768;
            #if __BYTE_ORDER == __LITTLE_ENDIAN
            case SNDRV_PCM_FORMAT_FLOAT_LE: *i = (*(int*)src); return (int)((f < -1.0f ? -1.0f : (f > 1.0f ? 1.0f : f)) * 32767.0f);
            case SNDRV_PCM_FORMAT_FLOAT_BE: *i = (((int)src[0] << 24) | ((int)src[1] << 16) | ((int)src[2] << 8) | (int)src[3]);
            return (int)((f < -1.0f ? -1.0f : (f > 1.0f ? 1.0f : f)) * 32767.0f);
            #else
            case SNDRV_PCM_FORMAT_FLOAT_LE: *i = (((int)src[0] << 24) | ((int)src[1] << 16) | ((int)src[2] << 8) | (int)src[3]);
            return (int)((f < -1.0f ? -1.0f : (f > 1.0f ? 1.0f : f)) * 32767.0f);
            case SNDRV_PCM_FORMAT_FLOAT_BE: *i = (*(int*)src); return (int)((f < -1.0f ? -1.0f : (f > 1.0f ? 1.0f : f)) * 32767.0f);
            #endif
        }
        return 0;
    }
    #endif

    /**
     * Send PCM data directly to channel
     */
    int alsa_write(alsa_t *ctx, void *buf, unsigned int numframes)
    {
        unsigned char *data = (unsigned char*)buf;
        struct snd_xferi xfer = { 0 };
        int ret, avail, s = ctx->sample_size * ctx->chan;
        #ifndef ALSA_NORESAMPLE
        unsigned int i;
        int j, k;
        short *smp;
        #endif

        if(!ctx || !data || numframes < 1) return ALSA_ERR_INP;
        #ifndef ALSA_NORESAMPLE
        if(ctx->needresample) {
            if(!ctx->resample || ctx->reslen < numframes) {
                ctx->reslen = numframes;
                ctx->resample = (short*)realloc(ctx->resample, numframes * 4);
                if(!ctx->resample) { ctx->reslen = 0; return ALSA_ERR_INP; }
            }
            for(smp = ctx->resample, i = 0; i < numframes; i++, data += s, smp += 2)
                switch(ctx->chan) {
                    case 1: /* mono -> stereo */
                        smp[0] = smp[1] = alsa_conv(ctx->fmt, data);
                        break;
                    case 2: /* stereo -> stereo */
                        smp[0] = alsa_conv(ctx->fmt, data);
                        smp[1] = alsa_conv(ctx->fmt, data + ctx->sample_size);
                        break;
                    case 6: /* dolby 5.1 -> stereo */
                        smp[0] = (alsa_conv(ctx->fmt, data) + alsa_conv(ctx->fmt, data + 2 * ctx->sample_size) +
                        + alsa_conv(ctx->fmt, data + 3 * ctx->sample_size) + alsa_conv(ctx->fmt, data + 5 * ctx->sample_size)) / 4;
                        smp[1] = (alsa_conv(ctx->fmt, data + ctx->sample_size) + alsa_conv(ctx->fmt, data + 2 * ctx->sample_size) +
                        + alsa_conv(ctx->fmt, data + 4 * ctx->sample_size) + alsa_conv(ctx->fmt, data + 5 * ctx->sample_size)) / 4;
                        break;
                    default: /* anything else -> mono -> stereo */
                        for(k = j = 0; j < ctx->chan; j++)
                            k += alsa_conv(ctx->fmt, data + j * ctx->sample_size);
                    smp[0] = smp[1] = k / ctx->chan;
                    break;
                }
                s = 4; data = (unsigned char*)ctx->resample;
        }
        #endif
        do {
            xfer.buf = data;
            xfer.frames = numframes > ctx->period_size ? ctx->period_size : numframes;
            xfer.result = 0;
            if(!(ret = ioctl(ctx->fd, SNDRV_PCM_IOCTL_WRITEI_FRAMES, &xfer))) {
                avail = ctx->mmap_status->hw_ptr + ALSA_BUFFER_SIZE - ctx->mmap_control->appl_ptr;
                if(avail < 0) avail += ctx->boundary; else
                    if((unsigned int)avail >= ctx->boundary) avail -= ctx->boundary;
                    numframes -= xfer.result;
                data += xfer.result * s;
            } else return ret;
        } while(numframes > 0);
            return ALSA_SUCCESS;
    }

    /**
     * Close the ALSA context
     */
    void alsa_close(alsa_t *ctx)
    {
        if(!ctx) return;
        #ifdef _PTHREAD_H
        alsa_stop(ctx);
        if(ctx->abuf) { free(ctx->abuf); ctx->abuf = NULL; }
        #endif
        #ifndef ALSA_NORESAMPLE
        if(ctx->resample) { free(ctx->resample); ctx->resample = NULL; }
        #endif
        if(ctx->mmap_status) munmap(ctx->mmap_status, 4096);
        if(ctx->mmap_control) munmap(ctx->mmap_control, 4096);
        if(ctx->fd > 0) {
            ioctl(ctx->fd, SNDRV_PCM_IOCTL_DRAIN);
            close(ctx->fd);
        }
        memset(ctx, 0, sizeof(alsa_t));
    }

    #ifdef _PTHREAD_H
    /**
     * Worker thread
     */
    static void *alsa_worker(void *data)
    {
        alsa_t *ctx = (alsa_t*)data;
        while(ctx->period_size) {
            (*ctx->cb)(ctx->abuf, ctx->sample_size, ctx->chan, ctx->period_size, ctx->data);
            alsa_write(ctx, ctx->abuf, ctx->period_size);
        }
        return NULL;
    }

    /**
     * Start channel
     */
    int alsa_start(alsa_t *ctx, alsa_callback_t callback, void *data)
    {
        if(!ctx || ctx->fd < 1 || !ctx->abuf || ctx->th || !callback) return ALSA_ERR_INP;
        ctx->cb = callback; ctx->data = data;
        return pthread_create(&ctx->th, NULL, alsa_worker, ctx) ? ALSA_ERR_THREAD : ALSA_SUCCESS;
    }

    /**
     * Stop channel
     */
    int alsa_stop(alsa_t *ctx)
    {
        if(!ctx) return ALSA_ERR_INP;
        ctx->period_size = 0;
        if(ctx->th) {
            pthread_cancel(ctx->th);
            pthread_join(ctx->th, NULL);
            ctx->th = 0;
        }
        if(ctx->fd > 0) ioctl(ctx->fd, SNDRV_PCM_IOCTL_DRAIN);
        return ALSA_SUCCESS;
    }
    #endif

    #endif /* ALSA_IMPLEMENTATION */

    #ifdef  __cplusplus
}
#endif

#endif /* _NANOALSA_H */
