/*
 * Copyright (C) 2016, Broadcom Corporation. All Rights Reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/io.h>
#include <linux/timer.h>
#include "pegasus-ssp.h"
#include "pegasus-pcm.h"

/* PERIOD_BYTES_MIN is the number of bytes to at which the interrupt will tick.
 * This number should be a multiple of 256
 */
#define PERIOD_BYTES_MIN 0x100

static int irq_num;
static struct pegasus_substream_data *gbrtd;

struct pegasus_substream_data {
	struct snd_pcm_substream *play_stream[MAX_PLAYBACK_PORTS];
	struct snd_pcm_substream *capture_stream[MAX_CAPTURE_PORTS];
	void __iomem *audio;
};

static const struct snd_pcm_hardware pegasus_pcm_hw = {
	.info = SNDRV_PCM_INFO_MMAP |
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_INTERLEAVED,
	.formats = SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S24_LE |
			SNDRV_PCM_FMTBIT_S32_LE,

	/* A period is basically an interrupt */
	.period_bytes_min = PERIOD_BYTES_MIN,
	.period_bytes_max = 0x10000,

	/* period_min/max gives range of approx interrupts per buffer */
	.periods_min = 2,
	.periods_max = 8,

	/* maximum buffer size in bytes = period_bytes_max * periods_max
	 * We allocate this amount of data for each enabled channel
	 */
	.buffer_bytes_max = 4 * 0x8000,
};

static u64 pegasus_dma_dmamask = DMA_BIT_MASK(32);

static int init_flag;

#ifdef CONFIG_SND_DEBUG
static struct timespec prev_time;
static struct timespec cur_time;
static struct timespec delta_time;
static unsigned g_intrpt_count;
static unsigned dtime_us;

unsigned diff_time(void)
{
	getnstimeofday(&cur_time);
	delta_time = timespec_sub(cur_time, prev_time);

	prev_time = cur_time;

	return delta_time.tv_nsec/1000;
}
#endif

static void ringbuf_set_initial(struct pegasus_aio *aio,
		struct ringbuf_regs *p_rbuf,
		int is_playback,
		u32 start,
		u32 periodsize,
		u32 bufsize)
{
	u32 initial_rd;
	u32 initial_wr;
	u32 end;
	u32 fmark_val; /* free or full mark */

	p_rbuf->period_bytes = periodsize;
	p_rbuf->buf_size = bufsize;

	if (is_playback) {
		/* Set the read pointer one period behind the write.
		 * This should cause an immediate freemark interrupt
		 */
		initial_rd = start;
		initial_wr = start + periodsize;
	} else {
		/* Set the write pointer one period behind the read.
		 * This should cause an immediate fullmark interrupt
		 */
		initial_rd = start + periodsize;
		initial_wr = start;
	}

	end = start + bufsize - 1;
	fmark_val = (bufsize - periodsize);

	writel(start,      aio->audio + p_rbuf->baseaddr);
	writel(end,        aio->audio + p_rbuf->endaddr);
	writel(fmark_val,  aio->audio + p_rbuf->fmark);
	writel(initial_rd, aio->audio + p_rbuf->rdaddr);
	writel(initial_wr, aio->audio + p_rbuf->wraddr);
}

static int configure_ringbuf_regs(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct pegasus_aio *aio;
	struct ringbuf_regs *p_rbuf;
	int status = 0;

	aio = snd_soc_dai_get_dma_data(soc_runtime->cpu_dai, substream);

	/* Map the ssp portnum to a set of ring buffers. */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		p_rbuf = &aio->play_rb_regs;

		if (aio->portnum == 0)
			*p_rbuf = RINGBUF_REG_PLAYBACK(0);
		else if (aio->portnum == 1)
			*p_rbuf = RINGBUF_REG_PLAYBACK(2);
		else if (aio->portnum == 2)
			*p_rbuf = RINGBUF_REG_PLAYBACK(4);
		else if (aio->portnum == 3)
			*p_rbuf = RINGBUF_REG_PLAYBACK(6); /*SPDIF */
		else
			status = -1;
	} else {
		p_rbuf = &aio->capture_rb_regs;

		if (aio->portnum == 0)
			*p_rbuf = RINGBUF_REG_CAPTURE(0);
		else if (aio->portnum == 1)
			*p_rbuf = RINGBUF_REG_CAPTURE(2);
		else if (aio->portnum == 2)
			*p_rbuf = RINGBUF_REG_CAPTURE(4);
		else
			status = -1;
	}

	return status;
}

static void ringbuf_inc(int is_playback, struct ringbuf_regs *p_rbuf)
{
	u32 regval, endval, active_ptr;

	if (is_playback)
		active_ptr = p_rbuf->wraddr;
	else
		active_ptr = p_rbuf->rdaddr;

	endval = readl(gbrtd->audio + p_rbuf->endaddr);
	regval = readl(gbrtd->audio + active_ptr);
	regval = regval + p_rbuf->period_bytes;
	if (regval > endval)
		regval -= p_rbuf->buf_size;

	writel(regval, gbrtd->audio + active_ptr);
}


static struct ringbuf_regs *get_ringbuf(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct pegasus_aio *aio;
	struct ringbuf_regs *p_rbuf = NULL;

	aio = snd_soc_dai_get_dma_data(soc_runtime->cpu_dai, substream);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		p_rbuf = &aio->play_rb_regs;
	else
		p_rbuf = &aio->capture_rb_regs;

	return p_rbuf;
}

static void enable_intr(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct pegasus_aio *aio;
	u32 mask;

	aio = snd_soc_dai_get_dma_data(soc_runtime->cpu_dai, substream);

	pr_debug("==> %s on port = %d\n", __func__, aio->portnum);

	mask = 1 << aio->portnum;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* Clear interrupt status before enabling them */
		writel(mask, aio->audio + ESR0_STATUS_CLR_OFFSET);
		writel(mask, aio->audio + ESR1_STATUS_CLR_OFFSET);
		writel(mask, aio->audio + ESR3_STATUS_CLR_OFFSET);
		/* Unmask the interrupts of the given port*/
		writel(mask, aio->audio + ESR0_MASK_CLR_OFFSET);
		writel(mask, aio->audio + ESR1_MASK_CLR_OFFSET);
		writel(mask, aio->audio + ESR3_MASK_CLR_OFFSET);
	} else {
		writel(mask, aio->audio + ESR2_STATUS_CLR_OFFSET);
		writel(mask, aio->audio + ESR4_STATUS_CLR_OFFSET);
		writel(mask, aio->audio + ESR2_MASK_CLR_OFFSET);
		writel(mask, aio->audio + ESR4_MASK_CLR_OFFSET);
	}

#ifdef CONFIG_SND_DEBUG
	dtime_us = diff_time();
#endif

	if (!init_flag) {
		/* One time clear all the ESR registers */
		writel(0x1F, aio->audio + INTH_R5F_CLEAR_OFFSET);
		writel(0x1F, aio->audio + INTH_R5F_MASK_CLEAR_OFFSET);
		pr_debug("==> %s on port = %d once\n", __func__, aio->portnum);
	}
	init_flag++;
}

static void disable_intr(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct pegasus_aio *aio;
	u32 mask;

	aio = snd_soc_dai_get_dma_data(soc_runtime->cpu_dai, substream);

	pr_debug("==> %s on port = %d\n", __func__, aio->portnum);

	mask = 1 << aio->portnum;
	init_flag--;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* Mask the interrupts of the given port*/
		writel(mask, aio->audio + ESR0_MASK_SET_OFFSET);
		writel(mask, aio->audio + ESR1_MASK_SET_OFFSET);
		writel(mask, aio->audio + ESR3_MASK_SET_OFFSET);
	} else {
		writel(mask, aio->audio + ESR2_MASK_SET_OFFSET);
		writel(mask, aio->audio + ESR4_MASK_SET_OFFSET);
	}
	if (!init_flag) {
		/* Disable all the ESR registers after all streams are closed*/
		writel(0x1F, aio->audio + INTH_R5F_MASK_SET_OFFSET);
		pr_debug("==> %s on port = %d once\n", __func__, aio->portnum);
	}
}

static int pegasus_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		enable_intr(substream);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		disable_intr(substream);
		break;
	}

	return ret;
}

static void pegasus_pcm_period_elapsed(struct snd_pcm_substream *substream)
{
	struct ringbuf_regs *p_rbuf = NULL;
	int is_play;

	p_rbuf = get_ringbuf(substream);

	is_play = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? 1 : 0;

	/* If free/full mark interrupt occurs, provide timestamp
	 * to ALSA and update appropriate idx by period_bytes
	 */
	snd_pcm_period_elapsed(substream);

	ringbuf_inc(is_play, p_rbuf);
}

/* ESR0/1/3 status  Description
*  0x1	I2S0_out port caused interrupt
*  0x2	I2S1_out port caused interrupt
*  0x4	I2S2_out port caused interrupt
*  0x8	SPDIF_out port caused interrupt
*/
static void handle_playback_irq(void)
{
	u32 port;
	u32 esr_status0, esr_status1, esr_status3;

	/* ESR status gets updates with/without interrupts enabled.
	 * So, check the ESR mask, which provides interrupt enable/
	 * disable status and use it to determine which ESR status
	 * should be serviced.
	 */
	esr_status0 = readl(gbrtd->audio + ESR0_STATUS_OFFSET);
	esr_status0 &= ~readl(gbrtd->audio + ESR0_MASK_STATUS_OFFSET);
	esr_status1 = readl(gbrtd->audio + ESR1_STATUS_OFFSET);
	esr_status1 &= ~readl(gbrtd->audio + ESR1_MASK_STATUS_OFFSET);
	esr_status3 = readl(gbrtd->audio + ESR3_STATUS_OFFSET);
	esr_status3 &= ~readl(gbrtd->audio + ESR3_MASK_STATUS_OFFSET);

	for (port = 0; port < MAX_PLAYBACK_PORTS; port++) {
		u32 esrmask = (1 << port);

		/* Ringbuffer or FIFO underflow
		 * If we get this interrupt then, it is also true that we have
		 * not yet responded to the freemark interrupt.
		 * Log a debug message.  The freemark handler below will
		 * handle getting everything going again.
		 */
		if ((esrmask & esr_status1) || (esrmask & esr_status0)) {
#ifdef CONFIG_SND_DEBUG
			pr_debug("Underrun %u (%u us) since prev intrpt\n",
				g_intrpt_count, dtime_us);
#endif
			pr_debug("Underrun: esr0=0x%x, esr1=0x%x esr3=0x%x\n",
				esr_status0, esr_status1, esr_status3);
		}

		/* Freemark is hit.  This is the normal interrupt.
		 * In typical operation the read and write regs will be equal
		 */
		if (esrmask & esr_status3) {
#ifdef CONFIG_SND_DEBUG
			pr_debug("Freemark %u (%u us) since prev intrpt\n",
				g_intrpt_count, dtime_us);
#endif
			pegasus_pcm_period_elapsed(gbrtd->play_stream[port]);
		}
	}

	/* Clear ESR interrupt */
	writel(esr_status0, gbrtd->audio + ESR0_STATUS_CLR_OFFSET);
	writel(esr_status1, gbrtd->audio + ESR1_STATUS_CLR_OFFSET);
	writel(esr_status3, gbrtd->audio + ESR3_STATUS_CLR_OFFSET);
	/* Rearm freemark logic by writing 1 to the correct bit */
	writel(esr_status3, gbrtd->audio + BF_REARM_FREE_MARK_OFFSET);
}

/* ESR2/4 status  Description
*  0x1	I2S0_in port caused interrupt
*  0x2	I2S1_in port caused interrupt
*  0x4	I2S2_in port caused interrupt
*/
static void handle_capture_irq(void)
{
	u32 port;
	u32 esr_status2, esr_status4;

	/*
	* ESR status gets updates with/without interrupts enabled.
	* So, check the ESR mask, which provides interrupt enable/
	* disable status and use it to determine which ESR status
	* should be serviced.
	*/
	esr_status2 = readl(gbrtd->audio + ESR2_STATUS_OFFSET);
	esr_status2 &= ~readl(gbrtd->audio + ESR2_MASK_STATUS_OFFSET);
	esr_status4 = readl(gbrtd->audio + ESR4_STATUS_OFFSET);
	esr_status4 &= ~readl(gbrtd->audio + ESR4_MASK_STATUS_OFFSET);

	for (port = 0; port < MAX_CAPTURE_PORTS; port++) {
		u32 esrmask = (1 << port);

		/* Ringbuffer or FIFO overflow
		 * If we get this interrupt then, it is also true that we have
		 * not yet responded to the fullmark interrupt.
		 * Log a debug message.  The fullmark handler below will
		 * handle getting everything going again.
		 */
		if (esrmask & esr_status2)
			pr_debug("Overflow: esr2=0x%x\n", esr_status2);

		if (esrmask & esr_status4)
			pegasus_pcm_period_elapsed(gbrtd->capture_stream[port]);
	}

	writel(esr_status2, gbrtd->audio + ESR2_STATUS_CLR_OFFSET);
	writel(esr_status4, gbrtd->audio + ESR4_STATUS_CLR_OFFSET);
	/* Rearm fullmark logic by writing 1 to the correct bit */
	writel(esr_status4, gbrtd->audio + BF_REARM_FULL_MARK_OFFSET);
}

static irqreturn_t pegasus_dma_irq(int irq, void *data)
{
	u32 r5_status;

#ifdef CONFIG_SND_DEBUG
	g_intrpt_count++;
	dtime_us = diff_time();
#endif
	if (!gbrtd) {
		pr_err("==> ERROR: brtd is NULL\n");
		return IRQ_HANDLED;
	}

	/* R5 status bits	Description
	*  0		ESR0 (playback FIFO interrupt)
	*  1		ESR1 (playback rbuf interrupt)
	*  2		ESR2 (capture rbuf interrupt)
	*  3		ESR3 (Freemark play. interrupt)
	*  4		ESR4 (Fullmark capt. interrupt)
	*/
	r5_status = readl(gbrtd->audio + INTH_R5F_STATUS_OFFSET);

	/* If playback interrupt happened */
	if (0xB & r5_status)
		handle_playback_irq();

	/* If  capture interrupt happened */
	if (0x14 & r5_status)
		handle_capture_irq();

	/* clear r5 interrupts after servicing them to avoid overwriting
	* esr_status
	*/
	writel(r5_status, gbrtd->audio + INTH_R5F_CLEAR_OFFSET);
	return IRQ_HANDLED;
}

static int pegasus_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pegasus_aio *aio;
	int ret;

	aio = snd_soc_dai_get_dma_data(soc_runtime->cpu_dai, substream);
	if (!aio)
		return -ENODEV;

	pr_debug("==>Enter %s of port = %d\n", __func__, aio->portnum);

	snd_soc_set_runtime_hwparams(substream, &pegasus_pcm_hw);

	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_PERIOD_BYTES, PERIOD_BYTES_MIN);
	if (ret < 0)
		return ret;

	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_BUFFER_BYTES, PERIOD_BYTES_MIN);
	if (ret < 0)
		return ret;

	/* Keep track of which substream belongs to which port.
	 * This info is needed by snd_pcm_period_elapsed() in irq_handler
	 */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		gbrtd->play_stream[aio->portnum] = substream;
	else
		gbrtd->capture_stream[aio->portnum] = substream;

	substream->runtime->private_data = gbrtd;

	return 0;
}

static int pegasus_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct snd_soc_dai *cpu_dai = soc_runtime->cpu_dai;
	struct pegasus_aio *aio;

	aio = snd_soc_dai_get_dma_data(cpu_dai, substream);
	pr_debug("==>Enter %s of port = %d\n", __func__, aio->portnum);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		gbrtd->play_stream[aio->portnum] = NULL;
	else
		gbrtd->capture_stream[aio->portnum] = NULL;

	if (!gbrtd->play_stream[aio->portnum]
		&& !gbrtd->capture_stream[aio->portnum])
		pr_debug("freed port = %d\n", aio->portnum);

	return 0;
}

static int pegasus_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct snd_soc_dai *cpu_dai = soc_runtime->cpu_dai;
	struct pegasus_aio *aio;
	int ret = 0;

	aio = snd_soc_dai_get_dma_data(cpu_dai, substream);
	pr_debug("==>Enter %s of port = %d\n", __func__, aio->portnum);

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	return ret;
}

static int pegasus_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct snd_soc_dai *cpu_dai = soc_runtime->cpu_dai;
	struct pegasus_aio *aio;

	aio = snd_soc_dai_get_dma_data(cpu_dai, substream);
	pr_debug("==>Enter %s of port = %d\n", __func__, aio->portnum);
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

/*
 *		Ringbuffer startup logic
 *
 *   Capture                             Playback
 * +---------+ <= rdaddr/baseaddr      +---------+ <= wraddr/baseaddr
 * |         |                         |         |
 * |         |                         |         |
 * |         |                         |         |
 * |         |                         |         |
 * |         |                         |         |
 * |         |                         |         |
 * |         |                         |         |
 * |         |                         |         |
 * |         | <= wraddr/fullmark      |         | <= rdaddr/freemark
 * |         |   (size-PERIOD_BYTES)   |         |   (size-PERIOD_BYTES)
 * |         |                         |         |
 * +---------+ <= endaddr              +---------+ <= endaddr
 *
 * size = endaddr - baseaddr
 *
 */

static int pegasus_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct snd_soc_dai *cpu_dai = soc_runtime->cpu_dai;
	struct pegasus_aio *aio;
	unsigned long bufsize, periodsize;
	int ret = 0;
	int is_play;
	u32 start;
	struct ringbuf_regs *p_rbuf = NULL;

	aio = snd_soc_dai_get_dma_data(cpu_dai, substream);
	pr_debug("%s port = %d\n", __func__, aio->portnum);

	if (aio->portnum > SPDIF) {
		pr_err("%s: Port number is not supported\n", __func__);
		return -EINVAL;
	}

	bufsize = snd_pcm_lib_buffer_bytes(substream);
	periodsize = snd_pcm_lib_period_bytes(substream);
	pr_debug("%s (buf_size %lu) (period_size %lu)\n", __func__,
		bufsize, periodsize);

	configure_ringbuf_regs(substream);

	p_rbuf = get_ringbuf(substream);

	start = runtime->dma_addr;

	is_play = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? 1 : 0;

	ringbuf_set_initial(aio, p_rbuf, is_play, start, periodsize, bufsize);

	return ret;
}

static snd_pcm_uframes_t pegasus_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct snd_soc_dai *cpu_dai = soc_runtime->cpu_dai;
	struct pegasus_aio *aio;
	int res = 0;
	unsigned int cur = 0, base = 0;
	struct ringbuf_regs *p_rbuf = NULL;

	aio = snd_soc_dai_get_dma_data(cpu_dai, substream);

	if (aio->portnum > SPDIF) {
		pr_err("Port number out of range %u\n", aio->portnum);
		return 0;
	}

	/* Get the offset of the current read (for playack) or write
	 * index (for capture).  Report this value back to the asoc framework.
	 */
	p_rbuf = get_ringbuf(substream);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		cur = readl(aio->audio + p_rbuf->rdaddr);
	else
		cur = readl(aio->audio + p_rbuf->wraddr);

	base = readl(aio->audio + p_rbuf->baseaddr);

	/* Mask off the MSB of the rdaddr,wraddr and baseaddr
	 * since MSB is not part of the address
	 */
	res = (cur & 0x7fffffff) - (base & 0x7fffffff);

	return bytes_to_frames(substream->runtime, res);
}

static int pegasus_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = pegasus_pcm_hw.buffer_bytes_max;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_coherent(pcm->card->dev, size,
			&buf->addr, GFP_KERNEL);

	pr_debug("==> %s: size 0x%x @ 0x%p\n",
			 __func__, (unsigned int)size, buf->area);

	if (!buf->area) {
		pr_err("==> %s: dma_alloc failed\n", __func__);
		return -ENOMEM;
	}
	buf->bytes = size;

	return 0;
}

/* This code is identical to what is done by the framework, when we do not
 * supply a 'copy' function.  Having our own copy hook in place allows for
 * us to easily add some diagnotics when needed.
 */
int pegasus_pcm_copy(struct snd_pcm_substream *substream, int channel,
		snd_pcm_uframes_t pos,
		void __user *buf, snd_pcm_uframes_t count)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	char *hwbuf = runtime->dma_area + frames_to_bytes(runtime, pos);
	int size = frames_to_bytes(runtime, count);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (copy_from_user(hwbuf, buf, size))
			return -EFAULT;
	} else {
		if (copy_to_user(buf, hwbuf, size))
			return -EFAULT;
	}
	return 0;
}

struct snd_pcm_ops pegasus_pcm_ops = {
	.open		= pegasus_pcm_open,
	.close		= pegasus_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= pegasus_pcm_hw_params,
	.hw_free	= pegasus_pcm_hw_free,
	.prepare	= pegasus_pcm_prepare,
	.trigger	= pegasus_pcm_trigger,
	.pointer	= pegasus_pcm_pointer,
	.copy		= pegasus_pcm_copy,
};


static int pegasus_dma_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	int ret;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &pegasus_dma_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		ret = pegasus_pcm_preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			return ret;
	}

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		ret = pegasus_pcm_preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			return ret;
	}

	return 0;
}

static void pegasus_dma_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;
		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_coherent(pcm->card->dev, buf->bytes,
				buf->area, buf->addr);
		buf->area = NULL;
	}
}

struct snd_soc_platform_driver pegasus_soc_platform = {
	.ops		= &pegasus_pcm_ops,
	.pcm_new	= pegasus_dma_new,
	.pcm_free	= pegasus_dma_free_dma_buffers,
};

static int pegasus_soc_platform_probe(struct platform_device *pdev)
{
	struct resource *res = pdev->resource;
	struct pegasus_substream_data *brtd;
	int err = 0;

	pr_debug("%s Enter\n", __func__);

	brtd = kzalloc(sizeof(struct pegasus_substream_data), GFP_KERNEL);
	if (!brtd) {
		err = -ENOMEM;
		return err;
	}
	dev_set_drvdata(&pdev->dev, brtd);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("pegasus_soc_platform_probe: get resource failed\n");
		err = -ENXIO;
		goto err_register;
	}

	brtd->audio = ioremap_nocache(res->start, resource_size(res));
	if (!brtd->audio) {
		pr_err("ioremap failed\n");
		err = -ENOMEM;
		goto err_register;
	}

	irq_num = platform_get_irq(pdev, 0);
	if (irq_num <= 0) {
		pr_err("platform_get_irq failed\n");
		err = -ENXIO;
		goto err_map_audio;
	}

	err = request_irq(irq_num, pegasus_dma_irq, IRQF_SHARED,
			  "pegasus-audio", brtd);
	if (err) {
		pr_err("request_irq error %d\n", err);
		goto err_map_audio;
	}

	err = snd_soc_register_platform(&pdev->dev, &pegasus_soc_platform);
	if (err) {
		pr_err("snd_soc_register_platform failed\n");
		goto err_free_irq;
	}

	pr_info("%s: snd_soc_register_platform.. Done\n", __func__);
	gbrtd = brtd;

	return err;

err_free_irq:
	free_irq(irq_num, brtd->audio);

err_map_audio:
	iounmap(brtd->audio);

err_register:
	kfree(brtd);
	return err;
}

static int pegasus_soc_platform_remove(struct platform_device *pdev)
{
	struct pegasus_substream_data *brtd = dev_get_drvdata(&pdev->dev);

	free_irq(irq_num, brtd->audio);
	iounmap(brtd->audio);
	kfree(brtd);

	snd_soc_unregister_platform(&pdev->dev);

	return 0;
}

static const struct of_device_id aio_of_match[] = {
	{ .compatible = "bcm,pegasus-pcm", },
	{ }
};

static struct platform_driver pegasus_pcm_driver = {
	.driver = {
			.name = "pegasus-pcm",
			.owner = THIS_MODULE,
			.of_match_table = aio_of_match,
	},

	.probe = pegasus_soc_platform_probe,
	.remove = pegasus_soc_platform_remove,
};

module_platform_driver(pegasus_pcm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Pegasus PCM module");
