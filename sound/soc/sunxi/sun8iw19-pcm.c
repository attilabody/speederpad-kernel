/*
 * sound\soc\sunxi\sun8iw19-pcm.c
 * (C) Copyright 2014-2019
 * AllWinner Technology Co., Ltd. <www.allwinnertech.com>
 * wolfgang huang <huangjinhui@allwinnertech.com>
 * yumingfeng <yumingfeng@allwinnertech.com>
 *
 * some simple description for this code
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma/sunxi-dma.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <asm/dma.h>
#include "sun8iw19-pcm.h"

#undef SUN8IW19_PCM_DEBUG

static u64 sunxi_pcm_mask = DMA_BIT_MASK(32);

#ifdef CONFIG_SUNXI_ADC_DAUDIO_SYNC
#include "sun8iw19-codec.h"

void *sunxi_daudio0;
struct regmap *sunxi_codec_regmap;

struct sunxi_pcm_trigger_spinlock adc_sync_spinlock;

int adc_sync_flag;
module_param(adc_sync_flag, int, 0444);
MODULE_PARM_DESC(adc_sync_flag, "SUNXI codec for dma engine trigger sync.");

static int substream_mode;
module_param(substream_mode, int, 0444);
MODULE_PARM_DESC(substream_mode, "SUNXI codec for dma engine trigger sync.");

/* only for debug */
static long playback_tv;
module_param(playback_tv, long, 0444);
MODULE_PARM_DESC(playback_tv, "SUNXI codec for dma engine trigger time(us).");

static long capture_tv;
module_param(capture_tv, long, 0444);
MODULE_PARM_DESC(capture_tv, "SUNXI codec for dma engine trigger time(us).");

static struct timeval play_tv;
static struct timeval cap_tv;

/* used for mixer control */
long long sunxi_codec_get_pcm_trigger_playback_tv(void)
{
	long long time_tv = play_tv.tv_sec * 1000000 + play_tv.tv_usec;
	playback_tv = time_tv;
	return time_tv;
}
EXPORT_SYMBOL_GPL(sunxi_codec_get_pcm_trigger_playback_tv);

long long sunxi_codec_get_pcm_trigger_capture_tv(void)
{
	long long time_tv = cap_tv.tv_sec * 1000000 + cap_tv.tv_usec;
	capture_tv = time_tv;
	return time_tv;
}
EXPORT_SYMBOL_GPL(sunxi_codec_get_pcm_trigger_capture_tv);

void sunxi_codec_set_pcm_trigger_playback_tv(void)
{
	do_gettimeofday(&play_tv);
	playback_tv = play_tv.tv_sec * 1000000 + play_tv.tv_usec;
}
EXPORT_SYMBOL_GPL(sunxi_codec_set_pcm_trigger_playback_tv);

void sunxi_codec_set_pcm_trigger_capture_tv(void)
{
	do_gettimeofday(&cap_tv);
	capture_tv = cap_tv.tv_sec * 1000000 + cap_tv.tv_usec;
}
EXPORT_SYMBOL_GPL(sunxi_codec_set_pcm_trigger_capture_tv);

int sunxi_codec_get_pcm_trigger_substream_mode(void)
{
	return substream_mode;
}
EXPORT_SYMBOL_GPL(sunxi_codec_get_pcm_trigger_substream_mode);

void sunxi_codec_set_pcm_trigger_substream_mode(int value)
{
	if (!((adc_sync_flag >> ADC_I2S_RUNNING) & 0x1)) {
		substream_mode = value;
	} else {
		pr_err("set the adc sync mode should be stop the record.\n");
	}
}
EXPORT_SYMBOL_GPL(sunxi_codec_set_pcm_trigger_substream_mode);

void sunxi_codec_set_pcm_adc_sync_flag(int value)
{
       adc_sync_flag = value;
}
EXPORT_SYMBOL_GPL(sunxi_codec_set_pcm_adc_sync_flag);

int sunxi_codec_get_pcm_adc_sync_flag(void)
{
	return adc_sync_flag;
}
EXPORT_SYMBOL_GPL(sunxi_codec_get_pcm_adc_sync_flag);

void sunxi_codec_set_pcm_adc_daudio0(void *sunxi_daudio)
{
	sunxi_daudio0 = (struct sunxi_daudio_info *)sunxi_daudio;
}
EXPORT_SYMBOL_GPL(sunxi_codec_set_pcm_adc_daudio0);

void *sunxi_codec_get_pcm_adc_daudio0(void)
{
	return (void *)sunxi_daudio0;
}
EXPORT_SYMBOL_GPL(sunxi_codec_get_pcm_adc_daudio0);

void sunxi_codec_set_regmap(struct regmap *codec_regmap)
{
	sunxi_codec_regmap = codec_regmap;
}
EXPORT_SYMBOL_GPL(sunxi_codec_set_regmap);

/* for adc and i2s rx sync */
void sunxi_cpudai_adc_drq_enable(bool enable)
{
	if (!sunxi_codec_regmap) {
		pr_err("sunxi_codec_regmap is null.\n");
		return;
	}

	if (enable)
		regmap_update_bits(sunxi_codec_regmap, SUNXI_ADC_FIFOC,
				(1 << ADC_DRQ_EN), (1 << ADC_DRQ_EN));
	else
		regmap_update_bits(sunxi_codec_regmap, SUNXI_ADC_FIFOC,
				(1 << ADC_DRQ_EN), (0 << ADC_DRQ_EN));
}
EXPORT_SYMBOL_GPL(sunxi_cpudai_adc_drq_enable);

/* for sync */
static void audio_trigger_lock_spinlock_init(void *__lock)
{
	struct sunxi_pcm_trigger_spinlock *lock = __lock;
	spin_lock_init(&lock->spinlock);
}

void audio_trigger_lock_spinlock(void)
{
	struct sunxi_pcm_trigger_spinlock *lock = &adc_sync_spinlock;
	unsigned long flags;

	spin_lock_irqsave(&lock->spinlock, flags);
	lock->spinlock_flags = flags;
}
EXPORT_SYMBOL_GPL(audio_trigger_lock_spinlock);

void audio_trigger_unlock_spinlock(void)
{
	struct sunxi_pcm_trigger_spinlock *lock = &adc_sync_spinlock;
	spin_unlock_irqrestore(&lock->spinlock, lock->spinlock_flags);
}
EXPORT_SYMBOL_GPL(audio_trigger_unlock_spinlock);

#endif

static int sunxi_set_runtime_hwparams(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_platform *platform = rtd->platform;
	struct device_node *np = platform->component.dev->of_node;
	unsigned int temp_val;
	size_t temp_long;
	int ret = 0;

	snd_soc_platform_get_drvdata(platform);

	runtime->hw.info		= SNDRV_PCM_INFO_INTERLEAVED
					| SNDRV_PCM_INFO_BLOCK_TRANSFER
					| SNDRV_PCM_INFO_MMAP
					| SNDRV_PCM_INFO_MMAP_VALID
					| SNDRV_PCM_INFO_PAUSE
					| SNDRV_PCM_INFO_RESUME;

	runtime->hw.formats		= SNDRV_PCM_FMTBIT_S8
					| SNDRV_PCM_FMTBIT_S16_LE
					| SNDRV_PCM_FMTBIT_S20_3LE
					| SNDRV_PCM_FMTBIT_S24_LE
					| SNDRV_PCM_FMTBIT_S32_LE;
	runtime->hw.rates		= SNDRV_PCM_RATE_8000_192000
					| SNDRV_PCM_RATE_KNOT;
	runtime->hw.rate_min		= 8000;
	runtime->hw.channels_min	= 1;
	runtime->hw.period_bytes_min	= 256;
	runtime->hw.periods_min		= 1;

	ret = of_property_read_u32(np, "rate_max", &temp_val);
	if (ret < 0) {
		runtime->hw.rate_max = 192000;
		pr_err("audio_driver: [%s] hwparams get failed!\n", __func__);
	} else {
		runtime->hw.rate_max = temp_val;
	}

	ret = of_property_read_u32(np, "channels_max", &temp_val);
	if (ret < 0) {
		runtime->hw.channels_max = 8;
		pr_err("audio_driver: [%s] hwparams get failed!\n", __func__);
	} else {
		runtime->hw.channels_max = temp_val;
	}

	ret = of_property_read_u32(np, "buffer_bytes_max", &temp_long);
	if (ret < 0) {
		runtime->hw.buffer_bytes_max = 1024*256;
		pr_err("audio_driver: [%s] hwparams get failed!\n", __func__);
	} else {
		runtime->hw.buffer_bytes_max = temp_long;
	}

	ret = of_property_read_u32(np, "period_bytes_max", &temp_long);
	if (ret < 0) {
		runtime->hw.period_bytes_max = 1024*128;
		pr_err("audio_driver: [%s] hwparams get failed!\n", __func__);
	} else {
		runtime->hw.period_bytes_max = temp_long;
	}

	ret = of_property_read_u32(np, "periods_max", &temp_val);
	if (ret < 0) {
		runtime->hw.periods_max = 8;
		pr_err("audio_driver: [%s] hwparams get failed!\n", __func__);
	} else {
		runtime->hw.periods_max = temp_val;
	}

	ret = of_property_read_u32(np, "fifo_size", &temp_long);
	if (ret < 0) {
		runtime->hw.fifo_size = 128;
		pr_err("audio_driver: [%s] hwparams get failed!\n", __func__);
	} else {
		runtime->hw.fifo_size = temp_long;
	}
#ifdef SUN8IW19_PCM_DEBUG
	pr_warn("audio_driver: rate_min: %d, rate_max: %d, channels_min: %d, "
		"channels_max: %d\n",
		runtime->hw.rate_min, runtime->hw.rate_max,
		runtime->hw.channels_min, runtime->hw.channels_max);

	pr_warn("audio_driver: buffer_bytes_max: 0x%x, period_bytes_min: 0x%x"
		" period_bytes_max: 0x%x, periods_min: %d, periods_max: %d"
		" fifo_size: 0x%x\n\n",
		runtime->hw.buffer_bytes_max, runtime->hw.period_bytes_min,
		runtime->hw.period_bytes_max, runtime->hw.periods_min,
		runtime->hw.periods_max, runtime->hw.fifo_size);
#endif
	return 0;
};

static int sunxi_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct sunxi_dma_params *dmap;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->platform->dev;
	struct dma_chan *chan;
	struct dma_slave_config slave_config;
	int ret;

	if (strcmp(rtd->card->name, "sun8iw19-codec") == 0) {
		dmap = snd_soc_dai_get_dma_data(rtd->codec_dai, substream);
	} else {
		dmap = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
	}

	ret = snd_hwparams_to_dma_slave_config(substream, params,
					&slave_config);
	if (ret) {
		dev_err(dev, "hw params config failed with err %d\n", ret);
		return ret;
	}

	slave_config.dst_maxburst = dmap->dst_maxburst;
	slave_config.src_maxburst = dmap->src_maxburst;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		slave_config.dst_addr = dmap->dma_addr;
		slave_config.src_addr_width = slave_config.dst_addr_width;
		slave_config.slave_id = sunxi_slave_id(dmap->dma_drq_type_num,
						DRQSRC_SDRAM);
	} else {
		slave_config.src_addr =	dmap->dma_addr;
		slave_config.dst_addr_width = slave_config.src_addr_width;
		slave_config.slave_id = sunxi_slave_id(DRQDST_SDRAM,
						dmap->dma_drq_type_num);
	}

	chan = snd_dmaengine_pcm_get_chan(substream);
	if (chan == NULL) {
		pr_err("[%s] dma pcm get chan failed! chan is NULL!\n", __func__);
		return -EINVAL;
	}

	ret = dmaengine_slave_config(chan, &slave_config);
	if (ret < 0) {
		dev_err(dev, "dma slave config failed with err %d\n", ret);
		return ret;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	return 0;
}

static int sunxi_pcm_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);

	return 0;
}

static int sunxi_pcm_hw_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static int sunxi_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			snd_dmaengine_pcm_trigger(substream,
					SNDRV_PCM_TRIGGER_START);
			break;
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			snd_dmaengine_pcm_trigger(substream,
					SNDRV_PCM_TRIGGER_STOP);
			break;
		}
	} else {
		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			snd_dmaengine_pcm_trigger(substream,
					SNDRV_PCM_TRIGGER_START);
			break;
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			snd_dmaengine_pcm_trigger(substream,
					SNDRV_PCM_TRIGGER_STOP);
			break;
		}
	}
	return 0;
}

static int sunxi_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->platform->dev;
	int ret = 0;

	/* Set HW params now that initialization is complete */
	/* dma platform Hw params get from dtsi config */
	ret = sunxi_set_runtime_hwparams(substream);
	if (ret < 0) {
		pr_err("audio_driver: [%s] dma platform hwparams set failed\n",
								__func__);
		return ret;
	}

	ret = snd_pcm_hw_constraint_integer(substream->runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	ret = snd_dmaengine_pcm_open_request_chan(substream, NULL, NULL);
	if (ret != 0) {
		dev_err(dev, "dmaengine pcm chan request failed or open failed with err %d\n", ret);
		pr_err("[%s] maybe the DMA configuration is unenabled\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int sunxi_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = NULL;

	if (substream->runtime != NULL) {
		runtime = substream->runtime;

		return dma_mmap_writecombine(substream->pcm->card->dev, vma,
					     runtime->dma_area,
					     runtime->dma_addr,
					     runtime->dma_bytes);
	} else {
		return -1;
	}

}

static struct snd_pcm_ops sunxi_pcm_ops = {
	.open		= sunxi_pcm_open,
	.close		= snd_dmaengine_pcm_close_release_chan,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= sunxi_pcm_hw_params,
	.prepare	= sunxi_pcm_hw_prepare,
	.hw_free	= sunxi_pcm_hw_free,
	.trigger	= sunxi_pcm_trigger,
	.pointer	= snd_dmaengine_pcm_pointer,
	.mmap		= sunxi_pcm_mmap,
};

static int sunxi_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream,
					struct snd_soc_pcm_runtime *rtd)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	struct snd_soc_platform *platform = rtd->platform;
	struct device_node *np = platform->component.dev->of_node;
	size_t temp_long;
	size_t size = 0;
	int ret = 0;

	/* To get the device_node */
	snd_soc_platform_get_drvdata(platform);

	ret = of_property_read_u32(np, "buffer_bytes_max", &temp_long);
	if (ret < 0) {
		size = 1024 * 256;
		pr_err("audio_driver: [%s] buffer_bytes_max get failed!\n",
							__func__);
	} else {
		size = temp_long;
	}

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_coherent(pcm->card->dev, size,
					&buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;

	return 0;
}

static void sunxi_pcm_free_dma_buffers(struct snd_pcm *pcm)
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

static int sunxi_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &sunxi_pcm_mask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = 0xffffffff;

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		ret = sunxi_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK, rtd);
		if (ret)
			goto out;
	}

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		ret = sunxi_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE, rtd);
		if (ret)
			goto out;
	}

out:
	return ret;
}

static struct snd_soc_platform_driver sunxi_soc_platform = {
	.ops		= &sunxi_pcm_ops,
	.pcm_new	= sunxi_pcm_new,
	.pcm_free	= sunxi_pcm_free_dma_buffers,
};

int asoc_dma_platform_register(struct device *dev, unsigned int flags)
{
#ifdef CONFIG_SUNXI_ADC_DAUDIO_SYNC
	if (adc_sync_spinlock.ref_count++ == 0)
		audio_trigger_lock_spinlock_init(&adc_sync_spinlock);
#endif
	return snd_soc_register_platform(dev, &sunxi_soc_platform);
}
EXPORT_SYMBOL_GPL(asoc_dma_platform_register);

void asoc_dma_platform_unregister(struct device *dev)
{
	snd_soc_unregister_platform(dev);
}
EXPORT_SYMBOL_GPL(asoc_dma_platform_unregister);

MODULE_AUTHOR("huangxin, liushaohua");
MODULE_DESCRIPTION("sunxi ASoC DMA Driver");
MODULE_LICENSE("GPL");
