/*
 * Copyright 2016 Broadcom Corporation.  All rights reserved.
 *
 * Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2, available at
 * http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
 */

/*
 * This is the ASoC machine file for Pegasus SVK boards.
 */
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>

#include "pegasus-ssp.h"
#include "../../codecs/tlv320aic3x.h"

/* Max string length of our dt property names */
#define PROP_LEN_MAX 40

struct pegasus_sspcfg_info {
	int clksrc;
	int codec_slave_mode;
};

struct pegasussvk_data {
	struct pegasus_sspcfg_info sspcfg_info[MAX_PLAYBACK_PORTS];
};

static int pegasussvk_hw_params_si3226x(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return 0;
}

static int pegasussvk_hw_params_aic3x(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct pegasussvk_data *card_data =
		snd_soc_card_get_drvdata(soc_card);
	struct pegasus_sspcfg_info *sspcfg;
	unsigned int format = 0;
	unsigned int mclk_freq = 0;
	int ret = 0;

	sspcfg = &card_data->sspcfg_info[0];

	switch (params_rate(params)) {
	case  8000:
	case 16000:
		mclk_freq = 6144000;
		break;
	case 32000:
	case 48000:
		mclk_freq = 12288000;
		break;
	case 96000:
	case 192000:
		mclk_freq = 24576000;
		break;
	case 11025:
	case 22050:
		mclk_freq = 5644800;
		break;
	case 44100:
		mclk_freq = 11289600;
		break;
	case  88200:
	case 176400:
		mclk_freq = 22579200;
		break;
	default:
		return -EINVAL;
	}

	if (sspcfg->codec_slave_mode == 1) {
		ret = snd_soc_dai_set_sysclk(codec_dai, CLKIN_MCLK,
			mclk_freq, SND_SOC_CLOCK_IN);
		if (ret < 0)
			return ret;

		ret = snd_soc_dai_set_sysclk(cpu_dai, sspcfg->clksrc,
					mclk_freq, SND_SOC_CLOCK_OUT);
		if (ret < 0)
			return ret;

		/* Set codec as I2S slave */
		format = SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S;
	} else {
		ret = snd_soc_dai_set_sysclk(codec_dai, CLKIN_MCLK,
						12000000, SND_SOC_CLOCK_IN);
		if (ret < 0)
			return ret;

		format = SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_I2S;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, format);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, format);
	if (ret < 0)
		return ret;

	return 0;
}

static int pegasussvk_startup_aic3x(struct snd_pcm_substream *substream)
{
	return 0;
}

static struct snd_soc_ops pegasussvk_ops_aic3x = {
	.startup = pegasussvk_startup_aic3x,
	.hw_params = pegasussvk_hw_params_aic3x,
};

static int handsfree_spk_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	return 0;
}

static int ext_spk_evt_aic3x(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	return 0;
}

/* Machine DAPM */
static const struct snd_soc_dapm_widget aic3106_dapm_widgets[] = {
	/* Outputs */
	SND_SOC_DAPM_HP("Headset Spk", NULL),
	SND_SOC_DAPM_HP("Handset Spk", NULL),
	SND_SOC_DAPM_LINE("Handsfree Spk", handsfree_spk_event),
	SND_SOC_DAPM_SPK("External Spk", ext_spk_evt_aic3x), /* 3.5 mm jack */

	/* Inputs */
	SND_SOC_DAPM_MIC("Handset Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Handsfree Mic", NULL),
	SND_SOC_DAPM_LINE("External Mic", NULL),   /* 3.5 mm jack */
};

static const struct snd_soc_dapm_route aic3106_audio_map[] = {
	/* Routings for outputs */
	{"Headset Spk", NULL, "HPLOUT"},
	{"Handset Spk", NULL, "HPROUT"},
	{"Handsfree Spk", NULL, "MONO_LOUT"},
	{"External Spk", NULL, "LLOUT"},
	{"External Spk", NULL, "RLOUT"},

	/* Routings for inputs */
	{"LINE1L", NULL, "Mic Bias"},
	{"LINE2L", NULL, "Mic Bias"},
	{"LINE2R", NULL, "Mic Bias"},

	{"Mic Bias", NULL, "Handset Mic"},
	{"Mic Bias", NULL, "Headset Mic"},
	{"Mic Bias", NULL, "Handsfree Mic"},

	{"MIC3R", NULL, "External Mic"},
};

static int pegasussvk_init_aic3106(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->component.dapm;

	snd_soc_dapm_new_controls(dapm, aic3106_dapm_widgets,
				ARRAY_SIZE(aic3106_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, aic3106_audio_map,
				ARRAY_SIZE(aic3106_audio_map));

	snd_soc_dapm_nc_pin(dapm, "LINE1R");
	snd_soc_dapm_nc_pin(dapm, "MIC3L");

	return 0;
}

static int pegasussvk_setup_i2c(void)
{
	struct platform_device *pdev;
	struct device *dev_i2c;
	struct device_node *np = NULL;
	char prop_name[PROP_LEN_MAX] = "brcm,iproc-i2c";

	np = of_find_node_with_property(np, prop_name);
	pdev = of_find_device_by_node(np);
	dev_i2c = &pdev->dev;
	return 0;
}

static int pegasussvk_startup_si3226x(struct snd_pcm_substream *substream)
{
	/* Select the appropriate I2C Mux channel to communicate
	 * with Daughter Card
	 */

	return pegasussvk_setup_i2c();
}

static struct snd_soc_ops pegasussvk_ops_si3226x = {
	.startup = pegasussvk_startup_si3226x,
	.hw_params = pegasussvk_hw_params_si3226x,
};

static int pegasussvk_init_si3226x(struct snd_soc_pcm_runtime *rtd)
{
	pr_debug("Enter %s\n", __func__);
	return 0;
}

static int pegasussvk_hw_params_spdif(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct pegasussvk_data *card_data =
		snd_soc_card_get_drvdata(soc_card);
	struct pegasus_sspcfg_info *sspcfg;
	unsigned int mclk_freq = 0;
	int ret = 0;

	sspcfg = &card_data->sspcfg_info[3];

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 48000:
	case 96000:
		mclk_freq = 24576000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk_freq = 22579200;
		break;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, sspcfg->clksrc,
				mclk_freq, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops pegasussvk_ops_spdif = {
	.hw_params = pegasussvk_hw_params_spdif,
};

static int pegasussvk_hw_params_testport_common(
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, int linknum)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct pegasussvk_data *card_data = snd_soc_card_get_drvdata(soc_card);
	struct pegasus_sspcfg_info *sspcfg;
	int sspmode;
	unsigned int format = 0;
	unsigned int mclk_freq = 0;
	int ret = 0;

	sspcfg = &card_data->sspcfg_info[linknum];

	switch (params_rate(params)) {
	case 8000:
	case 16000:
		mclk_freq = 4096000;
		break;
	case 48000:
	case 96000:
		mclk_freq = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk_freq = 11289600;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, sspcfg->clksrc,
				mclk_freq, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	/* Most systems will never need to do this get_mode, because the port
	 * will only every need to run in one mode.
	 * It is convientent on our verification system to initialize the board
	 * in different modes for verification puposes.
	 */
	sspmode = pegasus_ssp_get_mode(cpu_dai);
	if (sspmode == SSPMODE_TDM) {
		format = SND_SOC_DAIFMT_CBS_CFS
			| SND_SOC_DAIFMT_DSP_B
			| SND_SOC_DAIFMT_IB_NF;
	} else {
		format = SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S;
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, format);
	if (ret < 0)
		return ret;

	if (sspmode == SSPMODE_TDM) {
		unsigned int channel = 0;
		unsigned int width = 0;

		channel = params_channels(params);
		width = snd_pcm_format_physical_width(params_format(params));

		/* dai, tx_mask, rx_mask, slots, slot_width
		 */
		ret = snd_soc_dai_set_tdm_slot(cpu_dai, 1, 0, channel, width);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int pegasussvk_hw_params_testport0(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return pegasussvk_hw_params_testport_common(substream, params, 0);
}

static int pegasussvk_hw_params_testport1(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return pegasussvk_hw_params_testport_common(substream, params, 1);
}

static int pegasussvk_hw_params_testport2(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return pegasussvk_hw_params_testport_common(substream, params, 2);
}

static struct snd_soc_ops pegasussvk_ops_testport0 = {
	.hw_params = pegasussvk_hw_params_testport0,
};

static struct snd_soc_ops pegasussvk_ops_testport1 = {
	.hw_params = pegasussvk_hw_params_testport1,
};

static struct snd_soc_ops pegasussvk_ops_testport2 = {
	.hw_params = pegasussvk_hw_params_testport2,
};

/* digital audio interface glue - connects codec <--> CPU
 * Just use some very generic names for "name" and "stream_name".
 * Ideally we would use more meaningful names, but because we have made
 * this machine module so configurable it make is easier to just use
 * generic names.
 */
static struct snd_soc_dai_link pegasussvk_dai_links[] = {
{
	.name = "pegasussvkdev0",
	.stream_name = "pegasussvkdev0_stream",
	.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
},
{
	.name = "SPDIF",
	.stream_name = "SPDIF",
	.codec_dai_name = "snd-soc-dummy-dai",
	.codec_name = "snd-soc-dummy",

	.ops = &pegasussvk_ops_spdif,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
},
{
	.name = "pegasussvkdev1",
	.stream_name = "pegasussvkdev1_stream",
	.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
}
};

/* Audio machine driver */
static struct snd_soc_card pegasussvk_card = {
	.name = "bcm-pegasussvk",
	.owner = THIS_MODULE,
	.dai_link = pegasussvk_dai_links,
	.num_links = ARRAY_SIZE(pegasussvk_dai_links),
};

int get_dt_info(struct platform_device *pdev,
	struct pegasussvk_data *card_data, int linknum)
{
	struct device_node *np = pdev->dev.of_node;
	char prop_name[PROP_LEN_MAX];
	int ret = 0;
	const char *clk;
	struct snd_soc_dai_link *dai_link;
	struct device_node *cpu_np = NULL;
	struct pegasus_sspcfg_info *sspcfg;

	dai_link = &pegasussvk_dai_links[linknum];
	sspcfg = &card_data->sspcfg_info[linknum];

	/* Is there a codec attached to this link?
	 * If not use a dummy codec.
	 */
	snprintf(prop_name, PROP_LEN_MAX, "bcm,pegasus-link%d-codec", linknum);
	dai_link->codec_of_node = of_parse_phandle(np, prop_name, 0);
	if (dai_link->codec_of_node == NULL) {
		if (linknum == 0) {
			dai_link->codec_dai_name = "snd-soc-dummy-dai";
			dai_link->codec_name = "snd-soc-dummy";
			dai_link->ops = &pegasussvk_ops_testport0;
		} else if (linknum == 1) {
			dai_link->codec_dai_name = "snd-soc-dummy-dai";
			dai_link->codec_name = "snd-soc-dummy";
			dai_link->ops = &pegasussvk_ops_testport1;
		} else if (linknum == 2) {
			dai_link->codec_dai_name = "snd-soc-dummy-dai";
			dai_link->codec_name = "snd-soc-dummy";
			dai_link->ops = &pegasussvk_ops_testport2;
		}
	} else {
		if (linknum == 0) {
			dai_link->codec_dai_name = "tlv320aic3x-hifi";
			dai_link->ops = &pegasussvk_ops_aic3x;
			dai_link->init = pegasussvk_init_aic3106;
		}
		if (linknum == 2) {
			dai_link->codec_dai_name = "si3226x-slic";
			dai_link->ops = &pegasussvk_ops_si3226x;
			dai_link->init = pegasussvk_init_si3226x;
		}
	}

	snprintf(prop_name, PROP_LEN_MAX, "bcm,pegasus-link%d-slave", linknum);

	ret = of_property_read_u32(np, prop_name, &sspcfg->codec_slave_mode);
	if (ret) {
		dev_err(&pdev->dev,
			"Could not find %s property in device tree\n",
			prop_name);
		goto err_exit;
	}

	snprintf(prop_name, PROP_LEN_MAX, "bcm,pegasus-link%d-ssp", linknum);
	cpu_np = of_parse_phandle(np, prop_name, 0);
	dai_link->cpu_of_node = cpu_np;
	if (dai_link->cpu_of_node == NULL) {
		dev_err(&pdev->dev,
			"Property %s missing or invalid\n", prop_name);
		ret = -EINVAL;
		goto err_exit;
	}

	snprintf(prop_name, PROP_LEN_MAX, "bcm,pegasus-link%d-clk", linknum);
	if ((of_property_read_string(np, prop_name, &clk)) != 0) {
		dev_err(&pdev->dev,
			"Property %s missing or invalid\n", prop_name);
		ret = -EINVAL;
		goto err_exit;
	}

	if (strstr(clk, "pll"))
		sspcfg->clksrc = PEGASUS_SSP_CLKSRC_PLL;
	else
		sspcfg->clksrc = PEGASUS_SSP_CLKSRC_NCO_0;

err_exit:
	return ret;
}

static int pegasussvk_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &pegasussvk_card;
	struct pegasussvk_data *card_data;
	const struct device_node *pofn;
	int ret = 0;

	dev_dbg(&pdev->dev, "Enter %s\n", __func__);

	card->dev = &pdev->dev;

	card_data = devm_kzalloc(&pdev->dev, sizeof(*card_data), GFP_KERNEL);
	if (card_data == NULL)
		return -ENOMEM;

	/* Get codec and ssp info for Link 0 */
	ret = get_dt_info(pdev, card_data, 0);
	if (ret < 0)
		goto err_exit;

	/* Get codec and ssp info for Link 1 */
	ret = get_dt_info(pdev, card_data, 1);
	if (ret < 0)
		goto err_exit;

	/* Get codec and ssp info for Link 2 */
	ret = get_dt_info(pdev, card_data, 2);
	if (ret < 0)
		goto err_exit;

	/* Use the same platform driver for both links */
	pofn = of_parse_phandle(np, "bcm,pegasus-pcm", 0);
	if (pofn == NULL) {
		dev_err(&pdev->dev,
			"Property bcm,pegasus-pcm missing or invalid\n");
		ret = -EINVAL;
		goto err_exit;
	}

	/* Use the same platform driver for all links */
	pegasussvk_dai_links[0].platform_of_node = (struct device_node *)pofn;
	pegasussvk_dai_links[1].platform_of_node = (struct device_node *)pofn;
	pegasussvk_dai_links[2].platform_of_node = (struct device_node *)pofn;

	snd_soc_card_set_drvdata(card, card_data);

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n",
			ret);
		goto err_exit;
	}
	ret = pegasussvk_setup_i2c();
	if (ret)
		dev_err(&pdev->dev, "ns2svk_setup_i2c() failed: %d\n",
			ret);
	else
		pr_info("snd_soc_register_card: successful.\n");

	return ret;

err_exit:
	devm_kfree(&pdev->dev, card_data);
	return ret;
}

static int pegasussvk_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct pegasussvk_data *card_data;

	dev_dbg(&pdev->dev, "%s Enter\n", __func__);

	card_data = snd_soc_card_get_drvdata(card);

	snd_soc_unregister_card(card);
	devm_kfree(&pdev->dev, card_data);

	return 0;
}

static const struct of_device_id pegasus_mach_of_match[] = {
	{.compatible = "bcm,pegasussvk-machine", },
	{ },
};
MODULE_DEVICE_TABLE(of, pegasus_mach_of_match);

static struct platform_driver bcm_pegasus_driver = {
	.driver = {
		.name = "bcm-pegasussvk-machine",
		.owner = THIS_MODULE,
		.of_match_table = pegasus_mach_of_match,
	},
	.probe  = pegasussvk_probe,
	.remove = pegasussvk_remove,
};

module_platform_driver(bcm_pegasus_driver);

MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("ALSA SoC for Pegasus APs");
MODULE_LICENSE("GPL v2");
