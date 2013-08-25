/*-
 * Copyright (c) 2010 Rui Paulo <rpaulo@FreeBSD.org>
 * Copyright (c) 2013 Chenchong Qin <ccqin@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: soc2013/ccqin/head/sys/net80211/ieee80211_ratectl.c 256474 2013-08-25 09:37:15Z ccqin $");

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/socket.h>

#include <net/if.h>
#include <net/if_media.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_ratectl.h>

static const struct ieee80211_ratectl *ratectls[IEEE80211_RATECTL_MAX];

static const char *ratectl_modnames[IEEE80211_RATECTL_MAX] = {
	[IEEE80211_RATECTL_AMRR]	= "wlan_amrr",
	[IEEE80211_RATECTL_RSSADAPT]	= "wlan_rssadapt",
	[IEEE80211_RATECTL_ONOE]	= "wlan_onoe",
	[IEEE80211_RATECTL_SAMPLE]	= "wlan_sample",
	[IEEE80211_RATECTL_NONE]	= "wlan_none",
};

MALLOC_DEFINE(M_80211_RATECTL, "80211ratectl", "802.11 rate control");

enum {
	MCS_HT20,
	MCS_HT20_SGI,
	MCS_HT40,
	MCS_HT40_SGI,
};

int max_4ms_framelen[4][32] = {
	[MCS_HT20] = {
		3212,  6432,  9648,  12864,  19300,  25736,  28952,  32172,
		6424,  12852, 19280, 25708,  38568,  51424,  57852,  64280,
		9628,  19260, 28896, 38528,  57792,  65532,  65532,  65532,
		12828, 25656, 38488, 51320,  65532,  65532,  65532,  65532,
	},
	[MCS_HT20_SGI] = {
		3572,  7144,  10720,  14296,  21444,  28596,  32172,  35744,
		7140,  14284, 21428,  28568,  42856,  57144,  64288,  65532,
		10700, 21408, 32112,  42816,  64228,  65532,  65532,  65532,
		14256, 28516, 42780,  57040,  65532,  65532,  65532,  65532,
	},
	[MCS_HT40] = {
		6680,  13360,  20044,  26724,  40092,  53456,  60140,  65532,
		13348, 26700,  40052,  53400,  65532,  65532,  65532,  65532,
		20004, 40008,  60016,  65532,  65532,  65532,  65532,  65532,
		26644, 53292,  65532,  65532,  65532,  65532,  65532,  65532,
	},
	[MCS_HT40_SGI] = {
		7420,  14844,  22272,  29696,  44544,  59396,  65532,  65532,
		14832, 29668,  44504,  59340,  65532,  65532,  65532,  65532,
		22232, 44464,  65532,  65532,  65532,  65532,  65532,  65532,
		29616, 59232,  65532,  65532,  65532,  65532,  65532,  65532,
	}
};

void
ieee80211_ratectl_register(int type, const struct ieee80211_ratectl *ratectl)
{
	if (type >= IEEE80211_RATECTL_MAX)
		return;
	ratectls[type] = ratectl;
}

void
ieee80211_ratectl_unregister(int type)
{
	if (type >= IEEE80211_RATECTL_MAX)
		return;
	ratectls[type] = NULL;
}

void
ieee80211_ratectl_init(struct ieee80211vap *vap, uint32_t capabilities)
{
	if (vap->iv_rate == ratectls[IEEE80211_RATECTL_NONE])
		ieee80211_ratectl_set(vap, IEEE80211_RATECTL_AMRR);
	vap->iv_rate->ir_init(vap, capabilities);
}

void
ieee80211_ratectl_set(struct ieee80211vap *vap, int type)
{
	if (type >= IEEE80211_RATECTL_MAX)
		return;
	if (ratectls[type] == NULL) {
		ieee80211_load_module(ratectl_modnames[type]);
		if (ratectls[type] == NULL) {
			IEEE80211_DPRINTF(vap, IEEE80211_MSG_RATECTL,
			    "%s: unable to load algo %u, module %s\n",
			    __func__, type, ratectl_modnames[type]);
			vap->iv_rate = ratectls[IEEE80211_RATECTL_NONE];
			return;
		}
	}
	vap->iv_rate = ratectls[type];
}

void
ieee80211_ratectl_complete_rcflags(struct ieee80211_node *ni,
		struct ieee80211_rc_info *rc_info)
{
	struct ieee80211vap *vap = ni->ni_vap;
	const struct ieee80211_rate_table * rt = NULL;
	struct ieee80211_rc_series *rc = rc_info->iri_rc;
	/* int shortPreamble = rc_info->ri_shortPreamble; */
	uint8_t rate;
	int i;

	rt = ieee80211_get_ratetable(ni->ni_ic->ic_curchan);

	/* Make sure that rate control code doesn't mess it up.
	 * If enable rts/cts and is pre-802.11n, blank tries 1, 2, 3 
	 */

	if (! IEEE80211_RATECTL_HASCAP_MRRPROT(vap))
	{
		for (i = 1; i < IEEE80211_RATECTL_NUM; i++)
		{
			if (rc[0].flags & IEEE80211_RATECTL_FLAG_RTSCTS)
				rc[i].tries = 0;
			rc[i].flags &= ~IEEE80211_RATECTL_FLAG_RTSCTS; 
		}
	}

	for (i = 0; i < IEEE80211_RATECTL_NUM; i++) {
		
		if (rc[i].tries == 0)
			continue;

		rate = rt->info[rc[i].rix].dot11Rate;

		/*
		 * Only enable short preamble for legacy rates
		 */

		/* XXX how we get the non_ht ratecode here? */

		#if 0
		if ((! IS_HT_RATE(rate)) && shortPreamble)
			rate |= rt->info[rc[i].rix].shortPreamble;
		#endif

		/*
		 * Save this, used by the TX and completion code
		 */
		rc[i].ratecode = rate;

		/* Only enable shortgi, 2040, dual-stream if HT is set */
		if (IS_HT_RATE(rate)) {
			rc[i].flags |= IEEE80211_RATECTL_FLAG_HT;

			/*
			 * XXX TODO: LDPC
			 */

			/*
			 * Dual / Triple stream rate?
			 */
			if (HT_RC_2_STREAMS(rate) == 2)
				rc[i].flags |= IEEE80211_RATECTL_FLAG_DS;
			else if (HT_RC_2_STREAMS(rate) == 3)
				rc[i].flags |= IEEE80211_RATECTL_FLAG_TS;
		}

		/*
		 * Calculate the maximum TX power cap for the current
		 * node. 
		 * Rate control algo can't control TX power by now.
		 */
		rc[i].tx_power_cap = ieee80211_get_node_txpower(ni);
		
		/*
		 * Calculate the maximum 4ms frame length based
		 * on the MCS rate, SGI and channel width flags.
		 */
		if ((rc[i].flags & IEEE80211_RATECTL_FLAG_HT) &&
		    (HT_RC_2_MCS(rate) < 32)) {
			int j;
			if (rc[i].flags & IEEE80211_RATECTL_FLAG_CW40) {
				if (rc[i].flags & IEEE80211_RATECTL_FLAG_SGI)
					j = MCS_HT40_SGI;
				else
					j = MCS_HT40;
			} else {
				if (rc[i].flags & IEEE80211_RATECTL_FLAG_SGI)
					j = MCS_HT20_SGI;
				else
					j = MCS_HT20;
			}
			rc[i].max4msframelen =
			    max_4ms_framelen[j][HT_RC_2_MCS(rate)];
		} else
			rc[i].max4msframelen = 0;
	}
}

