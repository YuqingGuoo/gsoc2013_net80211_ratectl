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
 *
 * $FreeBSD: soc2013/ccqin/head/sys/net80211/ieee80211_ratectl.h 255101 2013-07-24 09:48:56Z ccqin $
 */
#ifndef _NET80211_IEEE80211_RATECTL_H_
#define _NET80211_IEEE80211_RATECTL_H_

enum ieee80211_ratealgs {
	IEEE80211_RATECTL_AMRR		= 0,
	IEEE80211_RATECTL_RSSADAPT	= 1,
	IEEE80211_RATECTL_ONOE		= 2,
	IEEE80211_RATECTL_SAMPLE	= 3,
	IEEE80211_RATECTL_NONE		= 4,
	IEEE80211_RATECTL_MAX
};

enum {
	MCS_HT20,
	MCS_HT20_SGI,
	MCS_HT40,
	MCS_HT40_SGI,
};

#define	IEEE80211_RATECTL_TX_SUCCESS	1
#define	IEEE80211_RATECTL_TX_FAILURE	0

#define	IEEE80211_RATECTL_TRUE		1
#define	IEEE80211_RATECTL_FALSE		0

#define	IEEE80211_RATECTL_NUM		4

#define	IEEE80211_RATECTL_DS_FLAG		0x01	/* dual-stream rate */
#define	IEEE80211_RATECTL_CW40_FLAG		0x02	/* use HT40 */
#define	IEEE80211_RATECTL_SGI_FLAG		0x04	/* use short-GI */
#define	IEEE80211_RATECTL_HT_FLAG		0x08	/* use HT */
#define	IEEE80211_RATECTL_RTSCTS_FLAG	0x10	/* enable RTS/CTS protection */
#define	IEEE80211_RATECTL_STBC_FLAG		0x20	/* enable STBC */
#define	IEEE80211_RATECTL_TS_FLAG		0x40	/* triple-stream rate */

/* Hardware CAPs chip offered to rate control code */
#define	IEEE80211_RATECTL_CAP_MRR			0x01	/* support MRR */
#define	IEEE80211_RATECTL_CAP_MRRPROT		0x02	/* support MRR + protect */
#define	IEEE80211_RATECTL_CAP_MULTXCHAIN	0x04	/* has more than 1 txchain */

#define IS_VAP_HT(vap)	((vap)->iv_htcaps & IEEE80211_HTC_HT)

#define IS_HT_RATE(_rate)   ((_rate) & 0x80)
#define HT_RC_2_MCS(_rc)    ((_rc) & 0x7f)
#define HT_RC_2_STREAMS(_rc)    ((((_rc) & 0x78) >> 3) + 1)


extern int max_4ms_framelen[4][32];

struct ieee80211_rc_series {
	uint8_t rix;		/* ratetable index, not rate code */
	uint8_t ratecode;	/* hardware rate code */
	uint8_t tries;
	uint8_t tx_power_cap;
	uint16_t flags;
	uint16_t max4msframelen;
};

struct ieee80211_ratectl {
	const char *ir_name;
	uint32_t ir_capabilities;		/* hardware capabilities offered to rc */

	int	(*ir_attach)(const struct ieee80211vap *);
	void	(*ir_detach)(const struct ieee80211vap *);
	void	(*ir_init)(struct ieee80211vap *);
	void	(*ir_deinit)(struct ieee80211vap *);
	void	(*ir_node_init)(struct ieee80211_node *);
	void	(*ir_node_deinit)(struct ieee80211_node *);
	int	(*ir_rate)(struct ieee80211_node *, void *, uint32_t);
	void	(*ir_rates)(struct ieee80211_node *, struct ieee80211_rc_series *, 
					  int, size_t);
	void	(*ir_tx_complete)(const struct ieee80211vap *,
	    			  const struct ieee80211_node *, int,
	    			  void *, void *);
	void	(*ir_tx_update)(const struct ieee80211vap *,
	    			const struct ieee80211_node *,
	    			void *, void *, void *);
	void	(*ir_setinterval)(const struct ieee80211vap *, int);
};

void	ieee80211_ratectl_register(int, const struct ieee80211_ratectl *);
void	ieee80211_ratectl_unregister(int);
void	ieee80211_ratectl_init(struct ieee80211vap *, uint32_t);
void	ieee80211_ratectl_set(struct ieee80211vap *, int);
void	ieee80211_ratectl_complete_rcflags(const struct ieee80211_node *, 
		struct ieee80211_rc_series *, int)

MALLOC_DECLARE(M_80211_RATECTL);

static void __inline
ieee80211_ratectl_deinit(struct ieee80211vap *vap)
{
	vap->iv_rate->ir_deinit(vap);
}

static void __inline
ieee80211_ratectl_node_init(struct ieee80211_node *ni)
{
	const struct ieee80211vap *vap = ni->ni_vap;

	vap->iv_rate->ir_node_init(ni);
}

static void __inline
ieee80211_ratectl_node_deinit(struct ieee80211_node *ni)
{
	const struct ieee80211vap *vap = ni->ni_vap;

	vap->iv_rate->ir_node_deinit(ni);
}

static int __inline
ieee80211_ratectl_rate(struct ieee80211_node *ni, void *arg, uint32_t iarg)
{
	const struct ieee80211vap *vap = ni->ni_vap;

	return vap->iv_rate->ir_rate(ni, arg, iarg);
}

static void __inline
ieee80211_ratectl_rates(struct ieee80211_node *ni, struct ieee80211_rc_series *rc,
		int shortPreamble, size_t frameLen)
{
	const struct ieee80211vap *vap = ni->ni_vap;

	vap->iv_rate->ir_rates(ni, rc, shortPreamble, frameLen);
	
	ieee80211_ratectl_complete_rcflags(ni, rc, shortPreamble);
}

static void __inline
ieee80211_ratectl_tx_complete(const struct ieee80211vap *vap,
    const struct ieee80211_node *ni, int status, void *arg1, void *arg2)
{
	vap->iv_rate->ir_tx_complete(vap, ni, status, arg1, arg2);
}

static void __inline
ieee80211_ratectl_tx_update(const struct ieee80211vap *vap,
    const struct ieee80211_node *ni, void *arg1, void *arg2, void *arg3)
{
	if (vap->iv_rate->ir_tx_update == NULL)
		return;
	vap->iv_rate->ir_tx_update(vap, ni, arg1, arg2, arg3);
}

static void __inline
ieee80211_ratectl_setinterval(const struct ieee80211vap *vap, int msecs)
{
	if (vap->iv_rate->ir_setinterval == NULL)
		return;
	vap->iv_rate->ir_setinterval(vap, msecs);
}

static int __inline
ieee80211_ratectl_hascap_cw40(const struct ieee80211vap *vap,
		const struct ieee80211_node *ni)
{
	return IS_VAP_HT(vap) && (ni->ni_chw == 40);
}

static int __inline
ieee80211_ratectl_hascap_shortgi(const struct ieee80211vap *vap,
		const struct ieee80211_node *ni)
{
	if (! IS_VAP_HT(vap))
		return IEEE80211_RATECTL_FALSE;

	if (ni->ni_chw == 40 &&
			vap->iv_htcaps & IEEE80211_HTCAP_SHORTGI40 &&
			ni->ni_htcap & IEEE80211_HTCAP_SHORTGI40)
		return IEEE80211_RATECTL_TRUE;

	if (ni->ni_chw == 20 &&
			vap->iv_htcaps & IEEE80211_HTCAP_SHORTGI20 &&
			ni->ni_htcap & IEEE80211_HTCAP_SHORTGI20)
		return IEEE80211_RATECTL_TRUE;
}


static int __inline
ieee80211_ratectl_hascap_stbc(const struct ieee80211vap *vap,
		const struct ieee80211_node *ni)
{
   return IS_VAP_HT(vap) && (vap->iv_htcaps & IEEE80211_HTCAP_TXSTBC) &&
			    (ni->ni_htcap & IEEE80211_HTCAP_RXSTBC_1STREAM) &&
			    (vap->iv_rate->ir_capabilities & IEEE80211_RATECTL_CAP_MULTXCHAIN);
}

#endif
