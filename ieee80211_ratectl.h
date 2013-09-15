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
 * $FreeBSD: soc2013/ccqin/head/sys/net80211/ieee80211_ratectl.h 257356 2013-09-15 03:47:06Z ccqin $
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

#define	IEEE80211_RATECTL_TX_SUCCESS	1
#define	IEEE80211_RATECTL_TX_FAILURE	0

#define	IEEE80211_RATECTL_TRUE		1
#define	IEEE80211_RATECTL_FALSE		0

#define	IEEE80211_RATECTL_NUM		4
#define	IEEE80211_RATECTL_TXMAXTRY	11

#define	IEEE80211_RATECTL_FLAG_DS		0x01	/* dual-stream rate */
#define	IEEE80211_RATECTL_FLAG_CW40		0x02	/* use HT40 */
#define	IEEE80211_RATECTL_FLAG_SGI		0x04	/* use short-GI */
#define	IEEE80211_RATECTL_FLAG_HT		0x08	/* use HT */
#define	IEEE80211_RATECTL_FLAG_RTSCTS	0x10	/* enable RTS/CTS protection */
#define	IEEE80211_RATECTL_FLAG_STBC		0x20	/* enable STBC */
#define	IEEE80211_RATECTL_FLAG_TS		0x40	/* triple-stream rate */
#define	IEEE80211_RATECTL_FLAG_SP		0x80	/* short preamble */

/* Hardware CAPs offered to rate control algo */
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

struct ieee80211_rc_info {
	struct ieee80211_rc_series iri_rc[IEEE80211_RATECTL_NUM];
	uint32_t iri_framelen;
	uint16_t iri_flags;			/* See below */
	uint16_t iri_maxaggrsize;	/* Maximum aggregate size */

	/* TX info */
	uint8_t iri_txcnt;			/* TX count */
	uint8_t iri_failcnt;		/* TX retry-fail count */
	uint8_t iri_okcnt;			/* TX ok with or without retry */
	uint8_t iri_retrycnt;		/* TX retry count */
	uint8_t iri_shortretry;
	uint8_t iri_longretry;
	uint8_t iri_finaltsi;
	uint8_t iri_txrate;			/* HW tx rate */
};

/* ieee80211_rc_info flags */
#define	IEEE80211_RATECTL_INFO_SP		0x01	/* short preamble */
#define	IEEE80211_RATECTL_INFO_AGGR		0x02	/* aggregation scenario */

#define NET80211_TAG_RATECTL   1   /* net80211 ratectl state */

/*
 * net80211 ratectl statistics. 
 * per vap ratectl seeting must start with this common state.
 */
struct ieee80211_rc_stat {
	uint32_t irs_capabilities;		/* hardware capabilities offered to rc */

	/* ratectl statistics */
	uint32_t irs_txcnt;
	uint32_t irs_failcnt;
	uint32_t irs_retrycnt;
	uint32_t irs_shortretry;
	uint32_t irs_longretry;
};

#define IEEE80211_RATECTL_STAT(_vap) \
	((struct ieee80211_rc_stat *)((_vap)->iv_rs))

#define	IEEE80211_RATECTL_HASCAP_MRR(_vap) \
	(IEEE80211_RATECTL_STAT(_vap)->irs_capabilities & IEEE80211_RATECTL_CAP_MRR)
#define	IEEE80211_RATECTL_HASCAP_MRRPROT(_vap) \
	(IEEE80211_RATECTL_STAT(_vap)->irs_capabilities & IEEE80211_RATECTL_CAP_MRRPROT)
#define	IEEE80211_RATECTL_HASCAP_MULTXCHAIN(_vap) \
	(IEEE80211_RATECTL_STAT(_vap)->irs_capabilities & IEEE80211_RATECTL_CAP_MULTXCHAIN)

struct ieee80211_ratectl {
	const char *ir_name;
	int	(*ir_attach)(const struct ieee80211vap *);
	void	(*ir_detach)(const struct ieee80211vap *);
	void	(*ir_init)(struct ieee80211vap *, uint32_t);
	void	(*ir_deinit)(struct ieee80211vap *);
	void	(*ir_node_init)(struct ieee80211_node *);
	void	(*ir_node_deinit)(struct ieee80211_node *);
	int	(*ir_rate)(struct ieee80211_node *, void *, uint32_t);
	void	(*ir_rates)(struct ieee80211_node *, struct ieee80211_rc_info *);
	void	(*ir_tx_complete)(const struct ieee80211vap *,
	    			  const struct ieee80211_node *, struct ieee80211_rc_info *);
	void	(*ir_tx_update)(const struct ieee80211vap *,
	    			const struct ieee80211_node *,
	    			void *, void *, void *);
	void	(*ir_setinterval)(const struct ieee80211vap *, int);
	void	(*ir_stats)(const struct ieee80211vap *);
};

void	ieee80211_ratectl_register(int, const struct ieee80211_ratectl *);
void	ieee80211_ratectl_unregister(int);
void	ieee80211_ratectl_init(struct ieee80211vap *, uint32_t);
void	ieee80211_ratectl_set(struct ieee80211vap *, int);
void	ieee80211_ratectl_complete_rcflags(struct ieee80211_node *, 
						struct ieee80211_rc_info*);

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
	IEEE80211_DPRINTF(vap, IEEE80211_MSG_RATECTL,
			"%s: net80211 ratectl node inited.\n", __func__);
}

static void __inline
ieee80211_ratectl_node_deinit(struct ieee80211_node *ni)
{
	const struct ieee80211vap *vap = ni->ni_vap;

	vap->iv_rate->ir_node_deinit(ni);
	IEEE80211_DPRINTF(vap, IEEE80211_MSG_RATECTL,
			"%s: net80211 ratectl node deinited.\n", __func__);
}

static int __inline
ieee80211_ratectl_rate(struct ieee80211_node *ni, void *arg, uint32_t iarg)
{
	const struct ieee80211vap *vap = ni->ni_vap;

	return vap->iv_rate->ir_rate(ni, arg, iarg);
}

static void __inline
ieee80211_ratectl_rates(struct ieee80211_node *ni, struct ieee80211_rc_info *rc_info)
{
	const struct ieee80211vap *vap = ni->ni_vap;

	if (rc_info->iri_flags & IEEE80211_RATECTL_INFO_AGGR) 
		rc_info->iri_framelen = 0;

	IEEE80211_DPRINTF(vap, IEEE80211_MSG_RATECTL,
			"%s: find rate sets for %saggregation scenario.\n", __func__, 
			(rc_info->iri_flags & IEEE80211_RATECTL_INFO_AGGR)? "" : "non-");

	vap->iv_rate->ir_rates(ni, rc_info);
	ieee80211_ratectl_complete_rcflags(ni, rc_info);
}

static void __inline
ieee80211_ratectl_tx_complete(const struct ieee80211vap *vap,
    const struct ieee80211_node *ni, struct ieee80211_rc_info *rc_info)
{
	vap->iv_rate->ir_tx_complete(vap, ni, rc_info);
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

static void __inline
ieee80211_ratectl_stats(const struct ieee80211vap *vap)
{
	printf("\n[%s]: net80211 ratectl statistics (%s)\n", 
			vap->iv_ifp->if_xname, vap->iv_rate->ir_name);
	if (vap->iv_rate->ir_stats == NULL)
		return;
	vap->iv_rate->ir_stats(vap);
}

static int __inline
ieee80211_ratectl_hascap_cw40(const struct ieee80211_node *ni)
{
	return ni->ni_chw == 40;
}

static int __inline
ieee80211_ratectl_hascap_shortgi(const struct ieee80211_node *ni)
{
	struct ieee80211com *ic = ni->ni_ic;

	if (ni->ni_chw == 40 &&
		ic->ic_htcaps & IEEE80211_HTCAP_SHORTGI40 &&
		ni->ni_htcap & IEEE80211_HTCAP_SHORTGI40)
		return IEEE80211_RATECTL_TRUE;

	if (ni->ni_chw == 20 &&
		ic->ic_htcaps & IEEE80211_HTCAP_SHORTGI20 &&
		ni->ni_htcap & IEEE80211_HTCAP_SHORTGI20)
		return IEEE80211_RATECTL_TRUE;

	return IEEE80211_RATECTL_FALSE;
}

static int __inline
ieee80211_ratectl_hascap_stbc(const struct ieee80211_node *ni)
{
	struct ieee80211com *ic = ni->ni_ic;

	if (ic->ic_htcaps & IEEE80211_HTCAP_TXSTBC &&
		ni->ni_htcap & IEEE80211_HTCAP_RXSTBC_1STREAM &&
		IEEE80211_RATECTL_HASCAP_MULTXCHAIN(ni->ni_vap))
		return IEEE80211_RATECTL_TRUE;
	
	return IEEE80211_RATECTL_FALSE;
}

static int __inline
ieee80211_ratectl_node_is11n(const struct ieee80211_node *ni)
{
	if (ni->ni_chan == NULL)
		return (0);
	if (ni->ni_chan == IEEE80211_CHAN_ANYC)
		return (0);
	return (IEEE80211_IS_CHAN_HT(ni->ni_chan));
}

__inline static const struct ieee80211_rateset *
ieee80211_ratectl_get_rateset(const struct ieee80211_node *ni)
{
	const struct ieee80211_rateset *rs = NULL;
	/* 11n or not? Pick the right rateset */
	if (ieee80211_ratectl_node_is11n(ni)) {
		IEEE80211_NOTE(ni->ni_vap, IEEE80211_MSG_RATECTL, ni,
				"%s: 11n node", __func__);
		rs = (struct ieee80211_rateset *) &ni->ni_htrates;
	} else {
		IEEE80211_NOTE(ni->ni_vap, IEEE80211_MSG_RATECTL, ni,
			    "%s: non-11n node", __func__);
		rs = &ni->ni_rates;
	}
	return rs;
}

static void __inline
ieee80211_ratectl_update_stat(const struct ieee80211vap *vap,
		const struct ieee80211_rc_info *rc_info)
{
	struct ieee80211_rc_stat * irs = IEEE80211_RATECTL_STAT(vap);
	irs->irs_txcnt += rc_info->iri_txcnt;
	irs->irs_failcnt += rc_info->iri_failcnt;
	irs->irs_retrycnt += rc_info->iri_retrycnt;
	irs->irs_shortretry += rc_info->iri_shortretry;
	irs->irs_longretry += rc_info->iri_longretry;
}

static void __inline
ieee80211_ratectl_rc_info_set(struct ieee80211_rc_info *rc_info,
		uint8_t txcnt, uint8_t failcnt, uint32_t framelen,
		uint8_t shortretry, uint8_t longretry,
		uint8_t finaltsi, uint8_t txrate)
{
	rc_info->iri_txcnt = txcnt;
	rc_info->iri_failcnt = failcnt;
	rc_info->iri_okcnt = txcnt - failcnt;
	rc_info->iri_framelen = framelen;
	rc_info->iri_shortretry = shortretry;
	rc_info->iri_longretry = longretry;
	rc_info->iri_retrycnt = shortretry + longretry;
	rc_info->iri_finaltsi = finaltsi;
	rc_info->iri_txrate = txrate;
}

__inline static struct ieee80211_rc_info *
ieee80211_ratectl_rc_info_get(struct ieee80211_node *ni,
		struct mbuf *m)
{
	struct m_tag *mtag;

    mtag = m_tag_locate(m, MTAG_ABI_NET80211, 
			NET80211_TAG_RATECTL, NULL);

	IEEE80211_NOTE(ni->ni_vap, IEEE80211_MSG_RATECTL, ni,
			"%s: %sratectl mbuf tag found.\n", __func__, 
			(NULL == mtag? "no ":""));
again:
	if (NULL == mtag) {
		mtag = m_tag_alloc(MTAG_ABI_NET80211, NET80211_TAG_RATECTL,
				sizeof(struct ieee80211_rc_info), M_NOWAIT);
		if (NULL == mtag) {
			IEEE80211_NOTE(ni->ni_vap, IEEE80211_MSG_RATECTL, ni,
					"%s: can't alloc mbuf tag for ratectl.\n", __func__);
			goto again;
		}
		bzero(mtag + 1, mtag->m_tag_len);
		m_tag_prepend(m, mtag);
	}
	return (struct ieee80211_rc_info*)(mtag + 1);
}

#endif
