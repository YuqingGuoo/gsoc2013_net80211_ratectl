/*-
 * Copyright (c) 2010 Rui Paulo <rpaulo@FreeBSD.org>
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
 * $FreeBSD: soc2013/ccqin/head/sys/net80211/ieee80211_ratectl.h 254864 2013-07-17 03:02:32Z ccqin $
 */

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

#define	IEEE80211_RATECTL_NUM		4

#define	IEEE80211_RATECTL_DS_FLAG		0x01	/* dual-stream rate */
#define	IEEE80211_RATECTL_CW40_FLAG		0x02	/* use HT40 */
#define	IEEE80211_RATECTL_SGI_FLAG		0x04	/* use short-GI */
#define	IEEE80211_RATECTL_HT_FLAG		0x08	/* use HT */
#define	IEEE80211_RATECTL_RTSCTS_FLAG	0x10	/* enable RTS/CTS protection */
#define	IEEE80211_RATECTL_STBC_FLAG		0x20	/* enable STBC */
#define	IEEE80211_RATECTL_TS_FLAG		0x40	/* triple-stream rate */

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
	int	(*ir_attach)(const struct ieee80211vap *);
	void	(*ir_detach)(const struct ieee80211vap *);
	void	(*ir_init)(struct ieee80211vap *);
	void	(*ir_deinit)(struct ieee80211vap *);
	void	(*ir_node_init)(struct ieee80211_node *);
	void	(*ir_node_deinit)(struct ieee80211_node *);
	int	(*ir_rate)(struct ieee80211_node *, void *, uint32_t);
	void	(*ir_rates)(struct ieee80211_node *, struct ieee80211_rc_series *, int, size_t);
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
void	ieee80211_ratectl_init(struct ieee80211vap *);
void	ieee80211_ratectl_set(struct ieee80211vap *, int);

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
	struct ieee80211com *ic = vap->iv_ic; 

	vap->iv_rate->ir_rates(ni, rc, shortPreamble, frameLen);

	/* if enable rts/cts and is pre-802.11n, blank tries 1, 2, 3 */
	if (!(ic->ic_htcaps & IEEE80211_HTC_HT))
	{
		if (rc[0].flags & IEEE80211_RATECTL_RTSCTS_FLAG)
			rc[1].tries = rc[2].tries = rc[3].tries = 0;
		rc[1].flags &= ~IEEE80211_RATECTL_RTSCTS_FLAG; 
		rc[2].flags &= ~IEEE80211_RATECTL_RTSCTS_FLAG; 
		rc[3].flags &= ~IEEE80211_RATECTL_RTSCTS_FLAG; 
	}
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
