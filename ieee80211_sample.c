/* $FreeBSD: head/sys/dev/ath/ath_rate/sample/sample.c 248573 2013-02-27 04:33:06Z adrian $*/

/*-
 * Copyright (c) 2005 John Bicket
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

#include "opt_wlan.h"

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/socket.h>
#include <sys/sysctl.h>

#include <net/if.h>
#include <net/if_media.h>
#include <net/ethernet.h>

#ifdef INET
#include <netinet/in.h>
#include <netinet/if_ether.h>
#endif

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_ht.h>
#include <net80211/ieee80211_ratectl.h>
#include <net80211/ieee80211_sample.h>
#include <net80211/ieee80211_sample_txsched.h>

static void	sample_init(struct ieee80211vap *, uint32_t);
static void	sample_deinit(struct ieee80211vap *);
static void	sample_node_init(struct ieee80211_node *);
static void	sample_node_deinit(struct ieee80211_node *);
static int	sample_rate(struct ieee80211_node *, void *, uint32_t);
static void	sample_rates(struct ieee80211_node *, struct ieee80211_rc_info *);
static void	sample_tx_complete(const struct ieee80211vap *,
    			const struct ieee80211_node *, struct ieee80211_rc_info *);
static void	sample_tx_update(const struct ieee80211vap *vap,
			const struct ieee80211_node *, void *, void *, void *);
static void	sample_stats(const struct ieee80211vap *);
static void	sample_setinterval(const struct ieee80211vap *, int);

static void	sample_sysctlattach(struct ieee80211vap *,
			struct sysctl_ctx_list *, struct sysctl_oid *);
static void	sample_update_static_rix(struct ieee80211_node *);

/* number of references from net80211 layer */
static	int nrefs = 0;

static const struct ieee80211_ratectl sample = {
	.ir_name	= "sample",
	.ir_attach	= NULL,
	.ir_detach	= NULL,
	.ir_init	= sample_init,
	.ir_deinit	= sample_deinit,
	.ir_node_init	= sample_node_init,
	.ir_node_deinit	= sample_node_deinit,
	.ir_rate	= sample_rate,
	.ir_rates	= sample_rates,
	.ir_tx_complete	= sample_tx_complete,
	.ir_tx_update	= sample_tx_update,
	.ir_setinterval	= sample_setinterval,
	.ir_stats	= sample_stats,
};
IEEE80211_RATECTL_MODULE(sample, 1);
IEEE80211_RATECTL_ALG(sample, IEEE80211_RATECTL_SAMPLE, sample);

static void
sample_init(struct ieee80211vap *vap, uint32_t capabilities)
{
	struct ieee80211_sample *sample;

	KASSERT(vap->iv_rs == NULL, ("%s called multiple times", __func__));

	sample = vap->iv_rs = malloc(sizeof(struct ieee80211_sample),
	    M_80211_RATECTL, M_NOWAIT|M_ZERO);
	if (sample == NULL) {
		if_printf(vap->iv_ifp, "couldn't alloc ratectl structure\n");
		return;
	}

	struct ieee80211_rc_stat * irs = IEEE80211_RATECTL_STAT(vap);
	irs->irs_capabilities = capabilities;

	sample->sample_smoothing_rate = 75;		/* ewma percentage ([0..99]) */
	sample->sample_smoothing_minpackets = 100 / (100 - sample->sample_smoothing_rate);
	sample->sample_rate = 10;			/* %time to try diff tx rates */
	sample->sample_max_successive_failures = 3;	/* threshold for rate sampling*/
	sample->sample_stale_failure_timeout = 10 * hz;	/* 10 seconds */
	sample->sample_min_switch = hz;			/* 1 second */
	sample_setinterval(vap, 500 /* ms */); 	/* actually do nothing */
	sample_sysctlattach(vap, vap->iv_sysctl, vap->iv_oid);
}

static void
sample_deinit(struct ieee80211vap *vap)
{
	free(vap->iv_rs, M_80211_RATECTL);
}

static const struct txschedule *mrr_schedules[IEEE80211_MODE_MAX+2] = {
	NULL,		/* IEEE80211_MODE_AUTO */
	series_11a,	/* IEEE80211_MODE_11A */
	series_11g,	/* IEEE80211_MODE_11B */
	series_11g,	/* IEEE80211_MODE_11G */
	NULL,		/* IEEE80211_MODE_FH */
	series_11a,	/* IEEE80211_MODE_TURBO_A */
	series_11g,	/* IEEE80211_MODE_TURBO_G */
	series_11a,	/* IEEE80211_MODE_STURBO_A */
	series_11na,	/* IEEE80211_MODE_11NA */
	series_11ng,	/* IEEE80211_MODE_11NG */
	series_half,	/* IEEE80211_MODE_HALF */
	series_quarter,	/* IEEE80211_MODE_QUARTER */
};

static void
sample_node_init(struct ieee80211_node *ni)
{
#define	RATE(_ix)	(ni->ni_rates.rs_rates[(_ix)] & IEEE80211_RATE_VAL)
#define	DOT11RATE(_ix)	(rt->info[(_ix)].dot11Rate & IEEE80211_RATE_VAL)
#define	MCS(_ix)	(ni->ni_htrates.rs_rates[_ix] | IEEE80211_RATE_MCS)
	
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211_sample *sample = vap->iv_rs;
	struct ieee80211_sample_node *san = NULL;

	const struct ieee80211_rate_table *rt = NULL;
	enum ieee80211_phymode curmode = ieee80211_chan2mode(vap->iv_ic->ic_curchan);
	int x, y, rix;

	if (ni->ni_rctls == NULL) {
		ni->ni_rctls = san = malloc(sizeof(struct ieee80211_sample_node),
		    M_80211_RATECTL, M_NOWAIT|M_ZERO);
		if (san == NULL) {
			if_printf(vap->iv_ifp, "couldn't alloc per-node ratectl "
			    "structure\n");
			return;
		}
	} else
		san = ni->ni_rctls;
	
	san->san_sample = sample;

	rt = ieee80211_get_ratetable(vap->iv_ic->ic_curchan);
	KASSERT(rt != NULL, ("no rate table, mode %u", curmode));

	san->sched = mrr_schedules[curmode];
	KASSERT(san->sched != NULL, ("no mrr schedule for mode %u", curmode));

	san->static_rix = -1;
	sample_update_static_rix(ni);

	/*
	 * Construct a bitmask of usable rates.  This has all
	 * negotiated rates minus those marked by the hal as
	 * to be ignored for doing rate control.
	 */
	san->ratemask = 0;

	/* MCS rates */
	if (ni->ni_flags & IEEE80211_NODE_HT) {
		for (x = 0; x < ni->ni_htrates.rs_nrates; x++) {
			rix = rt->rateCodeToIndex[MCS(x)];
			if (rix == 0xff)
				continue;
			KASSERT(rix < SAMPLE_MAXRATES,
			    ("mcs %u has rix %d", MCS(x), rix));
			san->ratemask |= (uint64_t) 1<<rix;
		}
	}

	/* Legacy rates */
	for (x = 0; x < ni->ni_rates.rs_nrates; x++) {
		rix = rt->rateCodeToIndex[RATE(x)];
		if (rix == 0xff)
			continue;
		KASSERT(rix < SAMPLE_MAXRATES,
		    ("rate %u has rix %d", RATE(x), rix));
		san->ratemask |= (uint64_t) 1<<rix;
	}

	for (y = 0; y < NUM_PACKET_SIZE_BINS; y++) {
		int size = bin_to_size(y);
		uint64_t mask;

		san->packets_sent[y] = 0;
		san->current_sample_rix[y] = -1;
		san->last_sample_rix[y] = 0;
		/* XXX start with first valid rate */
		san->current_rix[y] = ffs(san->ratemask)-1;
		
		/*
		 * Initialize the statistics buckets; these are
		 * indexed by the rate code index.
		 */
		for (rix = 0, mask = san->ratemask; mask != 0; rix++, mask >>= 1) {
			if ((mask & 1) == 0)		/* not a valid rate */
				continue;
			san->stats[y][rix].successive_failures = 0;
			san->stats[y][rix].tries = 0;
			san->stats[y][rix].total_packets = 0;
			san->stats[y][rix].packets_acked = 0;
			san->stats[y][rix].last_tx = 0;
			san->stats[y][rix].ewma_pct = 0;
			
			san->stats[y][rix].perfect_tx_time =
			    calc_usecs_unicast_packet(vap, size, rix, 0, 0,
			    (ni->ni_chw == 40));
			san->stats[y][rix].average_tx_time =
			    san->stats[y][rix].perfect_tx_time;
		}
	}

	/* set the visible bit-rate */
	if (san->static_rix != -1)
		ni->ni_txrate = DOT11RATE(san->static_rix);
	else
		ni->ni_txrate = RATE(0);
#undef RATE
#undef DOT11RATE
#undef MCS
}

static void
sample_node_deinit(struct ieee80211_node *ni)
{
	free(ni->ni_rctls, M_80211_RATECTL);
}

static int
dot11rate(const struct ieee80211_rate_table *rt, int rix)
{
	if (rix < 0)
		return -1;
	return rt->info[rix].phy == IEEE80211_T_HT ?
	    rt->info[rix].dot11Rate : (rt->info[rix].dot11Rate & IEEE80211_RATE_VAL) / 2;
}

static const char *
dot11rate_label(const struct ieee80211_rate_table *rt, int rix)
{
	if (rix < 0)
		return "";
	return rt->info[rix].phy == IEEE80211_T_HT ? "MCS" : "Mb ";
}

/*
 * Return the rix with the lowest average_tx_time,
 * or -1 if all the average_tx_times are 0.
 */
static __inline int
pick_best_rate(const struct ieee80211_node *ni, const struct ieee80211_rate_table *rt,
    int size_bin, int require_acked_before)
{
	struct ieee80211_sample_node *san = ni->ni_rctls;
    int best_rate_rix, best_rate_tt, best_rate_pct;
	uint64_t mask;
	int rix, tt, pct;

    best_rate_rix = 0;
    best_rate_tt = 0;
	best_rate_pct = 0;

	for (mask = san->ratemask, rix = 0; mask != 0; mask >>= 1, rix++) {
		if ((mask & 1) == 0)		/* not a supported rate */
			continue;

		/* Don't pick a non-HT rate for a HT node */
		if ((ni->ni_flags & IEEE80211_NODE_HT) &&
		    (rt->info[rix].phy != IEEE80211_T_HT)) {
			continue;
		}

		tt = san->stats[size_bin][rix].average_tx_time;
		if (tt <= 0 ||
		    (require_acked_before &&
		     !san->stats[size_bin][rix].packets_acked))
			continue;

		/* Calculate percentage if possible */
		if (san->stats[size_bin][rix].total_packets > 0) {
			pct = san->stats[size_bin][rix].ewma_pct;
		} else {
			/* XXX for now, assume 95% ok */
			pct = 95;
		}

		/* don't use a bit-rate that has been failing */
		if (san->stats[size_bin][rix].successive_failures > 3)
			continue;

		/*
		 * For HT, Don't use a bit rate that is much more
		 * lossy than the best.
		 *
		 * XXX this isn't optimal; it's just designed to
		 * eliminate rates that are going to be obviously
		 * worse.
		 */
		if (ni->ni_flags & IEEE80211_NODE_HT) {
			if (best_rate_pct > (pct + 50))
				continue;
		}

		/*
		 * For non-MCS rates, use the current average txtime for
		 * comparison.
		 */
		if (! (ni->ni_flags & IEEE80211_NODE_HT)) {
			if (best_rate_tt == 0 || tt <= best_rate_tt) {
				best_rate_tt = tt;
				best_rate_rix = rix;
				best_rate_pct = pct;
			}
		}

		/*
		 * Since 2 stream rates have slightly higher TX times,
		 * allow a little bit of leeway. This should later
		 * be abstracted out and properly handled.
		 */
		if (ni->ni_flags & IEEE80211_NODE_HT) {
			if (best_rate_tt == 0 || (tt * 8 <= best_rate_tt * 10)) {
				best_rate_tt = tt;
				best_rate_rix = rix;
				best_rate_pct = pct;
			}
		}
        }
        return (best_rate_tt ? best_rate_rix : -1);
}

/*
 * Pick a good "random" bit-rate to sample other than the current one.
 */
static __inline int
pick_sample_rate(const struct ieee80211_node *ni, 
		const struct ieee80211_rate_table *rt, int size_bin)
{
#define	DOT11RATE(ix)	(rt->info[ix].dot11Rate & IEEE80211_RATE_VAL)
#define	MCS(ix)		(rt->info[ix].dot11Rate | IEEE80211_RATE_MCS)
	struct ieee80211_sample_node *san = ni->ni_rctls;
	struct ieee80211_sample *sample = san->san_sample;
	int current_rix, rix;
	unsigned current_tt;
	uint64_t mask;
	
	current_rix = san->current_rix[size_bin];
	if (current_rix < 0) {
		/* no successes yet, send at the lowest bit-rate */
		/* XXX should return MCS0 if HT */
		return 0;
	}

	current_tt = san->stats[size_bin][current_rix].average_tx_time;

	rix = san->last_sample_rix[size_bin]+1;	/* next sample rate */
	mask = san->ratemask &~ ((uint64_t) 1<<current_rix);/* don't sample current rate */
	while (mask != 0) {
		if ((mask & ((uint64_t) 1<<rix)) == 0) {	/* not a supported rate */
	nextrate:
			if (++rix >= rt->rateCount)
				rix = 0;
			continue;
		}

		/*
		 * The following code stops trying to sample
		 * non-MCS rates when speaking to an MCS node.
		 * However, at least for CCK rates in 2.4GHz mode,
		 * the non-MCS rates MAY actually provide better
		 * PER at the very far edge of reception.
		 *
		 * However! Until ath_rate_form_aggr() grows
		 * some logic to not form aggregates if the
		 * selected rate is non-MCS, this won't work.
		 *
		 * So don't disable this code until you've taught
		 * ath_rate_form_aggr() to drop out if any of
		 * the selected rates are non-MCS.
		 */
#if 1
		/* if the node is HT and the rate isn't HT, don't bother sample */
		if ((ni->ni_flags & IEEE80211_NODE_HT) &&
		    (rt->info[rix].phy != IEEE80211_T_HT)) {
			mask &= ~((uint64_t) 1<<rix);
			goto nextrate;
		}
#endif

		/* this bit-rate is always worse than the current one */
		if (san->stats[size_bin][rix].perfect_tx_time > current_tt) {
			mask &= ~((uint64_t) 1<<rix);
			goto nextrate;
		}

		/* rarely sample bit-rates that fail a lot */
		if (san->stats[size_bin][rix].successive_failures > sample->sample_max_successive_failures &&
		    ticks - san->stats[size_bin][rix].last_tx < sample->sample_stale_failure_timeout) {
			mask &= ~((uint64_t) 1<<rix);
			goto nextrate;
		}

		/*
		 * For HT, only sample a few rates on either side of the
		 * current rix; there's quite likely a lot of them.
		 */
		if (ni->ni_flags & IEEE80211_NODE_HT) {
			if (rix < (current_rix - 3) ||
			    rix > (current_rix + 3)) {
				mask &= ~((uint64_t) 1<<rix);
				goto nextrate;
			}
		}

		/* Don't sample more than 2 rates higher for rates > 11M for non-HT rates */
		if (! (ni->ni_flags & IEEE80211_NODE_HT)) {
			if (DOT11RATE(rix) > 2*11 && rix > current_rix + 2) {
				mask &= ~((uint64_t) 1<<rix);
				goto nextrate;
			}
		}

		san->last_sample_rix[size_bin] = rix;
		return rix;
	}
	return current_rix;
#undef DOT11RATE
#undef MCS
}

static int
sample_get_static_rix(const struct ieee80211_node *ni)
{
#define	RATE(_ix)	(ni->ni_rates.rs_rates[(_ix)] & IEEE80211_RATE_VAL)
#define	MCS(_ix)	(ni->ni_htrates.rs_rates[_ix] | IEEE80211_RATE_MCS)
	struct ieee80211vap *vap = ni->ni_vap;
	const struct ieee80211_rate_table *rt = ieee80211_get_ratetable(vap->iv_ic->ic_curchan);
	const struct ieee80211_txparam *tp = ni->ni_txparms;
	int srate;

	/* Check MCS rates */
	for (srate = ni->ni_htrates.rs_nrates - 1; srate >= 0; srate--) {
		if (MCS(srate) == tp->ucastrate)
			return rt->rateCodeToIndex[tp->ucastrate];
	}

	/* Check legacy rates */
	for (srate = ni->ni_rates.rs_nrates - 1; srate >= 0; srate--) {
		if (RATE(srate) == tp->ucastrate)
			return rt->rateCodeToIndex[tp->ucastrate];
	}
	return -1;
#undef	RATE
#undef	MCS
}

static void
sample_update_static_rix(struct ieee80211_node *ni)
{
	struct ieee80211_sample_node *san = ni->ni_rctls;
	const struct ieee80211_txparam *tp = ni->ni_txparms;

	if (tp != NULL && tp->ucastrate != IEEE80211_FIXED_RATE_NONE) {
		/*
		 * A fixed rate is to be used; ucastrate is the IEEE code
		 * for this rate (sans basic bit).  Check this against the
		 * negotiated rate set for the node.  Note the fixed rate
		 * may not be available for various reasons so we only
		 * setup the static rate index if the lookup is successful.
		 */
		san->static_rix = sample_get_static_rix(ni);
	} else {
		san->static_rix = -1;
	}
}

/*
 * Pick a non-HT rate to begin using.
 */
static int
sample_pick_seed_rate_legacy(const struct ieee80211_node *ni, int frameLen)
{
#define	DOT11RATE(ix)	(rt->info[ix].dot11Rate & IEEE80211_RATE_VAL)
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211_sample_node *san = ni->ni_rctls;
	const struct ieee80211_rate_table *rt = ieee80211_get_ratetable(vap->iv_ic->ic_curchan);
	
	const int size_bin = size_to_bin(frameLen);
	int rix = -1;

	/* no packet has been sent successfully yet */
	for (rix = rt->rateCount-1; rix > 0; rix--) {
		if ((san->ratemask & ((uint64_t) 1<<rix)) == 0)
			continue;

		/* Skip HT rates */
		if (rt->info[rix].phy == IEEE80211_T_HT)
			continue;

		/*
		 * Pick the highest rate <= 36 Mbps
		 * that hasn't failed.
		 */
		if (DOT11RATE(rix) <= 72 &&
		    san->stats[size_bin][rix].successive_failures == 0) {
			break;
		}
	}
	return rix;
#undef DOT11RATE
}

/*
 * Pick a HT rate to begin using.
 *
 * Don't use any non-HT rates; only consider HT rates.
 */
static int
sample_pick_seed_rate_ht(const struct ieee80211_node *ni, int frameLen)
{
#define	MCS(ix)		(rt->info[ix].dot11Rate | IEEE80211_RATE_MCS)
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211_sample_node *san = ni->ni_rctls;
	const struct ieee80211_rate_table *rt = ieee80211_get_ratetable(vap->iv_ic->ic_curchan);
	
	const int size_bin = size_to_bin(frameLen);
	int rix = -1, ht_rix = -1;

	/* no packet has been sent successfully yet */
	for (rix = rt->rateCount-1; rix > 0; rix--) {
		/* Skip rates we can't use */
		if ((san->ratemask & ((uint64_t) 1<<rix)) == 0)
			continue;

		/* Keep a copy of the last seen HT rate index */
		if (rt->info[rix].phy == IEEE80211_T_HT)
			ht_rix = rix;

		/* Skip non-HT rates */
		if (rt->info[rix].phy != IEEE80211_T_HT)
			continue;

		/*
		 * Pick a medium-speed rate regardless of stream count
		 * which has not seen any failures. Higher rates may fail;
		 * we'll try them later.
		 */
		if (((MCS(rix) & 0x7) <= 4) &&
		    san->stats[size_bin][rix].successive_failures == 0) {
			break;
		}
	}

	/*
	 * If all the MCS rates have successive failures, rix should be
	 * > 0; otherwise use the lowest MCS rix (hopefully MCS 0.)
	 */
	return MAX(rix, ht_rix);
#undef MCS
}

static int
sample_rate(struct ieee80211_node *ni, void *arg __unused, uint32_t iarg __unused)
{
#define	DOT11RATE(ix)	(rt->info[ix].dot11Rate & IEEE80211_RATE_VAL)
#define	MCS(ix)		(rt->info[ix].dot11Rate | IEEE80211_RATE_MCS)
#define	RATE(ix)	(DOT11RATE(ix) / 2)
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211_sample_node *san = ni->ni_rctls;
	struct ieee80211_sample *sample = san->san_sample;
	const struct ieee80211_rate_table *rt = ieee80211_get_ratetable(vap->iv_ic->ic_curchan);
	
	int rix, mrr, best_rix, change_rates;
	unsigned average_tx_time;
	
	size_t frameLen = (size_t)iarg;
	const int size_bin = size_to_bin(frameLen);

	sample_update_static_rix(ni);

	if (san->static_rix != -1) {
		rix = san->static_rix;
		goto done;
	}

	if (IEEE80211_RATECTL_HASCAP_MRR(vap))
		mrr = 1;
	if (! IEEE80211_RATECTL_HASCAP_MRRPROT(vap))
		mrr = 0;

	best_rix = pick_best_rate(ni, rt, size_bin, !mrr);
	if (best_rix >= 0) {
		average_tx_time = san->stats[size_bin][best_rix].average_tx_time;
	} else {
		average_tx_time = 0;
	}

	/*
	 * Limit the time measuring the performance of other tx
	 * rates to sample_rate% of the total transmission time.
	 */
	if (san->sample_tt[size_bin] < average_tx_time * (san->packets_since_sample[size_bin] *
		sample->sample_rate/100)) {
		rix = pick_sample_rate(ni, rt, size_bin);
		IEEE80211_NOTE(vap, IEEE80211_MSG_RATECTL,
		     ni, "att %d sample_tt %d size %u sample rate %d %s current rate %d %s",
		     average_tx_time,
		     san->sample_tt[size_bin],
		     bin_to_size(size_bin),
		     dot11rate(rt, rix),
		     dot11rate_label(rt, rix),
		     dot11rate(rt, san->current_rix[size_bin]),
		     dot11rate_label(rt, san->current_rix[size_bin]));
		if (rix != san->current_rix[size_bin]) {
			san->current_sample_rix[size_bin] = rix;
		} else {
			san->current_sample_rix[size_bin] = -1;
		}
		san->packets_since_sample[size_bin] = 0;
	} else {
		change_rates = 0;
		if (!san->packets_sent[size_bin] || best_rix == -1) {
			/* no packet has been sent successfully yet */
			change_rates = 1;
			if (ni->ni_flags & IEEE80211_NODE_HT)
				best_rix = sample_pick_seed_rate_ht(ni, frameLen);
			else
				best_rix = sample_pick_seed_rate_legacy(ni, frameLen);
		} else if (san->packets_sent[size_bin] < 20) {
			/* let the bit-rate switch quickly during the first few packets */
			IEEE80211_NOTE(vap,
			    IEEE80211_MSG_RATECTL, ni,
			    "%s: switching quickly..", __func__);
			change_rates = 1;
		} else if (ticks - sample->sample_min_switch > san->ticks_since_switch[size_bin]) {
			/* min_switch seconds have gone by */
			IEEE80211_NOTE(vap,
			    IEEE80211_MSG_RATECTL, ni,
			    "%s: min_switch %d > ticks_since_switch %d..",
			    __func__, ticks - sample->sample_min_switch, san->ticks_since_switch[size_bin]);
			change_rates = 1;
		} else if ((! (ni->ni_flags & IEEE80211_NODE_HT)) &&
		    (2*average_tx_time < san->stats[size_bin][san->current_rix[size_bin]].average_tx_time)) {
			/* the current bit-rate is twice as slow as the best one */
			IEEE80211_NOTE(vap,
			    IEEE80211_MSG_RATECTL, ni,
			    "%s: 2x att (= %d) < cur_rix att %d",
			    __func__,
			    2 * average_tx_time, san->stats[size_bin][san->current_rix[size_bin]].average_tx_time);
			change_rates = 1;
		} else if ((ni->ni_flags & IEEE80211_NODE_HT)) {
			int cur_rix = san->current_rix[size_bin];
			int cur_att = san->stats[size_bin][cur_rix].average_tx_time;
			/*
			 * If the node is HT, upgrade it if the MCS rate is
			 * higher and the average tx time is within 20% of
			 * the current rate. It can fail a little.
			 *
			 * This is likely not optimal!
			 */
#if 0
			printf("cur rix/att %x/%d, best rix/att %x/%d\n",
			    MCS(cur_rix), cur_att, MCS(best_rix), average_tx_time);
#endif
			if ((MCS(best_rix) > MCS(cur_rix)) &&
			    (average_tx_time * 8) <= (cur_att * 10)) {
				IEEE80211_NOTE(vap,
				    IEEE80211_MSG_RATECTL, ni,
				    "%s: HT: best_rix 0x%d > cur_rix 0x%x, average_tx_time %d, cur_att %d",
				    __func__,
				    MCS(best_rix), MCS(cur_rix), average_tx_time, cur_att);
				change_rates = 1;
			}
		}

		san->packets_since_sample[size_bin]++;
		
		if (change_rates) {
			if (best_rix != san->current_rix[size_bin]) {
				IEEE80211_NOTE(vap,
				    IEEE80211_MSG_RATECTL,
				    ni,
"%s: size %d switch rate %d (%d/%d) -> %d (%d/%d) after %d packets mrr %d",
				    __func__,
				    bin_to_size(size_bin),
				    RATE(san->current_rix[size_bin]),
				    san->stats[size_bin][san->current_rix[size_bin]].average_tx_time,
				    san->stats[size_bin][san->current_rix[size_bin]].perfect_tx_time,
				    RATE(best_rix),
				    san->stats[size_bin][best_rix].average_tx_time,
				    san->stats[size_bin][best_rix].perfect_tx_time,
				    san->packets_since_switch[size_bin],
				    mrr);
			}
			san->packets_since_switch[size_bin] = 0;
			san->current_rix[size_bin] = best_rix;
			san->ticks_since_switch[size_bin] = ticks;
			/* 
			 * Set the visible txrate for this node.
			 */
			ni->ni_txrate = dot11rate(rt, best_rix);
		}
		rix = san->current_rix[size_bin];
		san->packets_since_switch[size_bin]++;
	}
	// *try0 = mrr ? san->sched[rix].t0 : ATH_TXMAXTRY;
done:

	/*
	 * This bug totally sucks and should be fixed.
	 *
	 * For now though, let's not panic, so we can start to figure
	 * out how to better reproduce it.
	 */
	if (rix < 0 || rix >= rt->rateCount) {
		printf("%s: ERROR: rix %d out of bounds (rateCount=%d)\n",
		    __func__,
		    rix,
		    rt->rateCount);
		    rix = 0;	/* XXX just default for now */
	}
	KASSERT(rix >= 0 && rix < rt->rateCount, ("rix is %d", rix));

	// *rix0 = rix;
	// *txrate = rt->info[rix].rateCode
	// 	| (shortPreamble ? rt->info[rix].shortPreamble : 0);
	san->packets_sent[size_bin]++;

	return rix;
#undef DOT11RATE
#undef MCS
#undef RATE
}

static void
sample_rates(struct ieee80211_node *ni, struct ieee80211_rc_info *rc_info)
{
	struct ieee80211_sample_node *san = ni->ni_rctls;
	uint8_t rix0 = sample_rate(ni, NULL, 0);
	const struct txschedule *sched = &san->sched[rix0];
	struct ieee80211_rc_series *rc = rc_info->iri_rc;

	KASSERT(rix0 == sched->r0, ("rix0 (%x) != sched->r0 (%x)!\n",
	    rix0, sched->r0));
	/* XXX */
	rc[0].flags = rc[1].flags = rc[2].flags = rc[3].flags = 0;

	rc[0].rix = sched->r0;
	rc[1].rix = sched->r1;
	rc[2].rix = sched->r2;
	rc[3].rix = sched->r3;

	rc[0].tries = sched->t0;
	rc[1].tries = sched->t1;
	rc[2].tries = sched->t2;
	rc[3].tries = sched->t3;
}

static void
update_stats(const struct ieee80211vap *vap,
    	  const struct ieee80211_node *ni,
		  int frame_size,
		  int rix0, int tries0,
		  int rix1, int tries1,
		  int rix2, int tries2,
		  int rix3, int tries3,
		  int short_tries, int tries,
		  int nframes, int nbad)
{
	struct ieee80211_sample_node *san = ni->ni_rctls;
	struct ieee80211_sample *sample = san->san_sample;

	const int size_bin = size_to_bin(frame_size);
	const int size = bin_to_size(size_bin);

	int is_ht40 = ieee80211_ratectl_hascap_cw40(ni);
	int tt, tries_so_far;
	int pct;

	if (!IS_RATE_DEFINED(san, rix0))
		return;
	tt = calc_usecs_unicast_packet(vap, size, rix0, short_tries,
		MIN(tries0, tries) - 1, is_ht40);
	tries_so_far = tries0;

	if (tries1 && tries_so_far < tries) {
		if (!IS_RATE_DEFINED(san, rix1))
			return;
		tt += calc_usecs_unicast_packet(vap, size, rix1, short_tries,
			MIN(tries1 + tries_so_far, tries) - tries_so_far - 1, is_ht40);
		tries_so_far += tries1;
	}

	if (tries2 && tries_so_far < tries) {
		if (!IS_RATE_DEFINED(san, rix2))
			return;
		tt += calc_usecs_unicast_packet(vap, size, rix2, short_tries,
			MIN(tries2 + tries_so_far, tries) - tries_so_far - 1, is_ht40);
		tries_so_far += tries2;
	}

	if (tries3 && tries_so_far < tries) {
		if (!IS_RATE_DEFINED(san, rix3))
			return;
		tt += calc_usecs_unicast_packet(vap, size, rix3, short_tries,
			MIN(tries3 + tries_so_far, tries) - tries_so_far - 1, is_ht40);
	}

	if (san->stats[size_bin][rix0].total_packets < sample->sample_smoothing_minpackets) {
		/* just average the first few packets */
		int avg_tx = san->stats[size_bin][rix0].average_tx_time;
		int packets = san->stats[size_bin][rix0].total_packets;
		san->stats[size_bin][rix0].average_tx_time = (tt+(avg_tx*packets))/(packets+nframes);
	} else {
		/* use a ewma */
		san->stats[size_bin][rix0].average_tx_time = 
			((san->stats[size_bin][rix0].average_tx_time * sample->sample_smoothing_rate) + 
			 (tt * (100 - sample->sample_smoothing_rate))) / 100;
	}
	
	/*
	 * XXX Don't mark the higher bit rates as also having failed; as this
	 * unfortunately stops those rates from being tasted when trying to
	 * TX. This happens with 11n aggregation.
	 */
	if (nframes == nbad) {
		san->stats[size_bin][rix0].successive_failures += nbad;

	} else {
		san->stats[size_bin][rix0].packets_acked += (nframes - nbad);
		san->stats[size_bin][rix0].successive_failures = 0;
	}
	san->stats[size_bin][rix0].tries += tries;
	san->stats[size_bin][rix0].last_tx = ticks;
	san->stats[size_bin][rix0].total_packets += nframes;

	/* update EWMA for this rix */

	/* Calculate percentage based on current rate */
	if (nframes == 0)
		nframes = nbad = 1;
	pct = ((nframes - nbad) * 1000) / nframes;

	if (san->stats[size_bin][rix0].total_packets <
	    sample->sample_smoothing_minpackets) {
		/* just average the first few packets */
		int a_pct = (san->stats[size_bin][rix0].packets_acked * 1000) /
		    (san->stats[size_bin][rix0].total_packets);
		san->stats[size_bin][rix0].ewma_pct = a_pct;
	} else {
		/* use a ewma */
		san->stats[size_bin][rix0].ewma_pct =
			((san->stats[size_bin][rix0].ewma_pct * sample->sample_smoothing_rate) +
			 (pct * (100 - sample->sample_smoothing_rate))) / 100;
	}

	if (rix0 == san->current_sample_rix[size_bin]) {
		san->sample_tt[size_bin] = tt;
		san->current_sample_rix[size_bin] = -1;
	}
}

static void
sample_tx_complete(const struct ieee80211vap *vap,
    const struct ieee80211_node *ni, struct ieee80211_rc_info *rc_info)
{
	struct ieee80211_sample_node *san = ni->ni_rctls;
	const struct ieee80211_rate_table *rt = NULL;
	struct ieee80211_rc_series *rc = NULL;
	int final_rix, short_tries, long_tries;
	int nframes, nbad;
	int frame_size, mrr;

	rt = ieee80211_get_ratetable(vap->iv_ic->ic_curchan);
	rc = rc_info->iri_rc;	
	/* update per vap statistics */
	ieee80211_ratectl_update_stat(vap, rc_info);

	final_rix = rt->rateCodeToIndex[rc_info->iri_txrate];
	short_tries = rc_info->iri_shortretry;
	long_tries = rc_info->iri_longretry + 1;

	nframes = rc_info->iri_txcnt;
	nbad = rc_info->iri_failcnt;

	frame_size = rc_info->iri_framelen;
	mrr = 0;

	if (nframes == 0) {
		/* XXX need some msg out */
		return;
	}

	if (frame_size == 0)		    /* NB: should not happen */
		frame_size = 1500;

	if (san->ratemask == 0) {
		return;
	}
	
	if (IEEE80211_RATECTL_HASCAP_MRR(vap))
		mrr = 1;
	/* XXX check HT protmode too */
	if (mrr && !IEEE80211_RATECTL_HASCAP_MRRPROT(vap))
		mrr = 0;
	
	if (!mrr || rc_info->iri_finaltsi == 0) {
		if (!IS_RATE_DEFINED(san, final_rix)) {
			return;
		}
		/*
		 * Only one rate was used; optimize work.
		 */
		update_stats(vap, ni, frame_size, 
			     final_rix, long_tries,
			     0, 0,
			     0, 0,
			     0, 0,
			     short_tries, long_tries,
			     nframes, nbad);

	} else {
		int finalTSIdx = rc_info->iri_finaltsi;

		/*
		 * NB: series > 0 are not penalized for failure
		 * based on the try counts under the assumption
		 * that losses are often bursty and since we
		 * sample higher rates 1 try at a time doing so
		 * may unfairly penalize them.
		 */
		if (rc[0].tries) {
			update_stats(vap, ni, frame_size,
				     rc[0].rix, rc[0].tries,
				     rc[1].rix, rc[1].tries,
				     rc[2].rix, rc[2].tries,
				     rc[3].rix, rc[3].tries,
				     short_tries, long_tries,
				     nframes, nbad);
			long_tries -= rc[0].tries;
		}
		
		if (rc[1].tries && finalTSIdx > 0) {
			update_stats(vap, ni, frame_size,
				     rc[1].rix, rc[1].tries,
				     rc[2].rix, rc[2].tries,
				     rc[3].rix, rc[3].tries,
				     0, 0,
				     short_tries, long_tries,
				     nframes, nbad);
			long_tries -= rc[1].tries;
		}

		if (rc[2].tries && finalTSIdx > 1) {
			update_stats(vap, ni, frame_size,
				     rc[2].rix, rc[2].tries,
				     rc[3].rix, rc[3].tries,
				     0, 0,
				     0, 0,
				     short_tries, long_tries,
				     nframes, nbad);
			long_tries -= rc[2].tries;
		}

		if (rc[3].tries && finalTSIdx > 2) {
			update_stats(vap, ni, frame_size,
				     rc[3].rix, rc[3].tries,
				     0, 0,
				     0, 0,
				     0, 0,
				     short_tries, long_tries,
				     nframes, nbad);
		}
	}
}

static void
sample_tx_update(const struct ieee80211vap *vap, const struct ieee80211_node *ni,
    void *arg1, void *arg2, void *arg3)
{
	/* nothing here. */
}

static void
sample_setinterval(const struct ieee80211vap *vap, int msecs)
{
	/* ieee80211_sample doesn't have the sample_interval field by now */
}

static void
sample_stats_node(void *arg, struct ieee80211_node *ni)
{
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211_sample_node *san = ni->ni_rctls;
	const struct ieee80211_rate_table *rt = ieee80211_get_ratetable(vap->iv_ic->ic_curchan);
	uint64_t mask;
	int rix, y;

	printf("\n[%s] refcnt %d static_rix (%d %s) ratemask 0x%jx\n",
	    ether_sprintf(ni->ni_macaddr), ieee80211_node_refcnt(ni),
	    dot11rate(rt, san->static_rix),
	    dot11rate_label(rt, san->static_rix),
	    (uintmax_t)san->ratemask);
	for (y = 0; y < NUM_PACKET_SIZE_BINS; y++) {
		printf("[%4u] cur rix %d (%d %s) since switch: packets %d ticks %u\n",
		    bin_to_size(y), san->current_rix[y],
		    dot11rate(rt, san->current_rix[y]),
		    dot11rate_label(rt, san->current_rix[y]),
		    san->packets_since_switch[y], san->ticks_since_switch[y]);
		printf("[%4u] last sample (%d %s) cur sample (%d %s) packets sent %d\n",
		    bin_to_size(y),
		    dot11rate(rt, san->last_sample_rix[y]),
		    dot11rate_label(rt, san->last_sample_rix[y]),
		    dot11rate(rt, san->current_sample_rix[y]),
		    dot11rate_label(rt, san->current_sample_rix[y]),
		    san->packets_sent[y]);
		printf("[%4u] packets since sample %d sample tt %u\n",
		    bin_to_size(y), san->packets_since_sample[y],
		    san->sample_tt[y]);
	}
	for (mask = san->ratemask, rix = 0; mask != 0; mask >>= 1, rix++) {
		if ((mask & 1) == 0)
				continue;
		for (y = 0; y < NUM_PACKET_SIZE_BINS; y++) {
			if (san->stats[y][rix].total_packets == 0)
				continue;
			printf("[%2u %s:%4u] %8ju:%-8ju (%3d%%) (EWMA %3d.%1d%%) T %8ju F %4d avg %5u last %u\n",
			    dot11rate(rt, rix), dot11rate_label(rt, rix),
			    bin_to_size(y),
			    (uintmax_t) san->stats[y][rix].total_packets,
			    (uintmax_t) san->stats[y][rix].packets_acked,
			    (int) ((san->stats[y][rix].packets_acked * 100ULL) /
			     san->stats[y][rix].total_packets),
			    san->stats[y][rix].ewma_pct / 10,
			    san->stats[y][rix].ewma_pct % 10,
			    (uintmax_t) san->stats[y][rix].tries,
			    san->stats[y][rix].successive_failures,
			    san->stats[y][rix].average_tx_time,
			    ticks - san->stats[y][rix].last_tx);
		}
	}
}
static void
sample_stats(const struct ieee80211vap *vap)
{
	struct ieee80211com *ic = vap->iv_ifp->if_l2com;
	struct ieee80211_rc_stat * irs = IEEE80211_RATECTL_STAT(vap);
	printf("tx count: %d (ok count: %d, fail count: %d)\n"
			"retry count: %d (short retry: %d, long retry: %d)\n",
			irs->irs_txcnt, irs->irs_txcnt - irs->irs_failcnt, irs->irs_failcnt,
			irs->irs_retrycnt, irs->irs_shortretry, irs->irs_longretry);
	ieee80211_iterate_nodes(&ic->ic_sta, sample_stats_node, NULL);
}

static int
sample_sysctl_smoothing_rate(SYSCTL_HANDLER_ARGS)
{
	struct ieee80211vap *vap = arg1;
	struct ieee80211_sample *sample = vap->iv_rs;
	int rate, error;

	rate = sample->sample_smoothing_rate;
	error = sysctl_handle_int(oidp, &rate, 0, req);
	if (error || !req->newptr)
		return error;
	if (!(0 <= rate && rate < 100))
		return EINVAL;
	sample->sample_smoothing_rate = rate;
	sample->sample_smoothing_minpackets = 100 / (100 - rate);
	return 0;
}

static int
sample_sysctl_sample_rate(SYSCTL_HANDLER_ARGS)
{
	struct ieee80211vap *vap = arg1;
	struct ieee80211_sample *sample = vap->iv_rs;
	int rate, error;

	rate = sample->sample_rate;
	error = sysctl_handle_int(oidp, &rate, 0, req);
	if (error || !req->newptr)
		return error;
	if (!(2 <= rate && rate <= 100))
		return EINVAL;
	sample->sample_rate = rate;
	return 0;
}

static void
sample_sysctlattach(struct ieee80211vap *vap,
    struct sysctl_ctx_list *ctx, struct sysctl_oid *tree)
{
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "sample_smoothing_rate", CTLTYPE_INT | CTLFLAG_RW, vap, 0,
	    sample_sysctl_smoothing_rate, "I",
	    "sample: smoothing rate for avg tx time (%%)");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "sample_rate", CTLTYPE_INT | CTLFLAG_RW, vap, 0,
	    sample_sysctl_sample_rate, "I",
	    "sample: percent air time devoted to sampling new rates (%%)");
}
