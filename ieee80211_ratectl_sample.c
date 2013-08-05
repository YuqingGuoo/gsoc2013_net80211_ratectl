/*-
 * Copyright (c) 2010 Bernhard Schmidt <bschmidt@FreeBSD.org>
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

#ifdef INET
#include <netinet/in.h>
#include <netinet/if_ether.h>
#endif

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_ht.h>
#include <net80211/ieee80211_ratectl.h>
#include <net80211/ieee80211_ratectl_sample.h>

static void	sample_init(struct ieee80211vap *);
static void	sample_deinit(struct ieee80211vap *);
static void	sample_node_init(struct ieee80211_node *);
static void	sample_node_deinit(struct ieee80211_node *);
static int	sample_rate(struct ieee80211_node *, void *, uint32_t);
static void	sample_tx_complete(const struct ieee80211vap *,
    			const struct ieee80211_node *, int, 
			void *, void *);
static void	sample_tx_update(const struct ieee80211vap *vap,
			const struct ieee80211_node *, void *, void *, void *);
static void	sample_setinterval(const struct ieee80211vap *, int);

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
	.ir_rates	= NULL,
	.ir_tx_complete	= sample_tx_complete,
	.ir_tx_update	= sample_tx_update,
	.ir_setinterval	= sample_setinterval,
};
IEEE80211_RATECTL_MODULE(sample, 1);
IEEE80211_RATECTL_ALG(sample, IEEE80211_RATECTL_SAMPLE, sample);

#define	DOT11RATE(ix)	(rs->info[ix].dot11Rate & IEEE80211_RATE_VAL)
#define	MCS(ix)		(rs->info[ix].dot11Rate | IEEE80211_RATE_MCS)
#define	RATE(ix)	(DOT11RATE(ix) / 2)

static void
sample_init(struct ieee80211vap *vap)
{
	struct ieee80211_sample *sample;

	KASSERT(vap->iv_rs == NULL, ("%s called multiple times", __func__));

	sample = vap->iv_rs = malloc(sizeof(struct ieee80211_sample),
	    M_80211_RATECTL, M_NOWAIT|M_ZERO);
	if (sample == NULL) {
		if_printf(vap->iv_ifp, "couldn't alloc ratectl structure\n");
		return;
	}
	sample->sample_smoothing_rate = 75;		/* ewma percentage ([0..99]) */
	sample->sample_smoothing_minpackets = 100 / (100 - sample->sample_smoothing_rate);
	sample->sample_rate = 10;			/* %time to try diff tx rates */
	sample->sample_max_successive_failures = 3;	/* threshold for rate sampling*/
	sample->sample_stale_failure_timeout = 10 * hz;	/* 10 seconds */
	sample->sample_min_switch = hz;			/* 1 second */
	sample_setinterval(vap, 500 /* ms */); 	/* actually set nothing */
	sample_sysctlattach(vap, vap->iv_sysctl, vap->iv_oid);
}

static void
sample_deinit(struct ieee80211vap *vap)
{
	free(vap->iv_rs, M_80211_RATECTL);
}

// XXX should be shared by ratectl algos
static int
sample_node_is_11n(struct ieee80211_node *ni)
{

	if (ni->ni_chan == NULL)
		return (0);
	if (ni->ni_chan == IEEE80211_CHAN_ANYC)
		return (0);
	return (IEEE80211_IS_CHAN_HT(ni->ni_chan));
}

static void
sample_node_init(struct ieee80211_node *ni)
{
	const struct ieee80211_rateset *rs = NULL;
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211_sample *sample = vap->iv_rs;
	struct ieee80211_sample_node *san;
	uint8_t rate;

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

// XXX not done yet

	san->san_success = 0;
	san->san_recovery = 0;
	san->san_txcnt = san->san_retrycnt = 0;
	san->san_success_threshold = sample->sample_min_success_threshold;

// XXX not done yet
	ni->ni_txrate = ni->ni_rates.rs_rates[0] & IEEE80211_RATE_VAL;
}

static void
sample_node_deinit(struct ieee80211_node *ni)
{
	free(ni->ni_rctls, M_80211_RATECTL);
}

static int
dot11rate(const ieee80211_rateset *rs, int rix)
{
	if (rix < 0)
		return -1;
	return rs->info[rix].phy == IEEE80211_T_HT ?
	    rs->info[rix].dot11Rate : (rs->info[rix].dot11Rate & IEEE80211_RATE_VAL) / 2;
}

static const char *
dot11rate_label(const ieee80211_rateset *rs, int rix)
{
	if (rix < 0)
		return "";
	return rs->info[rix].phy == IEEE80211_T_HT ? "MCS" : "Mb ";
}

/*
 * Return the rix with the lowest average_tx_time,
 * or -1 if all the average_tx_times are 0.
 */
static __inline int
pick_best_rate(const struct ieee80211_node *ni, const struct ieee80211_rateset *rs,
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
		if ((ni.ni_flags & IEEE80211_NODE_HT) &&
		    (rs->info[rix].phy != IEEE80211_T_HT)) {
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
		if (ni.ni_flags & IEEE80211_NODE_HT) {
			if (best_rate_pct > (pct + 50))
				continue;
		}

		/*
		 * For non-MCS rates, use the current average txtime for
		 * comparison.
		 */
		if (! (ni.ni_flags & IEEE80211_NODE_HT)) {
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
		if (ni.ni_flags & IEEE80211_NODE_HT) {
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
pick_sample_rate(const struct ieee80211_node *ni, const ieee80211_rateset *rs, 
	int size_bin)
{
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
			if (++rix >= rs->rateCount)
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
		if ((ni.ni_flags & IEEE80211_NODE_HT) &&
		    (rs->info[rix].phy != IEEE80211_T_HT)) {
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
		if (ni.ni_flags & IEEE80211_NODE_HT) {
			if (rix < (current_rix - 3) ||
			    rix > (current_rix + 3)) {
				mask &= ~((uint64_t) 1<<rix);
				goto nextrate;
			}
		}

		/* Don't sample more than 2 rates higher for rates > 11M for non-HT rates */
		if (! (ni.ni_flags & IEEE80211_NODE_HT)) {
			if (DOT11RATE(rix) > 2*11 && rix > current_rix + 2) {
				mask &= ~((uint64_t) 1<<rix);
				goto nextrate;
			}
		}

		san->last_sample_rix[size_bin] = rix;
		return rix;
	}
	return current_rix;
}

static int
sample_get_static_rix(const struct ieee80211_node *ni)
{
	const struct ieee80211_txparam *tp = ni->ni_txparms;
	const struct ieee80211_rateset *rs = sample_get_rateset(ni);
	
	return rs->rateCodeToIndex[tp->ucastrate];
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
	struct ieee80211_sample_node *san = ni->ni_rctls;
	const struct ieee80211_rateset *rs = &ni->ni_rates;
	
	const int size_bin = size_to_bin(frameLen);
	int rix = -1;

	/* no packet has been sent successfully yet */
	for (rix = rs->rateCount-1; rix > 0; rix--) {
		if ((san->ratemask & ((uint64_t) 1<<rix)) == 0)
			continue;

		/* Skip HT rates */
		if (rs->info[rix].phy == IEEE80211_T_HT)
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
}

/*
 * Pick a HT rate to begin using.
 *
 * Don't use any non-HT rates; only consider HT rates.
 */
static int
sample_pick_seed_rate_ht(const struct ieee80211_node *ni, int frameLen)
{
	struct ieee80211_sample_node *san = ni->ni_rctls;
	const struct ieee80211_rateset *rs = (struct ieee80211_rateset *) &ni->ni_htrates;
	
	const int size_bin = size_to_bin(frameLen);
	int rix = -1, ht_rix = -1;

	/* no packet has been sent successfully yet */
	for (rix = rs->rateCount-1; rix > 0; rix--) {
		/* Skip rates we can't use */
		if ((san->ratemask & ((uint64_t) 1<<rix)) == 0)
			continue;

		/* Keep a copy of the last seen HT rate index */
		if (rs->info[rix].phy == IEEE80211_T_HT)
			ht_rix = rix;

		/* Skip non-HT rates */
		if (rs->info[rix].phy != IEEE80211_T_HT)
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
}

static const struct ieee80211_rateset *
sample_get_rateset(const struct ieee80211_node *ni)
{
	const struct ieee80211_rateset *rs = NULL;
	/* 11n or not? Pick the right rateset */
	if (sample_node_is_11n(ni)) {
		/* XXX ew */
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

static int
sample_rate(struct ieee80211_node *ni, void *arg __unused, uint32_t iarg __unused)
{
	struct ieee80211_sample_node *san = ni->ni_rctls;
	struct ieee80211_sample *sample = san->san_sample;
	size_t frameLen = (size_t)iarg;

	const struct ieee80211vap *vap = ni->ni_vap;
	struct ifnet *ifp = vap->iv_ifp;
	struct ieee80211com *ic = ifp->if_l2com;

	int rix, mrr, best_rix, change_rates;
	unsigned average_tx_time;
	
	const struct ieee80211_rateset *rs = sample_get_rateset(ni);

	const int size_bin = size_to_bin(frameLen);

	sample_update_static_rix(ni);


	if (san->static_rix != -1) {
		rix = sn->static_rix;
		goto done;
	}

	if (vap->iv_rate->ir_capabilities & IEEE80211_RATECTL_CAP_MRR)
		mrr = 1;
	if (!(vap->iv_rate->ir_capabilities & IEEE80211_RATECTL_CAP_MRRPROT))
		mrr = 0;

	// XXX not done here.
	best_rix = pick_best_rate(ni, rs, size_bin, !mrr);
	if (best_rix >= 0) {
		average_tx_time = san->stats[size_bin][best_rix].average_tx_time;
	} else {
		average_tx_time = 0;
	}

	/*
	 * Limit the time measuring the performance of other tx
	 * rates to sample_rate% of the total transmission time.
	 */
	if (san->sample_tt[size_bin] < average_tx_time * (san->packets_since_sample[size_bin]*sample->sample_rate/100)) {
		// XXX not done here.
		rix = pick_sample_rate(ni, rs, size_bin);
		// XXX
		IEEE80211_NOTE(ni.ni_vap, IEEE80211_MSG_RATECTL,
		     &ni, "att %d sample_tt %d size %u sample rate %d %s current rate %d %s",
		     average_tx_time,
		     san->sample_tt[size_bin],
		     bin_to_size(size_bin),
		     dot11rate(rs, rix),
		     dot11rate_label(rs, rix),
		     dot11rate(rs, san->current_rix[size_bin]),
		     dot11rate_label(rs, san->current_rix[size_bin]));
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
			if (ni.ni_flags & IEEE80211_NODE_HT)
				best_rix = //XXX
				    sample_pick_seed_rate_ht(ni, frameLen);
			else
				best_rix = // XXX
				    sample_pick_seed_rate_legacy(ni, frameLen);
		} else if (san->packets_sent[size_bin] < 20) {
			/* let the bit-rate switch quickly during the first few packets */
			IEEE80211_NOTE(ni.ni_vap,
			    IEEE80211_MSG_RATECTL, &ni,
			    "%s: switching quickly..", __func__);
			change_rates = 1;
		} else if (ticks - sample->sample_min_switch > san->ticks_since_switch[size_bin]) {
			/* min_switch seconds have gone by */
			// XXX
			IEEE80211_NOTE(vap,
			    IEEE80211_MSG_RATECTL, &ni,
			    "%s: min_switch %d > ticks_since_switch %d..",
			    __func__, ticks - sample->sample_min_switch, san->ticks_since_switch[size_bin]);
			change_rates = 1;
		} else if ((! (ni.ni_flags & IEEE80211_NODE_HT)) &&
		    (2*average_tx_time < san->stats[size_bin][san->current_rix[size_bin]].average_tx_time)) {
			/* the current bit-rate is twice as slow as the best one */
			// XXX
			IEEE80211_NOTE(vap,
			    IEEE80211_MSG_RATECTL, &ni,
			    "%s: 2x att (= %d) < cur_rix att %d",
			    __func__,
			    2 * average_tx_time, san->stats[size_bin][san->current_rix[size_bin]].average_tx_time);
			change_rates = 1;
		} else if ((ni.ni_flags & IEEE80211_NODE_HT)) {
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
				    IEEE80211_MSG_RATECTL, &ni,
				    "%s: HT: best_rix 0x%d > cur_rix 0x%x, average_tx_time %d, cur_att %d",
				    __func__,
				    MCS(best_rix), MCS(cur_rix), average_tx_time, cur_att);
				change_rates = 1;
			}
		}

		san->packets_since_sample[size_bin]++;
		
		if (change_rates) {
			if (best_rix != san->current_rix[size_bin]) {
				// XXX
				IEEE80211_NOTE(vap,
				    IEEE80211_MSG_RATECTL,
				    &ni,
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
			ni.ni_txrate = (rs->info[best_rix].phy == IEEE80211_T_HT) ?  MCS(best_rix) : DOT11RATE(best_rix);
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
	if (rix < 0 || rix >= rs->rateCount) {
		printf("%s: ERROR: rix %d out of bounds (rateCount=%d)\n",
		    __func__,
		    rix,
		    rs->rateCount);
		    rix = 0;	/* XXX just default for now */
	}
	KASSERT(rix >= 0 && rix < rs->rateCount, ("rix is %d", rix));

	// *rix0 = rix;
	// *txrate = rs->info[rix].rateCode
	// 	| (shortPreamble ? rs->info[rix].shortPreamble : 0);
	san->packets_sent[size_bin]++;

	return rix;
}

static void
sample_tx_complete(const struct ieee80211vap *vap,
    const struct ieee80211_node *ni, int ok,
    void *arg1, void *arg2 __unused)
{
}

static void
sample_tx_update(const struct ieee80211vap *vap, const struct ieee80211_node *ni,
    void *arg1, void *arg2, void *arg3)
{
}

static void
sample_setinterval(const struct ieee80211vap *vap, int msecs)
{
	struct ieee80211_sample *sample = vap->iv_rs;
	int t;

	if (msecs < 100)
		msecs = 100;
	t = msecs_to_ticks(msecs);
	/* ieee80211_sample doesn't have the sample_interval field by now */
	// sample->sample_interval = (t < 1) ? 1 : t;
}

static void
sample_stats(void *arg, struct ieee80211_node *ni)
{
	struct ieee80211_sample_node *san = ni->ni_rctls;
	// XXX sc->sc_currates
	const ieee80211_rateset *rs = sample_get_rateset(ni); 
	uint64_t mask;
	int rix, y;

	printf("\n[%s] refcnt %d static_rix (%d %s) ratemask 0x%jx\n",
	    ether_sprintf(ni->ni_macaddr), ieee80211_node_refcnt(ni),
	    dot11rate(rs, san->static_rix),
	    dot11rate_label(rs, san->static_rix),
	    (uintmax_t)san->ratemask);
	for (y = 0; y < NUM_PACKET_SIZE_BINS; y++) {
		printf("[%4u] cur rix %d (%d %s) since switch: packets %d ticks %u\n",
		    bin_to_size(y), san->current_rix[y],
		    dot11rate(rs, san->current_rix[y]),
		    dot11rate_label(rs, san->current_rix[y]),
		    san->packets_since_switch[y], san->ticks_since_switch[y]);
		printf("[%4u] last sample (%d %s) cur sample (%d %s) packets sent %d\n",
		    bin_to_size(y),
		    dot11rate(rs, san->last_sample_rix[y]),
		    dot11rate_label(rs, san->last_sample_rix[y]),
		    dot11rate(rs, san->current_sample_rix[y]),
		    dot11rate_label(rs, san->current_sample_rix[y]),
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
			    dot11rate(rs, rix), dot11rate_label(rs, rix),
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

static int
sample_sysctl_stats(SYSCTL_HANDLER_ARGS)
{
	struct ieee80211vap *vap = arg1;
	struct ieee80211com *ic = vap->iv_ifp->if_l2com;
	int error, v;

	v = 0;
	error = sysctl_handle_int(oidp, &v, 0, req);
	if (error || !req->newptr)
		return error;
	ieee80211_iterate_nodes(&ic->ic_sta, sample_stats, NULL);
	return 0;
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
	/* XXX max_successive_failures, stale_failure_timeout, min_switch */
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "sample_stats", CTLTYPE_INT | CTLFLAG_RW, vap, 0,
	    sample_sysctl_stats, "I", "sample: print statistics");
}

#undef DOT11RATE
#undef MCS
#undef RATE