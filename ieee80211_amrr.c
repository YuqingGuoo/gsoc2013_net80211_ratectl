/*	$OpenBSD: ieee80211_amrr.c,v 1.1 2006/06/17 19:07:19 damien Exp $	*/

/*-
 * Copyright (c) 2010 Rui Paulo <rpaulo@FreeBSD.org>
 * Copyright (c) 2006
 *	Damien Bergamini <damien.bergamini@free.fr>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: soc2013/ccqin/head/sys/net80211/ieee80211_amrr.c 257398 2013-09-16 08:08:52Z ccqin $");

/*-
 * Naive implementation of the Adaptive Multi Rate Retry algorithm:
 *
 * "IEEE 802.11 Rate Adaptation: A Practical Approach"
 *  Mathieu Lacage, Hossein Manshaei, Thierry Turletti
 *  INRIA Sophia - Projet Planete
 *  http://www-sop.inria.fr/rapports/sophia/RR-5208.html
 */
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
#include <net80211/ieee80211_amrr.h>

#define is_success(amn)	\
	((amn)->amn_retrycnt < (amn)->amn_txcnt / 10)
#define is_failure(amn)	\
	((amn)->amn_retrycnt > (amn)->amn_txcnt / 3)
#define is_enough(amn)		\
	((amn)->amn_txcnt > 10)

static void	amrr_init(struct ieee80211vap *, uint32_t);
static void	amrr_deinit(struct ieee80211vap *);
static void	amrr_node_init(struct ieee80211_node *);
static void	amrr_node_deinit(struct ieee80211_node *);
static int	amrr_rate(struct ieee80211_node *, void *, uint32_t);
static void	amrr_rates(struct ieee80211_node *, struct ieee80211_rc_info *);
static void	amrr_tx_complete(const struct ieee80211vap *,
    			const struct ieee80211_node *, struct ieee80211_rc_info *);
static void	amrr_tx_update(const struct ieee80211vap *,
			const struct ieee80211_node *, void *, void *, void *);
static void	amrr_stats(const struct ieee80211vap *);
static void	amrr_setinterval(const struct ieee80211vap *, int);
static void	amrr_sysctlattach(struct ieee80211vap *,
			struct sysctl_ctx_list *, struct sysctl_oid *);

/* number of references from net80211 layer */
static	int nrefs = 0;

static const struct ieee80211_ratectl amrr = {
	.ir_name	= "amrr",
	.ir_attach	= NULL,
	.ir_detach	= NULL,
	.ir_init	= amrr_init,
	.ir_deinit	= amrr_deinit,
	.ir_node_init	= amrr_node_init,
	.ir_node_deinit	= amrr_node_deinit,
	.ir_rate	= amrr_rate,
	.ir_rates	= amrr_rates,
	.ir_tx_complete	= amrr_tx_complete,
	.ir_tx_update	= amrr_tx_update,
	.ir_setinterval	= amrr_setinterval,
	.ir_stats	= amrr_stats,
};
IEEE80211_RATECTL_MODULE(amrr, 1);
IEEE80211_RATECTL_ALG(amrr, IEEE80211_RATECTL_AMRR, amrr);

static void
amrr_setinterval(const struct ieee80211vap *vap, int msecs)
{
	struct ieee80211_amrr *amrr = vap->iv_rs;
	int t;

	if (msecs < 100)
		msecs = 100;
	t = msecs_to_ticks(msecs);
	amrr->amrr_interval = (t < 1) ? 1 : t;
}

static void
amrr_init(struct ieee80211vap *vap, uint32_t capabilities)
{
	struct ieee80211_amrr *amrr;

	KASSERT(vap->iv_rs == NULL, ("%s called multiple times", __func__));

	amrr = vap->iv_rs = malloc(sizeof(struct ieee80211_amrr),
	    M_80211_RATECTL, M_NOWAIT|M_ZERO);
	if (amrr == NULL) {
		if_printf(vap->iv_ifp, "couldn't alloc ratectl structure\n");
		return;
	}

	struct ieee80211_rc_stat * irs = IEEE80211_RATECTL_STAT(vap);
	irs->irs_capabilities = capabilities;

	amrr->amrr_min_success_threshold = IEEE80211_AMRR_MIN_SUCCESS_THRESHOLD;
	amrr->amrr_max_success_threshold = IEEE80211_AMRR_MAX_SUCCESS_THRESHOLD;
	amrr_setinterval(vap, 500 /* ms */);
	amrr_sysctlattach(vap, vap->iv_sysctl, vap->iv_oid);
}

static void
amrr_deinit(struct ieee80211vap *vap)
{
	free(vap->iv_rs, M_80211_RATECTL);
}

static void
amrr_node_init(struct ieee80211_node *ni)
{
	const struct ieee80211_rateset *rs = NULL;
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211_amrr *amrr = vap->iv_rs;
	struct ieee80211_amrr_node *amn;
	uint8_t rate;

	if (ni->ni_rctls == NULL) {
		ni->ni_rctls = amn = malloc(sizeof(struct ieee80211_amrr_node),
		    M_80211_RATECTL, M_NOWAIT|M_ZERO);
		if (amn == NULL) {
			if_printf(vap->iv_ifp, "couldn't alloc per-node ratectl "
			    "structure\n");
			return;
		}
	} else
		amn = ni->ni_rctls;

	amn->amn_amrr = amrr;
	amn->amn_success = 0;
	amn->amn_recovery = 0;
	amn->amn_txcnt = amn->amn_retrycnt = 0;
	amn->amn_success_threshold = amrr->amrr_min_success_threshold;

	rs = ieee80211_ratectl_get_rateset(ni);

	/* Initial rate - lowest */
	rate = rs->rs_rates[0];

	/* XXX clear the basic rate flag if it's not 11n */
	if (! ieee80211_ratectl_node_is11n(ni))
		rate &= IEEE80211_RATE_VAL;

	/* pick initial rate from the rateset - HT or otherwise */
	for (amn->amn_rix = rs->rs_nrates - 1; amn->amn_rix > 0;
	    amn->amn_rix--) {
		/* legacy - anything < 36mbit, stop searching */
		/* 11n - stop at MCS4 / MCS12 / MCS28 */
		if (ieee80211_ratectl_node_is11n(ni) &&
		    (rs->rs_rates[amn->amn_rix] & 0x7) < 4)
			break;
		else if ((rs->rs_rates[amn->amn_rix] & IEEE80211_RATE_VAL) <= 72)
			break;
		rate = rs->rs_rates[amn->amn_rix] & IEEE80211_RATE_VAL;
	}

	/* if the rate is an 11n rate, ensure the MCS bit is set */
	if (ieee80211_ratectl_node_is11n(ni))
		rate |= IEEE80211_RATE_MCS;

	/* Assign initial rate from the rateset */
	ni->ni_txrate = rate;
	amn->amn_ticks = ticks;

	IEEE80211_NOTE(ni->ni_vap, IEEE80211_MSG_RATECTL, ni,
	    "%s: AMRR: nrates=%d, initial rate %d",
		__func__, rs->rs_nrates, rate);
}

static void
amrr_node_deinit(struct ieee80211_node *ni)
{
	free(ni->ni_rctls, M_80211_RATECTL);
}

static int
amrr_update(struct ieee80211_amrr *amrr, struct ieee80211_amrr_node *amn,
    struct ieee80211_node *ni)
{
	int rix = amn->amn_rix;
	const struct ieee80211_rateset *rs = NULL;

	KASSERT(is_enough(amn), ("txcnt %d", amn->amn_txcnt));

	rs = ieee80211_ratectl_get_rateset(ni);


	IEEE80211_NOTE(ni->ni_vap, IEEE80211_MSG_RATECTL, ni,
	    "%s: AMRR: current rate %d, txcnt=%d, retrycnt=%d",
	    __func__, rs->rs_rates[rix] & IEEE80211_RATE_VAL,
	    amn->amn_txcnt,
	    amn->amn_retrycnt);

	/*
	 * XXX This is totally bogus for 11n, as although high MCS
	 * rates for each stream may be failing, the next stream
	 * should be checked.
	 *
	 * Eg, if MCS5 is ok but MCS6/7 isn't, and we can go up to
	 * MCS23, we should skip 6/7 and try 8 onwards.
	 */
	if (is_success(amn)) {
		amn->amn_success++;
		if (amn->amn_success >= amn->amn_success_threshold &&
		    rix + 1 < rs->rs_nrates) {
			amn->amn_recovery = 1;
			amn->amn_success = 0;
			rix++;
			IEEE80211_NOTE(ni->ni_vap, IEEE80211_MSG_RATECTL, ni,
			    "%s: AMRR increasing rate %d (txcnt=%d retrycnt=%d)",
			    __func__, rs->rs_rates[rix] & IEEE80211_RATE_VAL,
			    amn->amn_txcnt, amn->amn_retrycnt);
		} else {
			amn->amn_recovery = 0;
		}
	} else if (is_failure(amn)) {
		amn->amn_success = 0;
		if (rix > 0) {
			if (amn->amn_recovery) {
				amn->amn_success_threshold *= 2;
				if (amn->amn_success_threshold >
				    amrr->amrr_max_success_threshold)
					amn->amn_success_threshold =
					    amrr->amrr_max_success_threshold;
			} else {
				amn->amn_success_threshold =
				    amrr->amrr_min_success_threshold;
			}
			rix--;
			IEEE80211_NOTE(ni->ni_vap, IEEE80211_MSG_RATECTL, ni,
			    "%s: AMRR decreasing rate %d (txcnt=%d retrycnt=%d)",
			    __func__, rs->rs_rates[rix] & IEEE80211_RATE_VAL,
			    amn->amn_txcnt, amn->amn_retrycnt);
		}
		amn->amn_recovery = 0;
	}

	/* reset counters */
	amn->amn_txcnt = 0;
	amn->amn_retrycnt = 0;

	return rix;
}

/*
 * Return the rate index to use in sending a data frame.
 * Update our internal state if it's been long enough.
 * If the rate changes we also update ni_txrate to match.
 */
static int
amrr_rate(struct ieee80211_node *ni, void *arg __unused, uint32_t iarg __unused)
{
	struct ieee80211_amrr_node *amn = ni->ni_rctls;
	struct ieee80211_amrr *amrr = amn->amn_amrr;
	const struct ieee80211_rateset *rs = NULL;
	int rix;

	rs = ieee80211_ratectl_get_rateset(ni);

	if (is_enough(amn) && (ticks - amn->amn_ticks) > amrr->amrr_interval) {
		rix = amrr_update(amrr, amn, ni);
		if (rix != amn->amn_rix) {
			/* update public rate */
			ni->ni_txrate = rs->rs_rates[rix];
			/* XXX strip basic rate flag from txrate, if non-11n */
			if (ieee80211_ratectl_node_is11n(ni))
				ni->ni_txrate |= IEEE80211_RATE_MCS;
			else
				ni->ni_txrate &= IEEE80211_RATE_VAL;
			amn->amn_rix = rix;
		}
		amn->amn_ticks = ticks;
	} else
		rix = amn->amn_rix;
	return rix;
}

static void
amrr_rates(struct ieee80211_node *ni, struct ieee80211_rc_info *rc_info)
{
#define	RATE(_ix)	(rs->rs_rates[(_ix)] & IEEE80211_RATE_VAL)
#define	MCS(_ix)	(rs->rs_rates[(_ix)] | IEEE80211_RATE_MCS)
	struct ieee80211_rc_series *rc = rc_info->iri_rc;
	const struct ieee80211_rateset *rs = NULL;
	const struct ieee80211_rate_table *rt = NULL;
	int rix, code;

	rs = ieee80211_ratectl_get_rateset(ni);
	rt = ieee80211_get_ratetable(ni->ni_ic->ic_curchan);
	IEEE80211_DPRINTF(ni->ni_vap, IEEE80211_MSG_RATECTL, 
	    "%s: channel flags: 0x%08x\n", __func__, 
		ni->ni_ic->ic_curchan->ic_flags); 
	
	rix = amrr_rate(ni, NULL, 0);

	rc[0].flags = rc[1].flags = rc[2].flags = rc[3].flags = 0;

	if (rs->rs_nrates > 0) {
		code = ieee80211_ratectl_node_is11n(ni)? MCS(rix) : RATE(rix);
		rc[0].rix = rt->rateCodeToIndex[code];

		if (IEEE80211_RATECTL_HASCAP_MRR(ni->ni_vap)) {
			rc[0].tries = 1;
			rc[1].tries = 1;
			rc[2].tries = 1;
			rc[3].tries = 1;
			if (--rix >= 0) {
				code = ieee80211_ratectl_node_is11n(ni)? MCS(rix) : RATE(rix);
				rc[1].rix = rt->rateCodeToIndex[code];
			} else {
				rc[1].rix = rt->rateCodeToIndex[0];
			}
			if (--rix >= 0) {
				code = ieee80211_ratectl_node_is11n(ni)? MCS(rix) : RATE(rix);
				rc[2].rix = rt->rateCodeToIndex[code];
			} else {
				rc[2].rix = rt->rateCodeToIndex[0];
			}
			if (rix >= 0) {
				/* NB: only do this if we didn't already do it above */
				code = ieee80211_ratectl_node_is11n(ni)? MCS(0) : RATE(0);
				rc[3].rix = rt->rateCodeToIndex[code];
			} else {
				rc[3].rix = rt->rateCodeToIndex[0];
			}
		} else {
			rc[0].tries = IEEE80211_RATECTL_TXMAXTRY;

			rc[1].tries = 0;
			rc[2].tries = 0;
			rc[3].tries = 0;
			rc[1].rix = rt->rateCodeToIndex[0];
			rc[2].rix = rt->rateCodeToIndex[0];
			rc[3].rix = rt->rateCodeToIndex[0];
		}
	}
	IEEE80211_NOTE(ni->ni_vap, IEEE80211_MSG_RATECTL, ni,
	    "%s: AMRR rate sets(rix, tries). rc[0]:(%d, %d), rc[1]:(%d, %d), " 
		"rc[2]:(%d, %d), rc[3]:(%d, %d)", __func__, 
		rc[0].rix, rc[0].tries,
		rc[1].rix, rc[1].tries, 
		rc[2].rix, rc[2].tries, 
		rc[3].rix, rc[3].tries);
#undef RATE
#undef MCS
}

/*
 * Update statistics with tx complete status.  Ok is non-zero
 * if the packet is known to be ACK'd.  Retries has the number
 * retransmissions (i.e. xmit attempts - 1).
 */
static void
amrr_tx_complete(const struct ieee80211vap *vap,
    const struct ieee80211_node *ni, struct ieee80211_rc_info *rc_info)
{
	struct ieee80211_amrr_node *amn = ni->ni_rctls;

	/* update per vap statistics */
	ieee80211_ratectl_update_stat(vap, rc_info);

	amn->amn_txcnt += rc_info->iri_txcnt;
	amn->amn_success += (rc_info->iri_txcnt - rc_info->iri_failcnt);
	amn->amn_retrycnt += rc_info->iri_retrycnt;

	IEEE80211_NOTE(vap, IEEE80211_MSG_RATECTL, ni,
	    "%s: AMRR tx complete. txcnt=%d(%d) success=%d(%d) retrycnt=%d(%d)\n",
	    __func__, amn->amn_txcnt, rc_info->iri_txcnt,
		amn->amn_success, (rc_info->iri_txcnt - rc_info->iri_failcnt),
		amn->amn_retrycnt, rc_info->iri_retrycnt);
}

/*
 * Set tx count/retry statistics explicitly.  Intended for
 * drivers that poll the device for statistics maintained
 * in the device.
 */
static void
amrr_tx_update(const struct ieee80211vap *vap, const struct ieee80211_node *ni,
    void *arg1, void *arg2, void *arg3)
{
	struct ieee80211_amrr_node *amn = ni->ni_rctls;
	int txcnt = *(int *)arg1, success = *(int *)arg2, retrycnt = *(int *)arg3;

	amn->amn_txcnt = txcnt;
	amn->amn_success = success;
	amn->amn_retrycnt = retrycnt;
	IEEE80211_NOTE(vap, IEEE80211_MSG_RATECTL, ni,
	    "%s: AMRR tx update. txcnt=%d success=%d retrycnt=%d\n",
	    __func__, txcnt, success, retrycnt);
}

static void
amrr_stats(const struct ieee80211vap *vap)
{
	struct ieee80211_rc_stat * irs = IEEE80211_RATECTL_STAT(vap);
	printf("tx count: %d (ok count: %d, fail count: %d)\n"
			"retry count: %d (short retry: %d, long retry: %d)\n",
			irs->irs_txcnt, irs->irs_txcnt - irs->irs_failcnt, irs->irs_failcnt,
			irs->irs_retrycnt, irs->irs_shortretry, irs->irs_longretry);
}

static int
amrr_sysctl_interval(SYSCTL_HANDLER_ARGS)
{
	struct ieee80211vap *vap = arg1;
	struct ieee80211_amrr *amrr = vap->iv_rs;
	int msecs = ticks_to_msecs(amrr->amrr_interval);
	int error;

	error = sysctl_handle_int(oidp, &msecs, 0, req);
	if (error || !req->newptr)
		return error;
	amrr_setinterval(vap, msecs);
	return 0;
}

static void
amrr_sysctlattach(struct ieee80211vap *vap,
    struct sysctl_ctx_list *ctx, struct sysctl_oid *tree)
{
	struct ieee80211_amrr *amrr = vap->iv_rs;

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "amrr_rate_interval", CTLTYPE_INT | CTLFLAG_RW, vap,
	    0, amrr_sysctl_interval, "I", "amrr operation interval (ms)");
	/* XXX bounds check values */
	SYSCTL_ADD_UINT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "amrr_max_sucess_threshold", CTLFLAG_RW,
	    &amrr->amrr_max_success_threshold, 0, "");
	SYSCTL_ADD_UINT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "amrr_min_sucess_threshold", CTLFLAG_RW,
	    &amrr->amrr_min_success_threshold, 0, "");
}
