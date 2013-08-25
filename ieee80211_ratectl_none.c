/*-
 * Copyright (c) 2010 Bernhard Schmidt <bschmidt@FreeBSD.org>
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
__FBSDID("$FreeBSD: soc2013/ccqin/head/sys/net80211/ieee80211_ratectl_none.c 256491 2013-08-25 10:34:29Z ccqin $");

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
#include <net80211/ieee80211_ratectl.h>
#include <net80211/ieee80211_ratectl_none.h>

static void
none_init(struct ieee80211vap *vap, uint32_t capabilities)
{
	struct ieee80211_node *none;

	KASSERT(vap->iv_rs == NULL, ("%s called multiple times", __func__));

	none = vap->iv_rs = malloc(sizeof(struct ieee80211_none),
	    M_80211_RATECTL, M_NOWAIT|M_ZERO);
	if (none == NULL) {
		if_printf(vap->iv_ifp, "couldn't alloc ratectl structure\n");
		return;
	}

	struct ieee80211_rc_stat * irs = IEEE80211_RATECTL_STAT(vap);
	irs->irs_capabilities = capabilities;

	/* ... */
}

static void
none_deinit(struct ieee80211vap *vap)
{
	free(vap->iv_rs, M_80211_RATECTL);
}

static void
none_node_init(struct ieee80211_node *ni)
{
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211_none *none = vap->iv_rs;
	struct ieee80211_none_node *non;

	if (ni->ni_rctls == NULL) {
		ni->ni_rctls = non = malloc(sizeof(struct ieee80211_none_node),
		    M_80211_RATECTL, M_NOWAIT|M_ZERO);
		if (non == NULL) {
			if_printf(vap->iv_ifp, "couldn't alloc per-node ratectl "
			    "structure\n");
			return;
		}
	} else
		non = ni->ni_rctls;

	non->non_none = none;

	/* ... */
	
	ni->ni_txrate = ni->ni_rates.rs_rates[0] & IEEE80211_RATE_VAL;
}

static void
none_node_deinit(struct ieee80211_node *ni)
{
}

static int
none_rate(struct ieee80211_node *ni, void *arg __unused, uint32_t iarg __unused)
{
	int rix = 0;

	ni->ni_txrate = ni->ni_rates.rs_rates[rix] & IEEE80211_RATE_VAL;
	return rix;
}

static void
none_tx_complete(const struct ieee80211vap *vap,
    const struct ieee80211_node *ni, int ok,
    void *arg1, void *arg2 __unused)
{
}

static void
none_tx_update(const struct ieee80211vap *vap, const struct ieee80211_node *ni,
    void *arg1, void *arg2, void *arg3)
{
}

static void
none_setinterval(const struct ieee80211vap *vap, int msecs)
{
}

/* number of references from net80211 layer */
static	int nrefs = 0;

static const struct ieee80211_ratectl none = {
	.ir_name	= "none",
	.ir_attach	= NULL,
	.ir_detach	= NULL,
	.ir_init	= none_init,
	.ir_deinit	= none_deinit,
	.ir_node_init	= none_node_init,
	.ir_node_deinit	= none_node_deinit,
	.ir_rate	= none_rate,
	.ir_tx_complete	= none_tx_complete,
	.ir_tx_update	= none_tx_update,
	.ir_setinterval	= none_setinterval,
};
IEEE80211_RATECTL_MODULE(ratectl_none, 1);
IEEE80211_RATECTL_ALG(none, IEEE80211_RATECTL_NONE, none);
