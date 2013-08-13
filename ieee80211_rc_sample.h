/* $FreeBSD: head/sys/dev/ath/ath_rate/sample/sample.h 240382 2012-08-15 07:10:10Z adrian $*/

/*-
 * Copyright (c) 2013 Chenchong Qin <ccqin@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 */

#ifndef _NET80211_IEEE80211_RATECTL_SAMPLE_H_
#define _NET80211_IEEE80211_RATECTL_SAMPLE_H_

/*
 * for now, we track performance for three different packet
 * size buckets
 */
#define NUM_PACKET_SIZE_BINS 2
#define	SAMPLE_MAXRATES	64		/* NB: corresponds to hal info[32] */

/*
 * Rate control settings.
 */
struct ieee80211_sample {
	int	sample_smoothing_rate;			/* ewma percentage [0..99] */
	int	sample_smoothing_minpackets;
	int	sample_rate;			/* %time to try different tx rates */
	int	sample_max_successive_failures;
	int	sample_stale_failure_timeout;	/* how long to honor max_successive_failures */
	int	sample_min_switch;		/* min time between rate changes */
	int	sample_min_good_pct;	/* min good percentage for a rate to be considered */
};

struct rate_stats {	
	unsigned average_tx_time;
	int successive_failures;
	uint64_t tries;
	uint64_t total_packets;	/* pkts total since assoc */
	uint64_t packets_acked;	/* pkts acked since assoc */
	int ewma_pct;	/* EWMA percentage */
	unsigned perfect_tx_time; /* transmit time for 0 retries */
	int last_tx;
};

struct txschedule {
	uint8_t	t0, r0;		/* series 0: tries, rate code */
	uint8_t	t1, r1;		/* series 1: tries, rate code */
	uint8_t	t2, r2;		/* series 2: tries, rate code */
	uint8_t	t3, r3;		/* series 3: tries, rate code */
};

/*
 * Rate control state for a given node.
 */
/* XXX change naming conversion? */
struct ieee80211_sample_node {
	struct ieee80211_sample *san_sample;/* backpointer */
	int static_rix;			/* rate index of fixed tx rate */
	uint64_t ratemask;		/* bit mask of valid rate indices */
	const struct txschedule *sched;	/* tx schedule table */

	struct rate_stats stats[NUM_PACKET_SIZE_BINS][SAMPLE_MAXRATES];
	int last_sample_rix[NUM_PACKET_SIZE_BINS];

	int current_sample_rix[NUM_PACKET_SIZE_BINS];       
	int packets_sent[NUM_PACKET_SIZE_BINS];

	int current_rix[NUM_PACKET_SIZE_BINS];
	int packets_since_switch[NUM_PACKET_SIZE_BINS];
	unsigned ticks_since_switch[NUM_PACKET_SIZE_BINS];

	int packets_since_sample[NUM_PACKET_SIZE_BINS];
	unsigned sample_tt[NUM_PACKET_SIZE_BINS];
};

#define	IS_RATE_DEFINED(san, rix)	(((san)->ratemask & (1<<(rix))) != 0)

#ifndef MIN
#define	MIN(a,b)	((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define	MAX(a,b)	((a) > (b) ? (a) : (b))
#endif

static const int packet_size_bins[NUM_PACKET_SIZE_BINS]  = { 250, 1600 };

static inline int
bin_to_size(int index)
{
	return packet_size_bins[index];
}

static inline int
size_to_bin(int size) 
{
#if NUM_PACKET_SIZE_BINS > 1
	if (size <= packet_size_bins[0])
		return 0;
#endif
#if NUM_PACKET_SIZE_BINS > 2
	if (size <= packet_size_bins[1])
		return 1;
#endif
#if NUM_PACKET_SIZE_BINS > 3
	if (size <= packet_size_bins[2])
		return 2;
#endif
#if NUM_PACKET_SIZE_BINS > 4
#error "add support for more packet sizes"
#endif
	return NUM_PACKET_SIZE_BINS-1;
}

static uint32_t sample_pkt_txtime(const struct ieee80211_rate_table *rt,
	uint32_t frameLen, uint16_t rateix, int isht40, int isShortPreamble)
{
	uint8_t rc;
    int numStreams;

    rc = rt->info[rateix].rateCode;

    /* Legacy rate? Return the old way */
    if (! IS_HT_RATE(rc))
    	return ieee80211_compute_duration(rt, frameLen, rateix, isShortPreamble);

    /* 11n frame - extract out the number of spatial streams */
    numStreams = HT_RC_2_STREAMS(rc);
    KASSERT(numStreams > 0 && numStreams <= 4,
        ("number of spatial streams needs to be 1..3: MCS rate 0x%x!",
        rateix));

    return ieee80211_compute_duration_ht(frameLen, rc, numStreams, isht40, isShortPreamble);
}

#define WIFI_CW_MIN 31
#define WIFI_CW_MAX 1023

/*
 * Calculate the transmit duration of a frame.
 */
static unsigned calc_usecs_unicast_packet(const struct ieee80211vap *vap,
				int length,
				int rix, int short_retries,
				int long_retries, int is_ht40)
{
	const struct ieee80211_rate_table *rt = ieee80211_get_ratetable(vap->iv_ic->ic_curchan);
	struct ieee80211com *ic = vap->iv_ic;
	int curmode = ieee80211_chan2mode(vap->iv_ic->ic_curchan);
	
	unsigned t_slot, t_difs, t_sifs; 
	int rts, cts;
	int tt, x, cw, cix;

	int tt = 0;
	int x = 0;
	int cw = WIFI_CW_MIN;

	KASSERT(rt != NULL, ("no rate table, mode %u", curmode));

	if (rix >= rt->rateCount) {
		printf("bogus rix %d, max %u, mode %u\n",
		       rix, rt->rateCount, curmode);
		return 0;
	}
	cix = rt->info[rix].controlRate;
	/* 
	 * XXX getting mac/phy level timings should be fixed for turbo
	 * rates, and there is probably a way to get this from the
	 * hal...
	 */
	switch (rt->info[rix].phy) {
	case IEEE80211_T_OFDM:
		t_slot = 9;
		t_sifs = 16;
		t_difs = 28;
		/* fall through */
	case IEEE80211_T_TURBO:
		t_slot = 9;
		t_sifs = 8;
		t_difs = 28;
		break;
	case IEEE80211_T_HT:
		t_slot = 9;
		t_sifs = 8;
		t_difs = 28;
		break;
	case IEEE80211_T_DS:
		/* fall through to default */
	default:
		/* pg 205 ieee.802.11.pdf */
		t_slot = 20;
		t_difs = 50;
		t_sifs = 10;
	}

	rts = cts = 0;

	if ((ic->ic_flags & IEEE80211_F_USEPROT) &&
	    rt->info[rix].phy == IEEE80211_T_OFDM) {
		if (ic->ic_protmode == IEEE80211_PROT_RTSCTS)
			rts = 1;
		else if (ic->ic_protmode == IEEE80211_PROT_CTSONLY)
			cts = 1;

		int protrix;
    	if (curmode == IEEE80211_MODE_11G)
    	    protrix = rt->rateCodeToIndex[2*2];
    	else 
    	    protrix = rt->rateCodeToIndex[2*1];
    	if (0xff == protrix)
    		protrix = 0;

		cix = rt->info[protrix].controlRate;
	}

	if (0 /*length > ic->ic_rtsthreshold */) {
		rts = 1;
	}

	if (rts || cts) {
		int ctsrate;
		int ctsduration = 0;

		/* NB: this is intentionally not a runtime check */
		KASSERT(cix < rt->rateCount,
		    ("bogus cix %d, max %u, mode %u\n", cix, rt->rateCount,
		     curmode));

		ctsrate = rt->info[cix].rateCode | rt->info[cix].shortPreamble;
		if (rts)		/* SIFS + CTS */
			ctsduration += rt->info[cix].spAckDuration;

		/* XXX assumes short preamble */
		ctsduration += sample_pkt_txtime(rt, length, rix, is_ht40, 0);

		if (cts)	/* SIFS + ACK */
			ctsduration += rt->info[cix].spAckDuration;

		tt += (short_retries + 1) * ctsduration;
	}
	tt += t_difs;

	/* XXX assumes short preamble */
	tt += (long_retries+1)*sample_pkt_txtime(rt, length, rix, is_ht40, 0);

	tt += (long_retries+1)*(t_sifs + rt->info[rix].spAckDuration);

	for (x = 0; x <= short_retries + long_retries; x++) {
		cw = MIN(WIFI_CW_MAX, (cw + 1) * 2);
		tt += (t_slot * cw/2);
	}
	return tt;
}

#endif /* _NET80211_IEEE80211_RATECTL_SAMPLE_H_ */
