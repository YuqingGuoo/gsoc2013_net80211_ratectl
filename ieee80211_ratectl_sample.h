/*-
 * Copyright (c) 2005 John Bicket
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

#endif /* _NET80211_IEEE80211_RATECTL_SAMPLE_H_ */
