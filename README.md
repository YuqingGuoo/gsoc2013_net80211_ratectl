GSoC2013: net80211 rate control API
===========================

This a project of [Google Summer of Code 2013](http://www.google-melange.com/gsoc/homepage/google/gsoc2013).

Project Description
-------------------

A simplistic rate control API exists in net80211 of FreeBSD, which lack 
the support of 802.11n features. 802.11n brought a 10x maximum net data 
rate compared to its predecessor, but, unfortunately, the hard-won rate up 
can be easily wasted if rate control hasn't been properly performed. This 
project will extend the net80211 rate control API of FreeBSD to be 802.11n 
aware and be able to support multiple rate attempts. With the extended API, 
wireless throughput can be further imrpoved.

Improvements
------------
A summary of the feature improvements are listed below:

* Multiple Rate Attempts Support: The current rate control API of net80211 
in FreeBSD is quite straightforward. The way of performing rate control is 
just selecting the “best” rate found by rate adaptation algorithms. Rather 
than just selecting the “best” rate, the rate control API should be able to 
do some real “control” works. It should be able to support multiple rate 
attempts which ath(4) hardware support.
* 802.11n Features Support: The rate control API should be extended to 
support 802.11n features including MCS rates, 20/40MHz wide channels, 
short-gi, stbc, ldpc. 802.11n APs and STAs need to negotiate capabilities 
like the type of RF modulation, coding rate and channel width. They also must 
agree upon the guard interval and number of spatial streams to be used. The 
802.11n standard defines Modulation and Coding Scheme (MCS) a simple integer 
assigned to index 77 possible permutations of the factors that determine data 
rate. The standard also added support for short guard interval (short-gi), 
which provides an 11% increase in data rate. The rate control API of net80211 
in FreeBSD should support these features to perform better rate control 
operations in 802.11n environment.
* Statistics API: A interface for exposing the global, per-vap and per-device 
rate control statistics should be added to the rate control API of net80211 
in FreeBSD. This is pretty a useful feature for developers, academic researchers 
and, of course, end-users.
* 802.11n Aggregation Support: Frame aggregation is a feature of 802.11n that 
increases throughput by sending two or more data frames in a single transmission. 
Instead of having one frame be transmitted and received at a time, with A-MPDU, 
multiple subframes can be aggregated for one transmission and sender can be told 
of which subframes were successfully received. The current rate control API of 
net80211 in FreeBSD only works well for non-aggregate frames as it would be called 
each for a rate lookup and a single frame completion. With 802.11n aggregation, 
the rate control API should be extended to perform a single rate lookup for an 
aggregate, which will consist of multiple frames.

Links
----------------

* [Google Summer of Code 2013](http://www.google-melange.com/gsoc/homepage/google/gsoc2013)
* [FreeBSD GSoC2013](https://wiki.freebsd.org/SummerOfCode2013)
* [FreeBSD WifiIdeas Page](https://wiki.freebsd.org/WifiIdeasPage)
* [IEEE -- 802.11 network layer](http://www.freebsd.org/cgi/man.cgi?query=ieee80211&apropos=0&sektion=0&manpath=FreeBSD+9-current&arch=default&format=html)
* [Project Homepage](https://wiki.freebsd.org/SummerOfCode2013/80211RateControl80211nExtensions)
* [Chenchong@FreeBSD](https://wiki.freebsd.org/ChenchongQin)


