# trackfusion

A real-time multi-sensor air track fusion engine in Go. It ingests live ADS-B state vectors from the OpenSky Network, fuses them with a simulated radar sensor over the same airspace, and maintains one continuous set of tracks across everything flying in a bounded region.

## Why this exists

I wanted to understand how a system takes several asynchronous, incomplete, disagreeing streams of sensor data and produces a single coherent picture of what is actually out there. That problem shows up in air traffic control, maritime tracking, battle management, and anything else where you have more sensors than certainty. Reading about it was not getting me anywhere, so I decided to build one.

I picked ADS-B as the live source because it is public, real, and messy in useful ways. Aircraft drop off the feed, come back with gaps, report at irregular intervals, and occasionally report nonsense. A simulated radar gives me a second source I can control: I can dial up its noise, drop its returns, or make it lie, and see what that does to the picture.

## The two sensors

The interesting part of this setup is that the two sources are asymmetric in a way that mirrors reality.

ADS-B is cooperative. Each aircraft broadcasts its own identity (a 24-bit ICAO address), position, altitude, velocity, and heading. If you receive a message, you know exactly who sent it.

Radar is not. A return is a position and a timestamp. Nothing in it tells you which aircraft produced it, or whether it is an aircraft at all.

So one source hands you identity for free and the other makes you infer it. Fusing them means deciding, on every scan, which unlabeled returns belong to which tracks you are already holding.

## The problem worth solving

The Kalman filtering is the well-documented part. A constant-velocity model over state `[x, y, vx, vy]` predicts where a track should be at the next timestep, which matters here because the ADS-B feed only reports every five seconds and a jet covers a lot of ground in five seconds.

The part that actually determines whether the picture holds together is association: given a new radar return with a position and nothing else, which existing track does it belong to?

Nearest neighbor is the obvious answer, and it fails in a specific and instructive way. Two aircraft on converging paths will eventually produce returns that sit closer to each other's predicted position than to their own. The tracker swaps them. On its own that would be recoverable, except the filter then updates each track with the wrong measurement, sees a small residual because the wrong measurement is still close to the prediction, and shrinks its covariance accordingly. The gate is now tighter and centered on the wrong aircraft. The failure feeds itself: the more wrong the tracker gets, the more confident it becomes, and the swap becomes permanent.

Two things address this.

Gate on Mahalanobis distance rather than Euclidean, so the acceptance region around a prediction reflects the filter's own uncertainty instead of an arbitrary fixed radius. A track that has not been updated in a while should be willing to accept a return further away than one that was just corrected.

Then solve assignment globally across every return in a scan rather than greedily one return at a time. A return that is marginally closer to the wrong track can still be given up if the overall pairing across the whole scan is better. That is the Hungarian algorithm over a cost matrix of gated Mahalanobis distances.

Getting this right is most of the work, and it is the reason the project is interesting rather than an exercise.

## Design

```
cmd/trackfusion      entrypoint, config, wiring
internal/geo         WGS84 to local ENU tangent plane projection
internal/opensky     OAuth2 client, state vector polling, decoding
internal/radar       simulated sensor: sweep timing, noise, dropout
internal/track       track representation, lifecycle, track store
internal/filter      constant-velocity Kalman filter
internal/assoc       gating and assignment
internal/record      JSONL capture and deterministic replay
```

A few decisions worth naming:

Tracks are sensor-agnostic. A track does not know or care which sensor produced the measurement that updated it, which is what lets a third sensor type be added later without touching the estimator.

Everything runs in a local East-North-Up tangent plane rather than in latitude and longitude. A constant-velocity model needs a Cartesian frame to be linear in, and over a bounded region the projection error is small enough to ignore.

Every measurement that enters the system is written to a JSONL log, and the whole pipeline can be re-run from that log instead of from the live feed. This is the piece I care most about. Without it, "did that change improve anything" is answered by watching a map and forming an impression. With it, the same traffic can be replayed against two versions of the associator and the ID swap counts compared directly.

## Running it

Not yet. Once the OpenSky client lands, this section will cover credentials, the bounding box config, and replay usage.

## References

- Bar-Shalom, Willett, Tian. *Tracking and Data Fusion: A Handbook of Algorithms.* 2011.
- Blackman, Popoli. *Design and Analysis of Modern Tracking Systems.* 1999.
- Kuhn. "The Hungarian Method for the Assignment Problem." *Naval Research Logistics Quarterly*, 1955.
- [OpenSky Network REST API](https://openskynetwork.github.io/opensky-api/)