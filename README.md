# Heardwise

On-device geofencing and road-proximity alert system for free-grazing cattle. ESP32-based collar hardware, edge decision-making with zero network dependency, LoRa mesh backhaul, and a cloud layer for prediction and fleet visibility.

---

## System Overview

```
[Collar: GPS + IMU] --LoRa--> [Herd GSM Gateway Collar] --4G--> [Supabase/PostGIS]
                                                                        |
                                                                        v
                                                          [OSM road data + Monte Carlo
                                                           risk prediction + TinyML]
                                                                        |
                                                                        v
                                                              [Farmer App (multilingual)]
```

Only one collar per herd is GSM-equipped and acts as the gateway; the rest mesh to it over LoRa. The safety decision is made entirely on-device — the cloud layer is for prediction and fleet visibility, not real-time safety.

---

## Hardware

- **MCU:** ESP32 (dual-core, low-power modes)
- **Positioning:** GPS module
- **Motion:** IMU (accelerometer + gyroscope)
- **Radio:** LoRa (mesh) + GSM (single gateway collar per herd)
- **Power:** Solar-charged battery, deep-sleep + watchdog timer wake-on-motion

---

## Firmware / Edge Decision Engine

This is the safety-critical path. Everything here runs on the ESP32, per-collar, with no network dependency.

### 1. Boundary Containment — Ray Casting (Point-in-Polygon)

Determines whether the animal's current GPS fix is inside or outside its defined boundary polygon (field perimeter or road-exclusion zone).

- **How it works:** Cast a ray from the point outward in a fixed direction (e.g. due east) and count how many polygon edges it crosses. Odd crossing count = inside; even = outside.
- **Time complexity:** $O(n)$, where $n$ = number of vertices in the boundary polygon. Every edge must be tested once per query.
- **Space complexity:** $O(n)$ to store the polygon itself; $O(1)$ additional working memory for the crossing-count algorithm.
- **Practical $n$:** Boundary polygons for a single field/exclusion zone are small (tens of vertices, not thousands), so this is effectively constant-time on-device — the bound matters for correctness/memory budgeting, not runtime risk.
- **Edge cases to handle explicitly:** Ray passing exactly through a vertex (standard fix: treat vertex as slightly above/below the ray, or use the "vertex counts if its y-coordinate is $\ge$ ray's y, other endpoint's is $<$ " convention), horizontal edges (skip — no valid crossing), point exactly on the boundary (define as inside or outside by convention and stay consistent).

### 2. Nearest-Boundary Distance — Point-to-Segment

Computes the exact distance from the animal's position to the nearest edge of the polygon or nearest road segment, not just "inside/outside" — this is what drives the escalating alert (the risk score needs a continuous distance, not a binary flag).

- **How it works:** For each candidate segment, project the point onto the line containing the segment, clamp the projection to the segment's endpoints if it falls outside, then compute Euclidean distance to that clamped point.
- **Time complexity per segment:** $O(1)$ — a handful of arithmetic operations (dot product, clamp, distance).
- **Time complexity per query:** $O(m)$, where $m$ = number of segments being checked (polygon edges + nearby road segments). Naively this is a linear scan of every relevant segment.
- **Space complexity:** $O(1)$ per individual computation; $O(m)$ to hold the segment set in memory.
- **Optimization available:** If $m$ grows large (e.g. many road segments near a boundary field), a spatial index (grid bucket or bounding-box prefilter) reduces the effective segment set checked per query — but for a single collar's local boundary + locally-relevant road segments, $m$ is small enough (tens, not thousands) that the naive $O(m)$ scan is fine on an ESP32's clock budget for a sub-second decision loop.

### 3. Risk Scoring with Hysteresis

A simple finite-state machine, not a continuous controller — deliberately cheap.

- **State:** `ALERT` / `NO_ALERT`, plus the current distance value from step 2.
- **Transition:** `NO_ALERT` $\to$ `ALERT` only when distance drops below threshold $T_{high}$. `ALERT` $\to$ `NO_ALERT` only when distance rises above a separate, larger threshold $T_{low}$ ($T_{low} > T_{high}$). This deadband between the two thresholds is what prevents GPS jitter from flapping the alarm on and off near a single threshold.
- **Time complexity:** $O(1)$ — a couple of comparisons against the current state.
- **Space complexity:** $O(1)$ — one enum/state variable and two constant thresholds.

### 4. Actuation

Two-stage escalation on `ALERT`: vibration motor first, audio buzzer if the animal continues closing distance after vibration. This is a direct GPIO/PWM trigger — $O(1)$, no computation of note.

---

### Per-Cycle Decision Loop — Overall

One full decision cycle (executed multiple times per second) is:

$$\text{GPS/IMU read} \to \text{boundary check } (O(n)) \to \text{distance calc } (O(m)) \to \text{hysteresis update } (O(1)) \to \text{actuate } (O(1))$$

- **Dominant cost:** $O(n + m)$. Given $n$ and $m$ are both small (tens of elements) for a single field/road-segment set, this comfortably runs sub-second on an ESP32 with room to spare — the design choice that keeps it fast is bounding $n$ and $m$ at the data-modeling level (don't hand a collar a national-scale road dataset; only load the roads and boundary relevant to that animal's registered area).

---

## Communication Layer

- **Collar-to-gateway:** LoRa. Low bandwidth, long range, low power — sufficient because payload is small (position, battery, state, alert events), not continuous video/telemetry.
- **Gateway-to-cloud:** GSM/4G, one gateway per herd.
- **Payload size:** Dominated by GPS coordinates + status flags per message — bytes, not kilobytes, so LoRa's bandwidth ceiling isn't a bottleneck here.

---

## Backend

### Geospatial Storage & Query — PostGIS

- **Storage:** Postgres + PostGIS, animal positions and road/field geometries stored as geospatial types (points, polygons, linestrings).
- **Indexing:** PostGIS uses R-tree-based GiST indexes on geometry columns. Nearest-neighbor and containment queries (e.g. "which roads are near this animal," "is this point inside this field") run in roughly $O(\log n)$ average case against the indexed set, rather than a full table scan — this matters at the cloud layer because $n$ (all roads across all registered regions) is much larger than the small local $n/m$ used on-device.
- **Road data ingestion:** Pulled from OpenStreetMap, filtered to the region's national highways/main roads, stored as PostGIS linestrings.

### Risk Prediction — Monte Carlo Path Simulation

Predicts the probability that a given animal reaches a monitored road within a 5-minute window, based on its own recent movement.

- **How it works:** Build a simple motion model (e.g. a random walk or correlated random walk parameterized by the animal's recent speed/heading distribution), sample $K$ candidate future paths from that model, check how many of the $K$ sampled paths intersect a monitored road within the time horizon, and use that fraction as the probability estimate.
- **Time complexity:** $O(K \times T)$, where $K$ = number of sampled paths (500, per the current design) and $T$ = number of timesteps simulated per path within the 5-minute horizon. Each step is $O(1)$ (sample next position from the motion model) plus an intersection check against nearby road geometry, which — using the same spatial indexing as above — is roughly $O(\log r)$ per step against $r$ nearby road segments.
- **Full complexity:** $O(K \times T \times \log r)$.
- **Space complexity:** $O(K \times T)$ if all sampled paths are retained for later inspection/debugging; $O(T)$ if paths are simulated and discarded one at a time, only retaining the running intersection count (the memory-lean choice, and the one to actually implement given this runs server-side per-animal on a schedule, not per-collar).
- **Runs on:** Cloud, not collar — this is prediction/analytics, not the real-time safety decision, so the heavier cost is acceptable there.

### Behavior Classification — TinyML

A lightweight model (decision-tree ensemble or small quantized neural net — pick based on your actual training pipeline) classifying each animal's IMU-derived movement pattern (grazing / resting / agitated / walking) for longer-term analysis.

- **Inference time complexity:** Depends on model choice —
  - **Decision tree / random forest:** $O(d)$ per tree, where $d$ = tree depth; $O(d \times f)$ for a forest of $f$ trees.
  - **Small quantized NN:** $O(\sum \text{layer\_in} \times \text{layer\_out})$ — dominated by the largest fully-connected layer.
- **Space complexity:** TinyML models are deliberately kept small enough to fit embedded/edge memory budgets — typically tens of KB, not MB, achieved via quantization (int8 weights) and pruning. Exact footprint depends on the trained model; state it once the model is finalized rather than estimating here.
- **Where it runs:** Per the phase description, this is per-animal classification — confirm whether your current implementation runs it on-collar (in which case the memory budget is the ESP32's, and you want it firmly in the low tens-of-KB range) or server-side on ingested IMU data (looser budget, but loses the "runs without connectivity" property for this specific feature). This distinction should be made explicit in the writeup once decided, since it changes the complexity constraints materially.

---

## Application Layer

- **Live map:** Real-time position of every collared animal, monitored roads highlighted
- **Per-animal profile:** Location, battery, state, alert log (timestamp + response taken)
- **Geofencing UI:** Farmer places boundary points on a map; polygon pushed to relevant collars
- **Localization:** English, Hindi, Kannada

---

## Tech Stack Summary

| Layer | Technology | Core algorithm(s) |
|---|---|---|
| **Firmware** | ESP32, C/C++ | Ray casting ($O(n)$), point-to-segment distance ($O(m)$), hysteresis FSM ($O(1)$) |
| **Mesh networking** | LoRa (collar↔gateway), GSM (gateway↔cloud) | — |
| **Backend** | Supabase, PostgreSQL + PostGIS | GiST/R-tree spatial indexing ($O(\log n)$ queries) |
| **External data** | OpenStreetMap | Road geometry ingestion |
| **Prediction** | Monte Carlo simulation | $O(K \times T \times \log r)$ path sampling |
| **ML** | TinyML | $O(d)$ / $O(\sum \text{layer sizes})$ inference, depending on model type |

---

## Repository Structure

```text
/firmware       - ESP32 collar firmware (collar, leader/gateway, telemetry tests)
/src            - Web application & dashboard frontend (React/Vite)
/supabase       - Supabase migrations, edge functions & database schema
/landing        - Marketing and product landing page
/scripts        - Simulation, seed data, and utility scripts
/docs           - System documentation & collar contracts
```

---

## Setup

### Frontend Application
```bash
# Install dependencies
npm install

# Start development server
npm run dev
```

### Firmware (ESP32)
Build and flash the collar firmware using PlatformIO or Arduino IDE targeting ESP32 boards (located in `/firmware`).

### Backend / Supabase
Configure your `.env` with Supabase project URL and API keys (see `.env.example`). Apply database schemas in `/supabase`.

---

## Team

**Team JARVIS** — Smart India Hackathon
