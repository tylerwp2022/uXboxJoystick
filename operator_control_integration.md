# Operator Control Integration — uXboxJoystick + pRedirectWaypoint

How manual operator control is wired through the MCTF mission family: the two
shoreside operator apps, the uFldShoreBroker relay layer, and the vehicle-side
behavior contract in `meta_surveyor.bhv`. This documents the system as
integrated and sim-verified in July 2026.

Author: Tyler Errico, West Point Robotics Research Center

---

## 1. Component map

```
                        SHORESIDE                                VEHICLE (each boat)
  +-------------------+   +--------------------+       +--------------------------------+
  | uXboxJoystick     |   | pRedirectWaypoint  |       | pHelmIvP  (meta_surveyor.bhv)  |
  | dual-stick teleop |   | click-to-redirect  |       |   game bhvs   (pwt <= 100)     |
  +---------+---------+   +---------+----------+       |   station_hold      (pwt 140)  |
            |    per-vehicle vars   |                  |   waypt_redirect    (pwt 150)  |
            v                       v                  +---------------+----------------+
  +-------------------------------------------+                        |
  | uFldShoreBroker (qbridge) --> pShare      | ---------------------> | pMarinePID
  +-------------------------------------------+                        | uSimMarine
                                                                       v
```

Two operator systems share one vehicle-side contract:

- **pRedirectWaypoint** — behavior-level control. The helm never stands down;
  the operator's clicks drive the `waypt_redirect` behavior and the
  `REDIR_CONTROLLED` gate parks boats via `station_hold`.
- **uXboxJoystick** — actuator-level teleop. `MOOS_MANUAL_OVERRIDE` silences
  the helm and pMarinePID entirely; raw `DESIRED_THRUST/RUDDER` flow to
  uSimMarine. It *borrows* the REDIR gate for its handback (below).

## 2. Variable interface

| Variable (shoreside form) | Publisher | Bridged as | Consumer | Purpose |
|---|---|---|---|---|
| `DESIRED_THRUST_<V>` | uXboxJoystick | `DESIRED_THRUST` | uSimMarine | Teleop thrust, tick rate |
| `DESIRED_RUDDER_<V>` | uXboxJoystick | `DESIRED_RUDDER` | uSimMarine | Teleop rudder, tick rate |
| `MOOS_MANUAL_OVERRIDE_<V>` | uXboxJoystick, viewer buttons | `MOOS_MANUAL_OVERRIDE` | pHelmIvP, pMarinePID | Helm/PID stand-down. Re-asserted every tick while active |
| `MOOS_MANUAL_OVERIDE_<V>` | uXboxJoystick | `MOOS_MANUAL_OVERIDE` | pMarinePID (one-R-only builds) | Historic alias; without it PID tardy-stomps zeros |
| `REDIR_CONTROLLED_<V>` | pRedirectWaypoint, uXboxJoystick (hold mode) | `REDIR_CONTROLLED` | .bhv conditions | Operator gate: game bhvs OFF, station_hold ON |
| `REDIR_ACTIVE_<V>` | pRedirectWaypoint; uXboxJoystick posts false at acquire | `REDIR_ACTIVE` | .bhv conditions | Click-waypoint in flight |
| `REDIR_UPDATE_<V>` | pRedirectWaypoint | `REDIR_UPDATE` | waypt_redirect `updates` | Waypoint payload |
| `XBOX_CONTROL_STATUS` | uXboxJoystick | (shoreside only) | UIs, alogs | State-change announcements |
| `NODE_REPORT` | vehicles | — | both operator apps | Headings (global frame), positions (graphics) |
| `TAGGED_VEHICLES`, `TAGGED_<V>` | tag manager | — | uXboxJoystick, .bhv modes | Tag suppression / interlocks |

Required `uFldShoreBroker` lines (all present in `meta_shoreside.moos`):
`qbridge = DESIRED_THRUST, DESIRED_RUDDER`, `qbridge = MOOS_MANUAL_OVERRIDE`,
`qbridge = MOOS_MANUAL_OVERIDE`, `qbridge = REDIR_CONTROLLED`,
`qbridge = REDIR_ACTIVE`, `qbridge = REDIR_UPDATE`.

## 3. The .bhv contract (meta_surveyor.bhv)

### Initializes — MANDATORY

```
initialize REDIR_CONTROLLED = false
initialize REDIR_ACTIVE     = false
```

A helm condition referencing a never-posted variable makes that behavior
unrunnable. Without these two lines the operator gates below would kill every
game behavior at launch. Any boat that carries the gates MUST carry the
initializes.

### The operator gate

`condition = REDIR_CONTROLLED != true` is on exactly these six behaviors:
`waypt_grab_easy`, `waypt_grab_medium`, `loiter_passive`,
`cutrange_aggressive_`, `aggressive_station_keep`, `rlagent_attacker`.

Deliberately NOT gated:

- `waypt_untag` — a tagged boat may recover home even while operator-
  controlled (station_hold stands down for it via its own interlock).
- `waypt_disabled` — powerplay disable outranks operator control.
- everything in `common.bhv <bhvs>` — reactive collision avoidance stays
  active regardless of who is in charge. **Assumption:** nothing in common.bhv
  runs at pwt >= 140 except safety behaviors that *should* outbid the operator
  pair.

Note the gate is condition-line based, not MODE-based: a held boat's MODE
still computes normally (it can be `ATTACKING_EASY:TAGGED` etc. while held).
That is what lets waypt_untag keep working under a hold.

### Priority stack (highest bid wins)

| pwt | Behavior | Runs when |
|----:|----------|-----------|
| 150 | `waypt_redirect` | REDIR_CONTROLLED && REDIR_ACTIVE && !TAGGED && !DISABLED |
| 140 | `station_hold`   | REDIR_CONTROLLED && !TAGGED && !DISABLED |
| 100 | `waypt_disabled`, `loiter_passive`, `aggressive_station_keep` | per mode |
|  95 | `cutrange_aggressive_` | per mode + AGGRESSIVE |
|  50 | grab behaviors, `waypt_untag`, `rlagent_attacker` | per mode |

Invariant: **the operator pair must outbid every game behavior** (140/150 >
100 max). If a future game behavior is added above pwt 100, keep it below 140
or the hold breaks silently.

### Interlocks on the operator pair

- `TAGGED != true` — station_hold at 140 would otherwise outbid waypt_untag
  at 50 and pin a tagged boat on station forever. With the interlock, a
  tagged held boat drives home, untags, and station_hold re-captures wherever
  it re-activates (`center_activate = true`).
- `DISABLED != true` — a powerplay-disabled boat must sit dead in the water
  (waypt_disabled, speed 0), not actively station-keep.

## 4. State walkthroughs

**Xbox acquire (START, or cycling onto a boat).** App posts
`REDIR_ACTIVE=false` (cancels any in-flight click-waypoint),
`REDIR_CONTROLLED=true` (pre-latches the handback gate — inert now, decisive
later), then override=true (both spellings). Helm and PID stand down; raw
thrust/rudder drive uSimMarine. Ring appears.

**Xbox release (cycle-off or START).** One zero thrust/rudder post, override
drops (both spellings, false). The helm wakes and finds REDIR_CONTROLLED=true
already latched: game behaviors are gated off, station_hold `center_activate`
captures the boat's current position. **The boat actively parks where the
operator left it** — no race, because the gate was latched at acquire, not at
release. Release to game play later via pRedirectWaypoint (b/r + w) or a
click-waypoint.

**DEPLOY while teleop active.** The DEPLOY button blanket-posts
override=false to all boats. uXboxJoystick re-asserts override every tick, so
a held boat is exposed for at most one AppTick (~40 ms at 25 Hz). Precedence:
teleop owns its two boats; DEPLOY deploys everyone else; only START releases.

**pRedirectWaypoint toggles teams while Xbox holds a boat.** Its
recomputeControl() posts REDIR_CONTROLLED=false across uncontrolled teams —
including an Xbox-parked boat. uXboxJoystick re-asserts the gate at 1 Hz, so
the un-latch is transient. Two writers, one variable: acceptable for a single
operator, documented here so nobody chases the flicker in the alog.

**Tagged while held (Xbox).** `respect_tags=true`: the app streams zero
commands and hides the ring; the boat sits (still overridden). On release, the
TAGGED interlock keeps station_hold down, waypt_untag recovers the boat home,
then station_hold re-captures at the untag point.

**Powerplay-disabled while held.** Same shape via the DISABLED interlock:
after release the boat is dead in the water under waypt_disabled until
re-enabled, then station_hold catches it.

## 5. Porting checklist (new mission)

1. Vehicle `.bhv`: copy the two initializes, add the operator gate to every
   game behavior (audit for ones above pwt 100!), copy the operator pair,
   keep untag/disabled/collision-avoidance ungated.
2. Shoreside: the six qbridge lines; launch both operator apps (or just
   uXboxJoystick with `handback=autonomy` if the redirect feature is absent).
3. Verify pMarinePID is in the vehicle stack and honors at least one override
   spelling.
4. Run the 10-step test sequence in the uXboxJoystick README, then the four
   cross-system walkthroughs above.
