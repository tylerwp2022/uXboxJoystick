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
  +-------------------------------------------+        | uAutoGrab  (proximity flag     |
  | uFldShoreBroker (qbridge) --> pShare      | -----> |             grab requests)     |
  +-------------------------------------------+        | uGfxMask   (station-ring mask  |
  | uFldFlagHomeManager (stock, unmodified)   |        |             while XBOX_DRIVEN) |
  +-------------------------------------------+        | pMarinePID / uSimMarine        |
                                                       +--------------------------------+
```

Vehicle-side companion apps exist for one architectural reason each, both
discovered the hard way: uAutoGrab because the flag manager rejects grab
requests whose vname does not match the originating community (a shoreside
requester can never pass), and uGfxMask because pMarineViewer stores geo
shapes per source community (a shoreside erase can never remove a
vehicle-posted graphic). Ordinary MOOS apps keep running under
MOOS_MANUAL_OVERRIDE -- only the helm parks -- which is what makes
vehicle-side companions viable during teleop at all.

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
| `XBOX_DRIVEN_<V>` | uXboxJoystick (hold mode) | `XBOX_DRIVEN` | uGfxMask | Graphics-mask trigger: true at acquire, false at release. pMarineViewer keys geo shapes per SOURCE community, so only a vehicle-originated erase can remove the helm's station rings; uGfxMask erases them at 2 Hz while this is true |
| `FLAG_SUMMARY` | uFldFlagHomeManager | `bridge = src=FLAG_SUMMARY` (to all) | uAutoGrab | Flag positions, grab range, carrier |
| `FLAG_GRAB_REQUEST` | uAutoGrab (vehicle-local post) | vehicle->shore via node broker | uFldFlagHomeManager | Proximity grab; passes the vname==community anti-spoofing check because it originates on the vehicle |
| `REDIR_HOLD_UPDATE_<V>` | uXboxJoystick (hold mode) | `REDIR_HOLD_UPDATE` | station_hold `updates` | Explicit `station_pt=x,y` at every release. Needed because center_activate sees no idle→running edge across a manual-override period (the helm parks and never iterates behaviors), so without it a re-released boat returns to its FIRST captured point |
| `XBOX_CONTROL_STATUS` | uXboxJoystick | (shoreside only) | UIs, alogs | State-change announcements |
| `NODE_REPORT` | vehicles | — | both operator apps | Headings (global frame), positions (graphics) |
| `TAGGED_VEHICLES`, `TAGGED_<V>` | tag manager | — | uXboxJoystick, .bhv modes | Tag suppression / interlocks |

Required `uFldShoreBroker` lines (all present in `meta_shoreside.moos`):
`qbridge = DESIRED_THRUST, DESIRED_RUDDER`, `qbridge = MOOS_MANUAL_OVERRIDE`,
`qbridge = MOOS_MANUAL_OVERIDE`, `qbridge = REDIR_CONTROLLED`,
`qbridge = REDIR_ACTIVE`, `qbridge = REDIR_UPDATE`,
`qbridge = REDIR_HOLD_UPDATE`, `qbridge = XBOX_DRIVEN`, plus the plain
broadcast `bridge = src=FLAG_SUMMARY` for uAutoGrab. All verified present and
working, July 2026.

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

**Roster parking (START, hold mode, park_unselected=true).** Activation
latches REDIR_CONTROLLED=true on EVERY rostered boat, not just the selected
pair. Unselected boats wake out of their game behaviors into station_hold and
do nothing until a stick cycles onto them. A tagged parked boat autonomously
runs waypt_untag to the untag zone (game behaviors gated the whole way) and
re-parks. The latch is defended roster-wide by the 1 Hz re-assert and survives
deactivation — boats rejoin game play only via pRedirectWaypoint (b/r + w).
Two side effects of station_hold's pwt 140: the viewer RETURN button (pwt 100
waypt_return_home) does not move parked boats, and only recover (300) and
max_spd2 (500) from common.bhv can outbid a parked boat — both safety
behaviors, verified July 2026.

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

**Tagged while driven (Xbox).** The boat stays fully controllable (the app
marks it `(T)`; game legality is the operator's call). Pressing Y/B dispatches
an autonomous untag: override drops with the REDIR latch kept on, the helm
wakes into MODE==TAGGED, waypt_untag (ungated by design) recovers the boat
home under the REDIR gate, and uXboxJoystick re-acquires the instant the tag
clears — the stick gets the boat back automatically. A second press reclaims
mid-return. If instead the boat is simply released while tagged (cycle/START),
the TAGGED interlock keeps station_hold down, waypt_untag recovers it, then
station_hold re-captures at the untag point.

**Powerplay-disabled while held.** Same shape via the DISABLED interlock:
after release the boat is dead in the water under waypt_disabled until
re-enabled, then station_hold catches it.

**Station rings during teleop (uGfxMask, vehicle-side).** station_hold
erases its two viewable rings only on a running->idle transition, and an
override-parked helm never iterates behaviors -- so acquiring a boat orphans
the last-drawn rings on the viewer. No shoreside erase can remove them:
pMarineViewer files geo shapes per SOURCE community, so a shoreside-origin
erase lands in the wrong shape map regardless of label. uXboxJoystick
therefore raises XBOX_DRIVEN_<V> at acquire (dropped at release), and the
vehicle-side uGfxMask erases the ring labels at 2 Hz while it is up -- those
erases bridge up under the vehicle's community and land in the correct map.
On release the mask drops and the re-engaged behavior redraws its rings at
the new station point. Net operator semantics: rings only on boats not under
stick control.

**Flag grabs under teleop (uAutoGrab, vehicle-side).** Flag pickup is
request-driven, and uFldFlagHomeManager rejects any FLAG_GRAB_REQUEST whose
vname does not match the message's originating MOOS community (anti-spoofing;
its "invalid_vehicle_name" denial is really this check). The overridden helm
never posts the request and no shoreside app can pass the check, so the
companion uAutoGrab app runs ON each vehicle -- MOOS_MANUAL_OVERRIDE silences
only the helm, not apps -- and posts proximity grabs from the boat's own
community. Fires regardless of who is driving (teleop, station_hold, or
autonomy), gated locally on not-tagged and not-carrying, rate-limited, with
the stock manager still refereeing every request. Shoreside prerequisite:
bridge = src=FLAG_SUMMARY in uFldShoreBroker.

## 5. Porting checklist (new mission)

1. Vehicle `.bhv`: copy the two initializes, add the operator gate to every
   game behavior (audit for ones above pwt 100!), copy the operator pair with
   its `updates = REDIR_HOLD_UPDATE` line, keep untag/disabled/collision-
   avoidance ungated.
2. Vehicle apps: launch uAutoGrab and uGfxMask on each boat (defaults are
   correct for the MCTF naming/label conventions; `-e` on each documents the
   knobs).
3. Shoreside: the eight qbridge lines plus `bridge = src=FLAG_SUMMARY`;
   launch both operator apps (or just uXboxJoystick with `handback=autonomy`
   if the redirect feature is absent -- roster parking, hold handback, untag
   dispatch, and ring masking are all hold-mode features).
4. Verify pMarinePID is in the vehicle stack and honors at least one override
   spelling.
5. Run the test sequence in the uXboxJoystick README, then the cross-system
   walkthroughs above.

## 6. Debugging field notes (earned, July 2026)

Every hard bug in this integration fell to a scope, not a hypothesis. The
recurring patterns, in the order they will save time:

1. **Per-vehicle variable works shoreside, dead on the vehicle -> check the
   qbridge FIRST.** Three separate incidents (DESIRED_THRUST/RUDDER, the
   REDIR family, XBOX_DRIVEN) presented as app bugs and were each a missing
   uFldShoreBroker line. Remember the nsplug ritual: edit the META file, then
   kill and relaunch -- targ files are regenerated at launch.
2. **The vehicle DB is the clean observation point** for anything
   behavior-posted: `uXMS targ_<boat>.moos <VAR>` carries none of the
   shoreside apps' traffic, and behavior posts self-identify via their
   `source=` field.
3. **Error strings describe where code landed, not why.** The flag manager's
   `invalid_vehicle_name` is really a community-mismatch check; two rounds of
   name-casing fixes chased the message instead of the mechanism. One
   discriminating experiment (uPokeDB from the vehicle community) beats
   iterating on the likeliest hypothesis.
4. **Community is identity.** The flag manager validates request-vname
   against originating community; pMarineViewer files geo shapes per source
   community. Anything that must act *as* a vehicle -- grab requests, graphic
   erases -- must run *on* the vehicle. MOOS_MANUAL_OVERRIDE parks only the
   helm, so vehicle-side companion apps keep working during teleop.
5. **IVPHELM_STATE is the takeover ground truth**: PARK while driving means
   the override took; DRIVE with overrides true would mean the helm is
   fighting the operator. An override-parked helm also never gives behaviors
   the idle transition that erases their viewables or re-arms
   center_activate -- the root of both the stale-station-point and
   orphaned-ring bugs.
