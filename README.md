# uXboxJoystick

Shoreside dual-stick Xbox controller teleop for MOOS-IvP fleets, built for the
MCTF/Aquaticus mission family. Each analog stick drives one vehicle; the
operator can cycle either stick across the roster, switch each stick between a
body-relative and a world-relative control frame, and brake/reverse with the
analog triggers. On-viewer graphics show which boats are under manual control,
where they are being commanded, and how hard they are braking.

Author: Tyler Errico, West Point Robotics Research Center

---

## Controller layout (xpad defaults, all remappable)

| Input | Action |
|-------|--------|
| **Left stick**  | Drive the left-stick vehicle |
| **Right stick** | Drive the right-stick vehicle |
| **START** | Master teleop toggle: activate / release-all (see Handback) |
| **LB** | Cycle which vehicle the left stick controls |
| **RB** | Cycle which vehicle the right stick controls |
| **X**  | Toggle left stick frame (body <-> global) |
| **A**  | Toggle right stick frame (body <-> global) |
| **LT** | Analog brake/reverse, left-stick boat |
| **RT** | Analog brake/reverse, right-stick boat |

Cycling skips the boat the *other* stick owns, so two sticks can never fight
over one vehicle. All button/axis indices are config parameters because the
kernel `xpad` driver and `xboxdrv` number them differently — run
`jstest /dev/input/js0` to confirm yours.

## Control model

**Sticks are forward-only course commands.** In both frames: stick direction =
where to go, stick magnitude = forward thrust, rudder = `heading_kp * error`
(clamped at `max_rudder`). Sticks never command reverse — point the stick
astern and the boat drives forward under full rudder and loops around to face
the direction of travel. The frames differ only in the error reference:

- **Body frame** — error is the stick's angle off the *bow* (`atan2(x, y)`).
  Needs no NODE_REPORT.
- **Global frame** — the stick is a world course vector (up = north, right =
  east); error = course − actual heading from NODE_REPORT. Until the first
  report arrives the frame degrades safely to straight thrust, zero rudder,
  with a retractable run warning.

**Triggers are the only reverse source.** While pulled past
`trigger_deadzone`, the trigger is the *exclusive* thrust source
(`thrust = −pull × max_thrust`); the stick contributes rudder only, so the
boat steers while backing. Release, and thrust reverts to the stick.

The stick deadzone is radial, and inside it the app posts explicit zeros
rather than going silent, so a released boat coasts to a stop instead of
holding its last command.

## Activation, handback, and precedence

The app launches **inactive**: helms own the boats, nothing is posted. X/A/
LB/RB still work while inactive as silent pre-staging. START activates teleop.

On acquire (activation, or cycling onto a boat) the app posts, per vehicle:
`MOOS_MANUAL_OVERRIDE_<VNAME>=true` (both spellings — see Troubleshooting),
and in `handback=hold` mode it also pre-latches `REDIR_CONTROLLED_<VNAME>=true`
and cancels any in-flight click-waypoint (`REDIR_ACTIVE_<VNAME>=false`).

Every release (cycle-off or START) is the same non-configurable path: one zero
thrust/rudder post, then the override drops. What the helm wakes into is the
`handback` mode:

- **`hold`** (default) — the REDIR gate was latched at acquire, so the helm
  wakes directly into `station_hold`: the boat actively parks where the
  operator left it. Requires the click-to-redirect behavior patches
  (`station_hold` + `REDIR_CONTROLLED` condition gates) in the vehicle `.bhv`.
  Release the boat to game play later via the pRedirectWaypoint b/r + w
  workflow.
- **`autonomy`** — no `REDIR_*` variable is ever touched; the helm resumes
  whatever its behavior conditions dictate. Use on missions without the
  redirect patches.

**Precedence:** the override is re-asserted with every command pair (tick
rate), because the standard DEPLOY button posts `MOOS_MANUAL_OVERRIDE_ALL=false`
to every boat and would otherwise clobber a one-shot override mid-teleop.
Rule: while ACTIVE, teleop owns its two boats; DEPLOY deploys everyone else;
only START releases. The hold-mode REDIR gate is likewise re-asserted at 1 Hz
against `pRedirectWaypoint::recomputeControl()` stomps.

**Tag rules:** with `respect_tags=true` (default), a tagged boat receives a
zero-command stream while tagged — manual override is not a loophole in the
game's tag rules.

## Viewer graphics

All graphics are plain `VIEW_*` postings (pMarineViewer renders them with zero
configuration), labeled per-stick so cycling boats moves them, with
`label_color=invisible` so no text clutters the screen. Clears are full
parseable specs with `active=false` (some viewer builds reject label-only
clears).

- **Control ring** (`VIEW_CIRCLE`) — a filled halo following each controlled
  boat: green = left stick, dodger blue = right. Hidden while the boat is
  tagged, so the tag manager's own graphic shows unambiguously.
- **Command-intent arrowhead** (`VIEW_POLYGON`) — rides the ring, pointing
  where the stick is commanding the boat to go (frame-aware, so it doubles as
  the frame indicator); fill density = stick magnitude. Clears when there is
  no commanded direction to show.
- **Brake gauge** (`VIEW_POLYGON` pair) — a vertical fill-bar east of the
  ring, visible only while that trigger is pulled; red fill rises with pull
  depth inside an outline in the stick's color.

The app also publishes `XBOX_CONTROL_STATUS` on every state change (e.g.
`active=true,left=red_one:body,right=red_three:global`) for deploy UIs,
scoreboards, and alog scraping.

## Mission integration

**1. Shoreside `.moos`** — launch the app and add the command-stream relays to
`uFldShoreBroker` (the `REDIR_*` qbridges from the click-to-redirect feature
should already be present):

```
// ANTLER block
Run = uXboxJoystick @ NewConsole = true

// uFldShoreBroker block
qbridge = DESIRED_THRUST, DESIRED_RUDDER
qbridge = MOOS_MANUAL_OVERRIDE
qbridge = MOOS_MANUAL_OVERIDE   // historic one-R alias -- see Troubleshooting
```

**2. Vehicle `.bhv`** (only for `handback=hold`) — apply the click-to-redirect
patches: `initialize REDIR_CONTROLLED/REDIR_ACTIVE = false`, the
`condition = REDIR_CONTROLLED != true` operator gate on every game behavior
(including `cutrange_aggressive_`), and the `station_hold` / `waypt_redirect`
behavior pair with `TAGGED != true` and `DISABLED != true` interlocks.

**3. Vehicle sim stack** — pMarinePID must be in the loop and honoring the
override so the raw thrust/rudder reach uSimMarine uncontested.

## Example config

Run `uXboxJoystick -e` for the authoritative block. Summary:

| Parameter | Default | Notes |
|-----------|---------|-------|
| `vehicles` | — | Ordered roster; cycle order. Left stick starts on slot 1, right on slot 2 |
| `joystick_device` | `/dev/input/js0` | Hot-unplug tolerated; reopen retried each tick |
| `max_thrust` | 100 | Stick/trigger at full deflection |
| `max_rudder` | 45 | Degrees; also the P-controller clamp |
| `deadzone` | 0.10 | Radial, normalized stick units |
| `trigger_deadzone` | 0.05 | Pull below this = no brake |
| `heading_kp` | 1.5 | Rudder deg per deg of error, pre-clamp. Body-frame feel: rudder saturates at ~30° stick angle at 1.5; drop toward 0.5 for finer gradation |
| `handback` | `hold` | `hold` or `autonomy` (see above) |
| `respect_tags` | true | Tagged boat gets zero commands |
| `active_at_start` | false | Safe default: autonomy until START |
| `left_frame` / `right_frame` | `body` | Startup frame per stick |
| `ring_radius` | 10 | Meters |
| `left_ring_color` / `right_ring_color` | green / dodger_blue | |
| `triangle_length` / `triangle_width` | 4 / 2.5 | Intent arrowhead, meters |
| `brake_bar_width` / `brake_bar_height` / `brake_bar_color` | 2 / 12 / red | Brake gauge |
| `axis_*` / `button_*` | xpad numbering | Remap for xboxdrv etc. |
| `debug` | false | Timestamped event buffer in the appcast |

Recommended `AppTick`/`CommsTick`: 25 (smooth command stream).

## Build

Drop the folder into the mission tree's `src/` alongside the other apps; the
`CMakeLists.txt` follows the standard pattern (links `contacts`, `apputil`,
`geometry`, `mbutil`). Linux-only: reads the classic joystick API
(`linux/joystick.h`).

## Test sequence (sim, timewarp 4)

1. Launch: appcast shows `Teleop: inactive`, device OPEN, roster listed.
   Untouched controller: Brake column reads 0.00 (not 0.50 — INIT quirk guard).
2. DEPLOY: boats play normally; app posts nothing.
3. START: rings + `** ACTIVE **`; boats freeze then answer the sticks.
4. DEPLOY while active: held boats stay held (≤1 tick twitch); others deploy.
5. Stick astern in body frame: boat loops around forward.
6. Trigger: gauge appears, boat backs, stick steers it while backing.
7. LB cycle-off: released boat station-holds in place; ring jumps onward.
8. START release: both boats hold; all graphics clear.
9. Release a held boat to game play via pRedirectWaypoint (b/r + w).
10. Tag a held boat: commands zero, ring hides, boat recovers home, untags,
    re-stations.

## Troubleshooting

- **Boats don't move; vehicle DB shows thrust 0 but override true.** The
  command-stream qbridges are missing from `uFldShoreBroker` — this exact
  symptom is how it presents. Add them (see Mission integration), then
  relaunch: nsplug regenerates `targ_` files at launch, so editing the meta
  file while running does nothing (and editing `targ_` directly gets
  overwritten).
- **Vehicle `DESIRED_THRUST` alternates between stick values and 0, source
  flipping to `pMarinePID`.** That PID build only honors the historic one-R
  `MOOS_MANUAL_OVERIDE`; missing it, PID treats the silenced helm as tardy and
  zero-stomps an all-stop. The app posts both spellings — make sure the one-R
  qbridge is present too.
- **Boats run autonomy while "in controller mode" after DEPLOY.** Old binary:
  rebuild. The DEPLOY button blanket-posts override=false; current builds
  re-assert at tick rate.
- **Both boats reverse at ~50% on startup.** xpad reports raw 0 (= half pull)
  for untouched triggers in the INIT replay; current builds ignore trigger
  INIT events. Rebuild.
- **Text next to the graphics.** Labels render on some builds; current specs
  carry `label_color=invisible`. Rebuild.
- **D-pad/axis weirdness.** Wrong driver numbering — `jstest` and remap via
  the `axis_*`/`button_*` params.
