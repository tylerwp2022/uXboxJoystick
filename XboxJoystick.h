/*****************************************************************/
/*    NAME: Tyler Errico                                         */
/*    ORGN: West Point Robotics Research Center                  */
/*    FILE: XboxJoystick.h                                       */
/*    DATE: 2026                                                 */
/*                                                               */
/*  Shoreside dual-stick Xbox controller teleop for MOOS-IvP.    */
/*                                                               */
/*  PURPOSE                                                      */
/*  -------                                                      */
/*  Drive TWO vehicles simultaneously from one Xbox controller:  */
/*    LEFT stick  -> one vehicle                                 */
/*    RIGHT stick -> another vehicle                             */
/*                                                               */
/*  Button map (defaults, all remappable in config):             */
/*    START -> master ACTIVE toggle (see below)                  */
/*    LB -> cycle which vehicle the LEFT stick controls          */
/*    RB -> cycle which vehicle the RIGHT stick controls         */
/*    X  -> toggle LEFT stick control frame (body <-> global)    */
/*    A  -> toggle RIGHT stick control frame (body <-> global)   */
/*    LT -> proportional brake/reverse, LEFT stick's boat        */
/*    RT -> proportional brake/reverse, RIGHT stick's boat       */
/*  (Bumpers switch boats -- index fingers, thumbs never leave   */
/*  the sticks. Face buttons take the rarer frame toggle.)       */
/*                                                               */
/*  TRIGGER BRAKE/REVERSE -- THE ONLY REVERSE SOURCE             */
/*  ------------------------------------------------             */
/*  While pulled past its deadzone, the trigger is the EXCLUSIVE */
/*  thrust source: thrust = -pull * max_thrust, and the stick's  */
/*  magnitude is ignored for thrust entirely. The stick keeps    */
/*  contributing rudder, so the boat steers while backing; full  */
/*  trigger with the stick centered is straight-line reverse.    */
/*  Release the trigger and thrust reverts to the stick. Works   */
/*  identically in both frames.                                  */
/*                                                               */
/*  MASTER ACTIVE TOGGLE                                         */
/*  --------------------                                         */
/*  The app launches INACTIVE (config: active_at_start): boats   */
/*  run their normal autonomy and this app posts NO commands.    */
/*  Pressing START activates teleop:                             */
/*    ACTIVE:   MOOS_MANUAL_OVERRIDE_<VNAME>=true for both       */
/*              assigned boats; thrust/rudder stream at AppTick. */
/*    INACTIVE: both boats are parked (one zero thrust/rudder    */
/*              post) and MOOS_MANUAL_OVERRIDE_<VNAME>=false is  */
/*              posted -- full handback to the helm. Autonomy    */
/*              resumes; this app goes silent on the command     */
/*              variables.                                       */
/*  While inactive, X/A/LB/RB still work so the operator can     */
/*  pre-stage assignments and frames before taking control.      */
/*                                                               */
/*  PRECEDENCE VS. SHORESIDE BUTTONS: the override is RE-        */
/*  ASSERTED with every command pair (tick rate), because the    */
/*  standard DEPLOY button posts MOOS_MANUAL_OVERIDE_ALL=false   */
/*  to every boat and would otherwise clobber a one-shot         */
/*  override and wake the helm mid-teleop. Rule: while ACTIVE,   */
/*  teleop owns its two boats no matter what the viewer buttons  */
/*  post; DEPLOY deploys everyone else; only START releases.     */
/*  The hold-mode REDIR_CONTROLLED gate is likewise re-asserted  */
/*  at 1 Hz against pRedirectWaypoint recomputeControl() stomps. */
/*                                                               */
/*  VISUAL INDICATION                                            */
/*  -----------------                                            */
/*  While ACTIVE, each controlled boat gets a colored ring       */
/*  (VIEW_CIRCLE) that follows it via NODE_REPORT positions:     */
/*    left stick's boat  -> green ring   (left_ring_color)       */
/*    right stick's boat -> blue ring    (right_ring_color)      */
/*  Same house style as pRedirectWaypoint's magenta control      */
/*  halos (spec string, duration=0, light fill), so the two      */
/*  control systems read consistently on the viewer while        */
/*  remaining distinguishable by color. Rings are labeled per-   */
/*  STICK (xbox_left / xbox_right) so cycling vehicles just      */
/*  moves the ring. Tagged boats lose the ring while tagged      */
/*  (matching the redirect-halo convention; the tag manager's    */
/*  own circle shows instead). Clears are FULL circle specs with */
/*  active=false -- some pMarineViewer builds reject label-only  */
/*  clears as unhandled mail (lesson learned in                  */
/*  pRedirectWaypoint::clearControlCircle).                      */
/*                                                               */
/*  COMMAND-INTENT TRIANGLE (VIEW_POLYGON)                       */
/*  --------------------------------------                       */
/*  An arrowhead rides on each control ring, pointing in the     */
/*  direction the operator is COMMANDING the boat to go:         */
/*    global frame: the stick's compass course, atan2(x,y) --    */
/*      independent of where the bow currently points.           */
/*    body frame:   bow-relative, heading + atan2(x,y) -- stick  */
/*      up = out the bow, stick right = 90 off the bow, stick    */
/*      down (reverse) = astern.                                 */
/*  Toggling LB/RB re-renders the same deflection in the new     */
/*  frame instantly, so the triangle doubles as a visual         */
/*  confirmation of which frame each stick is in.                */
/*  Fill density is proportional to stick magnitude: a light tap */
/*  draws a faint wash, full deflection a solid arrowhead.       */
/*  Inside the deadzone there is no commanded direction, so the  */
/*  triangle CLEARS rather than pointing somewhere stale; ditto  */
/*  when tag-suppressed or when the needed NODE_REPORT data      */
/*  (position; heading too in body frame) is not yet known.      */
/*                                                               */
/*  BRAKE GAUGE (VIEW_POLYGON pair)                              */
/*  -------------------------------                              */
/*  A small vertical fill-bar just east of the control ring,     */
/*  visible ONLY while that stick's trigger is pulled past its   */
/*  deadzone -- zero standing clutter. Two rectangles: a fixed   */
/*  outline in the stick's color (identity) and a red fill that  */
/*  rises bottom-up with pull depth (brake semantics). Follows   */
/*  the boat and clears under the same conditions as the other   */
/*  graphics.                                                    */
/*                                                               */
/*  The app also publishes XBOX_CONTROL_STATUS on every state    */
/*  change (activation, cycle, frame toggle), e.g.:              */
/*    "active=true,left=red_one:body,right=red_three:global"     */
/*  so other apps (scoreboards, deploy UIs) can display or log   */
/*  who is under manual control.                                 */
/*                                                               */
/*  CONTROL FRAMES -- STICKS ARE FORWARD-ONLY COURSE COMMANDS    */
/*  -----------------------------------------------------        */
/*  Both frames share one model: stick direction = where to go,  */
/*  magnitude = forward thrust, rudder = kp * heading error.     */
/*  Sticks NEVER command reverse; point the stick astern and the */
/*  boat drives forward and loops around to face the direction   */
/*  of travel. The frames differ only in the error reference:    */
/*                                                               */
/*  BODY frame: error = stick angle off the BOW (atan2(x,y)).    */
/*    Needs no NODE_REPORT.                                      */
/*                                                               */
/*  GLOBAL frame: the stick is a COURSE VECTOR in the world      */
/*    (up=000, right=090); error = course - actual heading.      */
/*  The app closes the heading loop itself with a P-controller:  */
/*    rudder = kp * angle180(desired_hdg - actual_hdg)           */
/*  using the vehicle's heading from NODE_REPORT. Output is thus */
/*  ALWAYS thrust/rudder regardless of frame -- the vehicle side */
/*  never knows or cares which frame the operator is in.         */
/*                                                               */
/*  If no NODE_REPORT heading is known for a boat yet, global    */
/*  frame degrades safely: zero rudder, thrust from magnitude,   */
/*  plus a run warning until the first report arrives.           */
/*                                                               */
/*  OUTPUT / BRIDGING MODEL                                      */
/*  -----------------------                                      */
/*  This app runs SHORESIDE and publishes per-vehicle variables: */
/*    DESIRED_THRUST_<VNAME>                                     */
/*    DESIRED_RUDDER_<VNAME>                                     */
/*    MOOS_MANUAL_OVERRIDE_<VNAME>                               */
/*  plus (handback=hold) REDIR_CONTROLLED / REDIR_ACTIVE.        */
/*  uFldShoreBroker relays them via qbridge (house convention,   */
/*  same as pRedirectWaypoint's REDIR_* vars):                   */
/*    qbridge = DESIRED_THRUST                                   */
/*    qbridge = DESIRED_RUDDER                                   */
/*    qbridge = MOOS_MANUAL_OVERRIDE                             */
/*  (REDIR_CONTROLLED / REDIR_ACTIVE qbridges already exist for  */
/*  the click-to-redirect feature.)                              */
/*                                                               */
/*  HANDBACK MODEL (handback = hold | autonomy)                  */
/*  -------------------------------------------                  */
/*  With MOOS_MANUAL_OVERRIDE=true the helm stands down entirely */
/*  and uSimMarine consumes raw thrust/rudder. The interesting   */
/*  question is what happens when override DROPS. Two modes:     */
/*                                                               */
/*  hold (default -- requires the click-to-redirect bhv patches  */
/*  from pRedirectWaypoint: station_hold + REDIR_CONTROLLED      */
/*  condition gates on the game behaviors):                      */
/*    ACQUIRE: post REDIR_ACTIVE_<V>=false (cancel any in-flight */
/*      click-waypoint), REDIR_CONTROLLED_<V>=true, then         */
/*      MOOS_MANUAL_OVERRIDE_<V>=true. The REDIR gate is inert   */
/*      while the helm is overridden, but it is already latched  */
/*      on the vehicle -- so the eventual handback is race-free. */
/*    RELEASE: zero thrust/rudder + MOOS_MANUAL_OVERRIDE=false.  */
/*      The helm wakes with game behaviors gated OFF and         */
/*      station_hold active: the boat ACTIVELY PARKS at the spot */
/*      where the operator left it (not dead-stick drift, not    */
/*      surprise re-entry into game play). The operator then     */
/*      releases it to autonomy via the existing b/r + w         */
/*      workflow, or clicks it somewhere with pRedirectWaypoint. */
/*                                                               */
/*  autonomy (for missions WITHOUT the redirect bhv patches):    */
/*    ACQUIRE: MOOS_MANUAL_OVERRIDE=true only. No REDIR_* vars   */
/*      are touched in this mode -- clean separation, so a boat  */
/*      that was station-holding under pRedirectWaypoint returns */
/*      to exactly that state on release.                        */
/*    RELEASE: zero thrust/rudder + override=false; the helm     */
/*      resumes whatever its behavior conditions dictate.        */
/*                                                               */
/*  COEXISTENCE NOTE: if pRedirectWaypoint toggles team control  */
/*  (w) while the Xbox holds a boat of the OTHER team parked in  */
/*  hold mode, its recomputeControl() will post                  */
/*  REDIR_CONTROLLED=false for that boat and release it to game  */
/*  play. Two operator apps, one gate variable -- acceptable for */
/*  a single operator, worth knowing about.                      */
/*                                                               */
/*  DEVICE HANDLING                                              */
/*  ---------------                                              */
/*  Reads the classic Linux joystick API (/dev/input/js0) in     */
/*  non-blocking mode; every Iterate() drains all queued         */
/*  js_event structs. Hot-unplug is tolerated: the fd is closed  */
/*  on read error and re-open is attempted each Iterate() with a */
/*  retractable run warning. Axis/button numbering differs       */
/*  between the kernel xpad driver and xboxdrv, so every index   */
/*  is a config parameter (xpad defaults baked in). Trigger      */
/*  axes ignore the kernel's JS_EVENT_INIT replay: xpad reports  */
/*  raw 0 (= HALF PULL on a trigger's -32767..32767 range) until */
/*  first touched, which naively read would launch both boats    */
/*  backward at 50%% on startup. Triggers read fully-released    */
/*  until their first real event.                                */
/*                                                               */
/*  SAFETY BEHAVIORS                                             */
/*  ----------------                                             */
/*  * Every release (cycle or deactivate) is the same handback:  */
/*    one zero thrust/rudder post + override drop. Not           */
/*    configurable -- no config can leave a boat dead-stick or   */
/*    frozen under a stale override.                             */
/*  * respect_tags: a tagged (disabled) boat receives zero       */
/*    commands while tagged -- manual override must not become a */
/*    way to drive through the game's tag rules.                 */
/*  * Cycling skips the vehicle currently owned by the OTHER     */
/*    stick, so two sticks can never fight over one boat.        */
/*****************************************************************/

#ifndef XBOX_JOYSTICK_HEADER
#define XBOX_JOYSTICK_HEADER

#include <string>
#include <vector>
#include <list>
#include <set>
#include <map>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class XboxJoystick : public AppCastingMOOSApp
{
public:
  XboxJoystick();
  ~XboxJoystick();

protected: // Standard MOOSApp lifecycle
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();
  void registerVariables();
  bool buildReport();

protected: // Joystick device plumbing
  bool   openDevice();       // attempt open; true if fd is live after call
  void   closeDevice();
  void   pollDevice();       // drain all pending js_event structs
  void   handleAxis(unsigned int axis, int value, bool is_init);
  void   handleButton(unsigned int button, bool pressed);

protected: // Control logic
  // Identifies which physical stick a piece of state belongs to.
  enum StickID { LEFT_STICK = 0, RIGHT_STICK = 1 };

  // Per-stick bundle of live state. Kept in a fixed 2-slot array
  // (m_stick[LEFT_STICK], m_stick[RIGHT_STICK]).
  struct Stick {
    double x;              // normalized [-1,1], +right
    double y;              // normalized [-1,1], +UP (sign pre-flipped)
    double brake;          // trigger pull, [0,1] (0 = released)
    int    vix;            // index into m_vehicles, -1 = unassigned
    bool   global_frame;   // false = body frame
    double out_thrust;     // last computed command (for appcast)
    double out_rudder;
    Stick() : x(0), y(0), brake(0), vix(-1), global_frame(false),
              out_thrust(0), out_rudder(0) {}
  };

  void   cycleVehicle(StickID sid);       // X / A press
  void   toggleFrame(StickID sid);        // LB / RB press
  void   setActive(bool active);          // START press (master toggle)
  void   computeAndPost(StickID sid);     // one stick -> one boat
  void   releaseVehicle(int vix);         // zero + optional helm handback
  void   acquireVehicle(int vix);         // manual override on (if active)

  // Visual ring management (VIEW_CIRCLE, one per stick).
  void   postRing(StickID sid);           // draw/refresh at boat position
  void   clearRing(StickID sid);          // active=false the ring

  // Command-intent triangle (VIEW_POLYGON, one per stick): an
  // arrowhead on the ring pointing where the stick is commanding
  // the boat to go, fill density = stick magnitude.
  void   postDirTriangle(StickID sid);
  void   clearDirTriangle(StickID sid);

  // Brake gauge (VIEW_POLYGON pair per stick): outline + fill bar
  // east of the ring, fill height = trigger pull. Drawn only
  // while pulling.
  void   postBrakeBar(StickID sid);
  void   clearBrakeBar(StickID sid);

  // Publish XBOX_CONTROL_STATUS. Called on any state change.
  void   postStatus();

  // NODE_REPORT ingestion: cache heading per vehicle for the
  // global-frame P-controller, and x/y for ring placement.
  void   handleNodeReport(const std::string& sval);

  // TAGGED_VEHICLES / TAGGED_<VNAME> ingestion (same sources as
  // pRedirectWaypoint): tagged boats lose their control ring and,
  // if respect_tags, receive zero commands while tagged.
  void   handleTaggedVehicles(const std::string& csv);
  void   handleTagged(const std::string& vname, bool tagged);

  // Debug circular buffer (house style, cf. pRedirectWaypoint).
  void   debugLog(const std::string& line);

  std::string vehicleName(int vix) const; // "" if vix out of range

private: // Configuration (mission-portable knobs)
  std::vector<std::string> m_vehicles;   // roster, as-configured case
  std::string  m_device;                 // /dev/input/js0
  double       m_max_thrust;             // magnitude cap, e.g. 100
  double       m_max_rudder;             // degrees, e.g. 45
  double       m_deadzone;               // normalized, e.g. 0.10
  double       m_trigger_deadzone;       // pull below this = no brake
  double       m_heading_kp;             // global-frame P gain
  bool         m_active_at_start;        // launch already in teleop?
  bool         m_handback_hold;          // true=hold (REDIR park),
                                         // false=autonomy (no REDIR)
  bool         m_respect_tags;           // zero cmds to tagged boats

  // Ring appearance.
  double       m_ring_radius;
  std::string  m_left_ring_color;
  std::string  m_right_ring_color;

  // Intent-triangle geometry (base sits on the ring, apex points
  // outward along the commanded direction).
  double       m_tri_length;             // base-to-apex, meters
  double       m_tri_width;              // base width, meters

  // Brake-gauge geometry/appearance (meters; fill color fixed
  // semantics, outline takes the stick color).
  double       m_bar_width;
  double       m_bar_height;
  std::string  m_bar_fill_color;

  // Axis / button indices (xpad defaults; xboxdrv users remap).
  unsigned int m_ax_left_x,  m_ax_left_y;
  unsigned int m_ax_right_x, m_ax_right_y;
  unsigned int m_ax_left_trig, m_ax_right_trig;   // LT / RT
  unsigned int m_btn_cycle_left;    // X = 2
  unsigned int m_btn_cycle_right;   // A = 0
  unsigned int m_btn_frame_left;    // LB = 4
  unsigned int m_btn_frame_right;   // RB = 5
  unsigned int m_btn_active;        // START = 7

private: // Live state
  int          m_jsfd;                   // -1 = device not open
  bool         m_dev_warned;             // run-warning active for device
  bool         m_active;                 // master teleop state
  double       m_last_gate_assert;       // 1 Hz REDIR gate re-assert

  Stick        m_stick[2];

  // Latest heading and position per vehicle, keyed by LOWERCASE
  // vname. Presence in m_heading == at least one NODE_REPORT seen.
  std::map<std::string, double> m_heading;
  std::map<std::string, double> m_vx;
  std::map<std::string, double> m_vy;

  // Tagged (disabled) boats, lowercase vnames. Fed by
  // TAGGED_VEHICLES (authoritative csv) and TAGGED_<VNAME> edges.
  std::set<std::string> m_tagged;

  // Debug circular buffer (house style).
  bool                    m_debug;
  std::list<std::string>  m_debug_buffer;
  unsigned int            m_debug_buffer_max;

  // Appcast bookkeeping.
  unsigned int m_events_axis;
  unsigned int m_events_button;
  unsigned int m_posts;
};

#endif
