/*****************************************************************/
/*    NAME: Tyler Errico                                         */
/*    ORGN: West Point Robotics Research Center                  */
/*    FILE: XboxJoystick.cpp                                     */
/*    DATE: 2026                                                 */
/*****************************************************************/

#include <cmath>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>

#include "MBUtils.h"
#include "AngleUtils.h"      // angle180(), angle360(), radToDegrees()
#include "ACTable.h"
#include "NodeRecord.h"
#include "NodeRecordUtils.h" // string2NodeRecord()
#include "XboxJoystick.h"

using namespace std;

//---------------------------------------------------------------
// Constructor
//
// Defaults assume the in-kernel xpad driver:
//   axes:    0=LX  1=LY  2=LT  3=RX  4=RY  5=RT
//   buttons: 0=A  1=B  2=X  3=Y  4=LB  5=RB  6=BACK  7=START

XboxJoystick::XboxJoystick()
{
  m_device           = "/dev/input/js0";
  m_max_thrust       = 100;
  m_max_rudder       = 45;
  m_deadzone         = 0.10;
  m_trigger_deadzone = 0.05;
  m_heading_kp       = 1.5;
  m_active_at_start  = false;   // SAFE default: autonomy until START
  m_handback_hold    = true;    // hold: park via REDIR_CONTROLLED
                                // (needs click-to-redirect bhv patches)
  m_park_unselected  = true;    // hold mode: latch the WHOLE roster,
                                // so unselected boats hold, not play

  m_ring_radius      = 10;
  m_left_ring_color  = "green";
  m_right_ring_color = "dodger_blue";

  m_tri_length       = 4;    // arrowhead base-to-apex (meters)
  m_tri_width        = 2.5;  // arrowhead base width  (meters)

  m_bar_width        = 2;    // brake gauge width  (meters)
  m_bar_height       = 12;   // brake gauge height (meters)
  m_bar_fill_color   = "red";

  m_debug            = false;
  m_debug_buffer_max = 20;

  m_ax_left_x    = 0;
  m_ax_left_y    = 1;
  m_ax_right_x   = 3;
  m_ax_right_y   = 4;
  m_ax_left_trig  = 2;   // LT
  m_ax_right_trig = 5;   // RT

  m_btn_cycle_left  = 4;   // LB  (bumpers switch boats)
  m_btn_cycle_right = 5;   // RB
  m_btn_frame_left  = 2;   // X   (face buttons switch frames)
  m_btn_frame_right = 0;   // A
  m_btn_untag_left  = 3;   // Y   (dispatch auto-untag return)
  m_btn_untag_right = 1;   // B
  m_btn_active      = 7;   // START

  m_jsfd       = -1;
  m_dev_warned = false;
  m_active     = false;
  m_last_gate_assert = 0;

  m_events_axis   = 0;
  m_events_button = 0;
  m_posts         = 0;
}

//---------------------------------------------------------------
// Destructor

XboxJoystick::~XboxJoystick()
{
  closeDevice();
}

//---------------------------------------------------------------
// Procedure: OnNewMail

bool XboxJoystick::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if((key == "NODE_REPORT") || (key == "NODE_REPORT_LOCAL"))
      handleNodeReport(msg.GetString());
    else if(key == "TAGGED_VEHICLES")
      handleTaggedVehicles(msg.GetString());
    else if(strBegins(key, "TAGGED_")) {
      string vname = tolower(key.substr(7));
      handleTagged(vname, (tolower(msg.GetString()) == "true"));
    }
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }
  return(true);
}

//---------------------------------------------------------------
// Procedure: OnConnectToServer

bool XboxJoystick::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------------
// Procedure: Iterate()
//
// Order matters:
//   1) (Re)open the device if it is not open. Hot-unplug support.
//   2) Drain ALL queued joystick events. Button edges fire their
//      handlers immediately (cycle/frame changes); axis events
//      just update the cached stick positions.
//   3) Compute + post thrust/rudder once per stick per tick, from
//      the final cached stick position. Posting per-tick (not per
//      event) gives the boats a steady command stream at AppTick
//      rate no matter how fast the controller spams events.

bool XboxJoystick::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // (1) Device liveness -- retry open every tick until it works.
  if(m_jsfd < 0) {
    if(openDevice()) {
      if(m_dev_warned) {
        retractRunWarning("Joystick device unavailable: " + m_device);
        m_dev_warned = false;
      }
    }
    else if(!m_dev_warned) {
      reportRunWarning("Joystick device unavailable: " + m_device);
      m_dev_warned = true;
    }
  }

  // (2) Drain pending events.
  if(m_jsfd >= 0)
    pollDevice();

  // (3) Command the two active boats -- but ONLY in teleop mode.
  // While inactive this app is silent on all command variables;
  // the helms own their boats. Rings are refreshed here too so
  // they track the boats at AppTick rate.
  if(m_active) {
    // Untag-dispatch watcher: a dispatched boat is with its helm
    // running waypt_untag. The moment its tag clears, snap the
    // override back on -- the stick regains the boat without it
    // ever reaching a game behavior (the REDIR gate held those
    // off the whole return, hold mode).
    for(int i=0; i<2; i++) {
      Stick &stk = m_stick[i];
      if(!stk.untag_dispatch)
        continue;
      string vname = vehicleName(stk.vix);
      if(vname == "") {
        stk.untag_dispatch = false;
        continue;
      }
      if(m_tagged.count(tolower(vname)) == 0) {
        stk.untag_dispatch = false;
        clearHelmText((StickID)i);
        acquireVehicle(stk.vix);
        debugLog("Untag complete -- reclaimed " + vname);
        postStatus();
      }
    }

    computeAndPost(LEFT_STICK);
    computeAndPost(RIGHT_STICK);
    postRing(LEFT_STICK);
    postRing(RIGHT_STICK);
    postDirTriangle(LEFT_STICK);
    postDirTriangle(RIGHT_STICK);
    postBrakeBar(LEFT_STICK);
    postBrakeBar(RIGHT_STICK);
    postHelmText(LEFT_STICK);     // no-ops unless untag-dispatched
    postHelmText(RIGHT_STICK);
    postStickTag(LEFT_STICK);     // L / R while stick-driven
    postStickTag(RIGHT_STICK);

    // Slow (1 Hz) re-assert of the hold-mode handback gate. If
    // pRedirectWaypoint's recomputeControl() fires while we hold a
    // boat (operator toggles a team with 'w'), it posts
    // REDIR_CONTROLLED=false for every boat of uncontrolled teams
    // -- un-latching our parked handback. 1 Hz is enough: the gate
    // only matters at the moment override drops, and it costs two
    // small posts per second instead of 50.
    if((MOOSTime() - m_last_gate_assert) > 1.0) {
      if(m_handback_hold) {
        if(m_park_unselected) {
          // Roster-wide: parked boats need the latch defended
          // from pRedirectWaypoint recompute stomps just as much
          // as driven ones.
          for(unsigned int v=0; v<m_vehicles.size(); v++) {
            Notify("REDIR_CONTROLLED_" + toupper(m_vehicles[v]),
                   "true");
            m_posts++;
          }
        }
        else {
          for(int i=0; i<2; i++) {
            string vname = vehicleName(m_stick[i].vix);
            if(vname == "")
              continue;
            Notify("REDIR_CONTROLLED_" + toupper(vname), "true");
            m_posts++;
          }
        }
      }
      m_last_gate_assert = MOOSTime();
    }
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------------
// Procedure: OnStartUp()

bool XboxJoystick::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "vehicles") {
      // Comma-separated roster. Order defines cycle order.
      vector<string> svector = parseString(value, ',');
      for(unsigned int i=0; i<svector.size(); i++) {
        string vname = stripBlankEnds(svector[i]);
        if(vname != "")
          m_vehicles.push_back(vname);
      }
      handled = (m_vehicles.size() > 0);
    }
    else if(param == "joystick_device") {
      m_device = value;
      handled  = (value != "");
    }
    else if(param == "max_thrust")
      handled = setPosDoubleOnString(m_max_thrust, value);
    else if(param == "max_rudder")
      handled = setPosDoubleOnString(m_max_rudder, value);
    else if(param == "deadzone")
      handled = setNonNegDoubleOnString(m_deadzone, value);
    else if(param == "heading_kp")
      handled = setPosDoubleOnString(m_heading_kp, value);
    else if(param == "active_at_start")
      handled = setBooleanOnString(m_active_at_start, value);
    else if(param == "handback") {
      string lval = tolower(value);
      if(lval == "hold") {
        m_handback_hold = true;  handled = true; }
      else if(lval == "autonomy") {
        m_handback_hold = false; handled = true; }
    }
    else if(param == "park_unselected")
      handled = setBooleanOnString(m_park_unselected, value);
    else if(param == "debug")
      handled = setBooleanOnString(m_debug, value);
    else if(param == "ring_radius")
      handled = setPosDoubleOnString(m_ring_radius, value);
    else if(param == "left_ring_color") {
      m_left_ring_color = value;
      handled = (value != "");
    }
    else if(param == "right_ring_color") {
      m_right_ring_color = value;
      handled = (value != "");
    }
    else if(param == "triangle_length")
      handled = setPosDoubleOnString(m_tri_length, value);
    else if(param == "triangle_width")
      handled = setPosDoubleOnString(m_tri_width, value);
    else if(param == "brake_bar_width")
      handled = setPosDoubleOnString(m_bar_width, value);
    else if(param == "brake_bar_height")
      handled = setPosDoubleOnString(m_bar_height, value);
    else if(param == "brake_bar_color") {
      m_bar_fill_color = value;
      handled = (value != "");
    }
    else if(param == "left_frame") {
      string lval = tolower(value);
      if((lval == "global") || (lval == "body")) {
        m_stick[LEFT_STICK].global_frame = (lval == "global");
        handled = true;
      }
    }
    else if(param == "right_frame") {
      string lval = tolower(value);
      if((lval == "global") || (lval == "body")) {
        m_stick[RIGHT_STICK].global_frame = (lval == "global");
        handled = true;
      }
    }
    else if(param == "axis_left_x")
      handled = setUIntOnString(m_ax_left_x, value);
    else if(param == "axis_left_y")
      handled = setUIntOnString(m_ax_left_y, value);
    else if(param == "axis_right_x")
      handled = setUIntOnString(m_ax_right_x, value);
    else if(param == "axis_right_y")
      handled = setUIntOnString(m_ax_right_y, value);
    else if(param == "axis_left_trigger")
      handled = setUIntOnString(m_ax_left_trig, value);
    else if(param == "axis_right_trigger")
      handled = setUIntOnString(m_ax_right_trig, value);
    else if(param == "trigger_deadzone")
      handled = setNonNegDoubleOnString(m_trigger_deadzone, value);
    else if(param == "button_cycle_left")
      handled = setUIntOnString(m_btn_cycle_left, value);
    else if(param == "button_cycle_right")
      handled = setUIntOnString(m_btn_cycle_right, value);
    else if(param == "button_frame_left")
      handled = setUIntOnString(m_btn_frame_left, value);
    else if(param == "button_frame_right")
      handled = setUIntOnString(m_btn_frame_right, value);
    else if(param == "button_untag_left")
      handled = setUIntOnString(m_btn_untag_left, value);
    else if(param == "button_untag_right")
      handled = setUIntOnString(m_btn_untag_right, value);
    else if(param == "button_active")
      handled = setUIntOnString(m_btn_active, value);

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  if(m_vehicles.size() == 0)
    reportConfigWarning("No vehicles configured -- nothing to drive.");

  // Initial assignments: left stick takes roster slot 0, right
  // stick slot 1 (if it exists). No override is posted here --
  // setActive() below owns that, so a mission that launches with
  // active_at_start=false leaves the helms completely untouched.
  if(m_vehicles.size() >= 1)
    m_stick[LEFT_STICK].vix = 0;
  if(m_vehicles.size() >= 2)
    m_stick[RIGHT_STICK].vix = 1;

  // Enter the configured startup state. setActive(true) posts the
  // manual-override takeovers; setActive(false) is a no-op on the
  // command variables but still announces XBOX_CONTROL_STATUS.
  setActive(m_active_at_start);

  // Sanity: identical button assignments would make one physical
  // press fire two logical actions. Config error, warn loudly.
  unsigned int btns[7] = { m_btn_cycle_left, m_btn_cycle_right,
                           m_btn_frame_left, m_btn_frame_right,
                           m_btn_untag_left, m_btn_untag_right,
                           m_btn_active };
  for(unsigned int i=0; i<7; i++)
    for(unsigned int j=i+1; j<7; j++)
      if(btns[i] == btns[j])
        reportConfigWarning("Two actions share one button index.");

  openDevice();  // non-fatal if it fails; Iterate() retries

  registerVariables();
  return(true);
}

//---------------------------------------------------------------
// Procedure: registerVariables

void XboxJoystick::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT", 0);
  Register("NODE_REPORT_LOCAL", 0);
  Register("TAGGED_VEHICLES", 0);
}

//---------------------------------------------------------------
// Procedure: openDevice
//
// O_NONBLOCK is essential: Iterate() must never stall waiting on
// controller input. Returns true if the fd is live.

bool XboxJoystick::openDevice()
{
  if(m_jsfd >= 0)
    return(true);

  m_jsfd = open(m_device.c_str(), O_RDONLY | O_NONBLOCK);
  return(m_jsfd >= 0);
}

//---------------------------------------------------------------
// Procedure: closeDevice

void XboxJoystick::closeDevice()
{
  if(m_jsfd >= 0) {
    close(m_jsfd);
    m_jsfd = -1;
  }
}

//---------------------------------------------------------------
// Procedure: pollDevice
//
// Drain every queued js_event. Non-blocking read() returns -1
// with EAGAIN when the queue is empty (normal exit path). Any
// other error (or 0/short read after unplug) closes the fd so
// Iterate() falls back into reconnect mode.
//
// JS_EVENT_INIT-flagged events are the kernel's synthetic replay
// of current state on open. We accept INIT axis events (they seed
// true stick position) but IGNORE INIT button events -- otherwise
// a button physically held during app launch would fire a phantom
// cycle/frame action.

void XboxJoystick::pollDevice()
{
  struct js_event ev;
  while(true) {
    ssize_t n = read(m_jsfd, &ev, sizeof(ev));

    if(n < 0) {
      if((errno == EAGAIN) || (errno == EWOULDBLOCK))
        return;               // queue drained -- the normal case
      closeDevice();          // real error: unplugged, etc.
      return;
    }
    if(n != (ssize_t)sizeof(ev)) {
      closeDevice();          // short read: device went away
      return;
    }

    bool is_init = (ev.type & JS_EVENT_INIT);
    unsigned char type = ev.type & ~JS_EVENT_INIT;

    if(type == JS_EVENT_AXIS) {
      m_events_axis++;
      handleAxis(ev.number, ev.value, is_init);
    }
    else if((type == JS_EVENT_BUTTON) && !is_init) {
      m_events_button++;
      handleButton(ev.number, (ev.value != 0));
    }
  }
}

//---------------------------------------------------------------
// Procedure: handleAxis
//
// Kernel axis range is [-32767, 32767]. Sticks normalize to
// [-1, 1] with Y pre-flipped so +y always means STICK UP, which
// keeps all downstream math sign-sane (up = forward = north-ish).
//
// Triggers rest at -32767 and read +32767 fully pulled, so they
// remap to [0, 1]. CRITICAL is_init exception: xpad reports raw 0
// for an untouched trigger in the JS_EVENT_INIT replay -- which
// on the trigger scale is a HALF PULL. Accepting that would
// command 50%% reverse to both boats at startup, so trigger axes
// ignore INIT events and read fully-released until the first real
// pull. (Stick INIT events stay accepted: their rest is 0, which
// is also their true center.)

void XboxJoystick::handleAxis(unsigned int axis, int value, bool is_init)
{
  double norm = (double)value / 32767.0;
  if(norm > 1.0)  norm = 1.0;
  if(norm < -1.0) norm = -1.0;

  if(axis == m_ax_left_x)
    m_stick[LEFT_STICK].x = norm;
  else if(axis == m_ax_left_y)
    m_stick[LEFT_STICK].y = -norm;          // flip: kernel +y is DOWN
  else if(axis == m_ax_right_x)
    m_stick[RIGHT_STICK].x = norm;
  else if(axis == m_ax_right_y)
    m_stick[RIGHT_STICK].y = -norm;
  else if(axis == m_ax_left_trig) {
    if(!is_init)
      m_stick[LEFT_STICK].brake = (norm + 1.0) / 2.0;
  }
  else if(axis == m_ax_right_trig) {
    if(!is_init)
      m_stick[RIGHT_STICK].brake = (norm + 1.0) / 2.0;
  }
  // Any other axis (d-pad) is deliberately ignored.
}

//---------------------------------------------------------------
// Procedure: handleButton
//
// Edge-triggered on PRESS only (pressed==true). Releases ignored.

void XboxJoystick::handleButton(unsigned int button, bool pressed)
{
  if(!pressed)
    return;

  if(button == m_btn_active)
    setActive(!m_active);
  else if(button == m_btn_cycle_left)
    cycleVehicle(LEFT_STICK);
  else if(button == m_btn_cycle_right)
    cycleVehicle(RIGHT_STICK);
  else if(button == m_btn_frame_left)
    toggleFrame(LEFT_STICK);
  else if(button == m_btn_frame_right)
    toggleFrame(RIGHT_STICK);
  else if(button == m_btn_untag_left)
    dispatchUntag(LEFT_STICK);
  else if(button == m_btn_untag_right)
    dispatchUntag(RIGHT_STICK);
}

//---------------------------------------------------------------
// Procedure: cycleVehicle
//
// Advance this stick's assignment to the next roster slot,
// SKIPPING the slot currently owned by the other stick so the two
// sticks can never converge on one boat. With exactly 2 vehicles
// this is a no-op by construction (the only other boat is taken),
// which is the correct behavior.

void XboxJoystick::cycleVehicle(StickID sid)
{
  unsigned int nvec = m_vehicles.size();
  if(nvec == 0)
    return;

  StickID other = (sid == LEFT_STICK) ? RIGHT_STICK : LEFT_STICK;
  int cur = m_stick[sid].vix;

  // Probe forward at most nvec slots looking for a free one.
  for(unsigned int step=1; step<=nvec; step++) {
    int cand = (cur + (int)step) % (int)nvec;
    if(cand == m_stick[other].vix)
      continue;                       // other stick owns it -- skip
    if(cand == cur)
      return;                         // wrapped back: nowhere to go

    // A dispatched boat is already with its helm mid-return: no
    // second release; just drop the claim and let it finish (it
    // will station_hold after untagging, hold mode).
    if(m_stick[sid].untag_dispatch) {
      m_stick[sid].untag_dispatch = false;
      clearHelmText(sid);
    }
    else
      releaseVehicle(cur);
    m_stick[sid].vix = cand;
    acquireVehicle(cand);
    debugLog(((sid==LEFT_STICK)?string("LEFT"):string("RIGHT")) +
             " stick -> " + vehicleName(cand));
    postStatus();
    return;
  }
}

//---------------------------------------------------------------
// Procedure: toggleFrame

void XboxJoystick::toggleFrame(StickID sid)
{
  m_stick[sid].global_frame = !m_stick[sid].global_frame;
  debugLog(((sid==LEFT_STICK)?string("LEFT"):string("RIGHT")) +
           " frame -> " +
           (m_stick[sid].global_frame ? "global" : "body"));
  postStatus();
}

//---------------------------------------------------------------
// Procedure: dispatchUntag
//
// Y / B press: send the selected boat home to untag AUTONOMOUSLY,
// then automatically get it back.
//
// Mechanism -- a temporary delegation built on the handback path:
// zero-post + drop the override (releaseVehicle), but the hold-
// mode REDIR_CONTROLLED latch STAYS on (the 1 Hz re-assert keeps
// it there). The helm wakes into MODE==TAGGED; waypt_untag (the
// deliberately UNgated behavior) drives the boat home and clears
// the tag; the REDIR gate keeps every game behavior off the whole
// way. The Iterate() watcher re-acquires the instant the tag
// clears, so the stick gets the boat back at the home flag
// without any operator action.
//
// Toggle semantics: pressing again mid-return RECLAIMS the boat
// immediately (override back on, return abandoned) -- for when
// the return path is about to cost you tactically.
//
// No-ops: inactive teleop, unassigned stick, or boat not tagged.

void XboxJoystick::dispatchUntag(StickID sid)
{
  if(!m_active)
    return;

  Stick &stk = m_stick[sid];
  string vname = vehicleName(stk.vix);
  if(vname == "")
    return;

  if(stk.untag_dispatch) {
    // Second press: reclaim mid-return.
    stk.untag_dispatch = false;
    clearHelmText(sid);
    acquireVehicle(stk.vix);
    debugLog("Untag return CANCELED -- reclaimed " + vname);
    postStatus();
    return;
  }

  if(m_tagged.count(tolower(vname)) == 0) {
    debugLog("Untag ignored -- " + vname + " is not tagged");
    return;
  }

  releaseVehicle(stk.vix);        // zero-post + override drop;
                                  // REDIR latch deliberately kept
  stk.untag_dispatch = true;
  clearDirTriangle(sid);
  clearBrakeBar(sid);
  debugLog("Untag dispatch -- " + vname + " released to helm");
  postStatus();
}

//---------------------------------------------------------------
// Procedure: setActive
//
// The master teleop switch (START button, or startup state).
//
// ACTIVATE:   acquireVehicle() both assigned boats -- in hold
//             mode that pre-latches the REDIR_CONTROLLED gate on
//             each vehicle BEFORE the helm is overridden (see
//             acquireVehicle), then asserts manual override.
// DEACTIVATE: releaseVehicle() both assigned boats -- park the
//             actuators (zero thrust/rudder) and drop override.
//               hold mode:     the helm wakes with game behaviors
//                              gated off and station_hold active;
//                              the boat ACTIVELY PARKS where the
//                              operator left it. Release it later
//                              via the b/r + w redirect workflow.
//               autonomy mode: the helm wakes and resumes normal
//                              behavior-conditioned play. No
//                              REDIR_* variable is ever touched.
//             Both rings are cleared.

void XboxJoystick::setActive(bool active)
{
  // Order matters: acquireVehicle()/releaseVehicle() consult
  // m_active. Latch the flag such that both act:
  //   activating   -> set m_active=true first, then acquire
  //   deactivating -> release first (needs m_active still true),
  //                   then set m_active=false
  if(active)
    m_active = true;

  // ROSTER PARKING (hold mode): on activation, latch the REDIR
  // gate on EVERY rostered vehicle, not just the two the sticks
  // hold. Unselected boats therefore wake out of their game
  // behaviors into station_hold -- they do NOTHING until a stick
  // cycles onto them. The interlocks still apply per boat: a
  // tagged parked boat runs waypt_untag to the untag zone (its
  // gate keeps game behaviors off the whole way), then re-parks.
  // The latch is deliberately NOT dropped on deactivation --
  // "inactive" hands the ROSTER back parked, and boats return to
  // game play only via the pRedirectWaypoint b/r + w workflow.
  // Note: station_hold (140) also outbids waypt_return_home
  // (100), so the viewer RETURN button will not move parked
  // boats.
  if(active && m_handback_hold && m_park_unselected) {
    for(unsigned int v=0; v<m_vehicles.size(); v++) {
      string uv = toupper(m_vehicles[v]);
      Notify("REDIR_ACTIVE_" + uv, "false");
      Notify("REDIR_CONTROLLED_" + uv, "true");
      m_posts += 2;
    }
    debugLog("Roster parked: REDIR latch on " +
             uintToString(m_vehicles.size()) + " boats");
  }

  for(int i=0; i<2; i++) {
    int vix = m_stick[i].vix;
    if(vehicleName(vix) == "")
      continue;
    if(active)
      acquireVehicle(vix);
    else {
      // Dispatched boat: already released to its helm; a second
      // zero-post would momentarily fight it. Just clear the flag
      // and let the return finish.
      if(m_stick[i].untag_dispatch) {
        m_stick[i].untag_dispatch = false;
        clearHelmText((StickID)i);
      }
      else
        releaseVehicle(vix);
      m_stick[i].out_thrust = 0;
      m_stick[i].out_rudder = 0;
    }
  }

  if(!active) {
    m_active = false;
    clearRing(LEFT_STICK);
    clearRing(RIGHT_STICK);
    clearDirTriangle(LEFT_STICK);
    clearDirTriangle(RIGHT_STICK);
    clearBrakeBar(LEFT_STICK);
    clearBrakeBar(RIGHT_STICK);
    clearStickTag(LEFT_STICK);
    clearStickTag(RIGHT_STICK);
  }

  debugLog(string("Teleop -> ") + (active ? "ACTIVE" : "inactive"));
  postStatus();
}

//---------------------------------------------------------------
// Procedure: computeAndPost
//
// Translate one stick's cached position into a thrust/rudder pair
// for its assigned boat and publish. Called once per stick per
// Iterate() so vehicles see a steady AppTick-rate command stream.
//
// STICKS ARE FORWARD-ONLY COURSE COMMANDS. Both frames share one
// model: stick direction = where to go, stick magnitude = how
// much forward thrust, rudder = kp * heading_error (clamped).
// The only difference is the reference for the error:
//
//   BODY frame:   error = atan2(x, y) -- the stick's angle off
//     the bow, directly. Needs no NODE_REPORT at all. Point the
//     stick astern and the error is ~180: the boat drives
//     FORWARD under full rudder and loops around to face the
//     direction of travel. There is no stick reverse.
//   GLOBAL frame: error = angle180(course - actual_heading),
//     course = atan2(x, y) as compass degrees (up=000,
//     right=090 -- atan2(x,y) NOT atan2(y,x); the swap IS the
//     cartesian->compass conversion). Needs a heading; degrades
//     to straight thrust + zero rudder (with a run warning)
//     until the first NODE_REPORT.
//
// TRIGGER = THE ONLY REVERSE, AND AN EXCLUSIVE THRUST SOURCE.
// While brake > trigger_deadzone, thrust is -brake*max_thrust,
// full stop -- the stick's magnitude is IGNORED for thrust and
// contributes rudder only, so the boat steers while backing.
// (Astern, most hulls turn opposite the rudder's forward sense;
// the app passes the stick's rudder through unchanged and lets
// the operator close that loop visually.)
//
// Deadzone is RADIAL: inside it (and not braking), post explicit
// zeros rather than skipping the post, so a boat that just lost
// stick input coasts to a stop instead of holding its last
// command forever.

void XboxJoystick::computeAndPost(StickID sid)
{
  Stick &stk = m_stick[sid];
  string vname = vehicleName(stk.vix);
  if(vname == "")
    return;                           // stick unassigned

  // Untag dispatch: the boat is deliberately with its helm right
  // now (waypt_untag driving it home). Post NOTHING -- especially
  // not the tick-rate override re-assert, which would silence the
  // helm and strand the boat. Iterate() re-acquires when the tag
  // clears. Tagged boats are otherwise fully controllable; game
  // legality is the operator's call, not the app's.
  if(stk.untag_dispatch)
    return;

  double mag = hypot(stk.x, stk.y);
  double thrust = 0;
  double rudder = 0;

  bool braking = (stk.brake > m_trigger_deadzone);
  if(mag > 1.0)
    mag = 1.0;                        // diagonal corners exceed 1

  if(mag >= m_deadzone) {
    // ---- Rudder from the stick (both frames), and forward
    // ---- thrust from the stick only when NOT braking.
    double err = 0;
    bool   have_err = true;

    if(!stk.global_frame) {
      // BODY: the stick's angle off the bow IS the heading error.
      err = radToDegrees(atan2(stk.x, stk.y));
    }
    else {
      // GLOBAL: error relative to the boat's actual heading.
      double course = angle360(radToDegrees(atan2(stk.x, stk.y)));
      string lname = tolower(vname);
      map<string,double>::iterator q = m_heading.find(lname);
      if(q != m_heading.end()) {
        retractRunWarning("No heading yet for " + vname +
                          " (global frame degraded)");
        err = angle180(course - q->second);
      }
      else {
        // No NODE_REPORT yet: cannot close the heading loop.
        // Degrade to straight-line thrust, zero rudder, warn.
        have_err = false;
        reportRunWarning("No heading yet for " + vname +
                         " (global frame degraded)");
      }
    }

    if(have_err) {
      rudder = m_heading_kp * err;
      if(rudder >  m_max_rudder) rudder =  m_max_rudder;
      if(rudder < -m_max_rudder) rudder = -m_max_rudder;
    }

    if(!braking)
      thrust = mag * m_max_thrust;    // forward-only, always
  }

  // ---- Trigger: the exclusive reverse thrust source ----
  // While braking, thrust comes ONLY from the trigger; any stick
  // forward thrust was skipped above, but stick rudder stands, so
  // the boat steers while backing.
  if(braking) {
    double brake = (stk.brake > 1.0) ? 1.0 : stk.brake;
    thrust = -brake * m_max_thrust;
  }

  stk.out_thrust = thrust;
  stk.out_rudder = rudder;

  string uv = toupper(vname);
  Notify("DESIRED_THRUST_" + uv, thrust);
  Notify("DESIRED_RUDDER_" + uv, rudder);

  // RE-ASSERT the override with every command pair. The shoreside
  // DEPLOY button posts MOOS_MANUAL_OVERIDE_ALL=false (typically
  // the historic one-R spelling, which pHelmIvP honors as an
  // alias) and qbridges it to EVERY boat -- including ours. A
  // one-shot override at acquire time therefore gets clobbered by
  // any later DEPLOY press and the helm wakes mid-teleop. Posting
  // override=true at tick rate means a DEPLOY stomp lasts at most
  // one AppTick (~40ms at 25Hz) before teleop retakes the boat.
  // Precedence rule this creates: while ACTIVE, teleop owns its
  // two boats; DEPLOY deploys everyone else. START releases.
  // Both spellings: pHelmIvP takes either, but one-R-only
  // pMarinePID builds MUST see MOOS_MANUAL_OVERIDE or they
  // zero-stomp the command stream (see acquireVehicle).
  Notify("MOOS_MANUAL_OVERRIDE_" + uv, "true");
  Notify("MOOS_MANUAL_OVERIDE_"  + uv, "true");
  m_posts += 4;
}

//---------------------------------------------------------------
// Procedure: acquireVehicle
//
// Fires when a stick takes ownership of a boat WHILE ACTIVE.
// While INACTIVE this is a silent no-op -- cycling assignments is
// just pre-staging; no boat is touched until START.
//
// hold mode: BEFORE asserting override, cancel any in-flight
// click-waypoint (REDIR_ACTIVE=false, same hygiene as
// pRedirectWaypoint's newly-controlled clearing) and pre-latch
// the control gate (REDIR_CONTROLLED=true). Both are inert while
// the helm is overridden -- the point is that they are already
// sitting on the vehicle when override eventually drops, so the
// helm wakes DIRECTLY into station_hold with game behaviors gated
// off. No shore->vehicle ordering race at handback time.
//
// autonomy mode: override only; REDIR_* is never touched, so any
// pRedirectWaypoint state on this boat survives an Xbox session.

void XboxJoystick::acquireVehicle(int vix)
{
  if(!m_active)
    return;
  string vname = vehicleName(vix);
  if(vname == "")
    return;

  string uv = toupper(vname);
  if(m_handback_hold) {
    Notify("REDIR_ACTIVE_" + uv, "false");
    Notify("REDIR_CONTROLLED_" + uv, "true");

    // Raise the graphics mask: the vehicle-side uGfxMask app
    // erases station_hold's rings while XBOX_DRIVEN is true.
    // Shoreside erases CANNOT do this job -- pMarineViewer keys
    // geo shapes per SOURCE community, so only a vehicle-
    // originated erase lands in the map holding the vehicle-
    // posted rings (learned across four debugging rounds).
    Notify("XBOX_DRIVEN_" + uv, "true");
    m_posts += 3;
  }
  // BOTH spellings, deliberately. pHelmIvP honors either, but
  // some pMarinePID builds only honor the historic one-R
  // MOOS_MANUAL_OVERIDE. If PID misses the override it sees the
  // silenced helm as "tardy" and floods DESIRED_THRUST/RUDDER=0
  // as an all-stop safety -- zero-stomping our bridged commands
  // in the vehicle DB. (Found the hard way: override true on the
  // vehicle, thrust pinned at 0, source=pMarinePID.)
  Notify("MOOS_MANUAL_OVERRIDE_" + uv, "true");
  Notify("MOOS_MANUAL_OVERIDE_"  + uv, "true");
  m_posts += 2;
  debugLog("Acquired " + vname);
}

//---------------------------------------------------------------
// Procedure: releaseVehicle
//
// The ONE handback path, used both when a stick cycles OFF a boat
// and (via setActive) when teleop deactivates: park the actuators
// with a single zero thrust/rudder post, then drop the override.
// What the helm wakes into depends on the handback mode latched
// at acquire time:
//   hold:     REDIR_CONTROLLED=true is already on the vehicle, so
//             station_hold catches the boat where it sits.
//   autonomy: game behaviors resume per their own conditions.
// Silent no-op while inactive (nothing is held to release).

void XboxJoystick::releaseVehicle(int vix)
{
  if(!m_active)
    return;
  string vname = vehicleName(vix);
  if(vname == "")
    return;

  string uv = toupper(vname);
  Notify("DESIRED_THRUST_" + uv, 0.0);
  Notify("DESIRED_RUDDER_" + uv, 0.0);

  // Re-station the boat AT ITS CURRENT POSITION explicitly, via
  // station_hold's updates channel. center_activate=true only
  // re-captures on an idle->running edge, and across a manual-
  // override period the helm never iterates behaviors -- so on a
  // second (and every later) handback there is NO edge and the
  // boat would sail back to wherever the FIRST hold captured.
  // The updates-channel station_pt applies unconditionally.
  // Skipped if no NODE_REPORT position is known yet; in that
  // case the boat has never moved under our control and the
  // original center_activate capture is still correct.
  if(m_handback_hold) {
    string lname = tolower(vname);
    if(m_vx.find(lname) != m_vx.end()) {
      string spt = "station_pt=" +
        doubleToStringX(m_vx[lname], 1) + "," +
        doubleToStringX(m_vy[lname], 1);
      Notify("REDIR_HOLD_UPDATE_" + uv, spt);
      m_posts++;
    }
  }

  Notify("MOOS_MANUAL_OVERRIDE_" + uv, "false");
  Notify("MOOS_MANUAL_OVERIDE_"  + uv, "false");   // both spellings
  if(m_handback_hold) {
    // Drop the graphics mask: uGfxMask stops erasing and the
    // re-engaged station_hold redraws its rings at the new
    // station point -- rings return exactly when the boat stops
    // being operator-driven.
    Notify("XBOX_DRIVEN_" + uv, "false");
    m_posts++;
  }
  m_posts += 4;
  debugLog("Handback " + vname + " -> " +
           (m_handback_hold ? "hold (station)" : "autonomy"));
}

//---------------------------------------------------------------
// Procedure: postRing
//
// Draw/refresh the control-indicator ring for one stick at its
// boat's latest known position, in the same spec-string house
// style as pRedirectWaypoint's control halos (duration=0, light
// fill) so the two control systems read consistently on the
// viewer -- distinguishable by color (green/blue vs magenta).
//
// The ring is labeled per-STICK (xbox_left / xbox_right), so
// cycling vehicles moves the same labeled circle to the new boat.
// Tagged boats lose the ring while tagged (the tag manager's own
// circle shows instead), and boats with no known position can't
// be drawn -- both cases CLEAR the label rather than skip, so no
// stale ring lingers at the last drawn spot.

void XboxJoystick::postRing(StickID sid)
{
  string vname = vehicleName(m_stick[sid].vix);
  if(vname == "")
    return;

  string lname = tolower(vname);
  bool has_pos = (m_vx.find(lname) != m_vx.end());
  if(!has_pos) {
    clearRing(sid);
    return;
  }

  string spec;
  spec += "x=" + doubleToStringX(m_vx[lname], 2);
  spec += ",y=" + doubleToStringX(m_vy[lname], 2);
  spec += ",radius=" + doubleToStringX(m_ring_radius, 1);
  spec += ",duration=0";               // persists until updated/cleared
  spec += ",label=";
  spec += (sid == LEFT_STICK) ? "xbox_left" : "xbox_right";
  spec += ",label_color=invisible";   // key only, never rendered
  spec += ",edge_color=";
  spec += (sid == LEFT_STICK) ? m_left_ring_color : m_right_ring_color;
  spec += ",fill_color=";
  spec += (sid == LEFT_STICK) ? m_left_ring_color : m_right_ring_color;
  spec += ",fill_transparency=0.3";    // actively driven = filled
  spec += ",edge_size=2";
  spec += ",vertex_size=0";
  Notify("VIEW_CIRCLE", spec);
  m_posts++;
}

//---------------------------------------------------------------
// Procedure: clearRing
//
// Remove one stick's ring by re-posting it as inactive. NOTE:
// some pMarineViewer builds reject a VIEW_CIRCLE carrying only a
// label + active=false (no geometry) as unhandled mail, so post a
// full, parseable spec at the boat's last known position (0,0 if
// unknown) -- same fix as pRedirectWaypoint::clearControlCircle.

void XboxJoystick::clearRing(StickID sid)
{
  double x = 0, y = 0;
  string vname = vehicleName(m_stick[sid].vix);
  if(vname != "") {
    string lname = tolower(vname);
    if(m_vx.find(lname) != m_vx.end()) {
      x = m_vx[lname];
      y = m_vy[lname];
    }
  }

  string spec;
  spec += "x=" + doubleToStringX(x, 2);
  spec += ",y=" + doubleToStringX(y, 2);
  spec += ",radius=" + doubleToStringX(m_ring_radius, 1);
  spec += ",label=";
  spec += (sid == LEFT_STICK) ? "xbox_left" : "xbox_right";
  spec += ",active=false";
  Notify("VIEW_CIRCLE", spec);
  m_posts++;
}

//---------------------------------------------------------------
// Procedure: postDirTriangle
//
// The command-intent arrowhead: base sits ON the control ring,
// apex points outward along the direction the operator is
// commanding the boat to go.
//
//   global frame: dir = compass course of the stick vector,
//     angle360(radToDegrees(atan2(x,y))) -- pure stick, no
//     dependence on where the bow points.
//   body frame:   the stick vector is bow-relative, so
//     dir = heading + atan2(x,y). Stick up = out the bow, stick
//     down (reverse) = astern. Needs a known heading.
//
// Geometry (compass convention: ux=sin, uy=cos):
//   apex  = center + (R + tri_length) * u
//   base1 = center + R*u + (tri_width/2) * p
//   base2 = center + R*u - (tri_width/2) * p
// where u is the unit vector along dir and p is its
// perpendicular.
//
// Fill density encodes push magnitude: fill_transparency runs
// 0.15 (feather touch, just past deadzone) to 0.85 (full
// deflection). Edge stays solid so a faint triangle still reads.
//
// Cleared (not skipped -- no stale arrow) whenever there is no
// commanded direction to show: stick inside deadzone, boat
// untag-dispatched (stick not commanding), position unknown, or
// (body frame) heading unknown.

void XboxJoystick::postDirTriangle(StickID sid)
{
  Stick &stk = m_stick[sid];
  string vname = vehicleName(stk.vix);
  if(vname == "")
    return;

  string lname = tolower(vname);
  bool has_pos = (m_vx.find(lname) != m_vx.end());

  double mag = hypot(stk.x, stk.y);
  if(mag > 1.0)
    mag = 1.0;

  // No arrow while dispatched: the stick is not commanding.
  if((mag < m_deadzone) || stk.untag_dispatch || !has_pos) {
    clearDirTriangle(sid);
    return;
  }

  // Commanded direction in world/compass degrees, per frame.
  double dir;
  if(stk.global_frame)
    dir = angle360(radToDegrees(atan2(stk.x, stk.y)));
  else {
    map<string,double>::iterator q = m_heading.find(lname);
    if(q == m_heading.end()) {
      clearDirTriangle(sid);       // bow-relative needs a heading
      return;
    }
    dir = angle360(q->second + radToDegrees(atan2(stk.x, stk.y)));
  }

  double cx = m_vx[lname];
  double cy = m_vy[lname];
  double rad = degToRadians(dir);
  double ux = sin(rad),  uy = cos(rad);    // along dir (compass)
  double px = cos(rad),  py = -sin(rad);   // perpendicular

  double ax  = cx + (m_ring_radius + m_tri_length) * ux;
  double ay  = cy + (m_ring_radius + m_tri_length) * uy;
  double b1x = cx + m_ring_radius * ux + (m_tri_width/2.0) * px;
  double b1y = cy + m_ring_radius * uy + (m_tri_width/2.0) * py;
  double b2x = cx + m_ring_radius * ux - (m_tri_width/2.0) * px;
  double b2y = cy + m_ring_radius * uy - (m_tri_width/2.0) * py;

  // Push magnitude -> fill density. mag is in [deadzone, 1];
  // remap to [0,1] before scaling so the faintest visible arrow
  // corresponds to "just escaped the deadzone".
  double frac = (mag - m_deadzone) / (1.0 - m_deadzone);
  double transparency = 0.15 + 0.70 * frac;

  string color = (sid == LEFT_STICK) ?
                 m_left_ring_color : m_right_ring_color;

  string spec;
  spec += "pts={" + doubleToStringX(ax,2)  + "," + doubleToStringX(ay,2);
  spec += ":"     + doubleToStringX(b1x,2) + "," + doubleToStringX(b1y,2);
  spec += ":"     + doubleToStringX(b2x,2) + "," + doubleToStringX(b2y,2);
  spec += "}";
  spec += ",label=";
  spec += (sid == LEFT_STICK) ? "xbox_left_dir" : "xbox_right_dir";
  spec += ",label_color=invisible";   // label is the update/clear
                                      // key only -- never render it
  spec += ",duration=0";
  spec += ",edge_color=" + color;
  spec += ",fill_color=" + color;
  spec += ",fill_transparency=" + doubleToStringX(transparency, 2);
  spec += ",edge_size=1";
  spec += ",vertex_size=0";
  Notify("VIEW_POLYGON", spec);
  stk.tri_drawn = true;
  m_posts++;
}

//---------------------------------------------------------------
// Procedure: clearDirTriangle
//
// Remove one stick's intent triangle. Same lesson as clearRing:
// post a full, parseable polygon spec (degenerate but valid, at
// the boat's last known position) with active=false, rather than
// a label-only clear that some viewer builds reject.

void XboxJoystick::clearDirTriangle(StickID sid)
{
  // Transition-gated: posting a clear every tick while idle
  // floods VIEW_POLYGON (~150 posts/s with the brake clears) and
  // drowns any attempt to scope the variable -- learned the hard
  // way while hunting the station-keep circles.
  if(!m_stick[sid].tri_drawn)
    return;
  m_stick[sid].tri_drawn = false;

  double x = 0, y = 0;
  string vname = vehicleName(m_stick[sid].vix);
  if(vname != "") {
    string lname = tolower(vname);
    if(m_vx.find(lname) != m_vx.end()) {
      x = m_vx[lname];
      y = m_vy[lname];
    }
  }

  string spec;
  spec += "pts={" + doubleToStringX(x,2)   + "," + doubleToStringX(y,2);
  spec += ":"     + doubleToStringX(x+1,2) + "," + doubleToStringX(y,2);
  spec += ":"     + doubleToStringX(x,2)   + "," + doubleToStringX(y+1,2);
  spec += "}";
  spec += ",label=";
  spec += (sid == LEFT_STICK) ? "xbox_left_dir" : "xbox_right_dir";
  spec += ",active=false";
  Notify("VIEW_POLYGON", spec);
  m_posts++;
}

//---------------------------------------------------------------
// Procedure: postBrakeBar
//
// Trigger-pull fill gauge: a small vertical bar just east of the
// control ring. Drawn ONLY while the trigger is past its deadzone
// -- no standing clutter -- and cleared under the same conditions
// as the other graphics (released, untag-dispatched, no position).
//
// Two stacked rectangles:
//   *_brk_bg:   fixed outline, stick color, fill invisible --
//               the gauge housing, so partial pulls read against
//               a full-scale reference.
//   *_brk_fill: rises bottom-up, height = brake * bar_height,
//               in m_bar_fill_color (red: brake semantics; which
//               BOAT it belongs to is unambiguous from position
//               and the housing color).
//
// North-up geometry: "vertical" means world north. pMarineViewer
// is north-up by default; if the view is ever rotated the gauge
// rotates with the world -- acceptable for a debug/operator aid.

void XboxJoystick::postBrakeBar(StickID sid)
{
  Stick &stk = m_stick[sid];
  string vname = vehicleName(stk.vix);
  if(vname == "")
    return;

  string lname = tolower(vname);
  bool has_pos = (m_vx.find(lname) != m_vx.end());

  if((stk.brake <= m_trigger_deadzone) || stk.untag_dispatch || !has_pos) {
    clearBrakeBar(sid);
    return;
  }

  double cx = m_vx[lname];
  double cy = m_vy[lname];

  // Gauge housing: west edge sits 2m east of the ring.
  double x0 = cx + m_ring_radius + 2.0;          // west edge
  double x1 = x0 + m_bar_width;                  // east edge
  double y0 = cy - m_bar_height / 2.0;           // bottom
  double y1 = cy + m_bar_height / 2.0;           // top

  double brake = stk.brake;
  if(brake > 1.0)
    brake = 1.0;
  double yfill = y0 + brake * m_bar_height;      // fill top

  string housing_color = (sid == LEFT_STICK) ?
                         m_left_ring_color : m_right_ring_color;
  string lbl = (sid == LEFT_STICK) ? "xbox_left" : "xbox_right";

  // Housing (outline only).
  string bg;
  bg += "pts={" + doubleToStringX(x0,2) + "," + doubleToStringX(y0,2);
  bg += ":"     + doubleToStringX(x1,2) + "," + doubleToStringX(y0,2);
  bg += ":"     + doubleToStringX(x1,2) + "," + doubleToStringX(y1,2);
  bg += ":"     + doubleToStringX(x0,2) + "," + doubleToStringX(y1,2);
  bg += "}";
  bg += ",label=" + lbl + "_brk_bg";
  bg += ",label_color=invisible";
  bg += ",duration=0";
  bg += ",edge_color=" + housing_color;
  bg += ",fill_color=invisible";
  bg += ",edge_size=1";
  bg += ",vertex_size=0";
  Notify("VIEW_POLYGON", bg);

  // Fill (rises with pull).
  string fl;
  fl += "pts={" + doubleToStringX(x0,2) + "," + doubleToStringX(y0,2);
  fl += ":"     + doubleToStringX(x1,2) + "," + doubleToStringX(y0,2);
  fl += ":"     + doubleToStringX(x1,2) + "," + doubleToStringX(yfill,2);
  fl += ":"     + doubleToStringX(x0,2) + "," + doubleToStringX(yfill,2);
  fl += "}";
  fl += ",label=" + lbl + "_brk_fill";
  fl += ",label_color=invisible";
  fl += ",duration=0";
  fl += ",edge_color=" + m_bar_fill_color;
  fl += ",fill_color=" + m_bar_fill_color;
  fl += ",fill_transparency=0.7";
  fl += ",edge_size=1";
  fl += ",vertex_size=0";
  Notify("VIEW_POLYGON", fl);

  // REV text tag under the gauge so the bar is self-explanatory
  // (same invisible-vertex trick as the HELM tag: the label IS
  // the text, and its (L)/(R) suffix keeps the key unique).
  string tx;
  tx += "x=" + doubleToStringX(x0 + m_bar_width/2.0, 2);
  tx += ",y=" + doubleToStringX(y0 - 2.0, 2);
  tx += ",vertex_size=1";
  tx += ",vertex_color=invisible";
  tx += ",label=";
  tx += (sid == LEFT_STICK) ? "REV(L)" : "REV(R)";
  tx += ",label_color=" + m_bar_fill_color;
  Notify("VIEW_POINT", tx);
  stk.bar_drawn = true;
  m_posts += 3;
}

//---------------------------------------------------------------
// Procedure: clearBrakeBar
//
// Remove both gauge rectangles. Full parseable specs with
// active=false, per the house label-only-clear lesson.

void XboxJoystick::clearBrakeBar(StickID sid)
{
  if(!m_stick[sid].bar_drawn)      // transition-gated, see
    return;                        // clearDirTriangle
  m_stick[sid].bar_drawn = false;

  double x = 0, y = 0;
  string vname = vehicleName(m_stick[sid].vix);
  if(vname != "") {
    string lname = tolower(vname);
    if(m_vx.find(lname) != m_vx.end()) {
      x = m_vx[lname];
      y = m_vy[lname];
    }
  }

  string lbl = (sid == LEFT_STICK) ? "xbox_left" : "xbox_right";
  string base;
  base += "pts={" + doubleToStringX(x,2)   + "," + doubleToStringX(y,2);
  base += ":"     + doubleToStringX(x+1,2) + "," + doubleToStringX(y,2);
  base += ":"     + doubleToStringX(x,2)   + "," + doubleToStringX(y+1,2);
  base += "}";

  Notify("VIEW_POLYGON", base + ",label=" + lbl + "_brk_bg,active=false");
  Notify("VIEW_POLYGON", base + ",label=" + lbl + "_brk_fill,active=false");

  string tlbl = (sid == LEFT_STICK) ? "REV(L)" : "REV(R)";
  Notify("VIEW_POINT", "x=" + doubleToStringX(x,2) +
         ",y=" + doubleToStringX(y,2) + ",label=" + tlbl +
         ",active=false");
  m_posts += 3;
}

//---------------------------------------------------------------
// Procedure: postHelmText
//
// "Under helm control" indicator: a VIEW_POINT whose vertex is
// invisible, so the only thing that renders is its LABEL --
// HELM(L) / HELM(R) in the stick's color, just below the ring,
// following the boat. Deliberately the opposite trick to the
// label_color=invisible graphics: here the label IS the payload.
// Works on any viewer build (labels render by default), unlike
// the newer VIEW_TEXTBOX. The label doubles as the update/clear
// key, so it must differ per stick -- hence the (L)/(R) suffix.
//
// Drawn only while untag-dispatched; every code path that flips
// the dispatch flag off calls clearHelmText(), so there is no
// per-tick clear churn.

void XboxJoystick::postHelmText(StickID sid)
{
  Stick &stk = m_stick[sid];
  if(!stk.untag_dispatch)
    return;

  string vname = vehicleName(stk.vix);
  if(vname == "")
    return;
  string lname = tolower(vname);
  if(m_vx.find(lname) == m_vx.end())
    return;

  string spec;
  spec += "x=" + doubleToStringX(m_vx[lname], 2);
  spec += ",y=" + doubleToStringX(m_vy[lname] - (m_ring_radius + 3), 2);
  spec += ",vertex_size=1";
  spec += ",vertex_color=invisible";
  spec += ",label=";
  spec += (sid == LEFT_STICK) ? "HELM(L)" : "HELM(R)";
  spec += ",label_color=";
  spec += (sid == LEFT_STICK) ? m_left_ring_color : m_right_ring_color;
  Notify("VIEW_POINT", spec);
  m_posts++;
}

//---------------------------------------------------------------
// Procedure: clearHelmText
//
// Full parseable spec + active=false, per the house lesson on
// label-only clears.

void XboxJoystick::clearHelmText(StickID sid)
{
  double x = 0, y = 0;
  string vname = vehicleName(m_stick[sid].vix);
  if(vname != "") {
    string lname = tolower(vname);
    if(m_vx.find(lname) != m_vx.end()) {
      x = m_vx[lname];
      y = m_vy[lname];
    }
  }

  string spec;
  spec += "x=" + doubleToStringX(x, 2);
  spec += ",y=" + doubleToStringX(y, 2);
  spec += ",label=";
  spec += (sid == LEFT_STICK) ? "HELM(L)" : "HELM(R)";
  spec += ",active=false";
  Notify("VIEW_POINT", spec);
  m_posts++;
}

//---------------------------------------------------------------
// Procedure: postStickTag
//
// Stick-identity tag: a plain L / R in the stick's color just
// ABOVE the ring (the HELM tag lives below it), following the
// boat while the stick is actively driving. Cleared while untag-
// dispatched so the boat never wears both "stick-driven" and
// "helm-driven" text at once. Labels "L" / "R" are unique between
// the two sticks and double as the display text.

void XboxJoystick::postStickTag(StickID sid)
{
  Stick &stk = m_stick[sid];
  string vname = vehicleName(stk.vix);
  if(vname == "")
    return;

  string lname = tolower(vname);
  bool has_pos = (m_vx.find(lname) != m_vx.end());
  if(stk.untag_dispatch || !has_pos) {
    clearStickTag(sid);
    return;
  }

  string spec;
  spec += "x=" + doubleToStringX(m_vx[lname], 2);
  spec += ",y=" + doubleToStringX(m_vy[lname] + m_ring_radius + 3, 2);
  spec += ",vertex_size=1";
  spec += ",vertex_color=invisible";
  spec += ",label=";
  spec += (sid == LEFT_STICK) ? "L" : "R";
  spec += ",label_color=";
  spec += (sid == LEFT_STICK) ? m_left_ring_color : m_right_ring_color;
  Notify("VIEW_POINT", spec);
  stk.tag_drawn = true;
  m_posts++;
}

//---------------------------------------------------------------
// Procedure: clearStickTag

void XboxJoystick::clearStickTag(StickID sid)
{
  if(!m_stick[sid].tag_drawn)      // transition-gated
    return;
  m_stick[sid].tag_drawn = false;

  double x = 0, y = 0;
  string vname = vehicleName(m_stick[sid].vix);
  if(vname != "") {
    string lname = tolower(vname);
    if(m_vx.find(lname) != m_vx.end()) {
      x = m_vx[lname];
      y = m_vy[lname];
    }
  }

  string spec;
  spec += "x=" + doubleToStringX(x, 2);
  spec += ",y=" + doubleToStringX(y, 2);
  spec += ",label=";
  spec += (sid == LEFT_STICK) ? "L" : "R";
  spec += ",active=false";
  Notify("VIEW_POINT", spec);
  m_posts++;
}

//---------------------------------------------------------------
// Procedure: handleTaggedVehicles
//
// TAGGED_VEHICLES is the authoritative comma list of tagged
// boats: rebuild the set wholesale (same as pRedirectWaypoint).

void XboxJoystick::handleTaggedVehicles(const std::string& csv)
{
  m_tagged.clear();
  vector<string> names = parseString(csv, ',');
  for(unsigned int i=0; i<names.size(); i++) {
    string v = tolower(stripBlankEnds(names[i]));
    if(v != "")
      m_tagged.insert(v);
  }
}

//---------------------------------------------------------------
// Procedure: handleTagged
//
// TAGGED_<VNAME> = true/false edge for a single boat.

void XboxJoystick::handleTagged(const std::string& vname, bool tagged)
{
  if(tagged)
    m_tagged.insert(vname);
  else
    m_tagged.erase(vname);
}

//---------------------------------------------------------------
// Procedure: debugLog
//
// Timestamped circular buffer surfaced at the bottom of the
// appcast when debug=true (house style, cf. pRedirectWaypoint).

void XboxJoystick::debugLog(const std::string& line)
{
  if(!m_debug)
    return;
  string stamped = doubleToString(MOOSTime(), 2) + "  " + line;
  m_debug_buffer.push_back(stamped);
  while(m_debug_buffer.size() > m_debug_buffer_max)
    m_debug_buffer.pop_front();
}

//---------------------------------------------------------------
// Procedure: postStatus
//
// Announce the full teleop state on any change, e.g.:
//   XBOX_CONTROL_STATUS = active=true,left=red_one:body,
//                         right=red_three:global
// Consumers: deploy UIs, scoreboards, alogs. Posted on change
// only (activation, cycle, frame toggle), not per-tick.

void XboxJoystick::postStatus()
{
  string status = "active=";
  status += (m_active ? "true" : "false");

  for(int i=0; i<2; i++) {
    string vname = vehicleName(m_stick[i].vix);
    if(vname == "")
      vname = "none";
    status += (i == LEFT_STICK) ? ",left=" : ",right=";
    status += vname + ":";
    status += (m_stick[i].global_frame ? "global" : "body");
  }

  Notify("XBOX_CONTROL_STATUS", status);
  m_posts++;
}

//---------------------------------------------------------------
// Procedure: handleNodeReport
//
// Cache latest heading per vehicle (lowercase key) for the
// global-frame P-controller, plus x/y for ring placement.
// Malformed reports are dropped with a run warning rather than
// crashing the teleop link.

void XboxJoystick::handleNodeReport(const std::string& sval)
{
  NodeRecord record = string2NodeRecord(sval);
  if(!record.valid()) {
    reportRunWarning("Malformed NODE_REPORT: " + sval);
    return;
  }
  string lname = tolower(record.getName());
  m_heading[lname] = record.getHeading();
  m_vx[lname]      = record.getX();
  m_vy[lname]      = record.getY();
}

//---------------------------------------------------------------
// Procedure: vehicleName

string XboxJoystick::vehicleName(int vix) const
{
  if((vix < 0) || (vix >= (int)m_vehicles.size()))
    return("");
  return(m_vehicles[vix]);
}

//---------------------------------------------------------------
// Procedure: buildReport()
//
// Example appcast:
//
//   Device: /dev/input/js0 (OPEN)   axis_events=1042 btn_events=7
//   Roster: red_one, red_two, red_three, red_four
//
//   Stick  Vehicle    Frame   StickX  StickY  Thrust  Rudder
//   -----  -------    -----   ------  ------  ------  ------
//   LEFT   red_one    body     0.00    0.75    75.0     0.0
//   RIGHT  red_three  global   0.50   -0.20    53.8   -12.4

bool XboxJoystick::buildReport()
{
  m_msgs << "Teleop: " << (m_active ? "** ACTIVE **" : "inactive");
  m_msgs << "   (START toggles)" << endl;

  string dev_status = (m_jsfd >= 0) ? "OPEN" : "NOT OPEN";
  m_msgs << "Device: " << m_device << " (" << dev_status << ")";
  m_msgs << "   axis_events=" << m_events_axis;
  m_msgs << " btn_events=" << m_events_button << endl;

  string roster;
  for(unsigned int i=0; i<m_vehicles.size(); i++) {
    if(i > 0)
      roster += ", ";
    roster += m_vehicles[i];
  }
  m_msgs << "Roster: " << roster << endl;
  m_msgs << "Total posts: " << m_posts << endl << endl;

  ACTable actab(8);
  actab << "Stick | Vehicle | Frame | StickX | StickY | Brake | Thrust | Rudder";
  actab.addHeaderLines();

  for(int i=0; i<2; i++) {
    const Stick &stk = m_stick[i];
    string sname = (i == LEFT_STICK) ? "LEFT" : "RIGHT";
    string vname = vehicleName(stk.vix);
    if(vname == "")
      vname = "--";
    else if(m_tagged.count(tolower(vname)) > 0)
      vname += "(T)";
    string frame = stk.global_frame ? "global" : "body";
    if(stk.untag_dispatch)
      frame = "UNTAG";
    actab << sname << vname << frame
          << doubleToString(stk.x, 2)
          << doubleToString(stk.y, 2)
          << doubleToString(stk.brake, 2)
          << doubleToString(stk.out_thrust, 1)
          << doubleToString(stk.out_rudder, 1);
  }
  m_msgs << actab.getFormattedString();

  if(m_debug && !m_debug_buffer.empty()) {
    m_msgs << endl << endl << "Debug (newest last)" << endl;
    list<string>::iterator q;
    for(q=m_debug_buffer.begin(); q!=m_debug_buffer.end(); q++)
      m_msgs << "  " << *q << endl;
  }
  return(true);
}
