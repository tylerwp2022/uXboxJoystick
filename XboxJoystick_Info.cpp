/*****************************************************************/
/*    NAME: Tyler Errico                                         */
/*    ORGN: West Point Robotics Research Center                  */
/*    FILE: XboxJoystick_Info.cpp                                */
/*    DATE: 2026                                                 */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include "XboxJoystick_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  Shoreside dual-stick Xbox controller teleop. The LEFT and     ");
  blk("  RIGHT sticks each drive one vehicle. LB cycles the left       ");
  blk("  stick's vehicle; RB cycles the right stick's (bumpers switch  ");
  blk("  boats -- thumbs never leave the sticks). X / A toggle the     ");
  blk("  left / right stick's control frame independently:             ");
  blk("                                                                ");
  blk("  Sticks are FORWARD-ONLY course commands in both frames:       ");
  blk("  direction = where to go, magnitude = forward thrust, rudder = ");
  blk("  kp * heading error. Point the stick astern and the boat loops ");
  blk("  around forward -- sticks never command reverse.               ");
  blk("  BODY frame:   error = stick angle off the bow (no NODE_REPORT ");
  blk("                needed).                                        ");
  blk("  GLOBAL frame: stick is a world course vector (up=north);      ");
  blk("                error = course - heading from NODE_REPORT.      ");
  blk("                                                                ");
  blk("  LT / RT are the ONLY reverse source. While pulled, the        ");
  blk("  trigger is the exclusive thrust source (-pull * max_thrust);  ");
  blk("  the stick contributes rudder only, so steering stays live     ");
  blk("  while backing.                                                ");
  blk("                                                                ");
  blk("  START is the master switch. The app launches INACTIVE (boats  ");
  blk("  under autonomy, no commands posted). START activates teleop   ");
  blk("  (MOOS_MANUAL_OVERRIDE=true on both boats); pressing it again  ");
  blk("  parks both boats and hands them back to their helms. While    ");
  blk("  ACTIVE each controlled boat wears a colored VIEW_CIRCLE ring  ");
  blk("  (green=left stick, blue=right) that follows it in the viewer. ");
  blk("                                                                ");
  blk("  Output is always DESIRED_THRUST_<VNAME> / DESIRED_RUDDER_<V-  ");
  blk("  NAME> plus MOOS_MANUAL_OVERRIDE_<VNAME>, intended for pShare/ ");
  blk("  uFldShoreBroker bridging down to each vehicle community.      ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: uXboxJoystick file.moos [OPTIONS]                      ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch uXboxJoystick with the given process name.      ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display release version of uXboxJoystick.              ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("uXboxJoystick Example MOOS Configuration                      ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = uXboxJoystick                                ");
  blk("{                                                               ");
  blk("  AppTick   = 25   // command stream rate; 10-25 feels smooth   ");
  blk("  CommsTick = 25                                                ");
  blk("                                                                ");
  blk("  // Cycle order = roster order. Left stick starts on slot 1,   ");
  blk("  // right stick on slot 2.                                     ");
  blk("  vehicles = red_one,red_two,red_three,red_four                 ");
  blk("                                                                ");
  blk("  joystick_device = /dev/input/js0                              ");
  blk("                                                                ");
  blk("  max_thrust = 100    // stick at full deflection               ");
  blk("  max_rudder = 45     // degrees                                ");
  blk("  deadzone         = 0.10  // radial, normalized stick units    ");
  blk("  trigger_deadzone = 0.05  // pull below this = no brake        ");
  blk("  heading_kp = 1.5    // global-frame P gain (rudder deg per    ");
  blk("                      // degree of heading error, pre-clamp)    ");
  blk("                                                                ");
  blk("  active_at_start  = false  // SAFE default: autonomy runs      ");
  blk("                            //   until the operator hits START  ");
  blk("                                                                ");
  blk("  // What a released boat does (cycle-off or deactivate):       ");
  blk("  //  hold     = station-holds where you left it, via the       ");
  blk("  //             click-to-redirect REDIR_CONTROLLED gate.       ");
  blk("  //             REQUIRES the redirect bhv patches (station_    ");
  blk("  //             hold + condition lines). Release it later via  ");
  blk("  //             the b/r + w workflow or a click-waypoint.      ");
  blk("  //  autonomy = helm resumes game play; REDIR_* never touched. ");
  blk("  handback     = hold                                           ");
  blk("                                                                ");
  blk("  respect_tags = true   // tagged boat gets zero commands --    ");
  blk("                        //   teleop is not a tag-rule loophole  ");
  blk("  debug        = false  // appcast debug buffer (house style)   ");
  blk("                                                                ");
  blk("  // Control-indicator rings (VIEW_CIRCLE, follow the boats).   ");
  blk("  ring_radius      = 10                                         ");
  blk("  left_ring_color  = green                                      ");
  blk("  right_ring_color = dodger_blue                                ");
  blk("                                                                ");
  blk("  // Command-intent arrowhead (VIEW_POLYGON): rides the ring,   ");
  blk("  // points along the commanded direction (frame-aware), fill   ");
  blk("  // density = stick magnitude. Meters.                         ");
  blk("  triangle_length  = 4                                          ");
  blk("  triangle_width   = 2.5                                        ");
  blk("                                                                ");
  blk("  // Brake gauge (VIEW_POLYGON pair): vertical fill-bar east of ");
  blk("  // the ring, visible only while the trigger is pulled. Fill   ");
  blk("  // rises with pull depth; outline takes the stick color.      ");
  blk("  brake_bar_width  = 2                                          ");
  blk("  brake_bar_height = 12                                         ");
  blk("  brake_bar_color  = red                                        ");
  blk("                                                                ");
  blk("  left_frame  = body   // or global; startup frame per stick    ");
  blk("  right_frame = body                                            ");
  blk("                                                                ");
  blk("  // Index remaps for non-xpad drivers (xboxdrv etc). Defaults  ");
  blk("  // shown are the in-kernel xpad numbering.                    ");
  blk("  // axis_left_x        = 0                                     ");
  blk("  // axis_left_y        = 1                                     ");
  blk("  // axis_right_x       = 3                                     ");
  blk("  // axis_right_y       = 4                                     ");
  blk("  // axis_left_trigger  = 2   // LT (brake/reverse)             ");
  blk("  // axis_right_trigger = 5   // RT (brake/reverse)             ");
  blk("  // button_cycle_left  = 4   // LB (cycle left-stick boat)     ");
  blk("  // button_cycle_right = 5   // RB (cycle right-stick boat)    ");
  blk("  // button_frame_left  = 2   // X  (toggle left-stick frame)   ");
  blk("  // button_frame_right = 0   // A  (toggle right-stick frame)  ");
  blk("  // button_active      = 7   // START                          ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("uXboxJoystick INTERFACE                                       ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NODE_REPORT        (headings for global-frame ctrl; x/y for   ");
  blk("  NODE_REPORT_LOCAL   control rings)                            ");
  blk("  TAGGED_VEHICLES    (authoritative csv of tagged boats)        ");
  blk("  TAGGED_<VNAME>     (single-boat tag edges)                    ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  DESIRED_THRUST_<VNAME>        (double, [-max_thrust, +max])   ");
  blk("  DESIRED_RUDDER_<VNAME>        (double, degrees)               ");
  blk("  MOOS_MANUAL_OVERRIDE_<VNAME>  (true on activate/acquire,      ");
  blk("  MOOS_MANUAL_OVERIDE_<VNAME>    re-asserted at tick rate;      ");
  blk("                                 false on every release. BOTH   ");
  blk("                                 spellings: one-R-only          ");
  blk("                                 pMarinePID builds zero-stomp   ");
  blk("                                 commands if they miss it)      ");
  blk("  VIEW_CIRCLE                   (control-indicator rings:       ");
  blk("                                 green=left stick, blue=right)  ");
  blk("  VIEW_POLYGON                  (command-intent arrowhead per   ");
  blk("                                 stick, direction frame-aware,  ");
  blk("                                 fill = push magnitude; brake   ");
  blk("                                 gauge pair, fill = trigger)    ");
  blk("  XBOX_CONTROL_STATUS           (on state change, e.g.          ");
  blk("                                 active=true,left=red_one:body, ");
  blk("                                 right=red_three:global)        ");
  blk("  REDIR_CONTROLLED_<VNAME>      (handback=hold only: pre-       ");
  blk("  REDIR_ACTIVE_<VNAME>           latched at acquire so release  ");
  blk("                                 wakes the helm into            ");
  blk("                                 station_hold)                  ");
  blk("                                                                ");
  blk("Relay shoreside->vehicle via uFldShoreBroker (house style,     ");
  blk("next to the existing REDIR_* qbridges):                         ");
  blk("  qbridge = DESIRED_THRUST                                      ");
  blk("  qbridge = DESIRED_RUDDER                                      ");
  blk("  qbridge = MOOS_MANUAL_OVERRIDE                                ");
  blk("  qbridge = MOOS_MANUAL_OVERIDE   // historic one-R alias --    ");
  blk("                                  // often already present for  ");
  blk("                                  // the DEPLOY button          ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("uXboxJoystick", "gpl");
  exit(0);
}
