--    ---------------------------------------------------------------------------------
--          LICENSE
--    ---------------------------------------------------------------------------------
--    Copyright (c) 2026, Coussini
--
--    VER1.4 Coussini 2026 : Repairs the previous version to works with X-Plane 12
--    VER1.5 Coussini 2026 : Simbrief Integration using Simbrief ID
--    VER1.6 Coussini 2026 : New preferences file FollowMeXplane12.prf - clean rewrite, no accumulation
--    VER1.7 Coussini 2026 : Fix runways with no direct taxiway connection (e.g. MDPC RWY 27)
--                           Back-taxi runways are now reachable via the runway itself
--    VER1.8 Coussini 2026 : Back-taxi support - car now guides to the actual runway threshold
--                           instead of stopping at the first hotzone node on the runway
--    VER1.9 Coussini 2026 : Back-taxi GPS node - adds a virtual node at the exact runway threshold
--                           coordinates so the car drives in a straight line to the threshold
--                           and stops there with "Arrived at destination"
--    VER1.10 Coussini 2026: Fix back-taxi false detection - a runway with a taxiway exit within
--                           200m of the threshold is NOT a back-taxi (e.g. CYHU RWY 06L via K)
--                           Only truly isolated thresholds (e.g. MDPC RWY 27) trigger back-taxi
--    VER1.11 Coussini 2026: Universal runway entry rule - match_runway() now selects the runway
--                           node WITH a taxiway neighbour closest to the threshold, instead of
--                           the closest node regardless of connectivity. The virtual GPS threshold
--                           node handles all on-runway travel automatically (back-taxi, TNCB, etc.)
--                           is_backtaxi detection simplified: always add virtual node if entry
--                           is more than 50m from the threshold.
--    VER1.12 Coussini 2026: (1) full_reset() - single cleanup entry point that unloads all 3D
--                           objects and reinitialises all state variables. Called when X-Plane
--                           starts a new flight (fm_new_flight resets) and from exit_plugin().
--                           Fixes double-car bug after location change without restarting X-Plane.
--                           (2) Car proximity fix - if the car would spawn more than 150m from
--                           the aircraft, it is placed directly behind the aircraft instead so
--                           the pilot can always see it immediately.
--    VER1.15 Coussini 2026: Universal gate fallback - when determine_pos_on_segment() finds no
--                           reachable segment from a gate position (common when apt.dat has isolated
--                           1201 nodes near parking areas with no 1202 segments, e.g. CYQB has 88
--                           such nodes), the function now falls back to the nearest CONNECTED node
--                           (any node with at least one segment). This fixes "no taxiway" and
--                           routing-through-fields for all airports universally without patching
--                           individual airport cases.
--    VER1.16 Coussini 2026: Centreline projection -- last A* node (taxiway/runway junction)
--                           sits on the runway EDGE, not the centreline. Fix: project it
--                           perpendicularly onto the centreline axis (REPLACE in-place).
--                           All back-taxi walk nodes also projected. Car drives to exact
--                           runway centre, then straight to threshold. No curve, no field.
--    VER1.17 Coussini 2026: (1) Crash fix - airport change caused X-Plane access violation.
--                           FlyWithLua restarts the Lua engine on each airport/plane change
--                           (LUA_RUN increments). XPLMRegisterDataAccessor left dangling
--                           pointers -> crash. Fix: tire datarefs use XPLMFindDataRef +
--                           XPLMSetDatavf each frame. fm/anim/sign registered once on
--                           LUA_RUN==1, found via FindDataRef on subsequent runs.
--                           (2) Universal GPS threshold - for ALL departures (back-taxi
--                           or not, intersecting runways or not), the car now always
--                           stops at the exact GPS threshold of the requested runway.
--                           The A* route is followed only for taxiways; at the first
--                           runway/hotzone node the route is cut. The last taxiway node
--                           is projected onto the runway centreline, then a GPS threshold
--                           node is added. Eliminates wrong-runway stops at intersecting
--                           runways (e.g. KSYR RWY 28 stopping at RWY 33 threshold) and
--                           the diagonal-into-grass bug. is_backtaxi removed - no longer
--                           needed since all departures use the same GPS threshold logic.
--    VER1.19 Coussini 2026: Cross-runway taxiway fix - at airports where a taxiway passes
--                           through a runway threshold node (e.g. KSYR: taxiways M and L
--                           connect through the RWY 33 threshold node 126 to reach RWY 28),
--                           the previous code broke the route at node 126 because it was
--                           typed "runway", sending the car straight across the field.
--                           Fix: a runway/hotzone node that still has at least one non-runway
--                           taxiway segment is TRAVERSABLE - the car follows the taxiway
--                           through it normally (hotzone not set). Only a node whose ALL
--                           segments are runway/hotzone triggers the GPS threshold logic.
--                           Result: Y->M->threshold33->L->threshold28 followed on pavement, no field.
--    VER1.20 Coussini 2026: Route node logging + drive node fix + arrived fix:
--                           (1) RAW A* ROUTE logMsg: complete A* node sequence,
--                               identical to what the HTML pathfinder tool displays.
--                           (2) RAW A* NODES total=N: count of nodes in the raw route.
--                           (3) DRIVE NODES logMsg: waypoints actually followed by the
--                               car. The count is now always equal to the RAW count.
--                           Removal of the angle<=10 degree merge AND the l_last_taxiway scan
--                           + hotzone break: these two mechanisms were reducing the number
--                           of DRIVE NODES (e.g., KSYR: 16 RAW -> 11 drive). Every A* node
--                           is now inserted unconditionally. VER1.17 alone handles
--                           runway entry via centerline projection + threshold GPS.
--                           Premature "arrived" fix: the old condition
--                           hotzone=="1" + curr_node>=#t_node-2 triggered the
--                           arrived.wav sound and the STOP sign at the first hotzone node
--                           encountered (e.g., CYQB: node 78, RWY 11 back-taxi entry).
--                           Since VER1.17, the last DRIVE NODE is always the exact
--                           threshold GPS. The condition is simplified to curr_node>=#t_node
--                           for the 3 relevant areas (car_sign, initialise_routes,
--                           taxi_light kill). The sound now plays at the actual threshold.
--    VER1.21 Coussini 2026: Disambiguate duplicate gate names.
--    VER1.22 Coussini 2026: Add FROM/TO labels in RAW A* ROUTE and DRIVE NODES log lines.
--                           depart_arrive==1: FROM=gate/ramp  TO=RWY xx
--                           depart_arrive==2: FROM=RWY xx     TO=gate
--    VER1.23 Coussini 2026: 1206 ground vehicle edge filter - apt.dat row 1206 defines
--                           routing edges reserved for ground vehicles only (not aircraft).
--                           During apt.dat parsing, all 1206 node pairs are collected in
--                           t_filter_1206[]. After all 1202 segments are loaded,
--                           apply_1206_filter() removes any node that appears ONLY in 1206
--                           edges (never in a 1202 taxiway/runway segment). This keeps the
--                           taxi network clean: vehicle-only nodes are pruned, their entries
--                           in t_taxinode[].Segment are cleared so A* never visits them, and
--                           a logMsg reports how many nodes were filtered per airport.
--    VER1.24 Coussini 2026: Precise centreline projection using exact runway pair.
--                           VER1.17 searched all t_runway[] entries and kept the last
--                           non-matching one as "far threshold" - at airports with multiple
--                           runways (e.g. LOWI has 08/26 and 08L/26R), the far point could
--                           land on the wrong runway, shifting the centreline axis and causing
--                           the projected node to miss the centreline.
--                           Fix: decipher_runway() now stores a Pair index in each t_runway[]
--                           entry so that process_possible_routes() can find the exact opposite
--                           threshold of the same physical runway (same apt.dat line 100).
--                           The centreline is then defined by (far_threshold -> near_threshold)
--                           for that specific runway only.
--                           DRIVE NODES log now includes GPS lat/lon for each waypoint so the
--                           pilot can compare with the in-sim position directly.
--    VER1.25 Coussini 2026: Car initial heading fix - align car toward the aircraft at spawn.
--                           When the pilot was standing behind the car at route start, the car
--                           heading (t_node[1].heading) pointed away from the plane. This caused
--                           move_car() to enter the "too far + in_sight → accel=0" branch and
--                           freeze the car (car_speed=0, no accel possible). The car would not
--                           move until the pilot walked far enough behind it to trigger the
--                           l_car_is_behind cone (45°), which could take 20-30 seconds.
--                           Fix: start_car() now re-orients car_body_heading toward the plane
--                           whenever the car is NOT already in front of the aircraft. The car
--                           immediately begins driving toward its first node and the pilot sees
--                           the correct sign from the very first frame of the run.
--                           Add a log to check the car_sign process
--    VER2.0 Coussini 2026:  Major version that combine several follow me car situation and bug.
--                           Arrow pointer: replaced the yellow line with a filled
--                           arrowhead triangle (tip points toward the FM car,
--                           direction is aircraft-heading-relative, 0-360 normalised).
--                           Direction message: "Follow Me Car is ready" replaced by
--                           four context-aware messages based on the relative bearing
--                           of the car to the aircraft nose
--
--                           1-Front (300-360 and 0-60 deg) : "Follow Me Car is ahead"
--                           2-Right (60-120 deg)           : "Follow Me Car is on the right"
--                           3-Rear  (120-240 deg)          : "Follow Me Car is behind"
--                           4-Left  (240-300 deg)          : "Follow Me Car is on the left"
--    ---------------------------------------------------------------------------------

if not SUPPORTS_FLOATING_WINDOWS then
    logMsg("FollowMe : imgui not supported by your FlyWithLua version")
    return
end

require("bit")
require("graphics")

local socket_ok, socket = pcall(require, "socket")
local http_ok, http = pcall(require, "socket.http")

if not socket_ok or not http_ok then
    logMsg("FollowMe : ERROR - socket.lua or socket.http not found")
end

local ffi = require("ffi")
local XPLMlib = ""

if SYSTEM == "IBM" then
    if SYSTEM_ARCHITECTURE == 64 then
        XPLMlib = "XPLM_64"
    else
        XPLMlib = "XPLM"
    end
elseif SYSTEM == "LIN" then
    if SYSTEM_ARCHITECTURE == 64 then
        XPLMlib = "Resources/plugins/XPLM_64.so"
    else
        XPLMlib = "Resources/plugins/XPLM.so"
    end
elseif SYSTEM == "APL" then
    XPLMlib = "Resources/plugins/XPLM.framework/XPLM"
else
    return
end

local XPLM = ffi.load(XPLMlib)

local cdefs =
    [[ 
enum {
	xplm_ControlCameraUntilViewChanges       = 1
	,xplm_ControlCameraForever                = 2
};

typedef struct {
	int                       structSize;
	float                     x;
	float                     y;
	float                     z;
	float                     pitch;
	float                     heading;
	float                     roll;
	} XPLMDrawInfo_t;

	typedef struct {
		int                       structSize;
		float                     locationX;
		float                     locationY;
		float                     locationZ;
		float                     normalX;
		float                     normalY;
		float                     normalZ;
		float                     velocityX;
		float                     velocityY;
		float                     velocityZ;
		int                       is_wet;
		} XPLMProbeInfo_t;

		typedef struct {
			float                     x;
			float                     y;
			float                     z;
			float                     pitch;
			float                     heading;
			float                     roll;
			float                     zoom;
			} XPLMCameraPosition_t;

			typedef void *inRefcon;
			typedef void *XPLMDataRef;
			typedef void *XPLMObjectRef;
			typedef void *XPLMInstanceRef;
			typedef void *XPLMProbeRef;
			typedef int XPLMProbeType;
			typedef int XPLMProbeResult;
			typedef int XPLMCameraControlDuration;

			typedef int (* XPLMCameraControl_f)(XPLMCameraPosition_t *outCameraPosition,
			int inIsLosingControl,
			void *inRefcon);

			typedef void (*XPLMObjectLoaded_f)(XPLMObjectRef inObject, void *inRefcon);

			typedef int (*XPLMGetDatai_f)(void *inRefcon);
			typedef void (*XPLMSetDatai_f)(void *inRefcon, int inValue);
			typedef float (*XPLMGetDataf_f)(void *inRefcon);
			typedef void (*XPLMSetDataf_f)(void *inRefcon, float inValue);
			typedef double (*XPLMGetDatad_f)(void *inRefcon);
			typedef void (*XPLMSetDatad_f)(void *inRefcon, double inValue);
			typedef int (*XPLMGetDatavi_f)(void *inRefcon, int *outValues, int inOffset, int inMax);
			typedef void (*XPLMSetDatavi_f)(void *inRefcon, int *inValues, int inOffset, int inCount);
			typedef int (*XPLMGetDatavf_f)(void *inRefcon, float *outValues, int inOffset, int inMax);
			typedef void (*XPLMSetDatavf_f)(void *inRefcon, float *inValues, int inOffset, int inCount);
			typedef int (*XPLMGetDatab_f)(void *inRefcon, void *outValue, int inOffset, int inMaxLength);
			typedef void (*XPLMSetDatab_f)(void *inRefcon, void *inValue, int inOffset, int inLength);

			XPLMDataRef XPLMRegisterDataAccessor(
			const char *         inDataName,
			int                  inDataType,
			int                  inIsWritable,
			XPLMGetDatai_f       inReadInt,
			XPLMSetDatai_f       inWriteInt,
			XPLMGetDataf_f       inReadFloat,
			XPLMSetDataf_f       inWriteFloat,
			XPLMGetDatad_f       inReadDouble,
			XPLMSetDatad_f       inWriteDouble,
			XPLMGetDatavi_f      inReadIntArray,
			XPLMSetDatavi_f      inWriteIntArray,
			XPLMGetDatavf_f      inReadFloatArray,
			XPLMSetDatavf_f      inWriteFloatArray,
			XPLMGetDatab_f       inReadData,
			XPLMSetDatab_f       inWriteData,
			void *               inReadRefcon,
			void *               inWriteRefcon);

			XPLMObjectRef XPLMLoadObject( const char *inPath);
			void XPLMLoadObjectAsync( const char * inPath, XPLMObjectLoaded_f inCallback, void *inRefcon);
			XPLMInstanceRef XPLMCreateInstance(XPLMObjectRef obj, const char **datarefs);
			void XPLMInstanceSetPosition(XPLMInstanceRef instance, const XPLMDrawInfo_t *new_position, const float *data);
			XPLMProbeRef XPLMCreateProbe(XPLMProbeType inProbeType);
			XPLMProbeResult XPLMProbeTerrainXYZ( XPLMProbeRef inProbe, float inX, float inY, float inZ, XPLMProbeInfo_t *outInfo);
			void XPLMUnregisterDataAccessor(XPLMDataRef inDataRef);
			void XPLMDestroyInstance(XPLMInstanceRef instance);
			void XPLMUnloadObject(XPLMObjectRef inObject);
			void XPLMDestroyProbe(XPLMProbeRef inProbe);
			void XPLMControlCamera(XPLMCameraControlDuration inHowLong, XPLMCameraControl_f  inControlFunc, void *inRefcon);
			void XPLMDontControlCamera(void);
			int XPLMIsCameraBeingControlled(XPLMCameraControlDuration *outCameraControlDuration);
			void XPLMReadCameraPosition(XPLMCameraPosition_t *outCameraPosition);
			void XPLMWorldToLocal( double inLatitude, double inLongitude, double inAltitude, double *outX, double *outY, double *outZ);
			void XPLMLocalToWorld( double inX, double inY, double inZ, double *outLatitude, double *outLongitude, double *outAltitude);
			void XPLMGetSystemPath(char * outSystemPath);
			XPLMDataRef XPLMFindDataRef(const char * inDataRefName);
			void XPLMSetDataf(XPLMDataRef inDataRef, float inValue);
			void XPLMSetDatavf(XPLMDataRef inDataRef, float * inValues, int inOffset, int inCount);
			]]

ffi.cdef(cdefs)

local char_str = ffi.new("char[256]")
local datarefs_addr = ffi.new("const char**")
local dataref_name = ffi.new("char[150]")

local dataref_array = ffi.new("const char*[7]")

local dataref_array2 = ffi.new("const char*[2]")

-- VER1.17 : handles XPLMFindDataRef + buffers pour sync animation sans RegisterDataAccessor
local dr_tire_steer = nil
local dr_tire_rotate = nil
local dr_sign = nil
local ffi_steer_buf = ffi.new("float[2]")
local ffi_rotate_buf = ffi.new("float[4]")

local objref = ffi.new("XPLMObjectRef")
local signboardref = ffi.new("XPLMObjectRef")
local pathref = ffi.new("XPLMObjectRef")
local rampstartref = ffi.new("XPLMObjectRef")
local proberef = ffi.new("XPLMProbeRef")

local obj_instance = ffi.new("XPLMInstanceRef[1]")
local signboard_instance = ffi.new("XPLMInstanceRef[1]")
local path_instance = ffi.new("XPLMInstanceRef[100]")
local rampstart_instance = ffi.new("XPLMInstanceRef[1]")

local objpos_addr = ffi.new("const XPLMDrawInfo_t*")
local objpos_value = ffi.new("XPLMDrawInfo_t[1]")
local float_addr = ffi.new("const float*")
local float_value = ffi.new("float[1]")
local dataref_float_addr = ffi.new("const float*")
local dataref_float_value = ffi.new("float[7]")
local dataref_float_value2 = ffi.new("float[2]")
local probeinfo_addr = ffi.new("XPLMProbeInfo_t*")
local probeinfo_value = ffi.new("XPLMProbeInfo_t[1]")
local probetype = ffi.new("int[1]")
local x1_value = ffi.new("double[1]")
local y1_value = ffi.new("double[1]")
local z1_value = ffi.new("double[1]")

-- Configuration / State Variables
local syspath = ""
local BUFSIZE = 102400

local flightstart = 0

local followme_wnd = nil
local holder_wnd = nil

local screen_width = 1920
local Holder_len = 30
-- VER1.6 : prev_mouse_Y removed - holder position is fixed
local holder_drag = 0
local toggle_window = false
-- counter to block parasitic IsMouseReleased after hide_window
local ignore_next_release = 0
local window_is_open = false
local text_was_chg = false

local prepare_show_objects = false
local prepare_kill_objects = false
-- VER1.4 : true=manual cancel, false=auto (takeoff)
local kill_is_manual = false

local init_load = 0
local window_first_access = false
local FM_car_active = false

local depart_arrive = 0
local depart_gate, arrival_gate, depart_runway, gatetext = 0, 0, "", ""
-- VER1.8 : true when runway has no direct taxiway exit (back-taxi)
local is_backtaxi = false

local curr_ICAO, curr_ICAO_Name = "", ""
local t_runway, t_runway_node, t_gate, t_taxinode, t_segment = {}, {}, {}, {}, {}

-- VER1.3 : Runways without defined routes
local t_deleted_runway = {}

-- VER1.23 : collected 1206 ground-vehicle-only edge pairs (node1, node2)
local t_filter_1206 = {}

local t_possible_route = {}
local t_node = {}
local t_suitable_gates = {}

local Err_Msg = {}
Err_Msg[1] = {}
Err_Msg[2] = {}
Err_Msg[3] = {}

local ac_types = {
    "0 - Fighter",
    "1 - SUPER HEAVY JET - A-380, C-5, 747",
    "2 - HEAVY JET - A-340, A-330, 777",
    "3 - LARGE JET - A-320, 737, 757",
    "4 - LARGE PROP - A-400, Hercules, C130",
    "5 - MEDIUM JET - Regional Jets CRJ, ERJ",
    "6 - MEDIUM PROP - Regional Prop planes Dash8-100",
    "7 - LIGHT JET - Learjet, Gulfstream, Fighter",
    "8 - LIGHT PROP - GA prop planes"
}

-- Preference / Config Values
-- VER1.6 : fixed position - no longer saved/loaded from preferences
local Win_Y = 400
-- VER1.4 : default LIGHT PROP for Cessna
local Aircraft_Type = "8"
-- VER1.6 : aircraft types table { ICAO = type } loaded from FollowMeXplane12.prf
local t_aircraft = {}
-- VER1.5 : replaces get_from_FMS
local get_from_SimBrief = false
local show_path = false
local show_rampstart = false
local impose_restriction_chk = true
local random_gate = false
local vol = 5
local speed_limiter = false

local car_type_fmcar = "Auto"

-- VER1.5 : SimBrief variables
-- User's SimBrief numeric ID (max 10 chars)
local simbrief_id = ""
-- SimBrief departure airport ICAO (4 letters)
local sb_origin_icao = ""
-- SimBrief departure airport name
local sb_origin_name = ""
-- SimBrief arrival airport ICAO (4 letters)
local sb_dest_icao = ""
-- SimBrief arrival airport name
local sb_dest_name = ""
-- SimBrief takeoff runway
local sb_runway_takeoff = ""
-- SimBrief landing runway
local sb_runway_landing = ""
-- Last fetch status: "", "OK", "ERROR", "LOADING"
local sb_fetch_status = ""
-- true if SimBrief departure airport != current sim airport
local sb_airport_mismatch = false

dataref("viewext", "sim/graphics/view/view_is_external")
dataref("camera_z_position", "sim/graphics/view/pilots_head_z")
dataref("fm_gear1_gnd", "sim/flightmodel2/gear/on_ground", "readonly", 0)
dataref("fm_gear2_gnd", "sim/flightmodel2/gear/on_ground", "readonly", 1)

dataref("fm_new_flight", "sim/time/total_flight_time_sec")
dataref("fm_run_time", "sim/time/total_running_time_sec")
dataref("fm_sim_time", "sim/operation/misc/frame_rate_period")
dataref("fm_replay", "sim/time/is_in_replay")
dataref("fm_plane_x", "sim/flightmodel/position/local_x")
dataref("fm_plane_y", "sim/flightmodel/position/local_y")
dataref("fm_plane_z", "sim/flightmodel/position/local_z")

dataref("fm_gnd_spd", "sim/flightmodel/position/groundspeed", "readonly")
dataref("fm_plane_head", "sim/flightmodel/position/psi", "readonly")

-- VER1.4 : Automatic detection of Toliss vs other aircraft
local fm_taxi_light = 0
local fm_beacon_light = 0

if (string.lower(PLANE_AUTHOR) == "gliding kiwi") or (string.lower(PLANE_AUTHOR) == "glidingkiwi") then
    dataref("fm_taxi_light", "AirbusFBW/OHPLightSwitches", "readonly", 3)
    dataref("fm_beacon_light", "AirbusFBW/OHPLightSwitches", "readonly", 7)
else
    dataref("fm_taxi_light", "sim/cockpit/electrical/taxi_light_on", "readonly")
    dataref("fm_beacon_light", "sim/cockpit/electrical/strobe_lights_on", "readonly")
end

local snd_arrived = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/arrived.wav")
local snd_followme = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/followme.wav")
local snd_safeflight_bye = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/safeflight_goodbye.wav")
local snd_welcome = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/welcome_followme.wav")
local snd_welcome_bye = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/welcomeagain_goodbye.wav")
-- VER1.6 fix : speed warning sound (played when aircraft exceeds 20 kts and speed_limiter is active)
local snd_keep_speed = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/KeepYourSpeed20Kts.wav")

local path_is_shown = false
local rampstart_chg = false
local world_alt = 0

local gravity = 9.81
local cof = 0.35
local deccel_max = -8.9
local deccel_avg = -4
local accel_avg = 2

local car_weight = 1445
local tire_diameter = 0.79
local min_turn_radius = 10.8
local width_btw_midtire = 1.86
local car_front_to_back_wheel = 3.2
local car_rear_wheel_to_ref = 1.60
local time_to_100 = 3
local speed_max = 88.88
local place_above_the_car = 1.43
local place_Z_of_car = 0

local min_dist_from_plane = 30
local avg_dist_from_plane = 65
local max_dist_from_plane = 80

local accel_max = 27.7778 / time_to_100
local max_steering = math.deg(math.atan(car_front_to_back_wheel / min_turn_radius))
local min_rot_radius = min_turn_radius + (width_btw_midtire / 2)
local car_default_speed = speed_max

local i = 0
local curr_node = 1
local tire_rotate = 0
local steering = 0
local car_sign = 0

local elapsed_time = 0
local car_speed = 0
local car_accel = 0
local car_x, car_y, car_z = 0, 0, 0
local remaining_dist_leg = 0
local turning_is_active = 0
local prev_taxi_light = 0
local prev_beacon_light = 0
local ground_time = 0
-- VER1.12 : detect new flight when fm_new_flight resets
local prev_new_flight = 0
-- VER1.12 : detect teleport (delta pos > 100m in 1 frame)
local prev_plane_x = 0
local prev_plane_z = 0
local taxiway_network = ""
local play_time = 0
local play_text = ""
-- VER1.6 fix : cooldown timer to avoid repeating speed warning every frame
local speed_warn_time = 0

-- VER1.5 : SimBrief functions
-- ====================================================
-- Function: check_SimBrief
-- Description:
-- Fetches the active flight plan from the SimBrief API using the pilot's
-- numeric SimBrief user ID. Sends an HTTP GET to the SimBrief XML endpoint,
-- parses the response to extract departure ICAO, arrival ICAO, departure
-- runway, and landing runway. Sets sb_fetch_status to "OK", "ERROR",
-- "NO_ID", or "NO_DATA" accordingly. If the fetched origin ICAO does not
-- match the current X-Plane airport, sets sb_airport_mismatch = true so
-- the UI can warn the pilot. If auto-SimBrief mode (get_from_SimBrief) is
-- active, immediately calls apply_simbrief_runway() to pre-select the
-- runway in the UI.
-- ====================================================
function check_SimBrief()
    -- Reset minimal state
    sb_fetch_status = ""
    sb_airport_mismatch = false

    if simbrief_id == "" then
        sb_fetch_status = "NO_ID"
        return
    end

    sb_fetch_status = "LOADING"

    local response_body = {}
    local url = "https://www.simbrief.com/api/xml.fetcher.php?userid=" .. simbrief_id .. "&v=xml"

    local res, code =
        http.request {
        url = url,
        sink = ltn12.sink.table(response_body),
        redirect = true
    }

    -- If the request fails -> do not block anything
    if code ~= 200 or not response_body or #response_body == 0 then
        sb_fetch_status = "ERROR"
        return
    end

    local xml_body = table.concat(response_body)

    -- Secure extraction
    sb_origin_icao = string.match(xml_body, "<origin>.-<icao_code>(.-)</icao_code>") or ""
    sb_dest_icao = string.match(xml_body, "<destination>.-<icao_code>(.-)</icao_code>") or ""
    sb_origin_name = string.match(xml_body, "<origin>.-<name>(.-)</name>") or ""
    sb_dest_name = string.match(xml_body, "<destination>.-<name>(.-)</name>") or ""
    sb_runway_takeoff = string.match(xml_body, "<origin>.-<plan_rwy>(.-)</plan_rwy>") or ""
    sb_runway_landing = string.match(xml_body, "<destination>.-<plan_rwy>(.-)</plan_rwy>") or ""

    -- Minimal verification
    if sb_origin_icao == "" or sb_dest_icao == "" then
        sb_fetch_status = "NO_DATA"
        return
    end

    sb_fetch_status = "OK"

    -- Airport check is not equal to X-Plane
    sb_airport_mismatch = (sb_origin_icao ~= curr_ICAO)

    -- Automatic application if enabled
    if get_from_SimBrief then
        apply_simbrief_runway()
    end
end

-- ====================================================
-- Function: apply_simbrief_runway
-- Description:
-- Maps the SimBrief runway to the FollowMe route system. Reads the
-- departure or arrival runway from the last successful SimBrief fetch
-- and checks whether that runway exists in t_runway (the list of runways
-- that have a valid taxiway route at the current airport). If found,
-- sets depart_runway so the pilot does not have to pick it manually.
-- If not found (e.g. the SimBrief runway has no route in apt.dat),
-- clears depart_runway and posts error -19 to the status bar.
-- Called automatically after check_SimBrief() and whenever the pilot
-- toggles between Departure and Arrival mode.
-- ====================================================
function apply_simbrief_runway()
    -- Apply SimBrief runway based on departure/arrival mode
    -- and verify the runway exists in t_runway (runways with valid routes)

    depart_runway = ""

    if sb_fetch_status ~= "OK" then
        return
    end

    local l_rwy = ""

    if depart_arrive == 1 and sb_origin_icao == curr_ICAO then
        -- Departure: use SimBrief takeoff runway
        l_rwy = tostring(tonumber(sb_runway_takeoff))
    elseif depart_arrive == 2 and sb_dest_icao == curr_ICAO then
        -- Arrival: use SimBrief landing runway
        l_rwy = tostring(tonumber(sb_runway_landing))
    end

    if l_rwy == "" then
        return
    end

    -- Check that runway exists in t_runway (runways with valid routes)
    local l_found = false
    for i = 1, #t_runway do
        if t_runway[i].ID == l_rwy then
            l_found = true
            break
        end
    end

    if l_found then
        depart_runway = l_rwy
    else
        -- SimBrief runway has no defined route
        depart_runway = ""
        update_msg("-19")
    end
end

-- ====================================================
-- Function: start_car
-- Description:
-- Initialises all car motion state variables (speed, acceleration,
-- steering, sign, node counter) to zero and places the Follow Me car at
-- the first waypoint of the computed route. If the route start point is
-- more than 150 m away from the aircraft (e.g. the gate is on the far
-- side of the apron), the car is spawned directly 15 m behind the plane
-- instead so it is always immediately visible to the pilot. Determines
-- whether the car is already visible in front of the aircraft; if not,
-- rotates car_body_heading toward the plane so move_car() can begin
-- accelerating on the very first frame (VER1.25 freeze fix). Posts the
-- appropriate ready message ("3" = car ahead, "6" = car behind you).
-- ====================================================
function start_car()
    car_speed = 0
    car_accel = 0
    tire_rotate = 0
    steering = 0
    car_sign = 0
    remaining_dist_leg = 0
    turning_is_active = 0
    curr_node = 1

    -- Clear any previous messages to avoid collision (e.g. "Arrived" + "Follow Me")
    Err_Msg[1] = {}
    Err_Msg[2] = {}
    Err_Msg[3] = {}

    car_body_heading = t_node[1].heading
    car_x = t_node[1].x
    car_y = t_node[1].y
    car_z = t_node[1].z

    -- VER1.12 : if the route startpt is more than 150m away, place the car
    -- directly behind the aircraft so the pilot can always see it immediately.
    -- The car will then drive forward to join its route normally.
    local _, l_dist_to_plane = heading_n_dist(car_x, car_z, fm_plane_x, fm_plane_z)
    if l_dist_to_plane > 150 then
        local l_behind_heading = add_delta_clockwise(fm_plane_head, 180, 1)
        car_x, car_z = coordinates_of_adjusted_ref(fm_plane_x, fm_plane_z, 0, 15, l_behind_heading)
        car_y = probe_y(car_x, car_y, car_z)
        car_body_heading = fm_plane_head
    end

    local l_car_is_in_front = false
    local l_plane_is_in_front = false

    l_car_is_in_front = chk_line_of_sight(fm_plane_head, 120, 120, fm_plane_x, fm_plane_z, car_x, car_z)
    l_plane_is_in_front = chk_line_of_sight(car_body_heading, 120, 120, car_x, car_z, fm_plane_x, fm_plane_z)

    -- VER1.25 : align car_body_heading toward the aircraft at spawn when the car
    -- is NOT already visible in front of the plane.  Without this, if the pilot
    -- stands behind (or to the side of) the car, move_car() enters the
    -- "dist > max_dist AND in_sight" branch which sets accel=0 when speed is
    -- already 0 - freezing the car until the pilot happens to walk far enough
    -- behind it to trigger the l_car_is_behind 45-degree cone.
    -- Pointing the car toward the plane ensures it immediately falls into the
    -- correct speed-control branch and starts moving on frame 1.
    if not l_car_is_in_front then
        local l_head_to_plane, _ = heading_n_dist(car_x, car_z, fm_plane_x, fm_plane_z)
        car_body_heading = l_head_to_plane
    end

    if l_car_is_in_front == false and depart_gate == 0 then
        update_msg("6")
    else
        update_msg("3")
    end
end

-- ====================================================
-- Function: determine_exit_angle
-- Description:
-- Computes the arc exit angle for a tight turn (angle of curve < 90 deg).
-- Iterates over candidate arc angles starting from lever_angle and finds
-- the arc sweep that minimises the deviation between the tangent of the
-- outer turn circle and the heading toward the next waypoint. The result
-- is used by determine_dir_of_turn() to set head1_exit, the car heading
-- at the moment it leaves the curved section and enters the straight leg
-- to the next node. Ensures the FM car stays on paved surfaces through
-- sharp corners on narrow taxiways.
-- ====================================================
function determine_exit_angle(lever_angle)
    local l_prev_deviation, l_curr_deviation = 8888, 0
    local l_angle_btw_circle = 0
    local l_circle_radius = 0
    local l_dist_btw_circles = 0
    local l_angle_horizon_to_center_circle2
    local l_adjacent, l_hyper, l_x, l_z, l_delta_z = 0, 0, 0, 0, 0
    local l_delta_angle_rad = 0
    local l_delta_angle_deg = 0
    local l_total_angle = 0

    l_circle_radius = min_rot_radius
    l_dist_btw_circles = 2 * l_circle_radius

    for l_delta_angle_deg = lever_angle, lever_angle + 90, 0.1 do
        l_delta_angle_rad = math.rad(l_delta_angle_deg)
        l_x = l_dist_btw_circles * math.sin(l_delta_angle_rad)
        l_adjacent = l_x + l_circle_radius
        l_z = l_dist_btw_circles * math.cos(l_delta_angle_rad)
        l_delta_z = l_z - l_circle_radius
        l_hyper = math.sqrt((l_delta_z ^ 2) + (l_adjacent ^ 2))
        l_angle_horizon_to_center_circle2 = math.asin(l_delta_z / l_hyper)
        l_angle_horizon_to_center_circle2 = math.deg(l_angle_horizon_to_center_circle2)
        l_total_angle = math.asin(l_circle_radius / l_hyper)
        l_total_angle = math.deg(l_total_angle)
        l_curr_deviation = math.abs(l_total_angle - (l_angle_horizon_to_center_circle2 + lever_angle))

        if l_curr_deviation < l_prev_deviation then
            l_prev_deviation = l_curr_deviation
            l_angle_btw_circle = l_delta_angle_deg
        end
    end

    return 90 + l_angle_btw_circle
end

-- ====================================================
-- Function: chk_line_of_sight
-- Description:
-- Tests whether a target position (target_x, target_z) falls within the
-- angular field of view of a shooter located at (shooter_x, shooter_z)
-- facing shooter_heading. The cone spans shooter_left_angle degrees to
-- the left and shooter_right_angle degrees to the right. Returns a boolean
-- (within sight), the exact bearing from shooter to target, and the
-- straight-line distance. Used by object_physics() to detect whether the
-- aircraft can see the FM car (160-deg forward cone), and by move_car()
-- to detect whether the car can see the aircraft in its rear 90-deg cone.
-- ====================================================
function chk_line_of_sight(
    shooter_heading,
    shooter_left_angle,
    shooter_right_angle,
    shooter_x,
    shooter_z,
    target_x,
    target_z)
    local l_heading_to_target, l_dist_to_target = 0, 0
    local l_left_arc, l_right_arc = 0, 0
    local l_within_sight = false

    l_heading_to_target, l_dist_to_target = heading_n_dist(shooter_x, shooter_z, target_x, target_z)
    l_left_arc = add_delta_clockwise(shooter_heading, shooter_left_angle, -1)
    l_right_arc = math.fmod(shooter_heading + shooter_right_angle, 360)

    if
        l_heading_to_target >= l_left_arc and
            l_heading_to_target <= l_left_arc + shooter_left_angle + shooter_right_angle
     then
        l_within_sight = true
    elseif
        l_heading_to_target <= l_right_arc and
            l_heading_to_target >= l_right_arc - (shooter_left_angle + shooter_right_angle)
     then
        l_within_sight = true
    end

    return l_within_sight, l_heading_to_target, l_dist_to_target
end

-- ====================================================
-- Function: move_car
-- Description:
-- Decides the FM car acceleration for the current frame based on its
-- distance to the aircraft and whether the aircraft is within the car's
-- forward sight cone. Implements five behaviour zones:
--   dist >= max (80 m) AND visible : decelerate (car is too far ahead)
--   avg (65 m) <= dist < max, visible : match aircraft ground speed
--   min (30 m) <= dist < avg, visible : accelerate to catch up
--   dist < min OR aircraft is behind the car : full acceleration
--   out of sight AND far : emergency brake
-- If speed_limiter is active, the aircraft reference speed is capped at
-- speed_max (20 kts) so the car never exceeds the taxiway limit.
-- Delegates motion integration to manage_car_motion().
-- ====================================================
function move_car(in_dist, in_car_in_sight, in_car_is_behind)
    -- VER1.6 fix : when speed_limiter is active, cap the plane speed reference so
    -- the car never accelerates beyond speed_max to follow the aircraft
    local l_ref_spd = fm_gnd_spd
    if speed_limiter and l_ref_spd > speed_max then
        l_ref_spd = speed_max
    end

    if in_dist >= max_dist_from_plane and in_car_in_sight then
        if car_speed > 0 then
            if car_accel >= 0 then
                car_accel = deccel_avg
            end
        else
            car_accel = 0
        end
    elseif in_dist >= avg_dist_from_plane and in_dist < max_dist_from_plane and in_car_in_sight then
        if car_speed > 0 then
            if l_ref_spd > car_speed then
                car_accel = (l_ref_spd - car_speed) / elapsed_time
                if car_accel > accel_max then
                    car_accel = accel_max
                end
            elseif car_accel >= 0 then
                car_accel = deccel_avg
            end
        else
            car_accel = 0
        end
    elseif in_dist >= min_dist_from_plane and in_dist < avg_dist_from_plane and in_car_in_sight then
        if car_speed > 0 then
            if l_ref_spd > car_speed then
                car_accel = accel_max
            elseif car_accel >= 0 then
                car_accel = accel_avg
            end
        else
            car_accel = accel_max
        end
    elseif in_dist < min_dist_from_plane or in_car_is_behind == true then
        car_accel = accel_max
    else
        if car_speed > 0 then
            car_accel = deccel_max
        else
            car_accel = 0
        end
    end

    manage_car_motion()
end

-- ====================================================
-- Function: manage_car_motion
-- Description:
-- Integrates the car speed and position for one simulation frame.
-- Applies the current car_accel to car_speed, clamps speed to [0, speed_max],
-- and enforces the per-node turn speed limit when a curve is active. Computes
-- the distance travelled this frame and triggers anticipatory braking if the
-- car is close enough to the next node that it must start slowing now to reach
-- turn speed (or stop at the last GPS threshold node). Calls plot_position()
-- to advance the car along the route geometry by the computed distance.
-- ====================================================
function manage_car_motion()
    local l_dist = 0
    local l_dist_stop = 0
    local l_dist_turn = 0

    car_speed = car_speed + (car_accel * elapsed_time)

    if car_speed > speed_max then
        car_speed = speed_max
        if car_accel > 0 then
            car_accel = 0
        end
    elseif car_speed < 0 then
        car_speed = 0
    end

    if turning_is_active > 0 and t_node[curr_node + 1].speed ~= nil then
        if car_speed > t_node[curr_node + 1].speed then
            car_speed = t_node[curr_node + 1].speed
            if car_accel > 0 then
                car_accel = 0
            end
        end
    end

    l_dist = (car_speed * elapsed_time) + (0.5 * math.abs(car_accel) * math.pow(elapsed_time, 2))

    if remaining_dist_leg > 0 then
        l_dist_stop = math.abs(0.5 * (math.pow((car_speed / 2), 2) / deccel_avg))

        l_dist_turn = 0
        if t_node[curr_node + 1].dir ~= nil and t_node[curr_node + 1].speed ~= nil and curr_node + 1 ~= #t_node then
            if turning_is_active == 0 and t_node[curr_node + 1].dir ~= 0 and car_speed > t_node[curr_node + 1].speed then
                l_dist_turn = math.abs(0.5 * ((((car_speed + t_node[curr_node + 1].speed) / 2) ^ 2) / deccel_avg))

                if l_dist_turn > (remaining_dist_leg - l_dist) then
                    car_accel = deccel_avg
                    car_speed = car_speed + (car_accel * elapsed_time)
                    l_dist = (car_speed * elapsed_time) + (0.5 * math.abs(car_accel) * math.pow(elapsed_time, 2))
                end
            end
        end

        if curr_node + 1 == #t_node then
            if l_dist_stop > (remaining_dist_leg - l_dist) then
                car_accel = deccel_avg
                car_speed = car_speed + (car_accel * elapsed_time)
                l_dist = (car_speed * elapsed_time) + (0.5 * math.abs(car_accel) * math.pow(elapsed_time, 2))
            end
        end
    end

    plot_position(l_dist)
end

-- ====================================================
-- Function: plot_position
-- Description:
-- Advances the FM car along the pre-computed route by in_act_dist metres
-- for the current frame. Handles two movement phases: straight legs
-- (simple forward translation along the current heading) and curved arcs
-- (circular rotation around the Centre of Rotation computed by
-- CoR_coordinates_using_car_ref()). Detects when the car has consumed
-- the straight portion before a turn (dist_b4_turn reached) and
-- activates turning_is_active = 1 to enter the arc phase. Manages the
-- two-arc S-curve case (turning_is_active = 2) for tight reversing
-- manoeuvres. Advances curr_node when the full leg is consumed, updates
-- the signboard sign (turn left/right/stop), and calls
-- determine_dir_of_turn() to pre-compute the next node's turn geometry.
-- ====================================================
function plot_position(in_act_dist)
    local l_remaining_turn_dist = 0
    local l_remaining_act_dist = 0
    local l_remaining_leg_dist = 0
    local l_remaining_rot = 0
    local l_head1 = 0
    local l_head2 = 0
    local l_dist = 0
    local l_heading_from_center, l_heading_to_center = 0, 0
    local l_goto_nextnode = false
    local l_AoR = 0
    local l_act_dist = 0

    if curr_node == #t_node then
        return
    end

    l_act_dist = in_act_dist

    if turning_is_active == 0 then
        if curr_node + 1 ~= #t_node then
            remaining_dist_leg = 0
            l_rear_wheel_x, l_rear_wheel_z =
                coordinates_of_adjusted_ref(car_x, car_z, 0, car_rear_wheel_to_ref * -1, car_body_heading)
            l_head1, remaining_dist_leg =
                heading_n_dist(l_rear_wheel_x, l_rear_wheel_z, t_node[curr_node + 1].x, t_node[curr_node + 1].z)
        else
            l_head1 = t_node[curr_node].heading
        end

        if curr_node + 2 <= #t_node and t_node[curr_node + 1].dir == nil then
            determine_dir_of_turn(l_head1, t_node[curr_node + 1].heading, remaining_dist_leg)
        end

        t_node[curr_node].heading = l_head1
        steering = 0
        car_body_heading = l_head1

        if curr_node + 1 == #t_node then
            if in_act_dist > remaining_dist_leg then
                l_act_dist = remaining_dist_leg
            end
        elseif t_node[curr_node + 1].dir ~= 0 then
            remaining_dist_leg = remaining_dist_leg - t_node[curr_node + 1].dist_b4_turn
        end

        l_remaining_leg_dist = remaining_dist_leg - l_act_dist

        if l_remaining_leg_dist > 0 then
            car_x = car_x + math.sin(math.rad(l_head1)) * l_act_dist
            car_z = car_z + math.cos(math.rad(l_head1)) * l_act_dist * -1
            remaining_dist_leg = l_remaining_leg_dist
        end

        if l_remaining_leg_dist <= 0 then
            car_x = car_x + math.sin(math.rad(l_head1)) * remaining_dist_leg
            car_z = car_z + math.cos(math.rad(l_head1)) * remaining_dist_leg * -1
            remaining_dist_leg = 0
            l_act_dist = math.abs(l_remaining_leg_dist)

            if t_node[curr_node + 1].dir ~= 0 and curr_node + 1 ~= #t_node then
                turning_is_active = 1
                t_node[curr_node + 1].rot_x, t_node[curr_node + 1].rot_z =
                    CoR_coordinates_using_car_ref(
                    car_x,
                    car_z,
                    l_head1,
                    t_node[curr_node + 1].radius,
                    t_node[curr_node + 1].dir
                )
            else
                l_goto_nextnode = true
            end
        end
    end

    if turning_is_active == 1 then
        if t_node[curr_node + 1].head1_exit == nil then
            l_head2, l_dist =
                heading_n_dist(
                t_node[curr_node + 1].rot_x,
                t_node[curr_node + 1].rot_z,
                t_node[curr_node + 2].x,
                t_node[curr_node + 2].z
            )
            l_AoR = math.deg(math.acos(t_node[curr_node + 1].radius / l_dist))
            l_heading_from_center = minus_delta_clockwise(l_head2, l_AoR, t_node[curr_node + 1].dir)
            t_node[curr_node + 1].head1_exit = add_delta_clockwise(l_heading_from_center, 90, t_node[curr_node + 1].dir)
            t_node[curr_node + 1].heading = t_node[curr_node + 1].head1_exit
        end

        l_remaining_rot =
            compute_angle_diff_dir(car_body_heading, t_node[curr_node + 1].head1_exit, t_node[curr_node + 1].dir)
        l_remaining_turn_dist = (l_remaining_rot / 360) * (2 * math.pi * t_node[curr_node + 1].radius)
        l_remaining_act_dist = l_act_dist - l_remaining_turn_dist

        if l_remaining_act_dist <= 0 then
            l_AoR = 360 * l_act_dist / (2 * math.pi * t_node[curr_node + 1].radius)
        else
            l_AoR = l_remaining_rot
            l_act_dist = l_remaining_act_dist
            if t_node[curr_node + 1].heading ~= t_node[curr_node + 1].head1_exit then
                turning_is_active = 2
            else
                l_goto_nextnode = true
            end
        end

        car_body_heading = add_delta_clockwise(car_body_heading, l_AoR, t_node[curr_node + 1].dir)
        l_heading_from_center =
            minus_delta_clockwise(
            car_body_heading,
            90 - t_node[curr_node + 1].angle_rear_to_ref,
            t_node[curr_node + 1].dir
        )
        car_x =
            t_node[curr_node + 1].rot_x +
            math.sin(math.rad(l_heading_from_center)) * t_node[curr_node + 1].ref_rot_radius
        car_z =
            t_node[curr_node + 1].rot_z +
            math.cos(math.rad(l_heading_from_center)) * t_node[curr_node + 1].ref_rot_radius * -1
        determine_steering(l_AoR, l_remaining_turn_dist, t_node[curr_node + 1].dir, t_node[curr_node + 1].steering)

        if turning_is_active == 2 then
            l_head1 = heading_n_dist(car_x, car_z, t_node[curr_node + 2].x, t_node[curr_node + 2].z)
            t_node[curr_node + 1].rot2_x, t_node[curr_node + 1].rot2_z =
                CoR_coordinates_using_car_ref(
                car_x,
                car_z,
                car_body_heading,
                t_node[curr_node + 1].radius,
                t_node[curr_node + 1].dir * -1
            )
            l_head2, l_dist =
                heading_n_dist(
                t_node[curr_node + 1].rot2_x,
                t_node[curr_node + 1].rot2_z,
                t_node[curr_node + 2].x,
                t_node[curr_node + 2].z
            )
            l_AoR = math.deg(math.acos(t_node[curr_node + 1].radius / l_dist))
            l_heading_from_center = minus_delta_clockwise(l_head2, l_AoR, t_node[curr_node + 1].dir * -1)
            t_node[curr_node + 1].head1_exit =
                add_delta_clockwise(l_heading_from_center, 90, t_node[curr_node + 1].dir * -1)
        end
    end

    if turning_is_active == 2 then
        l_remaining_rot =
            compute_angle_diff_dir(car_body_heading, t_node[curr_node + 1].head1_exit, t_node[curr_node + 1].dir * -1)
        l_remaining_turn_dist = (l_remaining_rot / 360) * (2 * math.pi * t_node[curr_node + 1].radius)
        l_remaining_act_dist = l_act_dist - l_remaining_turn_dist

        if l_remaining_act_dist <= 0 then
            l_AoR = 360 * l_act_dist / (2 * math.pi * t_node[curr_node + 1].radius)
        else
            l_AoR = l_remaining_rot
            l_act_dist = l_remaining_act_dist
            l_goto_nextnode = true
        end

        car_body_heading = add_delta_clockwise(car_body_heading, l_AoR, t_node[curr_node + 1].dir * -1)
        l_heading_from_center =
            minus_delta_clockwise(
            car_body_heading,
            90 - t_node[curr_node + 1].angle_rear_to_ref,
            t_node[curr_node + 1].dir * -1
        )
        car_x =
            t_node[curr_node + 1].rot2_x +
            math.sin(math.rad(l_heading_from_center)) * t_node[curr_node + 1].ref_rot_radius
        car_z =
            t_node[curr_node + 1].rot2_z +
            math.cos(math.rad(l_heading_from_center)) * t_node[curr_node + 1].ref_rot_radius * -1
        determine_steering(l_AoR, l_remaining_turn_dist, t_node[curr_node + 1].dir * -1, t_node[curr_node + 1].steering)
    end

    if l_goto_nextnode then
        turning_is_active = 0
        curr_node = curr_node + 1
        steering = 0
        car_sign = 0

        if curr_node < #t_node then
            remaining_dist_leg = 0
            l_head1, remaining_dist_leg = heading_n_dist(car_x, car_z, t_node[curr_node + 1].x, t_node[curr_node + 1].z)
            t_node[curr_node].heading = l_head1
            remaining_dist_leg = remaining_dist_leg - l_act_dist
        elseif curr_node == #t_node then
            remaining_dist_leg = 0
        end
    end

    car_y = probe_y(car_x, car_y, car_z)
    tire_rotate = math.fmod(tire_rotate + (in_act_dist * 360 / (tire_diameter * math.pi)), 360)

    if car_sign == 0 or curr_node >= #t_node - 2 then
        if curr_node ~= #t_node then
            if
                remaining_dist_leg < avg_dist_from_plane and t_node[curr_node + 1].dir ~= nil and
                    t_node[curr_node + 1].AoC < 160
             then
                if t_node[curr_node + 1].dir == -1 then
                    car_sign = 3
                elseif t_node[curr_node + 1].dir == 1 then
                    car_sign = 2
                end
            end

            -- VER1.20: The arrived condition only triggers at the last node (#t_node = GPS threshold VER1.17).
            -- The old hotzone+curr_node>=#t_node-2 condition triggered too early (e.g., CYQB node 78).
            if depart_arrive == 1 and curr_node >= #t_node then
                if not is_backtaxi then
                    car_sign = 1
                    if not string.find(Err_Msg[1].text, "Arrived at destination") then
                        update_msg("5")
                    end
                end
            end
        else
            if depart_arrive ~= 1 then
                local l_gate_in_sight, l_to_gate_heading, _ =
                    chk_line_of_sight(
                    car_body_heading,
                    45,
                    45,
                    car_x,
                    car_z,
                    t_gate[arrival_gate].x,
                    t_gate[arrival_gate].z
                )
                if l_gate_in_sight then
                    car_sign = 1
                else
                    local l_turn_dir = 0
                    _, l_turn_dir = compute_angle_diff(car_body_heading, l_to_gate_heading)
                    if l_turn_dir == 1 then
                        car_sign = 2
                    else
                        car_sign = 3
                    end
                end

                flightstart = 0
                update_msg("5")
            else
                car_sign = 1
                -- VER1.9 : send "Arrived at destination" for departures (back-taxi and normal)
                if not string.find(Err_Msg[1].text or "", "Arrived at destination") then
                    update_msg("5")
                end
            end
        end
    end
end

-- ====================================================
-- Function: CoR_coordinates_using_car_ref
-- Description:
-- Computes the Centre of Rotation (CoR) coordinates for the FM car using
-- the car reference point (rear axle midpoint) as the origin. Given the
-- car's current position, heading, turn radius, and turn direction (+1 right,
-- -1 left), it offsets perpendicularly from the rear axle to find the exact
-- centre of the circular arc the car will follow. The result is stored in
-- t_node[].rot_x / rot_z and used by plot_position() to move the car along
-- the correct curved path through taxiway intersections.
-- ====================================================
function CoR_coordinates_using_car_ref(in_x, in_z, in_heading, in_radius, in_dir)
    local l_angle_rear_to_ref = math.deg(math.atan(car_rear_wheel_to_ref / in_radius))
    local l_ref_to_center_rot = math.sqrt((car_rear_wheel_to_ref ^ 2) + (in_radius ^ 2))
    local l_heading_to_center = add_delta_clockwise(in_heading, 90 + l_angle_rear_to_ref, in_dir)
    local l_rot_x = in_x + math.sin(math.rad(l_heading_to_center)) * l_ref_to_center_rot
    local l_rot_z = in_z + math.cos(math.rad(l_heading_to_center)) * l_ref_to_center_rot * -1
    return l_rot_x, l_rot_z
end

-- ====================================================
-- Function: determine_dir_of_turn
-- Description:
-- Pre-computes all turn parameters for the next waypoint before the car
-- reaches it. Determines the turn direction (+1 right, -1 left) and the
-- angle of change (AoC). For sharp turns (AoC < 90 deg) it uses minimum
-- turn radius and calls determine_exit_angle() for the arc geometry. For
-- wider turns it selects a physically safe speed limited by the lateral
-- friction coefficient, then derives the matching turn radius. If the
-- available straight distance before the node is too short, braking distance
-- and radius are revised to avoid running wide. Results (radius, speed,
-- dist_b4_turn, head1_exit, steering limit, AoC) are stored in
-- t_node[curr_node+1] for use by plot_position() and determine_steering().
-- ====================================================
function determine_dir_of_turn(in_head1, in_head2, in_dist)
    local l_AoC = 0
    local l_AoR = 0
    local l_speed_skid = math.sqrt(cof * gravity * min_rot_radius)

    l_AoR, t_node[curr_node + 1].dir = compute_angle_diff(in_head1, in_head2)

    if t_node[curr_node + 1].dir ~= 0 then
        l_AoC = 180 - l_AoR

        if l_AoC < 90 then
            t_node[curr_node + 1].radius = min_rot_radius
            t_node[curr_node + 1].speed = math.sqrt(cof * gravity * t_node[curr_node + 1].radius)
            t_node[curr_node + 1].dist_b4_turn = t_node[curr_node + 1].radius
            l_AoR = determine_exit_angle(90 - l_AoC)
            t_node[curr_node + 1].head1_exit = add_delta_clockwise(in_head1, l_AoR, t_node[curr_node + 1].dir)
        else
            local l_speed_reduction_strength = 0
            local l_turn_speed = l_speed_skid + (l_AoC - 90) / ((180 - 90) / (speed_max - l_speed_skid))

            if l_AoC < 140 then
                l_speed_reduction_strength = math.exp((l_AoC - 90) / (10 + 6 * (l_AoC % 90) / 10))
            else
                l_speed_reduction_strength = math.exp((180 - l_AoC) / 30)
            end

            t_node[curr_node + 1].speed = l_turn_speed / l_speed_reduction_strength
            t_node[curr_node + 1].radius = (t_node[curr_node + 1].speed ^ 2) / (cof * gravity)
            t_node[curr_node + 1].dist_b4_turn = t_node[curr_node + 1].radius * math.tan(math.rad(l_AoR / 2))

            local l_revised = false

            if t_node[curr_node + 1].dist_b4_turn + 15 > in_dist then
                t_node[curr_node + 1].dist_b4_turn = in_dist - 15
                if t_node[curr_node + 1].dist_b4_turn < min_rot_radius + car_rear_wheel_to_ref then
                    t_node[curr_node + 1].dist_b4_turn = min_rot_radius + car_rear_wheel_to_ref
                end
                l_revised = true
            end

            if
                t_node[curr_node + 1].dist_b4_turn + (min_rot_radius + car_rear_wheel_to_ref) >
                    t_node[curr_node + 1].dist
             then
                t_node[curr_node + 1].dist_b4_turn =
                    t_node[curr_node + 1].dist - (min_rot_radius + car_rear_wheel_to_ref)
                if t_node[curr_node + 1].dist_b4_turn < min_rot_radius * 2 + car_rear_wheel_to_ref then
                    t_node[curr_node + 1].dist_b4_turn = min_rot_radius + car_rear_wheel_to_ref
                end
                l_revised = true
            end

            if l_revised == true then
                t_node[curr_node + 1].radius = t_node[curr_node + 1].dist_b4_turn / math.tan(math.rad(l_AoR / 2))
                if t_node[curr_node + 1].radius < min_rot_radius then
                    t_node[curr_node + 1].radius = min_rot_radius
                end
                t_node[curr_node + 1].speed = math.sqrt(cof * gravity * t_node[curr_node + 1].radius)
                if t_node[curr_node + 1].speed > speed_max then
                    t_node[curr_node + 1].speed = speed_max
                end
            end
        end

        t_node[curr_node + 1].angle_rear_to_ref =
            math.deg(math.atan(car_rear_wheel_to_ref / t_node[curr_node + 1].radius))
        t_node[curr_node + 1].ref_rot_radius =
            math.sqrt((car_rear_wheel_to_ref ^ 2) + (t_node[curr_node + 1].radius ^ 2))
        t_node[curr_node + 1].steering =
            math.deg(math.atan(car_front_to_back_wheel / (t_node[curr_node + 1].radius - width_btw_midtire / 2)))
        t_node[curr_node + 1].AoC = l_AoC
    end
end

-- ====================================================
-- Function: determine_steering
-- Description:
-- Updates the front wheel steering angle (steering) for the current turn
-- frame. Ramps the steering toward the target lock angle while there is
-- enough arc remaining, then ramps it back toward zero as the car
-- approaches the exit point so the wheels are neutral when the car
-- returns to a straight leg. The rate of change is proportional to
-- the arc swept this frame divided by a 0.5 s settling constant.
-- The steering value is consumed each frame by draw_object() via
-- sync_anim_datarefs() to animate the front wheels of the 3-D car model.
-- ====================================================
function determine_steering(in_AoR, in_remaining_turn_dist, in_dir, in_steer_limit)
    local l_prev_steer_angle = math.abs(steering)
    local l_time_in_turn = (in_AoR / 360) * (2 * math.pi * t_node[curr_node + 1].radius)
    local l_steering_delta = l_time_in_turn * max_steering / 0.5

    if l_steering_delta > in_steer_limit then
        l_steering_delta = in_steer_limit
    end

    local time_to_exit_turn = in_remaining_turn_dist / car_speed
    local time_to_steer_neutral = l_prev_steer_angle * 0.5 / max_steering

    if time_to_exit_turn < time_to_steer_neutral then
        if in_dir == 1 then
            steering = steering - l_steering_delta
            if steering <= 0 then
                steering = 0
            end
        else
            steering = steering + l_steering_delta
            if steering >= 0 then
                steering = 0
            end
        end
    elseif l_prev_steer_angle < in_steer_limit and (time_to_exit_turn > time_to_steer_neutral) then
        if in_dir == 1 then
            steering = steering + l_steering_delta
            if steering >= in_steer_limit then
                steering = in_steer_limit
            end
        else
            steering = steering - in_AoR
            if steering <= in_steer_limit * -1 then
                steering = in_steer_limit * -1
            end
        end
    end
end

-- ====================================================
-- Function: coordinates_of_adjusted_ref
-- Description:
-- Returns a new (x, z) position obtained by shifting a reference point
-- (in_ref_x, in_ref_z) by a local offset (in_delta_x, in_delta_z)
-- rotated to align with in_heading. Used to compute spawning positions
-- relative to the aircraft (e.g. place the car 15 m behind the plane)
-- and to position the signboard above the car body taking into account
-- the car's current heading. Coordinates are in X-Plane local OpenGL
-- metres (x = east, z = south).
-- ====================================================
function coordinates_of_adjusted_ref(in_ref_x, in_ref_z, in_delta_x, in_delta_z, in_heading)
    local l_dist = math.sqrt((in_delta_x ^ 2) + (in_delta_z ^ 2))
    local l_heading = math.fmod((math.deg(math.atan2(in_delta_x, in_delta_z)) + 360), 360)
    local l_shifted_x = in_ref_x - math.sin(math.rad(in_heading - l_heading)) * l_dist * -1
    local l_shifted_z = in_ref_z - math.cos(math.rad(in_heading - l_heading)) * l_dist
    return l_shifted_x, l_shifted_z
end

-- ====================================================
-- Function: heading_n_dist
-- Description:
-- Core navigation utility used throughout the plugin. Given two points
-- in X-Plane local coordinates (x east, z south), returns the true
-- compass bearing (0-360 deg, 0 = north) from point 1 to point 2 and
-- the straight-line distance in metres. Called hundreds of times per
-- second for pathfinding, distance checks, turn geometry, centreline
-- projection, and the directional arrow in the UI.
-- ====================================================
function heading_n_dist(in_from_x1, in_from_z1, in_to_x2, in_to_z2)
    local l_heading = math.fmod((math.deg(math.atan2(in_to_x2 - in_from_x1, -(in_to_z2 - in_from_z1))) + 360), 360)
    local l_dist = math.sqrt(((in_to_x2 - in_from_x1) ^ 2) + ((in_to_z2 - in_from_z1) ^ 2))
    return l_heading, l_dist
end

-- ====================================================
-- Function: minus_delta_clockwise
-- Description:
-- Subtracts a heading delta in a direction-aware way. When in_direction
-- is +1 (clockwise turn), subtracts in_delta from in_heading (wrapping
-- below 0). When in_direction is -1 (counter-clockwise), adds in_delta
-- (wrapping via fmod). Used to back-calculate the heading from the centre
-- of rotation to the car entry point during arc geometry setup in
-- plot_position(), ensuring the correct tangent heading is always found
-- regardless of which side of the arc the car approaches from.
-- ====================================================
function minus_delta_clockwise(in_heading, in_delta, in_direction)
    local l_heading
    if in_direction == 1 then
        l_heading = in_heading - in_delta
        if l_heading < 0 then
            l_heading = l_heading + 360
        end
        return l_heading
    elseif in_direction == -1 then
        return math.fmod(in_heading + in_delta, 360)
    else
        return in_heading
    end
end

-- ====================================================
-- Function: add_delta_clockwise
-- Description:
-- Adds a heading delta in a direction-aware way. When in_direction is +1
-- (clockwise), adds in_delta and wraps with fmod 360. When in_direction
-- is -1 (counter-clockwise), subtracts in_delta and wraps below 0.
-- Companion to minus_delta_clockwise. Used throughout the turn-geometry
-- code and in chk_line_of_sight() to compute the left and right arc
-- limits of a vision cone.
-- ====================================================
function add_delta_clockwise(in_heading, in_delta, in_direction)
    local l_heading = 0
    if in_direction == 1 then
        return math.fmod(in_heading + in_delta, 360)
    elseif in_direction == -1 then
        l_heading = in_heading - in_delta
        if l_heading < 0 then
            l_heading = l_heading + 360
        end
        return l_heading
    else
        return in_heading
    end
end

-- ====================================================
-- Function: compute_angle_diff
-- Description:
-- Computes the shortest angular difference between two compass headings
-- and the direction of that difference. Returns (angle_deg, direction)
-- where angle_deg is always positive (0-180) and direction is +1 for a
-- clockwise turn or -1 for a counter-clockwise turn (0 if identical).
-- Used by determine_dir_of_turn() to decide which way the car turns at
-- each waypoint and by check_deadend_node() to evaluate gate entry angles.
-- ====================================================
function compute_angle_diff(in_from, in_to)
    if in_to == in_from then
        return 0, 0
    elseif in_to > in_from then
        if in_from + 180 > in_to then
            return (in_to - in_from), 1
        else
            return (in_from + (360 - in_to)), -1
        end
    elseif in_to < in_from then
        if in_to + 180 > in_from then
            return (in_from - in_to), -1
        else
            return (in_to + (360 - in_from)), 1
        end
    end
end

-- ====================================================
-- Function: compute_angle_diff_dir
-- Description:
-- Computes the angular distance from in_from to in_to measured strictly
-- in the given direction (in_dir: +1 = clockwise, -1 = counter-clockwise).
-- Unlike compute_angle_diff(), the result can exceed 180 deg when in_to
-- lies on the long arc side relative to in_dir. Used in plot_position()
-- to measure how many degrees of arc remain before the car reaches the
-- turn exit heading (head1_exit), driving the braking and steering logic
-- for the current rotation frame.
-- ====================================================
function compute_angle_diff_dir(in_from, in_to, in_dir)
    if in_to == in_from then
        return 0
    elseif in_dir > 0 then
        if in_to > in_from then
            return (in_to - in_from)
        else
            return (in_to + (360 - in_from))
        end
    elseif in_dir < 0 then
        if in_to > in_from then
            return (in_from + (360 - in_to))
        else
            return (in_from - in_to)
        end
    end
end

-- ====================================================
-- Function: local_to_latlon
-- Description:
-- Converts X-Plane local OpenGL coordinates (x, y, z in metres) to
-- geographic coordinates (latitude, longitude, altitude) by calling
-- XPLMLocalToWorld via the FFI interface. Used in process_possible_routes()
-- to log the GPS lat/lon of each drive waypoint so the developer can
-- compare computed positions against the in-sim map.
-- ====================================================
function local_to_latlon(l_x, l_y, l_z)
    x1_value[0] = l_x
    y1_value[0] = l_y
    z1_value[0] = l_z
    XPLM.XPLMLocalToWorld(x1_value[0], y1_value[0], z1_value[0], x1_value, y1_value, z1_value)
    return x1_value[0], y1_value[0], z1_value[0]
end

-- ====================================================
-- Function: latlon_to_local
-- Description:
-- Converts geographic coordinates (latitude, longitude, altitude) to
-- X-Plane local OpenGL metres by calling XPLMWorldToLocal via FFI.
-- Called by get_local_coordinates() when converting apt.dat lat/lon
-- positions (runways, gates, taxinodes) into the local coordinate space
-- used by all car physics and pathfinding calculations.
-- ====================================================
function latlon_to_local(in_lat, in_lon, in_alt)
    x1_value[0] = in_lat
    y1_value[0] = in_lon
    z1_value[0] = in_alt
    XPLM.XPLMWorldToLocal(x1_value[0], y1_value[0], z1_value[0], x1_value, y1_value, z1_value)
    return x1_value[0], y1_value[0], z1_value[0]
end

-- ====================================================
-- Function: get_local_coordinates
-- Description:
-- Converts a lat/lon/alt from apt.dat into X-Plane local coordinates
-- with terrain elevation snapping. First converts to local space via
-- latlon_to_local(), then uses XPLMProbeTerrainXYZ to snap the point
-- to the actual ground surface. The snapped altitude is then converted
-- back to lat/lon and re-projected to local space. This two-pass
-- approach ensures runway thresholds, gates, and taxinodes all sit
-- exactly on the mesh surface regardless of airport elevation.
-- ====================================================
function get_local_coordinates(in_lat, in_lon, in_alt)
    local l_x, l_y, l_z = 0, 0, 0
    if l_alt == 0 then
        l_x, l_y, l_z = latlon_to_local(in_lat, in_lon, in_alt)
        x1_value[0] = l_x
        y1_value[0] = l_y
        z1_value[0] = l_z
        XPLM.XPLMProbeTerrainXYZ(proberef, x1_value[0], y1_value[0], z1_value[0], probeinfo_addr)
        probeinfo_value = probeinfo_addr
        in_lat, in_lon, in_alt =
            local_to_latlon(probeinfo_value[0].locationX, probeinfo_value[0].locationY, probeinfo_value[0].locationZ)
    end

    l_x, l_y, l_z = latlon_to_local(in_lat, in_lon, in_alt)
    x1_value[0] = l_x
    y1_value[0] = l_y
    z1_value[0] = l_z
    XPLM.XPLMProbeTerrainXYZ(proberef, x1_value[0], y1_value[0], z1_value[0], probeinfo_addr)
    probeinfo_value = probeinfo_addr
    in_lat, in_lon, in_alt =
        local_to_latlon(probeinfo_value[0].locationX, probeinfo_value[0].locationY, probeinfo_value[0].locationZ)
    l_x, l_y, l_z = latlon_to_local(in_lat, in_lon, in_alt)
    return l_x, l_y, l_z, in_alt
end

-- ====================================================
-- Function: probe_y
-- Description:
-- Returns the terrain elevation (Y coordinate in local OpenGL metres)
-- at a given (x, z) position by calling XPLMProbeTerrainXYZ. Used every
-- frame in manage_car_motion() via plot_position() to keep the FM car
-- rolling exactly on the airport surface, following slopes, bridges, and
-- level changes rather than floating or clipping into the ground.
-- ====================================================
function probe_y(in_x, in_y, in_z)
    local l_lat, l_lon, l_alt = 0, 0, 0
    x1_value[0] = in_x
    y1_value[0] = in_y
    z1_value[0] = in_z
    XPLM.XPLMProbeTerrainXYZ(proberef, x1_value[0], y1_value[0], z1_value[0], probeinfo_addr)
    probeinfo_value = probeinfo_addr
    l_lat, l_lon, l_alt =
        local_to_latlon(probeinfo_value[0].locationX, probeinfo_value[0].locationY, probeinfo_value[0].locationZ)
    in_x, in_y, in_z = latlon_to_local(l_lat, l_lon, l_alt)
    return in_y
end

-- ====================================================
-- Function: draw_object
-- Description:
-- Positions the FM car 3-D model and its signboard in the X-Plane scene
-- for the current frame. Calls sync_anim_datarefs() to push the latest
-- steering and tire-rotation angles to the car's animation datarefs so
-- wheels turn and steer visually. Sets the car body position and heading
-- via XPLMInstanceSetPosition(), then places the signboard slightly
-- above and in front of the car, passing car_sign (0=none, 1=stop,
-- 2=turn-right, 3=turn-left) so the signboard .obj animates correctly.
-- ====================================================
function draw_object(in_x, in_y, in_z, in_heading)
    -- VER1.17 : populate dataref_float_value with the wheel animation values
    dataref_float_value[0] = steering
    dataref_float_value[1] = steering
    dataref_float_value[2] = tire_rotate
    dataref_float_value[3] = tire_rotate
    dataref_float_value[4] = tire_rotate
    dataref_float_value[5] = tire_rotate
    dataref_float_addr = dataref_float_value

    -- VER1.17 : Also synchronize the X-Plane datarefs for external visibility
    sync_anim_datarefs()

    objpos_value[0].x = in_x
    objpos_value[0].z = in_z
    objpos_value[0].y = in_y

    if car_accel == accel_max then
        objpos_value[0].pitch = 0.3
    elseif car_accel == deccel_max then
        objpos_value[0].pitch = -0.3
    else
        objpos_value[0].pitch = 0
    end

    objpos_value[0].heading = in_heading
    objpos_value[0].roll = 0
    objpos_value[0].structSize = ffi.sizeof(objpos_value[0])
    objpos_addr = objpos_value

    if obj_instance[0] ~= nil then
        XPLM.XPLMInstanceSetPosition(obj_instance[0], objpos_addr, dataref_float_addr)
    end

    dataref_float_value2[0] = car_sign
    dataref_float_value2[1] = 0
    dataref_float_addr = dataref_float_value2
    objpos_value[0].y = in_y + place_above_the_car
    objpos_value[0].heading = in_heading
    objpos_value[0].x, objpos_value[0].z = coordinates_of_adjusted_ref(in_x, in_z, 0, place_Z_of_car, in_heading)
    objpos_value[0].structSize = ffi.sizeof(objpos_value[0])
    objpos_addr = objpos_value

    if signboard_instance[0] ~= nil then
        XPLM.XPLMInstanceSetPosition(signboard_instance[0], objpos_addr, dataref_float_addr)
    end
end

-- ====================================================
-- Function: object_physics
-- Description:
-- Main per-frame physics driver, registered with do_every_frame().
-- Computes the delta-time since the last frame (clamped to [0, 1 s] to
-- ignore pauses or scene reloads). Handles path-pin and ramp-start marker
-- visibility changes. When the FM car is active, calls chk_line_of_sight()
-- to determine whether the aircraft can see the car and whether the car
-- can see the aircraft from behind, then calls move_car() with those
-- results to set acceleration, and draw_object() to refresh the 3-D
-- position. All car motion and rendering happen exclusively inside this
-- function and its callees.
-- ====================================================
function object_physics()
    -- VER1.4 : Corrected delta-time calculation
    local l_now = fm_run_time

    if elapsed_time == 0 then
        elapsed_time = l_now
    end

    local l_dt = l_now - elapsed_time

    if l_dt <= 0 or l_dt > 1 then
        elapsed_time = l_now
        return
    end

    elapsed_time = l_dt

    if path_instance[0] ~= nil and path_is_shown == false then
        draw_path()
    end

    if rampstart_chg == true then
        if show_rampstart then
            load_rampstart()
        else
            unload_rampstart()
        end
        if rampstart_instance[0] ~= nil then
            draw_rampstart()
        end
    end

    if obj_instance[0] ~= nil and #t_node > 0 and curr_node ~= #t_node then
        local l_dist = 0
        local l_car_in_sight, l_car_is_behind = false, false
        l_car_in_sight, _, l_dist = chk_line_of_sight(fm_plane_head, 80, 80, fm_plane_x, fm_plane_z, car_x, car_z)
        local l_car_to_plane_head, l_car_to_plane_dist = 0, 0
        if not l_car_in_sight and l_dist <= 200 then
            l_car_is_behind, l_car_to_plane_head, l_car_to_plane_dist =
                chk_line_of_sight(car_body_heading, 45, 45, car_x, car_z, fm_plane_x, fm_plane_z)
        end
        move_car(l_dist, l_car_in_sight, l_car_is_behind)
        draw_object(car_x, car_y, car_z, car_body_heading)
    end

    elapsed_time = fm_run_time
end

-- ====================================================
-- Function: load_probe
-- Description:
-- Creates the XPLMProbeRef terrain-probe object used by probe_y() and
-- get_local_coordinates(). Must be called once at plugin startup before
-- any coordinate conversion or car movement takes place. The probe is
-- a vertical ray-cast that returns the exact ground elevation at any
-- (x, z) position in the X-Plane scenery mesh.
-- ====================================================
function load_probe()
    probeinfo_value[0].structSize = ffi.sizeof(probeinfo_value[0])
    probeinfo_addr = probeinfo_value
    probetype[1] = 0
    proberef = XPLM.XPLMCreateProbe(probetype[1])
end

-- ====================================================
-- Function: load_object
-- Description:
-- Asynchronously loads the selected FM car .obj model and the signboard
-- .obj into the X-Plane scenery system and creates XPLMInstanceRef
-- handles for each. The car model is chosen based on car_type_fmcar
-- (Ferrari, Van, Truck, or random Auto). Each vehicle type has different
-- physical constants (weight, tire diameter, wheelbase, top speed, etc.)
-- that are set here immediately so car physics match the 3-D model.
-- The dataref arrays for tire steering and tire rotation are also bound
-- here so the model animates correctly once draw_object() starts calling
-- sync_anim_datarefs() every frame.
-- ====================================================
function load_object()
    ffi.copy(dataref_name, "sim/graphics/animation/ground_traffic/tire_steer_deg[0]")
    dataref_array[0] = dataref_name
    ffi.copy(dataref_name, "sim/graphics/animation/ground_traffic/tire_steer_deg[1]")
    dataref_array[1] = dataref_name
    ffi.copy(dataref_name, "sim/graphics/animation/ground_traffic/tire_rotation_angle_deg[0]")
    dataref_array[2] = dataref_name
    ffi.copy(dataref_name, "sim/graphics/animation/ground_traffic/tire_rotation_angle_deg[1]")
    dataref_array[3] = dataref_name
    ffi.copy(dataref_name, "sim/graphics/animation/ground_traffic/tire_rotation_angle_deg[2]")
    dataref_array[4] = dataref_name
    ffi.copy(dataref_name, "sim/graphics/animation/ground_traffic/tire_rotation_angle_deg[3]")
    dataref_array[5] = dataref_name
    dataref_array[6] = NULL
    datarefs_addr = dataref_array

    local l_auto_sel = 0
    if car_type_fmcar == "Auto" then
        math.randomseed(os.time())
        l_auto_sel = math.random(1, 3)
    end

    if car_type_fmcar == "Ferrari" or (car_type_fmcar == "Auto" and l_auto_sel == 1) then
        XPLM.XPLMLoadObjectAsync(
            syspath .. "Resources/default scenery/airport scenery/Dynamic_Vehicles/crew_car_ferrari.obj",
            function(inObject, inRefcon)
                obj_instance[0] = XPLM.XPLMCreateInstance(inObject, datarefs_addr)
                objref = inObject
            end,
            inRefcon
        )
        car_weight = 1445
        tire_diameter = 0.79
        min_turn_radius = 10.8
        width_btw_midtire = 1.86
        car_front_to_back_wheel = 3.2
        car_rear_wheel_to_ref = 1.60
        time_to_100 = 3
        speed_max = 88.88
        place_above_the_car = 1.43
        place_Z_of_car = 0
    elseif car_type_fmcar == "Van" or (car_type_fmcar == "Auto" and l_auto_sel == 2) then
        XPLM.XPLMLoadObjectAsync(
            SCRIPT_DIRECTORY .. "follow_me/objects/fm_van.obj",
            function(inObject, inRefcon)
                obj_instance[0] = XPLM.XPLMCreateInstance(inObject, datarefs_addr)
                objref = inObject
            end,
            inRefcon
        )
        car_weight = 1700
        tire_diameter = 0.65
        min_turn_radius = 10.8
        width_btw_midtire = 1.39
        car_front_to_back_wheel = 2.64
        car_rear_wheel_to_ref = 1.32
        time_to_100 = 10
        speed_max = 70
        place_above_the_car = 1.95
        place_Z_of_car = -1.625
    elseif car_type_fmcar == "Truck" or (car_type_fmcar == "Auto" and l_auto_sel == 3) then
        XPLM.XPLMLoadObjectAsync(
            SCRIPT_DIRECTORY .. "follow_me/objects/fm_truck.obj",
            function(inObject, inRefcon)
                obj_instance[0] = XPLM.XPLMCreateInstance(inObject, datarefs_addr)
                objref = inObject
            end,
            inRefcon
        )
        car_weight = 2000
        tire_diameter = 0.730
        min_turn_radius = 10.8
        width_btw_midtire = 1.578
        car_front_to_back_wheel = 3.137
        car_rear_wheel_to_ref = 1.5685
        time_to_100 = 7.5
        speed_max = 54
        place_above_the_car = 1.86
        place_Z_of_car = -0.4347
    end

    ffi.copy(dataref_name, "fm/anim/sign")
    dataref_array2[0] = dataref_name
    dataref_array2[1] = NULL
    datarefs_addr = dataref_array2

    XPLM.XPLMLoadObjectAsync(
        SCRIPT_DIRECTORY .. "follow_me/objects/signboard.obj",
        function(inObject, inRefcon)
            signboard_instance[0] = XPLM.XPLMCreateInstance(inObject, datarefs_addr)
            signboardref = inObject
        end,
        inRefcon
    )
end

-- ====================================================
-- Function: load_path
-- Description:
-- Loads the yellow pushpin .obj marker and creates one XPLMInstanceRef
-- per waypoint in t_node when the "Show Path" option is active. The
-- instances are positioned at each node's (x, y, z) by draw_path() so
-- the pilot can see the planned taxiway route as a row of pins on the
-- airport surface. Loading is asynchronous; draw_path() is called once
-- the callback fires and path_instance[0] becomes non-nil.
-- ====================================================
function load_path()
    if FM_car_active and show_path and #t_node > 0 then
        XPLM.XPLMLoadObjectAsync(
            SCRIPT_DIRECTORY .. "follow_me/objects/pushpin_yellow.obj",
            function(inObject, inRefcon)
                for i = 0, #t_node - 1 do
                    path_instance[i] = XPLM.XPLMCreateInstance(inObject, NULL)
                end
                pathref = inObject
            end,
            inRefcon
        )
    end
end

-- ====================================================
-- Function: load_rampstart
-- Description:
-- Loads the diamond_marker .obj and creates an XPLMInstanceRef for the
-- ramp-start indicator when "Show Ramp Start" is enabled. The marker is
-- placed at the selected departure gate or arrival gate by draw_rampstart()
-- so the pilot can visually identify the correct stand on the apron.
-- Loading is skipped if an instance already exists to avoid duplicates
-- across airport changes.
-- ====================================================
function load_rampstart()
    if rampstart_instance[0] == nil then
        XPLM.XPLMLoadObjectAsync(
            SCRIPT_DIRECTORY .. "follow_me/objects/diamond_marker.obj",
            function(inObject, inRefcon)
                rampstart_instance[0] = XPLM.XPLMCreateInstance(inObject, NULL)
                rampstartref = inObject
            end,
            inRefcon
        )
    end
end

-- ====================================================
-- Function: draw_path
-- Description:
-- Places each yellow pushpin instance at the (x, y, z) of the
-- corresponding waypoint in t_node so the full planned route is visible
-- on the airport surface. Called once after load_path() completes and
-- also by object_physics() if the instances exist but have not yet been
-- positioned (path_is_shown == false). Sets path_is_shown = true after
-- positioning so the pins are not repositioned every frame.
-- ====================================================
function draw_path()
    local l_index = 0
    float_value[0] = 0
    float_addr = float_value
    for l_index = 0, #t_node - 1 do
        objpos_value[0].x = t_node[l_index + 1].x
        objpos_value[0].y = fm_plane_y
        objpos_value[0].z = t_node[l_index + 1].z
        objpos_addr = objpos_value
        XPLM.XPLMInstanceSetPosition(path_instance[l_index], objpos_addr, float_addr)
    end
    path_is_shown = true
end

-- ====================================================
-- Function: draw_rampstart
-- Description:
-- Updates the position of the ramp-start diamond marker to the currently
-- selected gate (depart_gate in departure mode, arrival_gate in arrival
-- mode). If no gate is selected, the marker is moved to (0, plane_y, 0)
-- to hide it out of view. Called by object_physics() whenever
-- rampstart_chg is true (gate selection changed, airport reload, or
-- "Show Ramp Start" toggled). Clears rampstart_chg after updating.
-- ====================================================
function draw_rampstart()
    local l_index = 0
    float_value[0] = 0
    float_addr = float_value

    if (depart_arrive == 1 and depart_gate > 0) then
        l_index = depart_gate
    elseif (depart_arrive == 2 and arrival_gate > 0) then
        l_index = arrival_gate
    else
        l_index = 0
    end

    if l_index > 0 then
        objpos_value[0].x = t_gate[l_index].x
        objpos_value[0].y = t_gate[l_index].y
        objpos_value[0].z = t_gate[l_index].z
        objpos_value[0].heading = t_gate[l_index].Heading
    else
        objpos_value[0].x = 0.0
        objpos_value[0].y = fm_plane_y
        objpos_value[0].z = 0.0
    end

    objpos_addr = objpos_value
    XPLM.XPLMInstanceSetPosition(rampstart_instance[0], objpos_addr, float_addr)
    rampstart_chg = false
end

-- ====================================================
-- Function: unload_probe
-- Description:
-- Destroys the XPLM terrain probe created by load_probe() and sets
-- proberef to nil. Called from exit_plugin() during X-Plane shutdown.
-- Must be called after full_reset() because probe_y() (called by the
-- car physics) uses proberef every frame while the car is active.
-- ====================================================
function unload_probe()
    if proberef ~= nil then
        XPLM.XPLMDestroyProbe(proberef)
    end
    proberef = nil
end

-- ====================================================
-- Function: unload_object
-- Description:
-- Destroys the XPLMInstanceRef handles for the FM car body and the
-- signboard, then unloads their .obj files from the X-Plane scenery
-- system. Called by full_reset() whenever the pilot cancels the service,
-- takes off, changes airport, or quits. Nil-guards prevent double-free
-- crashes if the async load never completed before the unload was
-- requested.
-- ====================================================
function unload_object()
    if obj_instance[0] ~= nil then
        XPLM.XPLMDestroyInstance(obj_instance[0])
    end
    if objref ~= nil then
        XPLM.XPLMUnloadObject(objref)
    end
    obj_instance[0] = nil
    objref = nil
    if signboard_instance[0] ~= nil then
        XPLM.XPLMDestroyInstance(signboard_instance[0])
    end
    if signboardref ~= nil then
        XPLM.XPLMUnloadObject(signboardref)
    end
    signboard_instance[0] = nil
    signboardref = nil
end

-- ====================================================
-- Function: unload_path
-- Description:
-- Destroys all yellow pushpin XPLMInstanceRef handles and unloads the
-- pushpin .obj from memory. Iterates over the full t_node array so every
-- pin is removed even if t_node has already been partially consumed by
-- the car. Resets path_is_shown to false so load_path() / draw_path()
-- can be called cleanly on the next route request.
-- ====================================================
function unload_path()
    local l_index = 0
    if path_instance[0] ~= nil then
        for l_index = 0, #t_node - 1 do
            XPLM.XPLMDestroyInstance(path_instance[l_index])
        end
    end
    if pathref ~= nil then
        XPLM.XPLMUnloadObject(pathref)
    end
    path_instance[0] = nil
    pathref = nil
    path_is_shown = false
end

-- ====================================================
-- Function: unload_rampstart
-- Description:
-- Removes the ramp-start diamond marker from the scene. First moves the
-- marker underground (y = -9999) so X-Plane clears its visual before the
-- instance is destroyed - without this step a ghost diamond remains
-- visible for one extra frame. Then destroys the XPLMInstanceRef and
-- unloads the .obj. Called by full_reset() and by handle_plugin_window()
-- on manual cancel or after landing.
-- ====================================================
function unload_rampstart()
    if rampstart_instance[0] ~= nil then
        -- Move the marker underground before destroying so X-Plane removes it visually
        objpos_value[0].x = 0.0
        objpos_value[0].y = -9999.0
        objpos_value[0].z = 0.0
        objpos_addr = objpos_value
        float_value[0] = 0
        float_addr = float_value
        XPLM.XPLMInstanceSetPosition(rampstart_instance[0], objpos_addr, float_addr)
        XPLM.XPLMDestroyInstance(rampstart_instance[0])
    end
    if rampstartref ~= nil then
        XPLM.XPLMUnloadObject(rampstartref)
    end
    rampstart_instance[0] = nil
    rampstartref = nil
    rampstart_chg = false
end

-- VER1.17: XPLMRegisterDataAccessor removed - causes crashes when changing airports.
-- FlyWithLua restarts the Lua engine with each change -> old handles are lost
-- but X-Plane keeps the dangling pointers -> access violation.
-- Solution: XPLMFindDataRef for native datarefs (tire_steer/tire_rotate),
-- and XPLMRegisterDataAccessor protected by LUA_RUN==1 for fm/anim/sign.
-- If LUA_RUN>1, fm/anim/sign already exists -> FindDataRef to retrieve it.

-- ====================================================
-- Function: register_dataref
-- Description:
-- Sets up the XPLM datarefs needed by the FM car 3-D animation.
-- Uses XPLMFindDataRef for the two native X-Plane tire arrays
-- (tire_steer_deg and tire_rotation_angle_deg) which already exist in
-- the sim. Registers the custom read-only float dataref "fm/anim/sign"
-- that exposes car_sign (0=none 1=stop 2=right 3=left) to the
-- signboard .obj so its animation controller can switch signs. The
-- registration is protected by LUA_RUN so it only runs once even when
-- FlyWithLua reloads the Lua engine on airport/aircraft changes.
-- ====================================================
function register_dataref()

    -- Datarefs natifs X-Plane - FindDataRef, pas RegisterDataAccessor
    dr_tire_steer = XPLM.XPLMFindDataRef("sim/graphics/animation/ground_traffic/tire_steer_deg")
    dr_tire_rotate = XPLM.XPLMFindDataRef("sim/graphics/animation/ground_traffic/tire_rotation_angle_deg")

	-- fm/anim/sign: custom dataref
	-- LUA_RUN == 1 -> first boot -> it is registered
	-- LUA_RUN > 1 -> reload after airport change -> it already exists -> FindDataRef
    dr_sign =
        XPLM.XPLMRegisterDataAccessor(
        "fm/anim/sign",
        2,
        0,
        NULL,
        NULL,
        function(inRefcon)
            return car_sign
        end,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL
    )
end

-- VER1.17: called every frame in draw_object() to push steering/tire_rotate/car_sign

-- ====================================================
-- Function: sync_anim_datarefs
-- Description:
-- Pushes the current steering angle and tire rotation values to the
-- native X-Plane tire animation datarefs every frame so the car model's
-- front wheels steer and all four wheels roll visually. Also pushes
-- car_sign to the custom fm/anim/sign dataref so the signboard .obj
-- switches between no-sign, stop, turn-right, and turn-left frames.
-- Called from draw_object() once per simulation frame while the FM car
-- is active.
-- ====================================================
function sync_anim_datarefs()
    if dr_tire_steer ~= nil then
        ffi_steer_buf[0] = steering
        ffi_steer_buf[1] = steering
        XPLM.XPLMSetDatavf(dr_tire_steer, ffi_steer_buf, 0, 2)
    end
    if dr_tire_rotate ~= nil then
        ffi_rotate_buf[0] = tire_rotate
        ffi_rotate_buf[1] = tire_rotate
        ffi_rotate_buf[2] = tire_rotate
        ffi_rotate_buf[3] = tire_rotate
        XPLM.XPLMSetDatavf(dr_tire_rotate, ffi_rotate_buf, 0, 4)
    end
    if dr_sign ~= nil then
        XPLM.XPLMSetDataf(dr_sign, car_sign)
    end
end

-- ====================================================
-- Function: get_airport_elements
-- Description:
-- Detects the current X-Plane airport via XPLMFindNavAid / XPLMGetNavAidInfo
-- and, if the ICAO has changed since the last call, triggers a full parse
-- of the apt.dat file for that airport via read_apt_file(). Stores the
-- new ICAO and airport name in curr_ICAO / curr_ICAO_name. If the airport
-- has no taxiway network (taxiway_network != ""), speaks an audio warning
-- to the pilot. Called once when the main window opens for the first time
-- (window_first_access) and again on auto-trigger events (taxi/beacon
-- lights) so the network is always fresh for the current position.
-- ====================================================
function get_airport_elements()
    local l_airport_index = XPLMFindNavAid(nil, nil, LATITUDE, LONGITUDE, nil, xplm_Nav_Airport)
    local l_new_ICAO, l_new_ICAO_name = "", ""

    _, _, _, _, _, _, l_new_ICAO, l_new_ICAO_name = XPLMGetNavAidInfo(l_airport_index)

    if curr_ICAO ~= l_new_ICAO then
        world_alt = 0
        taxiway_network = read_apt_file(l_new_ICAO)
        curr_ICAO = l_new_ICAO
        curr_ICAO_name = l_new_ICAO_name

        if taxiway_network ~= "" then
            XPLMSpeakString("FM Service is not available at this airport")
            return
        end

        -- VER1.5 : Check if SimBrief departure airport matches the new sim airport
        if sb_fetch_status == "OK" then
            sb_airport_mismatch = (sb_origin_icao ~= curr_ICAO)
        end
    end

    if #t_deleted_runway > 0 then
        update_msg("-18")
    end

    depart_gate = check_gate()
    if depart_arrive == 0 then
        if flightstart == 9999 then
            if depart_gate == 0 then
                depart_arrive = 2
            else
                flightstart = 0
                depart_arrive = 1
            end
        else
            depart_arrive = 1
        end
    end

    -- VER1.5 : Apply SimBrief runway if SimBrief mode is active
    if get_from_SimBrief then
        apply_simbrief_runway()
    end
end

-- ====================================================
-- Function: read_apt_file
-- Description:
-- Locates and parses the apt.dat file(s) for the given airport ICAO.
-- Searches X-Plane's scenery_packs.ini to find custom and default scenery
-- layers in priority order, then falls back to the Global Airports
-- apt.dat. For each apt.dat file, scans for the airport header line
-- matching the ICAO, then streams all rows until the next airport header
-- or end-of-file. Dispatches each recognised row type to the appropriate
-- decipher_* function: row 100 (runway), 1300 (gate), 1301 (gate ops),
-- 1201 (taxinode), 1202 (segment), 1204 (hotzone), 1206 (vehicle edge).
-- After parsing, calls determine_runway_node() to link runways to nodes
-- and apply_1206_filter() to clean vehicle-only nodes. Returns "" on
-- success or a negative error code if no taxiway network is found.
-- ====================================================
function read_apt_file(in_ICAO)
    local l_filename1, l_filename2 = "", ""
    local l_file1, l_file2
    local l_line1, l_line2, l_rest = "", "", ""
    local l_str1 = ""
    local l_airport_found = false
    local l_processed_1204 = false
    local l_processed_1300 = false
    local l_new_lines = ""
    local l_start = 0
    local l_not_first_line = false
    local l_terminate_loop = false

    l_filename1 = syspath .. "Custom Scenery/scenery_packs.ini"
    l_file1 = io.open(l_filename1, "r")
    if l_file1 == nil then
        return "-11"
    end

    repeat
        l_line1 = l_file1:read("*l")
        if l_line1 then
            l_str1 = string.match(l_line1, "SCENERY_PACK (.*)")
            if l_str1 then
                if l_str1 == "*GLOBAL_AIRPORTS*" then
                    l_str1 = "Global Scenery/Global Airports/"
                end
                if string.find(l_str1, "\\") then
                    l_filename2 = l_str1 .. "Earth nav data/apt.dat"
                else
                    l_filename2 = syspath .. l_str1 .. "Earth nav data/apt.dat"
                end

                l_file2 = io.open(l_filename2, "r")
                if l_file2 then
                    while true do
                        l_new_lines, l_rest = l_file2:read(BUFSIZE, "*l")
                        if not l_new_lines then
                            break
                        end
                        if l_rest then
                            l_new_lines = l_new_lines .. l_rest .. "\n"
                        end

                        if l_airport_found == false then
                            l_start = 0
                            while true do
                                if l_start > 0 then
                                    l_start = l_start + 200
                                else
                                    l_start = 1
                                end
                                l_start = string.find(l_new_lines, in_ICAO, l_start)
                                if l_start == nil then
                                    break
                                end
                                if l_start - 15 < 1 then
                                    l_start = 1
                                else
                                    l_start = l_start - 15
                                end
                                l_new_lines = string.sub(l_new_lines, l_start)
                                for l_line2 in l_new_lines:gmatch("[^\r\n]+") do
                                    if string.find(l_line2, in_ICAO) then
                                        if
                                            string.match(l_line2, "^%d+ %s*[^%s]+%s*[^%s]+%s*[^%s]+%s*([^%s]+)%s*.*") ==
                                                in_ICAO
                                         then
                                            l_airport_found = true
                                            l_not_first_line = false
                                            initialise_airport()
                                        end
                                        break
                                    end
                                end
                                if l_airport_found == true then
                                    break
                                end
                            end
                        end

                        if l_airport_found then
                            for l_line2 in l_new_lines:gmatch("[^\r\n]+") do
                                if l_not_first_line == false then
                                    l_not_first_line = true
                                else
                                    if string.find(l_line2, "^1%s") or l_line2 == "99" then
                                        table.sort(
                                            t_gate,
                                            function(a, b)
                                                return a.ID < b.ID
                                            end
                                        )
                                        -- VER1.21: Disambiguate duplicate gate names
                                        -- Phase 1: append terminal letter to duplicates
                                        local l_id_count1 = {}
                                        for _, g in ipairs(t_gate) do
                                            l_id_count1[g.ID] = (l_id_count1[g.ID] or 0) + 1
                                        end
                                        for _, g in ipairs(t_gate) do
                                            if l_id_count1[g.ID] > 1 and g.Terminal and g.Terminal ~= "" then
                                                g.ID = g.ID .. " " .. g.Terminal
                                            end
                                        end
                                        -- Phase 2: if still duplicates, append numeric counter
                                        local l_id_count2 = {}
                                        for _, g in ipairs(t_gate) do
                                            l_id_count2[g.ID] = (l_id_count2[g.ID] or 0) + 1
                                        end
                                        local l_id_index = {}
                                        for _, g in ipairs(t_gate) do
                                            if l_id_count2[g.ID] > 1 then
                                                l_id_index[g.ID] = (l_id_index[g.ID] or 0) + 1
                                                g.ID = g.ID .. " " .. l_id_index[g.ID]
                                            end
                                        end
                                        determine_runway_node()
                                        -- VER1.23 : apply ground-vehicle edge filter after all segments loaded
                                        apply_1206_filter()
                                        l_terminate_loop = true
                                        break
                                    end
                                end

                                if string.match(l_line2, "^100%s") then
                                    decipher_runway(l_line2, t_runway)
                                end
                                if string.match(l_line2, "^1301%s") and l_processed_1300 == true then
                                    decipher_ramp_operation(l_line2)
                                end
                                if string.match(l_line2, "^1300%s") then
                                    l_processed_1300 = decipher_ramp(l_line2, t_gate)
                                end
                                if string.match(l_line2, "^1201%s") then
                                    decipher_taxinode(l_line2)
                                end
                                if string.match(l_line2, "^1202%s") then
                                    l_processed_1204 = false
                                    decipher_taxisegment(l_line2)
                                end
                                if string.match(l_line2, "^1206%s") then
                                    decipher_vehicle_edge(l_line2)
                                end
                                if string.match(l_line2, "^1204%s") then
                                    if l_processed_1204 == false then
                                        decipher_taxisegment_hotzone(l_line2)
                                        l_processed_1204 = true
                                    end
                                end
                            end
                            if l_terminate_loop == true then
                                break
                            end
                        end
                    end
                    l_file2:close()
                    if l_terminate_loop == true then
                        break
                    end
                end
            end
        end
    until not l_line1
    l_file1:close()

    if l_airport_found == false and l_terminate_loop == false then
        l_filename2 = syspath .. "Global Scenery/Global Airports/Earth nav data/apt.dat"
        l_file2 = io.open(l_filename2, "r")
        if l_file2 then
            for l_line2 in l_file2:lines() do
                if string.find(l_line2, in_ICAO) then
                    if string.match(l_line2, "^%d+ %s*[^%s]+%s*[^%s]+%s*[^%s]+%s*([^%s]+)%s*.*") == in_ICAO then
                        l_airport_found = true
                        initialise_airport()
                        break
                    end
                end
            end
            l_file2:close()
        end
    end

    if #t_taxinode == 0 then
        return "-15"
    else
        rampstart_chg = true
        return ""
    end
end

-- ====================================================
-- Function: decipher_runway
-- Description:
-- Parses one apt.dat row-100 (paved runway) line and appends two entries
-- to t_runway[] - one for each threshold of the same physical runway.
-- Extracts the runway IDs, threshold lat/lon, converts them to local
-- coordinates via get_local_coordinates(), and cross-references them with
-- a Pair index so process_possible_routes() can always find the exact
-- opposite threshold for centreline projection (VER1.24). Node is
-- initialised to -1 (unassigned) and filled in later by match_runway().
-- ====================================================
function decipher_runway(in_str)
    local l_str1, l_str2, l_str3, l_str4 = "", "", "", ""
    i = #t_runway + 1
    t_runway[i] = {}
    t_runway[i + 1] = {}
    t_runway[i].ID, l_str1, l_str2, t_runway[i + 1].ID, l_str3, l_str4 =
        string.match(
        in_str,
        "100 %s*[^%s]+%s*[^%s]+%s*[^%s]+%s*[^%s]+%s*[^%s]+%s*[^%s]+%s*[^%s]+%s*" ..
            "([^%s]+)%s*([^%s]+)%s*([^%s]+)%s*" ..
                "[^%s]+%s*[^%s]+%s*[^%s]+%s*[^%s]+%s*[^%s]+%s*[^%s]+%s*" ..
                    "([^%s]+)%s*([^%s]+)%s*([^%s]+)%s*" .. "(.*)"
    )
    t_runway[i].Lat = tonumber(l_str1)
    t_runway[i].Lon = tonumber(l_str2)
	-- VER1.7: -1 = not assigned (0 is a valid node_id)
    t_runway[i].Node = -1
    t_runway[i + 1].Lat = tonumber(l_str3)
    t_runway[i + 1].Lon = tonumber(l_str4)
	-- VER1.7: -1 = not assigned (0 is a valid node_id)
    t_runway[i + 1].Node = -1
    -- VER1.24 : store cross-references so each threshold knows its opposite (same apt.dat line 100)
    t_runway[i].Pair = i + 1
    t_runway[i + 1].Pair = i
    t_runway[i].x, _, t_runway[i].z, world_alt = get_local_coordinates(t_runway[i].Lat, t_runway[i].Lon, world_alt)
    t_runway[i + 1].x, _, t_runway[i + 1].z, world_alt =
        get_local_coordinates(t_runway[i + 1].Lat, t_runway[i + 1].Lon, world_alt)
end

-- ====================================================
-- Function: decipher_ramp
-- Description:
-- Parses one apt.dat row-1300 (ramp start) line and appends an entry to
-- t_gate[]. Extracts lat/lon, heading, ramp type, aircraft size classes,
-- and gate name. Normalises the raw size-class string (heavy/jets/
-- turboprops/props/all/helos) into a space-separated list of FollowMe
-- aircraft-type numbers (0-8) stored in t_gate[i].Types. This list is
-- later compared against the pilot's selected Aircraft_Type to filter
-- suitable gates in the UI and in auto_assign_gate().
-- ====================================================
function decipher_ramp(in_str)
    local l_str1, l_str2, l_str3, l_str4, l_str5, l_str6 = "", "", "", "", "", ""
    l_str1, l_str2, l_str3, l_str4, l_str5, l_str6 =
        string.match(in_str, "1300 %s*([^%s]+)%s*([^%s]+)%s*([^%s]+)%s*([^%s]+)%s*([^%s]+)%s*(.*)")
    i = #t_gate + 1
    t_gate[i] = {}
    t_gate[i].Lat = tonumber(l_str1)
    t_gate[i].Lon = tonumber(l_str2)
    t_gate[i].Heading = tonumber(l_str3)
    t_gate[i].Types = l_str5
    t_gate[i].ID = l_str6
    t_gate[i].Ramptype = l_str4
    t_gate[i].x, t_gate[i].y, t_gate[i].z, world_alt = get_local_coordinates(t_gate[i].Lat, t_gate[i].Lon, world_alt)

    if l_str5 == "helos" then
        t_gate[i].Types = ""
    elseif string.find(t_gate[i].Types, "all") then
        t_gate[i].Types = "1 2 3 4 5 6 7 8"
    else
        local l_heavy, l_jet, l_turbo, l_prop = "", "", "", ""
        if string.find(t_gate[i].Types, "heavy") then
            l_heavy = "1 2 "
        end
        if string.find(t_gate[i].Types, "jets") then
            if l_heavy ~= "" then
                l_jet = "3 5 "
            else
                l_jet = "3 5 7 "
            end
        end
        if string.find(t_gate[i].Types, "turboprops") then
            if l_jet == "3 5 " then
                l_jet = "3 5 7 "
            end
            l_turbo = "4 6 "
        end
        if string.find(t_gate[i].Types, "|props") or string.find(t_gate[i].Types, "^%s*props") then
            l_prop = "8"
        end
        t_gate[i].Types = l_heavy .. l_jet .. l_turbo .. l_prop
    end
    return true
end

-- ====================================================
-- Function: decipher_ramp_operation
-- Description:
-- Parses the optional apt.dat row-1301 (ramp operation type) line that
-- follows a row-1300. Stores the terminal letter in t_gate[i].Terminal
-- for use in gate name disambiguation (VER1.21). Sets t_gate[i].Cargo
-- or t_gate[i].Military flags, and removes the super-heavy type "1" from
-- the allowed aircraft list for "E" (east / international) terminal gates
-- that restrict wide-body operations.
-- ====================================================
function decipher_ramp_operation(in_str)
    local l_str1, l_str2 = "", ""
    local l_types = ""
    i = #t_gate
    l_types = t_gate[i].Types
    if l_types == "" then
        return
    end
    l_str1, l_str2 = string.match(in_str, "1301 %s*(%a)%s*([^%s]+)%s*")
	-- VER1.21: store terminal letter for disambiguation
    t_gate[i].Terminal = l_str1 or ""
    if l_str1 == "E" and string.find(l_types, "1") then
        t_gate[i].Types = string.sub(l_types, 3)
    end
    if l_str2 == "cargo" then
        t_gate[i].Cargo = "1"
    elseif l_str2 == "military" then
        t_gate[i].Military = "1"
        t_gate[i].Types = "0 " .. t_gate[i].Types
    end
end

-- ====================================================
-- Function: decipher_taxinode
-- Description:
-- Parses one apt.dat row-1201 (taxiway node) line and appends an entry
-- to t_taxinode[]. Converts the node lat/lon to local coordinates.
-- Initialises A* pathfinding fields (f_value, g_value, h_value, parent,
-- cost) to nil and routing fields (Type, Runway, Segment) to empty so
-- decipher_taxisegment() can populate them as the segments are read.
-- The apt.dat node index is (table_index - 1); all node lookups use
-- node_id + 1 to translate between the two.
-- ====================================================
function decipher_taxinode(in_str)
    local l_str1, l_str2 = "", ""
    i = #t_taxinode + 1
    t_taxinode[i] = {}
    l_str1, l_str2 = string.match(in_str, "1201 %s*([^%s]+)%s*([^%s]+)%s*")
    t_taxinode[i].Lat = tonumber(l_str1)
    t_taxinode[i].Lon = tonumber(l_str2)
    t_taxinode[i].Type = ""
    t_taxinode[i].Runway = ""
    t_taxinode[i].Segment = ""
    t_taxinode[i].f_value = nil
    t_taxinode[i].g_value = nil
    t_taxinode[i].h_value = nil
    t_taxinode[i].parent = nil
    t_taxinode[i].cost = nil
    t_taxinode[i].heading = nil
    t_taxinode[i].x, t_taxinode[i].y, t_taxinode[i].z, world_alt =
        get_local_coordinates(t_taxinode[i].Lat, t_taxinode[i].Lon, world_alt)
end

-- ====================================================
-- Function: decipher_taxisegment
-- Description:
-- Parses one apt.dat row-1202 (taxiway segment) line and appends an
-- entry to t_segment[]. Records the two endpoint node IDs, direction
-- flag, taxiway size code (A-E), segment type (runway or taxiway), and
-- segment name. Marks both endpoint nodes as "runway" typed when the
-- segment is a runway surface. Appends the new segment index to the
-- Segment field of both endpoint nodes so the A* search can quickly
-- enumerate neighbours. Computes the segment heading and length via
-- heading_n_dist() for use during pathfinding and geometry.
-- ====================================================
function decipher_taxisegment(in_str)
    local l_str1, l_str2, l_str3, l_str4, l_str5 = "", "", "", "", ""
    local l_idx = 0
    i = #t_segment + 1
    t_segment[i] = {}
    l_str1, l_str2, l_str3, l_str4, l_str5 =
        string.match(in_str, "1202 %s*([^%s]+)%s*([^%s]+)%s*([^%s]+)%s*([^%s]+)%s*([^%s]+)%s*")
    t_segment[i].Node1 = tonumber(l_str1)
    t_segment[i].Node2 = tonumber(l_str2)
    t_segment[i].Dir = l_str3

    if string.find(l_str4, "runway") then
        t_segment[i].Type = "runway"
        t_taxinode[t_segment[i].Node1 + 1].Type = "runway"
        t_taxinode[t_segment[i].Node2 + 1].Type = "runway"
        l_idx = #t_runway_node + 1
        t_runway_node[l_idx] = t_segment[i].Node1
        t_runway_node[l_idx + 1] = t_segment[i].Node2
    else
        if string.sub(l_str4, string.len(l_str4) - 1, -2) == "_" then
            t_segment[i].Size = string.sub(l_str4, string.len(l_str4), -1)
        else
            t_segment[i].Size = ""
        end
        t_segment[i].Type = "taxiway"
    end

    t_segment[i].ID = l_str5
    t_segment[i].Hotzone = ""
    t_segment[i].Heading, t_segment[i].Dist =
        heading_n_dist(
        t_taxinode[t_segment[i].Node1 + 1].x,
        t_taxinode[t_segment[i].Node1 + 1].z,
        t_taxinode[t_segment[i].Node2 + 1].x,
        t_taxinode[t_segment[i].Node2 + 1].z
    )
    if t_taxinode[t_segment[i].Node1 + 1].Segment == "" then
        t_taxinode[t_segment[i].Node1 + 1].Segment = tostring(i)
    else
        t_taxinode[t_segment[i].Node1 + 1].Segment = t_taxinode[t_segment[i].Node1 + 1].Segment .. "," .. tostring(i)
    end
    if t_taxinode[t_segment[i].Node2 + 1].Segment == "" then
        t_taxinode[t_segment[i].Node2 + 1].Segment = tostring(i)
    else
        t_taxinode[t_segment[i].Node2 + 1].Segment = t_taxinode[t_segment[i].Node2 + 1].Segment .. "," .. tostring(i)
    end
end

-- ====================================================
-- Function: decipher_taxisegment_hotzone
-- Description:
-- Parses the optional apt.dat row-1204 (segment hotzone annotation) that
-- immediately follows a row-1202. Sets the Hotzone field on the last
-- parsed segment. For taxiway segments that cross a runway hold-short
-- line, marks both endpoint nodes as "hotzone" type (unless already
-- typed "runway"). Hotzone nodes are used by process_possible_routes()
-- to detect the runway boundary and insert the GPS threshold logic.
-- ====================================================
function decipher_taxisegment_hotzone(in_str)
    i = #t_segment
    t_segment[i].Hotzone = string.match(in_str, "1204 %s*[^%s]+%s*([^%s]+)%s*")
    if t_segment[i].Type == "taxiway" then
        if t_taxinode[t_segment[i].Node1 + 1].Type ~= "runway" then
            t_taxinode[t_segment[i].Node1 + 1].Type = "hotzone"
        end
        if t_taxinode[t_segment[i].Node2 + 1].Type ~= "runway" then
            t_taxinode[t_segment[i].Node2 + 1].Type = "hotzone"
        end
    end
end

-- VER1.23 : collect a 1206 ground-vehicle routing edge
-- Format: 1206 <node1> <node2> <direction> [name]

-- ====================================================
-- Function: decipher_vehicle_edge
-- Description:
-- Parses one apt.dat row-1206 (ground vehicle routing edge) line and
-- appends the node pair to t_filter_1206[]. Row-1206 edges are reserved
-- for service vehicles only - aircraft should never follow them. After
-- all segments are loaded, apply_1206_filter() uses this list to prune
-- nodes that appear exclusively in 1206 edges and never in any 1202
-- segment, keeping the taxi network clean for A* pathfinding.
-- ====================================================
function decipher_vehicle_edge(in_str)
    local l_str1, l_str2 = string.match(in_str, "1206 %s*([^%s]+)%s*([^%s]+)%s*")
    if l_str1 and l_str2 then
        local l_n1 = tonumber(l_str1)
        local l_n2 = tonumber(l_str2)
        if l_n1 ~= nil and l_n2 ~= nil then
            local l_idx = #t_filter_1206 + 1
            t_filter_1206[l_idx] = {Node1 = l_n1, Node2 = l_n2}
        end
    end
end

-- VER1.23 : remove nodes that exist ONLY in 1206 vehicle edges and never in
-- any 1202 taxiway/runway segment.  These are ground-vehicle-only waypoints
-- that aircraft should never visit.
-- Logic mirrors the HTML pathfinder's apply1206Filter():
--   1. Build the set of nodes referenced by at least one 1202 segment.
--   2. A 1206 node is removable only if it is NOT in that set.
--   3. Remove those nodes from t_taxinode (nil them out) and drop any
--      segment whose endpoint is a removed node.
--   4. Also scrub the Segment index string of each surviving node so A*
--      doesn't chase a dangling segment reference.

-- ====================================================
-- Function: apply_1206_filter
-- Description:
-- Post-processing step called once after all apt.dat segments are parsed.
-- Identifies taxinodes that appear exclusively in row-1206 ground-vehicle
-- edges and never in any row-1202 taxiway/runway segment. These
-- vehicle-only nodes are flagged for removal: any t_segment referencing
-- them is dropped and the Segment index strings of all surviving nodes
-- are rebuilt from scratch. This prevents the A* search from routing
-- the FM car through service-vehicle-only paths (fuel lanes, cargo
-- roads) that bypass the aircraft taxiway network.
-- ====================================================
function apply_1206_filter()
    if #t_filter_1206 == 0 then
        return
    end

    -- Step 1: build set of nodes used in 1202 segments
    local l_used = {}
    for l_s = 1, #t_segment do
        l_used[t_segment[l_s].Node1] = true
        l_used[t_segment[l_s].Node2] = true
    end

    -- Step 2: collect nodes that appear only in 1206 (never in 1202)
    local l_remove = {}
    for l_f = 1, #t_filter_1206 do
        local l_n1 = t_filter_1206[l_f].Node1
        local l_n2 = t_filter_1206[l_f].Node2
        if not l_used[l_n1] then
            l_remove[l_n1] = true
        end
        if not l_used[l_n2] then
            l_remove[l_n2] = true
        end
    end

    -- Step 3: count vehicle-only nodes (do NOT nil them - nilling creates holes in t_taxinode
    --          that crash every loop using  for i=1,#t_taxinode do  without a nil-guard).
    --          Nodes with no 1202 segment will get Segment="" in Step 5, so A* ignores them.
    local l_count = 0
    for l_id, _ in pairs(l_remove) do
        if t_taxinode[l_id + 1] ~= nil then
            l_count = l_count + 1
        end
    end

    -- Step 4: remove segments that reference a removed node
    -- (These are ADD_NEWSEGMENT stubs that may have been built from vehicle nodes;
    --  regular 1202 segments already survived Step 1.)
    local l_seg_count = 0
    local l_new_segments = {}
    for l_s = 1, #t_segment do
        if l_remove[t_segment[l_s].Node1] or l_remove[t_segment[l_s].Node2] then
            l_seg_count = l_seg_count + 1
        else
            l_new_segments[#l_new_segments + 1] = t_segment[l_s]
        end
    end
    -- Rebuild t_segment as a clean array (avoids nil holes)
    for l_s = 1, #l_new_segments do
        t_segment[l_s] = l_new_segments[l_s]
    end
    for l_s = #l_new_segments + 1, #t_segment + l_seg_count do
        t_segment[l_s] = nil
    end

    -- Step 5: rebuild Segment index strings for surviving nodes so A* is consistent
    -- Clear all index strings first
    for l_idx = 1, #t_taxinode do
        if t_taxinode[l_idx] ~= nil then
            t_taxinode[l_idx].Segment = ""
        end
    end
    -- Repopulate from the cleaned segment list
    for l_s = 1, #t_segment do
        local l_n1idx = t_segment[l_s].Node1 + 1
        local l_n2idx = t_segment[l_s].Node2 + 1
        if t_taxinode[l_n1idx] ~= nil then
            if t_taxinode[l_n1idx].Segment == "" then
                t_taxinode[l_n1idx].Segment = tostring(l_s)
            else
                t_taxinode[l_n1idx].Segment = t_taxinode[l_n1idx].Segment .. "," .. tostring(l_s)
            end
        end
        if t_taxinode[l_n2idx] ~= nil then
            if t_taxinode[l_n2idx].Segment == "" then
                t_taxinode[l_n2idx].Segment = tostring(l_s)
            else
                t_taxinode[l_n2idx].Segment = t_taxinode[l_n2idx].Segment .. "," .. tostring(l_s)
            end
        end
    end
end

-- ====================================================
-- Function: determine_runway_node
-- Description:
-- Assigns an apt.dat taxinode to each runway threshold by calling
-- match_runway() for every entry in t_runway[]. After matching, removes
-- any runway entry whose Node field is still -1 (no reachable taxiway
-- node found) and adds its ID to t_deleted_runway[] so the UI can warn
-- the pilot that no route exists for that runway. The surviving entries
-- in t_runway[] are the only runways shown in the "To Runway" dropdown.
-- ====================================================
function determine_runway_node()
    -- VER1.11 : Universal rule - for each runway, find the runway node that:
    --   1. Has at least one taxiway neighbour (accessible from the taxi network)
    --   2. Is closest to the runway threshold
    -- This replaces the non-duplicate filtering which excluded valid intersection
    -- nodes (e.g. CYHU node 59 on taxiway K was a duplicate and got ignored).
    -- If no node has a taxiway neighbour, fall back to closest node (any type).

    local l_idx = 0

    for l_idx = 1, #t_runway do
        match_runway(l_idx)
    end

    local l_rows = #t_runway
    l_idx = 1
    while l_idx <= l_rows do
		-- VER1.7 : -1 = unassigned
        if t_runway[l_idx].Node == -1 then
            t_deleted_runway[#t_deleted_runway + 1] = t_runway[l_idx].ID
            table.remove(t_runway, l_idx)
            l_rows = l_rows - 1
        else
            l_idx = l_idx + 1
        end
    end
end

-- ====================================================
-- Function: match_runway
-- Description:
-- Finds the best taxinode entry point for a specific runway threshold.
-- First builds the set of nodes that belong to segments of THIS runway
-- (matching by runway ID within the combined segment name, e.g. "11"
-- in "11/29"). Among those nodes, selects the one closest to the
-- threshold that also has at least one taxiway (non-runway) neighbour -
-- this ensures the car can leave the taxiway network and reach the
-- centreline. Falls back to the closest node of any type if none has a
-- taxiway neighbour. Stores the chosen node ID in t_runway[in_runway_idx].Node.
-- ====================================================
function match_runway(in_runway_idx)
    -- VER1.15 : Only consider nodes that appear on a segment belonging to THIS runway.
    -- VER1.11 searched all runway-typed nodes globally - this caused CYQB RWY 11 to
    -- pick node 12 (on taxiway H + piste 06/24, 405m from seuil 11) instead of
    -- node 78 (on taxiway D+G + piste 11/29, 690m from seuil 11). The car then
    -- entered on 06/24 and cut through the field to reach the threshold.
    -- Fix: build the candidate set from segments whose ID matches the runway ID.
    local l_idx = 0
    local l_curr_dist = 0
    local l_min_dist_twy = 99999
    local l_min_dist_any = 99999
    local l_best_twy = -1
    local l_best_any = -1
    local l_rwy_x = t_runway[in_runway_idx].x
    local l_rwy_z = t_runway[in_runway_idx].z
	-- l_rwy_id : e.g. "11", "29", "06", "24"
    local l_rwy_id = t_runway[in_runway_idx].ID

    -- Build nodes on THIS runway's segments
    -- Segment IDs in apt.dat use combined names like "06/24" or "11/29"
    -- while t_runway[].ID holds individual names like "06" or "11".
    -- Use string.find to match: "11" matches "11/29".
    local l_rwy_nodes = {}
    for l_seg = 1, #t_segment do
        if t_segment[l_seg].Type == "runway" then
            local l_seg_id = t_segment[l_seg].ID or ""
            -- match if rwy_id appears as a component of the segment name
            local l_match = false
            for l_part in (l_seg_id .. "/"):gmatch("([^/]+)/") do
                if l_part == l_rwy_id then
                    l_match = true
                    break
                end
            end
            if l_match then
                l_rwy_nodes[t_segment[l_seg].Node1] = true
                l_rwy_nodes[t_segment[l_seg].Node2] = true
            end
        end
    end

    -- If no segments found for this runway, fall back to global search
    local l_has_own_segs = false
    for _ in pairs(l_rwy_nodes) do
        l_has_own_segs = true
        break
    end

    for l_idx = 1, #t_taxinode do
        if t_taxinode[l_idx].Type == "runway" then
            local l_node_id = l_idx - 1
            -- VER1.15 : only consider nodes on this runway's segments (when available)
            if l_has_own_segs and not l_rwy_nodes[l_node_id] then
                -- skip nodes that belong to a different runway
            else
                _, l_curr_dist = heading_n_dist(t_taxinode[l_idx].x, t_taxinode[l_idx].z, l_rwy_x, l_rwy_z)
                if l_curr_dist < l_min_dist_any then
                    l_min_dist_any = l_curr_dist
                    l_best_any = l_node_id
                end
                local l_has_twy = false
                for l_seg = 1, #t_segment do
                    if
                        (t_segment[l_seg].Node1 == l_node_id or t_segment[l_seg].Node2 == l_node_id) and
                            t_segment[l_seg].Type ~= "runway"
                     then
                        l_has_twy = true
                        break
                    end
                end
                if l_has_twy and l_curr_dist < l_min_dist_twy then
                    l_min_dist_twy = l_curr_dist
                    l_best_twy = l_node_id
                end
            end
        end
    end

    local l_chosen = l_best_twy
    local l_chosen_dist = l_min_dist_twy
    if l_chosen == -1 then
        l_chosen = l_best_any
        l_chosen_dist = l_min_dist_any
    end

    if l_chosen >= 0 then
        t_runway[in_runway_idx].Node = l_chosen
        t_taxinode[l_chosen + 1].Runway = l_rwy_id
    end
end

-- VER1.12 : Single cleanup entry point - called on new flight or location change.
-- Unloads all 3D objects, clears all state variables, ready for fresh use.
-- Does NOT touch dataref registration (that is only done at load/exit).

-- ====================================================
-- Function: full_reset
-- Description:
-- Single cleanup entry point that returns the plugin to a pristine state.
-- Unloads all 3-D objects (car, signboard, path pins, ramp marker) if
-- the FM car is currently active, then resets all motion, routing, and
-- airport state variables. Forces a fresh apt.dat reload on the next
-- get_airport_elements() call by clearing curr_ICAO. Called by
-- handle_plugin_window() when X-Plane starts a new flight (fm_new_flight
-- resets) or when the aircraft teleports more than 1000 m, and from
-- exit_plugin() on shutdown. Fixes the double-car bug that occurred after
-- a location change without restarting X-Plane (VER1.12).
-- ====================================================
function full_reset()
    if FM_car_active then
        unload_object()
        unload_path()
        unload_rampstart()
    end
    FM_car_active = false
    prepare_kill_objects = false
    prepare_show_objects = false
    kill_is_manual = false
    ground_time = 0
    flightstart = 0
	-- new airport = fresh runway list
    t_deleted_runway = {}
	-- VER1.12 : force apt.dat reload on next get_airport_elements()
    curr_ICAO = ""
    initialise_airport()
    initialise_routes()
    logMsg("FollowMe : full_reset() completed")
end

-- ====================================================
-- Function: initialise_airport
-- Description:
-- Clears all airport-data tables (t_runway, t_runway_node, t_gate,
-- t_taxinode, t_segment, t_filter_1206) and resets the depart/arrive
-- mode and gate/runway selections to defaults. Called by full_reset()
-- whenever a new airport must be loaded. Note: t_deleted_runway is
-- intentionally preserved across calls so runways known to lack routes
-- are not repeatedly re-tested after airport re-reads.
-- ====================================================
function initialise_airport()
    t_runway, t_runway_node, t_gate, t_taxinode, t_segment = {}, {}, {}, {}, {}
    t_filter_1206 = {} -- VER1.23 : reset vehicle edge filter list
    -- VER1.4 : t_deleted_runway is intentionally NOT reset here
    -- (runways without routes must persist across airport re-reads)
    -- t_deleted_runway = {}
    depart_arrive = 0
    depart_gate, arrival_gate, depart_runway, gatetext = 0, 0, "", ""
end

-- ====================================================
-- Function: initialise_routes
-- Description:
-- Clears the active drive waypoint list (t_node), resets all status
-- messages, and resets the depart/arrive mode flag to 0 (none). If a
-- route was in progress, plays the appropriate farewell sound (safe-
-- flight for departure, welcome-bye for arrival) before clearing.
-- Called by full_reset(), by handle_plugin_window() on manual cancel
-- (with routing state preserved so the pilot can immediately re-request),
-- and on auto-cancel after takeoff or landing.
-- ====================================================
function initialise_routes()
    if #t_node > 0 then
        if depart_arrive == 1 and curr_node >= #t_node and flightstart ~= 9999 and not kill_is_manual then
            play_sound(snd_safeflight_bye)
        elseif depart_arrive == 2 and curr_node == #t_node then
            play_sound(snd_welcome_bye)
        end
    end
    t_node = {}
    Err_Msg[1] = {}
    Err_Msg[2] = {}
    Err_Msg[3] = {}
    depart_arrive = 0
	-- VER1.8
    is_backtaxi = false
    window_first_access = true
end

-- ====================================================
-- Function: handle_plugin_window
-- Description:
-- Master per-frame event dispatcher, registered with do_every_frame().
-- Handles: new-flight detection and teleport detection (both trigger
-- full_reset()), airborne timer that auto-cancels the FM car 3 min after
-- takeoff, window show/hide toggle from the holder button, taxi-light
-- off event that auto-kills the car when the route is complete, SimBrief
-- auto-trigger when taxi or beacon light turns on, speed-warning sound
-- when the aircraft exceeds 20 kts with speed_limiter active, arrival
-- auto-route when the aircraft lands with random_gate or a pre-selected
-- gate, deferred XPLMSpeakString for gate announcements, and the
-- prepare_show / prepare_kill object lifecycle flags that load or unload
-- the 3-D assets and start or stop the car.
-- ====================================================
function handle_plugin_window()
    -- VER1.12 : detect new flight / location change - fm_new_flight resets to 0
    if fm_new_flight < prev_new_flight and prev_new_flight > 5 then
        full_reset()
    end
    prev_new_flight = fm_new_flight

    -- VER1.12 : detect teleport in same flight - position jumps > 100m in 1 frame
    -- Max realistic speed 250kts = 128m/s at 20fps = 6.4m/frame, never > 100m
    if prev_plane_x ~= 0 then
        local _, l_teleport_dist = heading_n_dist(prev_plane_x, prev_plane_z, fm_plane_x, fm_plane_z)
		-- VER1.12 fix: 1000m avoids false reset during scene loading (plane can jump 300-900m)
        if l_teleport_dist >= 1000 then
            full_reset()
        end
    end
    prev_plane_x = fm_plane_x
    prev_plane_z = fm_plane_z

    if (fm_gear1_gnd == 0 and fm_gear2_gnd == 0) and fm_new_flight > 1 then
        if flightstart == 0 then
            flightstart = fm_run_time + 180
            -- VER1.6 : close the main window automatically at takeoff
            if followme_wnd ~= nil then
                hide_window()
            end
        end
        if fm_run_time > flightstart and flightstart ~= 9999 then
            arrival_gate = 0
            flightstart = 9999
            ground_time = 0
            prepare_kill_objects = true
        end
    end

    -- Process window toggle unconditionally (not inside else)
    -- so it works regardless of flight/ground state
    if holder_wnd and toggle_window == true then
        toggle_window = false
        if followme_wnd ~= nil then
            hide_window()
        else
            window_is_open = true
            show_window()
        end
    end

    local l_err = ""

    if FM_car_active == true and #t_node > 0 and prev_taxi_light ~= fm_taxi_light and fm_taxi_light == 0 then
        if (depart_arrive == 1 and curr_node >= #t_node) or (depart_arrive == 2 and curr_node == #t_node) then
            prepare_kill_objects = true
        end
    end

    -- VER1.5 : Auto-trigger via SimBrief (replaces FMS)
    if FM_car_active == false and get_from_SimBrief == true and fm_gear1_gnd == 1 and holder_wnd then
        if
            (prev_taxi_light ~= fm_taxi_light and fm_taxi_light == 1) or
                (prev_beacon_light ~= fm_beacon_light and fm_beacon_light == 1)
         then
            get_airport_elements()
            if taxiway_network == "" then
                if depart_runway ~= "" then
                    l_err = determine_XP_route()
                    if l_err == "" or (l_err ~= "" and tonumber(l_err) > 0) then
                        prepare_show_objects = true
                    else
                        update_msg(l_err)
                    end
                end
            else
                update_msg("-15")
            end
        end
    end
    prev_taxi_light = fm_taxi_light
    prev_beacon_light = fm_beacon_light

    -- VER1.6 fix : speed warning when aircraft exceeds 20 kts and speed_limiter is active
    -- 10.288 m/s = 20 kts ; warning repeats every 15 seconds max
    if
        speed_limiter and FM_car_active and fm_gear1_gnd == 1 and fm_gear2_gnd == 1 and fm_gnd_spd > 10.288 and
            fm_run_time > speed_warn_time
     then
        play_sound(snd_keep_speed)
		-- repeat at most every 15 seconds
        speed_warn_time = fm_run_time + 15
    end

    if
        FM_car_active == false and flightstart == 9999 and (fm_gear1_gnd == 1 and fm_gear2_gnd == 1) and
            ground_time == 0 and
            fm_taxi_light == 1 and
            fm_gnd_spd < 15
     then
        get_airport_elements()
        if taxiway_network == "" then
            if random_gate == true or arrival_gate > 0 then
                if arrival_gate == 0 then
                    l_err = auto_assign_gate()
                    if l_err == "1" then
                        play_text =
                            "No suitable gate for this aircraft. Randomly assigned to gate " .. t_gate[arrival_gate].ID
                    elseif l_err == "2" then
                        play_text = "Assigned to " .. t_gate[arrival_gate].ID
                    end
                    play_time = fm_run_time + 5
                end

                if window_is_open == false then
                    l_err = determine_XP_route()
                    if l_err == "" or (l_err ~= "" and tonumber(l_err) > 0) then
                        prepare_show_objects = true
                    else
                        update_msg(l_err)
                    end
                end
            end
        else
            update_msg("-15")
        end
        ground_time = 1
    end

    if play_time ~= 0 then
        if fm_run_time > play_time then
            XPLMSpeakString(play_text)
            play_text = ""
            play_time = 0
        end
    end

    if prepare_show_objects == true then
        FM_car_active = true
        load_object()
        load_path()
        start_car()
        rampstart_chg = true
        prepare_show_objects = false
    end

    if prepare_kill_objects == true then
        FM_car_active = false
        unload_object()
        unload_path()
        -- VER1.4 : behavior based on cancel type
        if kill_is_manual then
            unload_rampstart()
            rampstart_chg = false
            kill_is_manual = false
            -- VER1.9 : say goodbye when user manually cancels
            update_msg("7")
            -- VER1.12 : force airport reload so window_first_access re-reads apt.dat
            curr_ICAO = ""
            -- Preserve all routing state so the user can immediately re-request
            -- without having to re-select runway, mode, or gate
            local l_saved_runway = depart_runway
            local l_saved_depart = depart_arrive
            local l_saved_arrival_gate = arrival_gate
            local l_saved_gatetext = gatetext
            local l_saved_depart_gate = depart_gate
            initialise_routes()
            depart_runway = l_saved_runway
            depart_arrive = l_saved_depart
            arrival_gate = l_saved_arrival_gate
            gatetext = l_saved_gatetext
            depart_gate = l_saved_depart_gate
            -- Only redraw ramp marker if show_rampstart is active AND a gate is selected
            if show_rampstart and (arrival_gate > 0 or depart_gate > 0) then
                rampstart_chg = true
            end
        else
            if flightstart == 9999 then
                unload_rampstart()
            end
            initialise_routes()
        end -- end kill_is_manual
        prepare_kill_objects = false
    end

    if not holder_wnd and fm_new_flight > 1 then
        screen_width = SCREEN_WIDTH
        show_holder()
    end

    if holder_wnd and screen_width ~= SCREEN_WIDTH then
        screen_width = SCREEN_WIDTH
        float_wnd_set_position(holder_wnd, screen_width - Holder_len, Win_Y)
        if followme_wnd ~= nil then
            float_wnd_set_position(followme_wnd, screen_width - 430, Win_Y)
        end
    end

    if holder_wnd then
        if
            MOUSE_X >= SCREEN_WIDTH - Holder_len and MOUSE_X <= SCREEN_WIDTH and
                MOUSE_Y >= SCREEN_HIGHT - (Win_Y + Holder_len) and
                MOUSE_Y <= SCREEN_HIGHT - Win_Y
         then
            float_wnd_bring_to_front(holder_wnd)
        end
        if holder_drag == 3 and followme_wnd ~= nil then
            float_wnd_set_position(followme_wnd, screen_width - 430, Win_Y)
            holder_drag = 2
        end
    end

    if followme_wnd ~= nil then
        -- Only bring FM window to front if mouse is over it but NOT over the holder icon
        local l_over_holder =
            (MOUSE_X >= SCREEN_WIDTH - Holder_len and MOUSE_X <= SCREEN_WIDTH and
            MOUSE_Y >= SCREEN_HIGHT - (Win_Y + Holder_len) and
            MOUSE_Y <= SCREEN_HIGHT - Win_Y)
        if not l_over_holder then
            if
                MOUSE_X >= SCREEN_WIDTH - 400 - Holder_len and MOUSE_X <= SCREEN_WIDTH - Holder_len and
                    MOUSE_Y >= SCREEN_HIGHT - (Win_Y + 400) and
                    MOUSE_Y <= SCREEN_HIGHT - Win_Y
             then
                float_wnd_bring_to_front(followme_wnd)
            end
        end
    end
end

-- ====================================================
-- Function: show_holder
-- Description:
-- Creates the small "FM" badge floating window in the top-right corner
-- of the screen. Loads user preferences on first call (init_load guard).
-- The badge is always visible after the first flight starts and acts as
-- the click target to toggle the main FollowMe window open or closed.
-- Its appearance changes between white (on ground) and grey with a red
-- diagonal bar (in flight) to indicate service availability.
-- ====================================================
function show_holder()
    if init_load == 0 then
        load_config()
        init_load = 1
    end
    holder_wnd = float_wnd_create(Holder_len, Holder_len, 2, true)
    float_wnd_set_position(holder_wnd, screen_width - Holder_len, Win_Y)
    float_wnd_set_imgui_builder(holder_wnd, "build_holder")
    float_wnd_set_onclose(holder_wnd, "closed_holder")
end

-- ====================================================
-- Function: hide_holder
-- Description:
-- Destroys the "FM" badge floating window and sets holder_wnd to nil.
-- Not currently called during normal operation (the badge persists for
-- the entire flight session) but available for cleanup if needed.
-- ====================================================
function hide_holder()
    if holder_wnd then
        float_wnd_destroy(holder_wnd)
        holder_wnd = nil
    end
end

-- ====================================================
-- Function: show_window
-- Description:
-- Creates the main FollowMe control panel (405 x 525 px) and attaches
-- the build_window() imgui builder callback to it. Sets window_first_access
-- = true so build_window() triggers a fresh get_airport_elements() call
-- on the first rendered frame. The window is positioned to the left of
-- the FM badge, anchored to the right edge of the screen.
-- ====================================================
function show_window()
    window_first_access = true
    followme_wnd = float_wnd_create(405, 530, 2, true)
    float_wnd_set_position(followme_wnd, screen_width - 405 - Holder_len, Win_Y)
    float_wnd_set_imgui_builder(followme_wnd, "build_window")
    float_wnd_set_onclose(followme_wnd, "closed_window")
end

-- ====================================================
-- Function: hide_window
-- Description:
-- Destroys the main FollowMe control panel, sets followme_wnd to nil,
-- and marks window_is_open = false. Called automatically at takeoff
-- (handle_plugin_window), by the window close button (closed_window
-- callback), and by the toggle logic when the pilot clicks the FM badge
-- while the window is open.
-- ====================================================
function hide_window()
    if followme_wnd ~= nil then
        float_wnd_destroy(followme_wnd)
        followme_wnd = nil
        window_is_open = false
    end
end

-- ====================================================
-- Function: build_window
-- Description:
-- imgui draw callback invoked every frame while the main window is open.
-- Renders the complete FollowMe control panel: airport header (departure
-- and arrival ICAO from X-Plane or SimBrief), Departure/Arrival radio
-- buttons with runway and gate selectors, the Request/Cancel button,
-- the directional arrowhead pointing toward the FM car, the GND SPD /
-- FM Car speed row, the aircraft model and type dropdowns, Show Path /
-- Show Ramp Start checkboxes, volume slider, vehicle type selector,
-- SimBrief ID input and fetch button, and the three-line status message
-- area. Also updates the direction label (ahead / right / behind / left)
-- in real time from the current bearing to the car.
-- ====================================================
function build_window(wnd, x, y)
    local l_err = ""
    local l_is_selected = false
    local l_changed = false
    local l_newtext = ""
    local l_newval = 0

    local l_flags =
        bit.bor(
        imgui.constant.WindowFlags.NoTitleBar,
        imgui.constant.WindowFlags.NoResize,
        imgui.constant.WindowFlags.NoMove,
        imgui.constant.WindowFlags.HorizontalScrollbar,
        imgui.constant.WindowFlags.NoSavedSettings
    )

    if window_first_access == true then
        get_airport_elements()
        l_err = taxiway_network
        window_first_access = false
    end

    if FM_car_active == true or #t_runway == 0 then
        imgui.PushStyleVar(imgui.constant.StyleVar.Alpha, 0.7)
        imgui.PushStyleVar(imgui.constant.StyleVar.WindowBorderSize, 0)
        if FM_car_active == false then
            imgui.SetNextWindowFocus()
        end
        if FM_car_active == true then
            imgui.SetNextWindowPos(0, 75)
            imgui.SetNextWindowSize(405, 120)
        else
            imgui.SetNextWindowPos(0, 75)
            imgui.SetNextWindowSize(405, 170)
        end
        if imgui.Begin("##Disabled", nil, l_flags) then
            imgui.End()
        end
        imgui.PopStyleVar()
        imgui.PopStyleVar()
        if FM_car_active == false then
            imgui.SetNextWindowFocus()
        end
    end

    --=====================================================
    -- Show Departure and Arrival airport (ICAO + name)
    --=====================================================
    imgui.SetWindowFontScale(1.1)
    imgui.SetCursorPosY(5)
    imgui.SetCursorPosX(10)
	-- light gray
    imgui.PushStyleColor(imgui.constant.Col.Text, 0xFF888888)

    if get_from_SimBrief then
        imgui.TextUnformatted("Simbrief Departure Airport")
    else
        imgui.TextUnformatted("Departure Airport")
    end

    imgui.PopStyleColor()
    imgui.SetCursorPosY(18)
    imgui.SetCursorPosX(10)
    imgui.SetWindowFontScale(1.2)

    if get_from_SimBrief and sb_fetch_status == "OK" and sb_origin_icao ~= "" then
        imgui.TextUnformatted(sb_origin_icao .. "   " .. sb_origin_name)
    else
        -- Fallback: display current sim airport
        imgui.TextUnformatted(curr_ICAO .. "   " .. curr_ICAO_name)
    end

    imgui.SetWindowFontScale(1.1)
    imgui.SetCursorPosY(35)
    imgui.SetCursorPosX(10)
	-- light gray
    imgui.PushStyleColor(imgui.constant.Col.Text, 0xFF888888)

    if get_from_SimBrief then
        imgui.TextUnformatted("Simbrief Arrival Airport")
    else
        imgui.TextUnformatted("Arrival Airport")
    end

    imgui.PopStyleColor()

    imgui.SetCursorPosY(48)
    imgui.SetCursorPosX(10)
    imgui.SetWindowFontScale(1.2)

    if get_from_SimBrief and sb_fetch_status == "OK" and sb_dest_icao ~= "" then
        imgui.TextUnformatted(sb_dest_icao .. "   " .. sb_dest_name)
    else
        imgui.PushStyleColor(imgui.constant.Col.Text, 0xFF666666)
        imgui.TextUnformatted("---")
        imgui.PopStyleColor()
    end

    imgui.SetWindowFontScale(1.0)
    imgui.SetCursorPosY(65)
    imgui.Separator()

    --=====================================================
    -- Allow selection for Departure & Arrival
    -- Show "Request Follow Me Car" button or "Cancel Follow Me Car"
    -- Show Aircraft Speed and Follow Me car Speed
    -- Show Follow Me car Pointer (yellow triangle)
    --=====================================================
    imgui.SetCursorPosY(75)
    imgui.SetCursorPosX(18)
    if imgui.RadioButton(" Departure", depart_arrive == 1, false) then
        depart_arrive = 1
        flightstart = 0
        rampstart_chg = true
        if get_from_SimBrief then
            apply_simbrief_runway()
        end
    end

    if depart_gate > 0 then
        imgui.SameLine()
        --imgui.SetCursorPosX(160)
        imgui.SetCursorPosX(134)
        imgui.TextUnformatted(t_gate[depart_gate].ID)
    end

    imgui.SetCursorPosY(100)
    imgui.SetCursorPosX(48)
    imgui.TextUnformatted("To Runway : ")

    imgui.SameLine()
    imgui.SetCursorPosX(130)
    imgui.PushItemWidth(55)

    -- VER1.5 : If 'Get SimBrief data' checked -> ReadOnly field with SimBrief runway
    if get_from_SimBrief then
        imgui.InputText("##text_runway", sb_runway_takeoff, 5, imgui.constant.InputTextFlags.ReadOnly)
    else
        local l_depart_runway = depart_runway
        if imgui.BeginCombo("##select_runway", l_depart_runway) then
            for i = 1, #t_runway do
                l_is_selected = (l_depart_runway == t_runway[i].ID)
                if imgui.Selectable(t_runway[i].ID, l_is_selected) then
                    depart_runway = t_runway[i].ID
                    depart_arrive = 1
                    flightstart = 0
                end
                if l_is_selected then
                    imgui.SetItemDefaultFocus()
                end
            end
            imgui.EndCombo()
        end
    end
    imgui.PopItemWidth()

    imgui.SameLine()
    imgui.SetCursorPosX(230)

    -- VER1.5 : Checkbox 'Get SimBrief data' (replaces 'Get from FMS')
    l_changed, l_newval = imgui.Checkbox("##Get SimBrief data", get_from_SimBrief)
    if l_changed then
        get_from_SimBrief = l_newval
        if get_from_SimBrief then
            -- Trigger SimBrief fetch if ID is configured
            if simbrief_id ~= "" then
                check_SimBrief()
            else
                update_msg("-20") -- No SimBrief ID configured
            end
        else
            depart_runway = ""
        end
    end

    imgui.SameLine()
    imgui.TextUnformatted("Get SimBrief data")

    imgui.SetCursorPosY(135)
    imgui.SetCursorPosX(18)

    if imgui.RadioButton(" Arrival", depart_arrive == 2, false) then
        depart_arrive = 2
        rampstart_chg = true
        if get_from_SimBrief then
            apply_simbrief_runway()
        end
    end

    if depart_arrive == 2 and random_gate == true and arrival_gate == 0 then
        auto_assign_gate()
    end

    imgui.SameLine()
    imgui.SetCursorPosX(230)

    l_changed, l_newval = imgui.Checkbox("##Auto Assign", random_gate)
    if l_changed then
        random_gate = l_newval
        if random_gate == true and arrival_gate == 0 then
            auto_assign_gate()
        end
    end

    imgui.SameLine()
    imgui.TextUnformatted("Auto Assign")

    imgui.SetCursorPosY(160)
    imgui.SetCursorPosX(48)
    imgui.TextUnformatted("To Gate/Ramp : ")

    imgui.SameLine()
    imgui.SetCursorPosX(160)
    imgui.PushItemWidth(150)

    l_changed, l_newtext = imgui.InputText("##filter_gate", gatetext, 30)
    if l_changed then
        gatetext = l_newtext
        text_was_chg = true
    end
    imgui.PopItemWidth()

    if imgui.IsItemClicked(0) then
        combo_filter_list = true
    end

    if combo_filter_list == true then
        imgui.SetNextWindowFocus()
        imgui.SetNextWindowPos(160, 180)
        imgui.SetNextWindowSize(150, 80)
        if imgui.Begin("##combo_filter", nil, l_flags) then
            local l_gate = ""
            if arrival_gate > 0 then
                l_gate = t_gate[arrival_gate].ID
            end
            t_suitable_gates = {}
            for i = 1, #t_gate do
                if
                    string.match(t_gate[i].Types, Aircraft_Type) == Aircraft_Type or
                        (Aircraft_Type == "0" and string.match(t_gate[i].Types, "7"))
                 then
                    t_suitable_gates[#t_suitable_gates + 1] = i
                end
            end
            if #t_suitable_gates == 0 then
                l_err = "-17"
                for i = 1, #t_gate do
                    t_suitable_gates[#t_suitable_gates + 1] = i
                end
            end
            local l_into_list = true
            for i = 1, #t_suitable_gates do
                if text_was_chg then
                    if string.match(t_gate[t_suitable_gates[i]].ID, gatetext) == gatetext then
                        l_into_list = true
                    else
                        l_into_list = false
                    end
                end
                if l_into_list == true then
                    l_is_selected = (l_gate == t_gate[t_suitable_gates[i]].ID)
                    if imgui.Selectable(t_gate[t_suitable_gates[i]].ID, l_is_selected) then
                        depart_arrive = 2
                        arrival_gate = t_suitable_gates[i]
                        gatetext = t_gate[arrival_gate].ID
                        rampstart_chg = true
                        combo_filter_list = false
                        text_was_chg = false
                    end
                    if l_is_selected then
                        imgui.SetItemDefaultFocus()
                    end
                end
            end
            imgui.End()
        end
    end

    if l_err == "-17" then
        if Err_Msg[1].text ~= nil and string.find(Err_Msg[1].text, "No suitable gate for this plane. Lift") then
            l_err = ""
        end
    end

    imgui.SameLine()
    if imgui.Button("X##clear_filter", 15, 15) then
        arrival_gate = 0
        gatetext = ""
        combo_filter_list = false
        text_was_chg = false
        rampstart_chg = true
    end

    imgui.SetCursorPosY(200)
    imgui.SetCursorPosX(230)
    l_changed, l_newval = imgui.Checkbox("##Limit Car Speed", speed_limiter)
    if l_changed then
        speed_limiter = l_newval
        if speed_limiter == true then
			-- VER1.6 fix : 20 kts = 10.288 m/s (previously was 20 m/s = ~39 kts)
            speed_max = 10.288
        else
            speed_max = car_default_speed
        end
    end
    imgui.SameLine()
    imgui.TextUnformatted("Limit speed \nto 20kts")

    imgui.PushStyleColor(imgui.constant.Col.Button, 0xFF2D5A27)
    imgui.PushStyleColor(imgui.constant.Col.ButtonHovered, 0xFF3D7A35)
    imgui.PushStyleColor(imgui.constant.Col.ButtonActive, 0xFF4D9A43)

    imgui.SetCursorPosY(200)
    imgui.SetCursorPosX(48)

    if FM_car_active == false then
        if imgui.Button("Request Follow Me Car", 170, 30) then
            combo_filter_list = false
            l_err = determine_XP_route()
            if l_err == "" or (l_err ~= "" and tonumber(l_err) > 0) then
                prepare_show_objects = true
            end
        end
    else
        if imgui.Button("Cancel Follow Me Car", 170, 30) then
            prepare_kill_objects = true
            kill_is_manual = true
        end
    end

    imgui.PopStyleColor(3)

    -- VER2.0 : ARROW POINTER FOLLOW ME CAR
    -- Replaced simple line with a filled arrowhead triangle pointing toward the FM car
    -- relative to the aircraft heading. The tip points in the car direction.
	if FM_car_active and car_x ~= 0 then
	    -- Absolute bearing from aircraft to FM car
	    local l_bearing_to_car, l_dist_to_car = heading_n_dist(fm_plane_x, fm_plane_z, car_x, car_z)

	    -- Relative angle: 0=front, 90=right, 180=rear, 270=left  (aircraft-relative, 0-360)
	    local l_rel_angle = l_bearing_to_car - fm_plane_head
	    while l_rel_angle < 0   do l_rel_angle = l_rel_angle + 360 end
	    while l_rel_angle >= 360 do l_rel_angle = l_rel_angle - 360 end

	    -- Draw a filled yellow arrow triangle pointing toward the car
	    local cx, cy    = 370, 220
		-- distance from center to tip
	    local tip_len   = 18
		-- half-width of arrow base
	    local wing_len  = 10
		-- distance from center to base (behind center)
	    local tail_len  = 8

	    -- Tip direction (0 deg = up = north = forward)
	    local tip_rad   = math.rad(l_rel_angle - 90)
	    -- Base direction (opposite of tip)
	    local base_rad  = math.rad(l_rel_angle + 90)
	    -- Wing perpendicular directions
		-- perpendicular left
	    local left_rad  = math.rad(l_rel_angle - 90 + 90)
		-- perpendicular right
	    local right_rad = math.rad(l_rel_angle - 90 - 90)

	    local tip_x  = cx + math.cos(tip_rad)   * tip_len
	    local tip_y  = cy + math.sin(tip_rad)   * tip_len
	    local base_x = cx + math.cos(base_rad)  * tail_len
	    local base_y = cy + math.sin(base_rad)  * tail_len

	    local lw_x = base_x + math.cos(left_rad)  * wing_len
	    local lw_y = base_y + math.sin(left_rad)  * wing_len
	    local rw_x = base_x + math.cos(right_rad) * wing_len
	    local rw_y = base_y + math.sin(right_rad) * wing_len

	    -- Filled yellow triangle (tip, left-wing, right-wing)
	    imgui.DrawList_AddTriangleFilled(tip_x, tip_y, lw_x, lw_y, rw_x, rw_y, 0xFF00FFFF)
	end

    -- VER1.6 : Speed display zone - reserved read-only row below button
    imgui.SetCursorPosY(239)
    imgui.SetCursorPosX(48)
    imgui.PushStyleColor(imgui.constant.Col.Text, 0xFF666666)
    imgui.TextUnformatted("GND SPD")
    imgui.PopStyleColor()
    imgui.SameLine()
    imgui.SetCursorPosX(108)
    if FM_car_active then
        local l_plane_kts = fm_gnd_spd * 1.94384
        local l_spd_color = (speed_limiter and l_plane_kts > 20) and 0xFF0000FF or 0xFF00FF00
        imgui.PushStyleColor(imgui.constant.Col.Text, l_spd_color)
        imgui.TextUnformatted(string.format("%.1f kts", l_plane_kts))
        imgui.PopStyleColor()
    else
        imgui.PushStyleColor(imgui.constant.Col.Text, 0xFF444444)
        imgui.TextUnformatted("---")
        imgui.PopStyleColor()
    end
    imgui.SameLine()
    imgui.SetCursorPosX(216)
    imgui.PushStyleColor(imgui.constant.Col.Text, 0xFF666666)
    imgui.TextUnformatted("FM Car")
    imgui.PopStyleColor()
    imgui.SameLine()
    imgui.SetCursorPosX(263)
    if FM_car_active then
        local l_car_kts = car_speed * 1.94384
        imgui.PushStyleColor(imgui.constant.Col.Text, 0xFFCCCCCC)
        imgui.TextUnformatted(string.format("%.1f kts", l_car_kts))
        imgui.PopStyleColor()
    else
        imgui.PushStyleColor(imgui.constant.Col.Text, 0xFF444444)
        imgui.TextUnformatted("---")
        imgui.PopStyleColor()
    end

    imgui.SetCursorPosY(265)
    imgui.Separator()

    imgui.SetCursorPosY(275)
    imgui.SetCursorPosX(20)
    imgui.TextUnformatted("Model")
    imgui.SameLine()
    imgui.SetCursorPosX(90)
    imgui.TextUnformatted("Type")
    imgui.SameLine()
    imgui.SetCursorPosX(160)
    imgui.TextUnformatted("* Helps find suitable gate/ramp")

    imgui.SetCursorPosY(295)
    imgui.SetCursorPosX(20)
    imgui.TextUnformatted(PLANE_ICAO)
    imgui.SameLine()
    imgui.SetCursorPosX(85)
    imgui.TextUnformatted("*")
    imgui.SameLine()

    local l_ac_types = ""

    if Aircraft_Type == "" then
        l_ac_types = ""
    else
        l_ac_types = ac_types[tonumber(Aircraft_Type) + 1]
    end

    if FM_car_active == true and depart_arrive == 2 then
        imgui.InputText("##text_actype", l_ac_types, 250, imgui.constant.InputTextFlags.ReadOnly)
    else
        imgui.PushItemWidth(250)
        if imgui.BeginCombo("##select_type", l_ac_types) then
            for i = 1, #ac_types do
                l_is_selected = (l_ac_types == ac_types[i])
                if imgui.Selectable(ac_types[i], l_is_selected) then
                    l_ac_types = ac_types[i]
                    Aircraft_Type = tostring(i - 1)
                    gatetext = ""
                    arrival_gate = 0
                    rampstart_chg = true
                    Err_Msg[1] = {}
                    Err_Msg[2] = {}
                    Err_Msg[3] = {}
                    depart_gate = check_gate()
                end
                if l_is_selected then
                    imgui.SetItemDefaultFocus()
                end
            end
            imgui.EndCombo()
        end
        imgui.PopItemWidth()
        imgui.SameLine()
        if imgui.Button("X##clear_type", 20, 20) then
            Aircraft_Type = ""
            gatetext = ""
            arrival_gate = 0
            rampstart_chg = true
            Err_Msg[1] = {}
            Err_Msg[2] = {}
            Err_Msg[3] = {}
            depart_gate = check_gate()
        end
    end

    imgui.SetCursorPosY(330)
    imgui.SetCursorPosX(20)
    l_changed, l_newval = imgui.Checkbox(" Show Path", show_path)
    if l_changed then
        show_path = l_newval
        if show_path then
            load_path()
        else
            unload_path()
        end
    end

    imgui.SetCursorPosY(355)
    imgui.SetCursorPosX(20)
    l_changed, l_newval = imgui.Checkbox(" Show Ramp Start", show_rampstart)
    if l_changed then
        show_rampstart = l_newval
        if show_rampstart then
            rampstart_chg = true
        else
            unload_rampstart()
        end
    end

	-- Dark green (RGBA hex)
	imgui.PushStyleColor(imgui.constant.Col.Button, 0xFF2D5A27)
	-- Lighter green on hover
	imgui.PushStyleColor(imgui.constant.Col.ButtonHovered, 0xFF3D7A35)
	-- Green when clicked
	imgui.PushStyleColor(imgui.constant.Col.ButtonActive, 0xFF4D9A43)

	imgui.SetCursorPosY(335)
	imgui.SetCursorPosX(220)
	if imgui.Button("Save Preferences", 130, 30) then
	    l_err = save_config()
	end

	-- VERY IMPORTANT: Pop styles to avoid affecting other buttons
	imgui.PopStyleColor(3)

    imgui.SetCursorPosY(385)
    imgui.SetCursorPosX(20)
    imgui.TextUnformatted("Vol ")
    imgui.SameLine()
    imgui.SetCursorPosX(60)
    imgui.PushItemWidth(150)
    l_changed, l_newval = imgui.SliderFloat("##Vol", vol, 1, 10, "%.0f")
    imgui.PopItemWidth()
    if l_changed then
        vol = l_newval
        set_sound_vol()
    end
    imgui.SetCursorPosY(380)
    imgui.SetCursorPosX(220)
    if imgui.Button("Test Volume", 90, 30) then
        play_sound(snd_welcome)
    end

    -- Vehicle selection
    imgui.SetCursorPosX(20)
    imgui.SetCursorPosY(413)
    if imgui.RadioButton(" Ferrari", car_type_fmcar == "Ferrari", false) then
        car_type_fmcar = "Ferrari"
    end
    imgui.SetCursorPosX(110)
    imgui.SetCursorPosY(413)
    if imgui.RadioButton(" FM Van", car_type_fmcar == "Van", false) then
        car_type_fmcar = "Van"
    end
    imgui.SetCursorPosX(195)
    imgui.SetCursorPosY(413)
    if imgui.RadioButton(" FM Truck", car_type_fmcar == "Truck", false) then
        car_type_fmcar = "Truck"
    end
    imgui.SetCursorPosX(290)
    imgui.SetCursorPosY(413)
    if imgui.RadioButton(" Auto Select", car_type_fmcar == "Auto", false) then
        car_type_fmcar = "Auto"
    end

    -- VER1.5 : SimBrief ID row
    imgui.SetCursorPosY(438)
    imgui.Separator()

    imgui.SetCursorPosY(446)
    imgui.SetCursorPosX(10)
    imgui.TextUnformatted("SimBrief ID")

    imgui.SameLine()
    imgui.SetCursorPosX(95)
    imgui.PushItemWidth(90)
    l_changed, l_newtext = imgui.InputText("##simbrief_id", simbrief_id, 11, imgui.constant.InputTextFlags.CharsDecimal)
    if l_changed then
        if string.len(l_newtext) <= 10 then
            simbrief_id = l_newtext
        else
            simbrief_id = string.sub(l_newtext, 1, 10)
        end
    end
    imgui.PopItemWidth()

    imgui.SameLine()
    imgui.SetCursorPosX(195)
    if imgui.Button("Fetch SimBrief", 120, 30) then
        if simbrief_id ~= "" then
            check_SimBrief()
        else
            update_msg("-20")
        end
    end

    -- SimBrief fetch status display
    imgui.SetCursorPosY(480)
    imgui.SetCursorPosX(10)
    if sb_fetch_status == "OK" then
        if get_from_SimBrief then
            if sb_airport_mismatch then
                imgui.PushStyleColor(imgui.constant.Col.Text, 0xFF0000FF)
                imgui.TextUnformatted("The current airport does not match the Simbrief data")
                imgui.PopStyleColor()
            else
                imgui.PushStyleColor(imgui.constant.Col.Text, 0xFF00FF00)
                imgui.TextUnformatted("SimBrief Runways  Dep:" .. sb_runway_takeoff .. "  Arr:" .. sb_runway_landing)
                imgui.PopStyleColor()
            end
        end
    elseif sb_fetch_status == "LOADING" then
        imgui.PushStyleColor(imgui.constant.Col.Text, 0xFF00FFFF)
        imgui.TextUnformatted("Fetching SimBrief data...")
        imgui.PopStyleColor()
    elseif sb_fetch_status == "ERROR" then
        imgui.PushStyleColor(imgui.constant.Col.Text, 0xFF0000FF)
        imgui.TextUnformatted("SimBrief fetch error")
        imgui.PopStyleColor()
    end

    imgui.SetCursorPosY(500)
    imgui.Separator()

    if l_err ~= "" then
        update_msg(l_err)
    end

    -- VER2.0 : Real-time directional message - updated every frame when FM car is active.
    -- Overrides Err_Msg[1] with the current direction of the car relative to the aircraft.
    -- The four zones match the arrow pointer zones exactly:
    --   Front (300-360 / 0-60 deg) : "Follow Me Car is ahead"
    --   Right (60-120 deg)         : "Follow Me Car is on the right"
    --   Rear  (120-240 deg)        : "Follow Me Car is behind"
    --   Left  (240-300 deg)        : "Follow Me Car is on the left"
    if FM_car_active and car_x ~= 0 then
        local l_bear_rt, _ = heading_n_dist(fm_plane_x, fm_plane_z, car_x, car_z)
        local l_rel_rt = l_bear_rt - fm_plane_head
        while l_rel_rt < 0   do l_rel_rt = l_rel_rt + 360 end
        while l_rel_rt >= 360 do l_rel_rt = l_rel_rt - 360 end
        local l_dir_msg
        if l_rel_rt > 300 or l_rel_rt < 60 then
            l_dir_msg = "Follow Me Car is ahead"
        elseif l_rel_rt >= 60 and l_rel_rt < 120 then
            l_dir_msg = "Follow Me Car is on the right"
        elseif l_rel_rt >= 120 and l_rel_rt < 240 then
            l_dir_msg = "Follow Me Car is behind"
        else
            l_dir_msg = "Follow Me Car is on the left"
        end
        Err_Msg[1].text  = l_dir_msg
        Err_Msg[1].color = "GREEN"
    end

    local l_y2 = 16
    for i = 1, 3 do
        imgui.SetCursorPosY(488 + l_y2 * i)
        imgui.SetCursorPosX(10)
        if not Err_Msg[i].text then
            break
        end
        if i == 1 then
            if Err_Msg[i].color == "RED" then
                imgui.PushStyleColor(imgui.constant.Col.Text, 0xFF0000FF)
            else
                imgui.PushStyleColor(imgui.constant.Col.Text, 0xFF00FF00)
            end
        else
            imgui.PushStyleColor(imgui.constant.Col.Text, 0xFF696969)
        end
        imgui.TextUnformatted(Err_Msg[i].text)
        imgui.PopStyleColor()
    end
end

-- ====================================================
-- Function: set_sound_vol
-- Description:
-- Applies the current vol preference (1-10) to all six FM sound effects
-- by calling FlyWithLua's set_sound_gain() with vol/10 as the gain
-- multiplier. Called whenever the pilot moves the volume slider in the
-- UI and on startup when load_config() reads a saved vol value.
-- ====================================================
function set_sound_vol()
    set_sound_gain(snd_arrived, vol / 10)
    set_sound_gain(snd_followme, vol / 10)
    set_sound_gain(snd_safeflight_bye, vol / 10)
    set_sound_gain(snd_welcome, vol / 10)
    set_sound_gain(snd_welcome_bye, vol / 10)
	-- VER1.6 fix : speed warning
    set_sound_gain(snd_keep_speed, vol / 10)
end

-- ====================================================
-- Function: update_msg
-- Description:
-- Posts a status message to the three-line message area at the bottom of
-- the main window. Accepts numeric string codes (e.g. "3", "-14") and
-- maps them to human-readable text. Positive codes produce green text;
-- negative codes produce red text. Shifts the previous message down to
-- line 2 and line 2 to line 3 before setting line 1, giving a scrolling
-- history of the last three events. Duplicate messages and identical
-- runway-error strings are suppressed to avoid visual clutter. Key codes:
--   "3"  = FM Car ready (direction-aware in VER2.0)
--   "5"  = Arrived at destination
--   "6"  = FM Car ready, car spawned behind you
--   "7"  = Have a safe flight (manual cancel)
--   "-14" = No route found
--   "-15" = No taxi network at this airport
-- ====================================================
function update_msg(in_msg)
    if in_msg == nil then
        return
    end

    if tonumber(in_msg) ~= nil and tonumber(in_msg) < 0 then
        Err_Msg[1].color = "RED"
    else
        Err_Msg[1].color = "GREEN"
    end

    if in_msg == "-18" then
        in_msg = "Routes not defined for runway "
        for i = 1, #t_deleted_runway do
            if i == #t_deleted_runway then
                in_msg = in_msg .. t_deleted_runway[i]
            else
                in_msg = in_msg .. t_deleted_runway[i] .. ", "
            end
        end
        if Err_Msg[1].text ~= nil and Err_Msg[1].text == in_msg then
            return
        end
    end

    if in_msg == "-19" then
        if depart_arrive == 1 and sb_origin_icao == curr_ICAO then
            -- Departure: use SimBrief takeoff runway
            l_rwy = sb_runway_takeoff
        elseif depart_arrive == 2 and sb_dest_icao == curr_ICAO then
            -- Arrival: use SimBrief landing runway
            l_rwy = sb_runway_landing
        end
        in_msg = "Routes not defined for Simbrief runway " .. l_rwy
        if Err_Msg[1].text ~= nil and Err_Msg[1].text == in_msg then
            return
        end
    end

    if Err_Msg[1].text then
        Err_Msg[3].text = Err_Msg[2].text
        Err_Msg[2].text = Err_Msg[1].text
    end

    if in_msg == "6" then
        in_msg = "Follow Me Car ready. Car is behind you."
        if depart_arrive == 1 then
            play_sound(snd_followme)
        else
            play_sound(snd_welcome)
        end
    elseif in_msg == "7" then
        in_msg = "Have a safe flight !"
        play_sound(snd_safeflight_bye)
    elseif in_msg == "5" then
        in_msg = "Arrived at destination"
        play_sound(snd_arrived)
    elseif in_msg == "4" then
        in_msg = "No route found. Remove taxiway limitation, trying again."
    elseif in_msg == "3" then
        -- VER2.0 : Replace static "Follow Me Car is ready" with a direction-aware message.
        -- Compute the absolute bearing from the aircraft to the FM car (0-360),
        -- then map it to one of four compass zones relative to the aircraft heading:
        --   Front : bearing within 60 deg on each side of the aircraft nose (300-360 and 0-60)
        --   Right : bearing 60 to 120 deg (right side)
        --   Rear  : bearing 120 to 240 deg (behind)
        --   Left  : bearing 240 to 300 deg (left side)
        if FM_car_active and car_x ~= 0 then
            local l_bear, _ = heading_n_dist(fm_plane_x, fm_plane_z, car_x, car_z)
            local l_rel = l_bear - fm_plane_head
            while l_rel < 0   do l_rel = l_rel + 360 end
            while l_rel >= 360 do l_rel = l_rel - 360 end
            if l_rel > 300 or l_rel < 60 then
                in_msg = "Follow Me Car is ahead"
            elseif l_rel >= 60 and l_rel < 120 then
                in_msg = "Follow Me Car is on the right"
            elseif l_rel >= 120 and l_rel < 240 then
                in_msg = "Follow Me Car is behind"
            else
                -- 240 <= l_rel <= 300
                in_msg = "Follow Me Car is on the left"
            end
        else
            in_msg = "Follow Me Car is ready"
        end
        if depart_arrive == 1 then
            play_sound(snd_followme)
        else
            play_sound(snd_welcome)
        end
    elseif in_msg == "2" then
        in_msg = "Preference Saved"
    elseif in_msg == "-1" then
        in_msg = "Specify the Aircraft Type"
    elseif in_msg == "-2" then
        in_msg = "Preference file not found. Apply default values."
    elseif in_msg == "-4" then
        in_msg = "Unable to complete Save operation"
    elseif in_msg == "-5" then
        if get_from_SimBrief then
            in_msg = "No runway from SimBrief data"
        else
            in_msg = "Select depart runway"
        end
    elseif in_msg == "-6" then
        in_msg = "Select a gate/ramp"
    elseif in_msg == "-11" then
        in_msg = "Unable to locate scenery_packs.ini"
    elseif in_msg == "-12" then
        in_msg = "Unable to find a suitable start pt"
    elseif in_msg == "-13" then
        in_msg = "Unable to find a suitable end pt"
    elseif in_msg == "-14" then
        in_msg = "Unable to find a suitable route"
    elseif in_msg == "-15" then
        in_msg = "Taxi Routes not defined for this airport"
    elseif in_msg == "-16" then
        in_msg = "Can't find start pt. Get off runway and request again"
    elseif in_msg == "-17" then
        -- VER1.5 : New SimBrief error codes
        in_msg = "No suitable gate for this plane. Lift restriction."
    elseif in_msg == "-20" then
        in_msg = "No SimBrief ID configured. Enter your ID below."
    elseif in_msg == "-21" then
        in_msg = "SimBrief connection failed. Check network."
    end

    Err_Msg[1].text = in_msg
end

-- ====================================================
-- Function: determine_XP_route
-- Description:
-- Entry-point validation layer called when the pilot presses "Request
-- Follow Me Car". Checks that the taxi network is loaded, an aircraft
-- type is selected, a mode (departure/arrival) is chosen, a runway is
-- set for departure or a gate is set for arrival, and that the requested
-- runway exists in t_runway[]. Returns an error code string on any
-- failure so build_window() can post it via update_msg(). On success,
-- calls check_gate() to detect if the aircraft is already parked at a
-- known gate, then calls determine_possible_routes() to run the A*
-- pathfinder and build t_node[].
-- ====================================================
function determine_XP_route()
    if #t_taxinode == 0 then
        return "-15"
    end
    if Aircraft_Type == "" then
        return "-1"
    end
    if depart_arrive == 0 then
        return "-5"
    end
    if depart_arrive == 2 and arrival_gate == 0 then
        return "-6"
    end

    if depart_arrive == 1 then
        if depart_runway == "" then
            return "-5"
        end

        -- VER1.4 : Verify the runway exists in t_runway (with valid route)
        local l_found = false
        for i = 1, #t_runway do
            if t_runway[i].ID == depart_runway then
                l_found = true
                break
            end
        end
        if not l_found then
            return "-18"
        end
    end

    depart_gate = check_gate()
    return determine_possible_routes()
end

-- ====================================================
-- Function: determine_possible_routes
-- Description:
-- Resolves the start and end nodes for the FM car route and runs the
-- A* search. For departure, the start node is found from the depart_gate
-- position (or the aircraft position if not at a gate) and the end node
-- is the runway entry node from t_runway[]. For arrival, positions are
-- reversed. Pre-computes A* heuristic (h_value = straight-line distance
-- to end node) for all taxinodes. Calls transverse() once with heading
-- restriction to favour the natural taxi direction; if no route is found,
-- relaxes the restriction and tries again. If still no route, posts
-- message "4" and tries once more without any restrictions. Calls
-- process_possible_routes() to convert the raw A* result into t_node[].
-- ====================================================
function determine_possible_routes()
    local l_startpt_node, l_startpt_x, l_startpt_z = 0, 0, 0, 0
    local l_endpt_node, l_endpt_x, l_endpt_z = 0, 0, 0
    local l_index = 0
    local l_found = false
    local l_rev_heading = add_delta_clockwise(fm_plane_head, 180, 1)

    if depart_gate ~= 0 then
        l_found, l_startpt_node, l_startpt_x, l_startpt_z =
            determine_pos_on_segment(
            t_gate[depart_gate].Heading,
            t_gate[depart_gate].x,
            t_gate[depart_gate].z,
            t_gate[depart_gate].Ramptype
        )
    else
        local l_adj_fm_plane_x = 0
        local l_adj_fm_plane_z = 0
        l_adj_fm_plane_x, l_adj_fm_plane_z = coordinates_of_adjusted_ref(fm_plane_x, fm_plane_z, 0, 60, fm_plane_head)
        l_found, l_startpt_node, l_startpt_x, l_startpt_z =
            determine_pos_on_segment(fm_plane_head, l_adj_fm_plane_x, l_adj_fm_plane_z, "tie_down")
        if l_found == false then
            l_found, l_startpt_node, l_startpt_x, l_startpt_z =
                determine_pos_on_segment(l_rev_heading, fm_plane_x, fm_plane_z, "tie_down")
        end
        if l_found == false and depart_arrive == 2 then
            return "-16"
        end
    end

    if l_found == false then
        return "-12"
    end

    if depart_arrive == 1 then
        for l_index = 1, #t_runway do
            if t_runway[l_index].ID == depart_runway then
                l_endpt_node = t_runway[l_index].Node
                break
            end
        end
		-- VER1.17: is_backtaxi removed
		-- the new universal logic in process_possible_routes() ALWAYS sends the car to the GPS threshold,
		-- whether there is a backtaxi or not. is_backtaxi remains permanently false.
        is_backtaxi = false
    else
        l_found, l_endpt_node, l_endpt_x, l_endpt_z =
            determine_pos_on_segment(
            t_gate[arrival_gate].Heading,
            t_gate[arrival_gate].x,
            t_gate[arrival_gate].z,
            "gate"
        )
        if l_found == false then
            return "-13"
        end
    end

    impose_restriction_chk = true

    for l_index = 1, #t_taxinode do
        _, t_taxinode[l_index].h_value =
            heading_n_dist(
            t_taxinode[l_index].x,
            t_taxinode[l_index].z,
            t_taxinode[l_endpt_node + 1].x,
            t_taxinode[l_endpt_node + 1].z
        )
    end

    if depart_gate ~= 0 then
        transverse(l_startpt_node, l_endpt_node, -1)
    else
        transverse(l_startpt_node, l_endpt_node, fm_plane_head)
        if #t_possible_route == 0 then
            transverse(l_startpt_node, l_endpt_node, -1)
        end
    end

    if #t_possible_route == 0 then
        update_msg("4")
        impose_restriction_chk = false
        transverse(l_startpt_node, l_endpt_node, -1)
        if #t_possible_route == 0 then
            return "-14"
        end
    end

    process_possible_routes()
    return ""
end

-- ====================================================
-- Function: transverse
-- Description:
-- Implements the A* pathfinding algorithm over the apt.dat taxiway graph.
-- Maintains an open list and a closed list of taxinodes. At each step,
-- expands the node with the lowest f_value (g + h) by examining all
-- connected segments. The inner evaluate_node() helper enforces taxiway
-- size restrictions (codes A-E vs Aircraft_Type) and filters already-
-- visited nodes. When the end node is reached, back-traces the parent
-- chain to build t_possible_route[]. The in_heading parameter optionally
-- biases the search away from segments that would require an immediate
-- U-turn from the aircraft's current heading, preventing the car from
-- being routed backward through a gate.
-- ====================================================
function transverse(in_startnode, in_endnode, in_heading)
    function evaluate_node(in_node, in_size, t_open, t_close)
        local l_in_open, l_in_close = false, false
        for i = 1, #t_open do
            if t_open[i].Node == in_node then
                l_in_open = true
                break
            end
        end
        for i = 1, #t_close do
            if t_close[i].Node == in_node then
                l_in_close = true
                break
            end
        end
        if l_in_open == true or l_in_close == true then
            return false
        end
        if not impose_restriction_chk then
            return true
        else
            if in_size ~= "" then
                local l_aircraft_type = tonumber(Aircraft_Type)
                if in_size == "A" then
                    if (l_aircraft_type > 0 and l_aircraft_type < 7) then
                        return false
                    end
                elseif (in_size == "B" or in_size == "C" or in_size == "D") then
                    if (l_aircraft_type > 0 and l_aircraft_type < 3) then
                        return false
                    end
                elseif in_size == "E" then
                    if (l_aircraft_type == 1) then
                        return false
                    end
                end
            end
            return true
        end
        return false
    end

    local t_close, t_open = {}, {}
    local l_pass = false
    local l_curr_node, l_node, l_idx, l_curr_heading, l_AoC = 0, 0, 0, 0, 0
    local l_segment_idx, l_g_value, l_f_value = 0, 0, 0
    local l_string = ""

    t_possible_route = {}
    for l_idx = 1, #t_taxinode do
        t_taxinode[l_idx].f_value = nil
        t_taxinode[l_idx].g_value = nil
        t_taxinode[l_idx].parent = nil
        t_taxinode[l_idx].cost = nil
        t_taxinode[l_idx].heading = nil
    end

    t_taxinode[in_startnode + 1].f_value = t_taxinode[in_startnode + 1].h_value
    if in_heading ~= -1 then
        t_taxinode[in_startnode + 1].heading = in_heading
    end
    l_curr_node = in_startnode

    t_open[1] = {}
    t_open[1].Node = in_startnode
    t_open[1].f_value = t_taxinode[in_startnode + 1].f_value

    while l_curr_node ~= in_endnode and l_curr_node ~= -1 do
        for l_string in t_taxinode[l_curr_node + 1].Segment:gmatch("[^,]+") do
            l_segment_idx = tonumber(l_string)
            l_pass = false
            if
                t_segment[l_segment_idx].Type ~= "runway" or
                    (t_segment[l_segment_idx].Type == "runway" and not impose_restriction_chk)
             then
                if t_segment[l_segment_idx].Node1 == l_curr_node then
                    l_node = t_segment[l_segment_idx].Node2
                    l_pass = evaluate_node(l_node, t_segment[l_segment_idx].Size, t_open, t_close)
                    l_curr_heading = t_segment[l_segment_idx].Heading
                elseif (t_segment[l_segment_idx].Node2 == l_curr_node and t_segment[l_segment_idx].Dir == "twoway") then
                    l_node = t_segment[l_segment_idx].Node1
                    l_pass = evaluate_node(l_node, t_segment[l_segment_idx].Size, t_open, t_close)
                    l_curr_heading = add_delta_clockwise(t_segment[l_segment_idx].Heading, 180, 1)
                end
            end

            if l_pass then
                l_idx = #t_open + 1
                t_open[l_idx] = {}
                t_open[l_idx].Node = l_node
                if t_taxinode[l_curr_node + 1].g_value == nil then
                    l_g_value = t_segment[l_segment_idx].Dist
                else
                    l_g_value = t_taxinode[l_curr_node + 1].g_value + t_segment[l_segment_idx].Dist
                end
                l_f_value = l_g_value + t_taxinode[l_node + 1].h_value
                if t_taxinode[l_node + 1].f_value == nil or l_f_value < t_taxinode[l_node + 1].f_value then
                    t_taxinode[l_node + 1].g_value = l_g_value
                    t_taxinode[l_node + 1].f_value = l_f_value
                    t_taxinode[l_node + 1].parent = l_curr_node
                    if t_taxinode[l_curr_node + 1].heading == nil then
                        t_taxinode[l_node + 1].cost = 0
                    else
                        if t_taxinode[l_curr_node + 1].cost == nil then
                            t_taxinode[l_curr_node + 1].cost = 0
                        end
                        l_AoC = 180 - compute_angle_diff(t_taxinode[l_curr_node + 1].heading, l_curr_heading)
                        if l_AoC < 20 then
                            t_taxinode[l_node + 1].cost = t_taxinode[l_curr_node + 1].cost + 150
                        elseif l_AoC >= 20 and l_AoC <= 80 then
                            t_taxinode[l_node + 1].cost = t_taxinode[l_curr_node + 1].cost + 12
                        elseif l_AoC > 80 and l_AoC < 90 then
                            t_taxinode[l_node + 1].cost = t_taxinode[l_curr_node + 1].cost + 3
                        elseif l_AoC >= 90 and l_AoC <= 140 then
                            t_taxinode[l_node + 1].cost = t_taxinode[l_curr_node + 1].cost + 1
                        elseif l_AoC > 140 and l_AoC <= 170 then
                            t_taxinode[l_node + 1].cost = t_taxinode[l_curr_node + 1].cost + 1
                        elseif l_AoC > 170 then
                            t_taxinode[l_node + 1].cost = t_taxinode[l_curr_node + 1].cost
                        end
                    end
                    t_taxinode[l_node + 1].heading = l_curr_heading
                    t_open[l_idx].f_value = l_f_value
                    t_open[l_idx].cost = t_taxinode[l_node + 1].cost
                end
            end
        end

        for l_idx = #t_open, 1, -1 do
            if t_open[l_idx].Node == l_curr_node then
                table.remove(t_open, l_idx)
                break
            end
        end
        l_idx = #t_close + 1
        t_close[l_idx] = {}
        t_close[l_idx].Node = l_curr_node

        table.sort(
            t_open,
            function(a, b)
                return a.f_value > b.f_value
            end
        )
        for l_idx = #t_open, 1, -1 do
            if l_idx == #t_open then
                l_f_value = t_open[l_idx].f_value
                t_open[l_idx].order = 1
            else
                if math.abs(t_open[l_idx].f_value - l_f_value) < 300 then
                    t_open[l_idx].order = 1
                else
                    t_open[l_idx].order = 2
                end
            end
        end
        table.sort(
            t_open,
            function(a, b)
                return a.order > b.order or (a.order == b.order and a.cost > b.cost)
            end
        )
        if #t_open > 0 then
            l_curr_node = t_open[#t_open].Node
        else
            l_curr_node = -1
        end
    end

    if t_taxinode[in_endnode + 1].f_value ~= nil then
        t_possible_route[1] = {}
        t_possible_route[1].Dist = t_taxinode[in_endnode + 1].g_value
        t_possible_route[1].Cost = t_taxinode[in_endnode + 1].cost
        l_node = in_endnode
        t_possible_route[1].Route = in_endnode
        while l_node ~= in_startnode do
            l_node = t_taxinode[l_node + 1].parent
            t_possible_route[1].Route = l_node .. " " .. t_possible_route[1].Route
        end
        local l_from_label, l_to_label = "", ""
        if depart_arrive == 1 then
            l_from_label = (depart_gate > 0) and t_gate[depart_gate].ID or "ramp"
            l_to_label = "RWY " .. depart_runway
        elseif depart_arrive == 2 then
            l_from_label = "RWY " .. depart_runway
            l_to_label = (arrival_gate > 0) and t_gate[arrival_gate].ID or "gate"
        end
        logMsg(
            "FollowMe : RAW A* ROUTE" ..
                " AIRPORT=" .. curr_ICAO .. " FROM=" .. l_from_label .. " TO=" .. l_to_label
        )
        logMsg("FollowMe : RAW A* NODES=[ " .. t_possible_route[1].Route .. " ]")
        local l_raw_count = 0
        for _ in t_possible_route[1].Route:gmatch("[^%s]+") do
            l_raw_count = l_raw_count + 1
        end
        logMsg("FollowMe : RAW A* NODES TOTAL=" .. l_raw_count)
    end
end

-- ====================================================
-- Function: process_possible_routes
-- Description:
-- Converts the raw A* node sequence (t_possible_route[]) into the final
-- drive waypoint list (t_node[]) used by the car physics. Copies each
-- node's local coordinates, computes per-leg headings and distances via
-- heading_n_dist(), and inserts temporary virtual nodes and new segments
-- at the start and end positions found by determine_pos_on_segment().
-- For departure routes, appends the centreline projection node and the
-- exact GPS runway threshold node (VER1.17 / VER1.24) so the car always
-- stops at the correct threshold regardless of intersecting runways.
-- Cleans up any temporary "New" taxinodes and "ADD_NEWSEGMENT" entries
-- added during the route build. Logs the complete drive route with GPS
-- lat/lon for each waypoint (VER1.22 / VER1.24).
-- ====================================================
function process_possible_routes()
    if #t_possible_route == 0 then
        return
    end

    local l_new_head, l_new_dist = 0, 0
    local l_index = 1
    local l_node = 0
    t_node = {}
    local l_min_dist_btw_nodes = min_rot_radius * 2 + car_rear_wheel_to_ref
    local t_route_nodes = {}
    local routenode_cnt = 1

    for l_node in t_possible_route[1].Route:gmatch("[^%s]+") do
        t_route_nodes[routenode_cnt] = l_node
        routenode_cnt = routenode_cnt + 1
    end

	-- VER1.20: Scan l_last_taxiway and break hotzone removed.
	-- All A* nodes are inserted as DRIVE NODE unconditionally.
	-- VER1.17 handles runway entry independently (centerline projection + GPS threshold).
    for routenode_cnt = 1, #t_route_nodes do
        l_node = t_route_nodes[routenode_cnt]
        if l_index > 1 then
            l_new_head, l_new_dist =
                heading_n_dist(
                t_node[l_index - 1].x,
                t_node[l_index - 1].z,
                t_taxinode[l_node + 1].x,
                t_taxinode[l_node + 1].z
            )
        end
        t_node[l_index] = {}
        t_node[l_index].hotzone = ""
        t_node[l_index].x = t_taxinode[l_node + 1].x
        t_node[l_index].y = t_taxinode[l_node + 1].y
        t_node[l_index].z = t_taxinode[l_node + 1].z
        if l_index > 1 then
            t_node[l_index - 1].heading = l_new_head
            t_node[l_index - 1].dist = l_new_dist
        end
        l_index = l_index + 1
    end

    for l_index = 1, #t_node - 2 do
        if t_node[l_index].dist < l_min_dist_btw_nodes then
            t_node[l_index + 1].x, t_node[l_index + 1].z =
                coordinates_of_adjusted_ref(
                t_node[l_index + 1].x,
                t_node[l_index + 1].z,
                0,
                (l_min_dist_btw_nodes - t_node[l_index].dist),
                t_node[l_index].heading
            )
            t_node[l_index].dist = l_min_dist_btw_nodes
            t_node[l_index + 1].heading, t_node[l_index + 1].dist =
                heading_n_dist(
                t_node[l_index + 1].x,
                t_node[l_index + 1].z,
                t_node[l_index + 2].x,
                t_node[l_index + 2].z
            )
        end
    end

    while t_taxinode[#t_taxinode].Type == "New" do
        t_taxinode[#t_taxinode] = nil
    end

    while t_segment[#t_segment].ID == "ADD_NEWSEGMENT" do
        if t_taxinode[t_segment[#t_segment].Node1 + 1] ~= nil then
            t_taxinode[t_segment[#t_segment].Node1 + 1].Segment =
                string.gsub(t_taxinode[t_segment[#t_segment].Node1 + 1].Segment, "," .. tostring(#t_segment), "")
        end
        if t_taxinode[t_segment[#t_segment].Node2 + 1] ~= nil then
            t_taxinode[t_segment[#t_segment].Node2 + 1].Segment =
                string.gsub(t_taxinode[t_segment[#t_segment].Node2 + 1].Segment, "," .. tostring(#t_segment), "")
        end
        t_segment[#t_segment] = nil
    end

    -- VER1.17 : Universal runway threshold - for ALL departures (back-taxi or not),
    -- replace the runway portion of the A* route with a direct GPS path to the
    -- exact runway threshold. This eliminates wrong-runway stops at intersecting
    -- runways (e.g. KSYR RWY 28 vs 33) and the diagonal-into-grass bug.
    -- The last t_node is the taxiway/runway junction (edge of runway). We project
    -- it perpendicularly onto the centreline, then add the GPS threshold node.
    -- Result: car follows taxiways normally, arrives on centreline, drives straight
    -- to threshold. Universal - no special case needed for back-taxi.
    -- VER1.24 : use the exact paired opposite threshold (same apt.dat line 100) to
    -- build the centreline axis, instead of "any other runway entry" which could
    -- hit a different physical runway at airports with multiple parallel runways.
    if depart_arrive == 1 and #t_node > 0 then
        local l_rwy_x, l_rwy_y, l_rwy_z = 0, 0, 0
        local l_far_x, l_far_z = 0, 0
        local l_rwy_idx = 0

        -- Find the departure runway entry and its exact opposite threshold (Pair)
        for l_index = 1, #t_runway do
            if t_runway[l_index].ID == depart_runway then
                l_rwy_idx = l_index
                l_rwy_x = t_runway[l_index].x
                l_rwy_y = probe_y(t_runway[l_index].x, 0, t_runway[l_index].z)
                l_rwy_z = t_runway[l_index].z
                break
            end
        end

        -- VER1.24 : use the Pair index set by decipher_runway() for an exact centreline.
        -- Fallback: if Pair is not set (shouldn't happen) search by proximity.
        if l_rwy_idx > 0 and t_runway[l_rwy_idx].Pair ~= nil then
            local l_pair_idx = t_runway[l_rwy_idx].Pair
            if t_runway[l_pair_idx] ~= nil then
                l_far_x = t_runway[l_pair_idx].x
                l_far_z = t_runway[l_pair_idx].z
                logMsg(
                    "FollowMe : RWY " ..
                        depart_runway ..
                            " centreline from pair idx=" ..
                                l_pair_idx ..
                                    " ID=" ..
                                        (t_runway[l_pair_idx].ID or "?") ..
                                            " lat=" ..
                                                string.format("%.6f", t_runway[l_pair_idx].Lat or 0) ..
                                                    " lon=" .. string.format("%.6f", t_runway[l_pair_idx].Lon or 0)
                )
            end
        end

        -- Safety: if l_far is still 0,0 (Pair missing), fall back to old scan
        if l_far_x == 0 and l_far_z == 0 then
            for l_index = 1, #t_runway do
                if t_runway[l_index].ID ~= depart_runway then
                    l_far_x = t_runway[l_index].x
                    l_far_z = t_runway[l_index].z
                end
            end
            logMsg("FollowMe : RWY " .. depart_runway .. " Pair fallback used (scan)")
        end

        -- Centreline unit vector: far threshold -> near threshold (depart_runway)
        local l_ax = l_rwy_x - l_far_x
        local l_az = l_rwy_z - l_far_z
        local l_alen = math.sqrt(l_ax * l_ax + l_az * l_az)
        if l_alen < 1 then
            l_alen = 1
        end
        local l_ux = l_ax / l_alen
        local l_uz = l_az / l_alen

        -- Project last t_node (taxiway/runway junction) onto centreline
        local l_edge_x = t_node[#t_node].x
        local l_edge_z = t_node[#t_node].z
        local l_vx = l_edge_x - l_far_x
        local l_vz = l_edge_z - l_far_z
        local l_t = l_vx * l_ux + l_vz * l_uz
        local l_proj_x = l_far_x + l_t * l_ux
        local l_proj_z = l_far_z + l_t * l_uz
        local _, l_offset = heading_n_dist(l_edge_x, l_edge_z, l_proj_x, l_proj_z)
        local _, l_proj_to_thr = heading_n_dist(l_proj_x, l_proj_z, l_rwy_x, l_rwy_z)

        if l_offset > 2 and l_proj_to_thr > 10 then
            -- Junction is off-centreline -> replace last node with centreline projection
            local l_proj_y = probe_y(l_proj_x, 0, l_proj_z)
            t_node[#t_node].x = l_proj_x
            t_node[#t_node].y = l_proj_y
            t_node[#t_node].z = l_proj_z
            t_node[#t_node].hotzone = "1"
            if #t_node >= 2 then
                t_node[#t_node - 1].heading, t_node[#t_node - 1].dist =
                    heading_n_dist(t_node[#t_node - 1].x, t_node[#t_node - 1].z, l_proj_x, l_proj_z)
            end
        else
            -- Already on centreline (or very close)
            t_node[#t_node].hotzone = "1"
        end

        -- Final node: exact GPS threshold
        local l_last = #t_node
        local _, l_gap = heading_n_dist(t_node[l_last].x, t_node[l_last].z, l_rwy_x, l_rwy_z)
        if l_gap > 5 then
            t_node[l_last].heading, t_node[l_last].dist =
                heading_n_dist(t_node[l_last].x, t_node[l_last].z, l_rwy_x, l_rwy_z)
            t_node[l_last + 1] = {}
            t_node[l_last + 1].x = l_rwy_x
            t_node[l_last + 1].y = l_rwy_y
            t_node[l_last + 1].z = l_rwy_z
            t_node[l_last + 1].hotzone = "1"
            t_node[l_last + 1].heading = t_node[l_last].heading
            t_node[l_last + 1].dist = 0
        end
    end

    -- VER1.22 : log the final drive waypoints actually followed by the car
    if #t_node > 0 then
        local l_from_lbl, l_to_lbl = "", ""
        if depart_arrive == 1 then
            l_from_lbl = (depart_gate > 0) and t_gate[depart_gate].ID or "ramp"
            l_to_lbl = "RWY " .. depart_runway
        elseif depart_arrive == 2 then
            l_from_lbl = "RWY " .. depart_runway
            l_to_lbl = (arrival_gate > 0) and t_gate[arrival_gate].ID or "gate"
        end
        local l_node_list = ""
        for l_ni = 1, #t_node do
            local l_hz = (t_node[l_ni].hotzone == "1") and "[H]" or ""
            -- VER1.24 : also log GPS lat/lon for each waypoint
            local l_gps_lat, l_gps_lon = 0.0, 0.0
            if t_node[l_ni].x ~= nil and t_node[l_ni].y ~= nil and t_node[l_ni].z ~= nil then
                l_gps_lat, _, l_gps_lon = local_to_latlon(t_node[l_ni].x, t_node[l_ni].y or 0, t_node[l_ni].z)
            end
            l_node_list =
                l_node_list ..
                l_ni ..
                    l_hz ..
                        "(lat=" ..
                            string.format("%.6f", l_gps_lat) ..
                                " lon=" ..
                                    string.format("%.6f", l_gps_lon) ..
                                        " x=" ..
                                            string.format("%.1f", t_node[l_ni].x or 0) ..
                                                " z=" ..
                                                    string.format("%.1f", t_node[l_ni].z or 0) ..
                                                        " hdg=" ..
                                                            string.format("%.0f", t_node[l_ni].heading or 0) ..
                                                                " d=" ..
                                                                    string.format("%.0f", t_node[l_ni].dist or 0) ..
                                                                        "m) "
        end
        logMsg(
            "FollowMe : DRIVE ROUTE" .. " AIRPORT=" .. curr_ICAO .. " FROM=" .. l_from_lbl .. " TO=" .. l_to_lbl
        )
        logMsg("FollowMe : DRIVE NODES : " .. l_node_list)
        logMsg("FollowMe : DRIVE NODES TOTAL=" .. #t_node)
    end
end

-- ====================================================
-- Function: determine_pos_on_segment
-- Description:
-- Finds the nearest reachable taxiway connection point for a given
-- position and heading (gate exit, aircraft nose, or runway entry).
-- Searches all t_segment[] entries for the closest perpendicular
-- intersection point, the closest dead-end node, or the closest tangent
-- point (for tie-down / hangar types) within a heading-restricted cone.
-- Returns the winning candidate as a new virtual taxinode inserted at
-- the intersection. If no segment is reachable (e.g. CYQB isolated
-- parking nodes with no 1202 segments), falls back to the nearest
-- connected node within 500 m (VER1.15 universal fallback). This is the
-- function that bridges the gap between real-world parking positions and
-- the apt.dat taxiway graph.
-- ====================================================
function determine_pos_on_segment(in_heading, in_x, in_z, in_type)
    local l_within_sight, l_intersect_x, l_intersect_z, l_dist_to_intersect = false, 0, 0, 0
    local l_ret_intersect_dist = 9999
    local l_ret_segment_index, l_ret_x, l_ret_z = 0, 0, 0
    local l_deadnode, l_dist_to_deadnode
    local l_ret_node_dist = 9999
    local l_ret_nodesegment_index, l_ret_deadnode = 0, 0
    local l_in_rev_heading = 0
    local l_goto_node = 0
    local l_min_dist = 25
    local l_max_dist = 300
    local l_max_dist_node = 130
    local l_aircraft_type = tonumber(Aircraft_Type)

    if l_aircraft_type == 0 or l_aircraft_type >= 7 then
        l_min_dist = 5
    elseif l_aircraft_type >= 3 and l_aircraft_type < 7 then
        l_min_dist = 15
    elseif l_aircraft_type == 2 then
        l_min_dist = 30
    elseif l_aircraft_type == 1 then
        l_min_dist = 30
    end

    for i = 1, #t_segment do
        if (in_type == "gate" or in_type == "misc") then
            l_within_sight, l_intersect_x, l_intersect_z, l_dist_to_intersect =
                compute_intersection(in_type, in_heading, in_x, in_z, i)
            if l_within_sight and l_dist_to_intersect <= l_max_dist then
                if l_dist_to_intersect <= l_ret_intersect_dist then
                    l_ret_intersect_dist = l_dist_to_intersect
                    l_ret_segment_index = i
                    l_ret_x = l_intersect_x
                    l_ret_z = l_intersect_z
                end
            end
            l_within_sight, l_deadnode, l_dist_to_deadnode = check_deadend_node(in_type, in_heading, in_x, in_z, i)
            if l_within_sight then
                if l_dist_to_deadnode >= l_min_dist and l_dist_to_deadnode <= l_ret_node_dist then
                    l_ret_deadnode = l_deadnode
                    l_ret_nodesegment_index = i
                    l_ret_node_dist = l_dist_to_deadnode
                end
            end
        end

        if (in_type == "tie_down" or in_type == "hangar") then
            l_within_sight, l_deadnode, l_dist_to_deadnode = check_deadend_node(in_type, in_heading, in_x, in_z, i)
            if l_within_sight then
                if l_dist_to_deadnode <= l_max_dist_node and l_dist_to_deadnode <= l_ret_node_dist then
                    l_ret_deadnode = l_deadnode
                    l_ret_nodesegment_index = i
                    l_ret_node_dist = l_dist_to_deadnode
                end
            end
            l_within_sight, l_intersect_x, l_intersect_z, l_dist_to_intersect =
                compute_intersection(in_type, in_heading, in_x, in_z, i)
            if l_within_sight and l_dist_to_intersect <= l_max_dist then
                if l_dist_to_intersect <= l_ret_intersect_dist then
                    l_ret_intersect_dist = l_dist_to_intersect
                    l_ret_segment_index = i
                    l_ret_x = l_intersect_x
                    l_ret_z = l_intersect_z
                end
            end
            l_within_sight, l_tangent_dist, l_tangent_x, l_tangent_z, l_goto_node =
                compute_tangent_dist(in_heading, in_x, in_z, i)
            if l_within_sight and l_tangent_dist <= l_ret_intersect_dist and l_tangent_dist <= l_max_dist_node then
                l_ret_intersect_dist = l_tangent_dist
                l_ret_segment_index = i
                l_ret_x = l_tangent_x
                l_ret_z = l_tangent_z
            end
        end
    end

    local l_decision = 0
    if l_ret_node_dist < 9999 and l_ret_intersect_dist < 9999 then
        if l_ret_intersect_dist <= l_ret_node_dist then
            l_decision = 1
        else
            l_decision = 2
        end
    elseif l_ret_intersect_dist < 9999 then
        l_decision = 1
    elseif l_ret_node_dist < 9999 then
        l_decision = 2
    end

    if l_decision == 0 then
        -- VER1.15 : Universal fallback - when no segment is reachable from the gate
        -- (e.g. CYQB has 88 isolated nodes near gates, causing "no taxiway"),
        -- find the nearest CONNECTED node (has at least one segment) instead.
        -- This works universally for all airports regardless of apt.dat quality.
        if in_type == "gate" or in_type == "misc" or in_type == "tie_down" or in_type == "hangar" then
            local l_best_d = 99999
            local l_best_nid = -1
            for l_ni = 1, #t_taxinode do
                if t_taxinode[l_ni] and t_taxinode[l_ni].Segment and t_taxinode[l_ni].Segment ~= "" then
                    local l_dd = math.sqrt((in_x - t_taxinode[l_ni].x) ^ 2 + (in_z - t_taxinode[l_ni].z) ^ 2)
                    if l_dd < l_best_d and l_dd < 500 then
                        l_best_d = l_dd
                        l_best_nid = l_ni - 1
                    end
                end
            end
            if l_best_nid >= 0 then
                return true, l_best_nid, t_taxinode[l_best_nid + 1].x, t_taxinode[l_best_nid + 1].z, 0
            end
        end
        return false, 0, 0, 0, 0
    elseif l_decision == 1 then
        local l_new_node, l_new_segment =
            add_new_taxinode_segment(l_ret_segment_index, l_ret_x, l_ret_z, l_ret_intersect_dist)
        return true, l_new_node, l_ret_x, l_ret_z, l_new_segment
    elseif l_decision == 2 then
        return true, l_ret_deadnode, 0, 0, 0
    end
end

-- ====================================================
-- Function: compute_tangent_dist
-- Description:
-- Computes the perpendicular distance from a tie-down or hangar position
-- to a taxiway segment and the coordinates of the foot of that
-- perpendicular (the tangent point). First checks which segment endpoint
-- is visible within a 120-deg forward cone from the given heading to
-- ensure the result is in the direction of travel. Validates that the
-- tangent point actually falls between the two segment endpoints so the
-- car does not snap to a virtual point outside the physical taxiway.
-- Used by determine_pos_on_segment() for small-GA parking spots where
-- no direct intersection exists.
-- ====================================================
function compute_tangent_dist(in_heading, in_x, in_z, in_idx)
    local l_angle, l_dir, l_tangent_heading = 0, 0, 0
    local l_head, l_dist = 0, 0
    local l_tangent_dist, l_tangent_x, l_tangent_z, l_goto_node = 0, 0, 0, 0
    local l_in_sight = false
    local l_node1_x = t_taxinode[t_segment[in_idx].Node1 + 1].x
    local l_node1_z = t_taxinode[t_segment[in_idx].Node1 + 1].z
    local l_node2_x = t_taxinode[t_segment[in_idx].Node2 + 1].x
    local l_node2_z = t_taxinode[t_segment[in_idx].Node2 + 1].z
    local l_seg_heading = t_segment[in_idx].Heading

    l_in_sight = chk_line_of_sight(in_heading, 60, 60, in_x, in_z, l_node2_x, l_node2_z)
    if l_in_sight then
        l_goto_node = t_segment[in_idx].Node2
    else
        l_in_sight = chk_line_of_sight(in_heading, 60, 60, in_x, in_z, l_node1_x, l_node1_z)
        if l_in_sight then
            l_goto_node = t_segment[in_idx].Node1
        else
            return false, 0, 0, 0, 0
        end
    end

    l_head, l_dist = heading_n_dist(in_x, in_z, l_node2_x, l_node2_z)
    l_angle, l_dir = compute_angle_diff(l_seg_heading, l_head)
    l_tangent_dist = math.abs(math.sin(math.rad(l_angle)) * l_dist)
    l_tangent_heading = add_delta_clockwise(l_seg_heading, 90, l_dir)
    l_tangent_x = in_x + math.sin(math.rad(l_tangent_heading)) * l_tangent_dist
    l_tangent_z = in_z + math.cos(math.rad(l_tangent_heading)) * l_tangent_dist * -1

    if
        ((l_tangent_x >= l_node1_x and l_tangent_x <= l_node2_x) or
            (l_tangent_x >= l_node2_x and l_tangent_x <= l_node1_x)) or
            ((l_tangent_z >= l_node1_z and l_tangent_z <= l_node2_z) or
                (l_tangent_z >= l_node2_z and l_tangent_z <= l_node1_z))
     then
        return true, l_tangent_dist, l_tangent_x, l_tangent_z, l_goto_node
    else
        return false, 0, 0, 0, 0
    end
end

-- ====================================================
-- Function: compute_intersection
-- Description:
-- Computes the exact intersection point between a ray cast from position
-- (in_x, in_z) along in_heading and a taxiway segment defined by its
-- two endpoint nodes. Handles the degenerate cases where the ray is
-- exactly north/south or east/west. Returns a boolean (intersection
-- found within segment bounds), the intersection coordinates, and the
-- distance from the origin. Used by determine_pos_on_segment() for gate
-- and miscellaneous position types to find where the gate exit heading
-- crosses the nearest taxiway.
-- ====================================================
function compute_intersection(in_type, in_heading, in_x, in_z, in_idx)
    local l_node1_x = t_taxinode[t_segment[in_idx].Node1 + 1].x
    local l_node1_z = t_taxinode[t_segment[in_idx].Node1 + 1].z
    local l_node2_x = t_taxinode[t_segment[in_idx].Node2 + 1].x
    local l_node2_z = t_taxinode[t_segment[in_idx].Node2 + 1].z
    local l_seg_heading = t_segment[in_idx].Heading

    if l_seg_heading == in_heading or l_seg_heading == add_delta_clockwise(in_heading, 180, 1) then
        return false, 0, 0, 0
    end

    local l_intersect_x, l_intersect_z = 0, 0

    if in_heading == 0 or in_heading == 180 then
        l_intersect_z = math.tan(math.rad(90 - l_seg_heading)) * (in_x - l_node1_x) + (l_node1_z * -1)
        l_intersect_z = -1 * l_intersect_z
        l_intersect_x = in_x
    elseif l_seg_heading == 0 or l_seg_heading == 180 then
        l_intersect_z = math.tan(math.rad(90 - in_heading)) * (l_node1_x - in_x) + (in_z * -1)
        l_intersect_z = -1 * l_intersect_z
        l_intersect_x = l_node1_x
    else
        local l_heading_ratio = math.tan(math.rad(90 - in_heading)) / math.tan(math.rad(90 - l_seg_heading))
        local l_var1 = -1 * (l_heading_ratio * (l_node1_z * -1))
        local l_var2 = math.tan(math.rad(90 - in_heading)) * (l_node1_x - in_x)
        local l_var3 = in_z * -1
        local l_var4 = 1 - l_heading_ratio
        l_intersect_z = -1 * ((l_var1 + l_var2 + l_var3) / l_var4)
        l_var1 = -1 * (1 / l_heading_ratio) * l_node1_x
        l_var2 = -1 * (l_node1_z - in_z) / math.tan(math.rad(90 - in_heading))
        l_var3 = in_x
        l_var4 = 1 - (1 / l_heading_ratio)
        l_intersect_x = (l_var1 + l_var2 + l_var3) / l_var4
    end

    if
        ((l_intersect_x >= l_node1_x and l_intersect_x <= l_node2_x) or
            (l_intersect_x >= l_node2_x and l_intersect_x <= l_node1_x)) and
            ((l_intersect_z >= l_node1_z and l_intersect_z <= l_node2_z) or
                (l_intersect_z >= l_node2_z and l_intersect_z <= l_node1_z))
     then
    else
        return false, 0, 0, 0
    end

    local l_heading = 0
    if in_type == "gate" or in_type == "misc" then
        l_heading = add_delta_clockwise(in_heading, 180, 1)
    else
        l_heading = in_heading
    end

    local l_in_sight, _, l_dist_to_intersect =
        chk_line_of_sight(l_heading, 70, 70, in_x, in_z, l_intersect_x, l_intersect_z)

    if l_in_sight == false and (in_type == "tie_down" or in_type == "hangar") then
        l_heading = add_delta_clockwise(in_heading, 180, 1)
        l_in_sight, _, l_dist_to_intersect =
            chk_line_of_sight(l_heading, 70, 70, in_x, in_z, l_intersect_x, l_intersect_z)
    end

    return l_in_sight, l_intersect_x, l_intersect_z, l_dist_to_intersect
end

-- ====================================================
-- Function: check_deadend_node
-- Description:
-- Tests whether a taxiway segment has a dead-end node (a node connected
-- to only one segment, i.e. a stub or gate finger) and whether that
-- node falls within the heading cone from the current position. A node
-- is considered a dead-end when its Segment index string contains no
-- comma (only one segment reference). For gate and misc types, the check
-- is done from the reverse heading so the car exits toward the taxiway.
-- Used by determine_pos_on_segment() to snap the start or end point to
-- a dead-end node at a gate stub rather than computing a perpendicular
-- intersection point on the segment.
-- ====================================================
function check_deadend_node(in_type, in_heading, in_x, in_z, in_idx)
    local l_deadnode = -1
    if not string.find(t_taxinode[t_segment[in_idx].Node1 + 1].Segment, ",") then
        l_deadnode = t_segment[in_idx].Node1
    elseif not string.find(t_taxinode[t_segment[in_idx].Node2 + 1].Segment, ",") then
        l_deadnode = t_segment[in_idx].Node2
    end

    if l_deadnode ~= -1 then
        local l_in_rev_heading = 0
        local l_node_x = t_taxinode[l_deadnode + 1].x
        local l_node_z = t_taxinode[l_deadnode + 1].z
        local l_heading = 0
        l_in_rev_heading = add_delta_clockwise(in_heading, 180, 1)
        if in_type == "gate" or in_type == "misc" then
            l_heading = l_in_rev_heading
        else
            l_heading = in_heading
        end
        local l_in_sight, _, l_dist_to_node = chk_line_of_sight(l_heading, 80, 80, in_x, in_z, l_node_x, l_node_z)
        return l_in_sight, l_deadnode, l_dist_to_node
    else
        return false, 0, 0
    end
end

-- ====================================================
-- Function: add_new_taxinode_segment
-- Description:
-- Inserts a virtual taxinode at a computed intersection or tangent point
-- on an existing taxiway segment, splitting that segment into two halves.
-- Creates a new t_taxinode[] entry at (in_x, in_z) typed "New", then
-- creates two new t_segment[] entries ("ADD_NEWSEGMENT") connecting the
-- new node to each original endpoint. Updates the Segment index strings
-- of all affected nodes so the A* search can traverse through the new
-- virtual node. These temporary nodes and segments are removed by
-- process_possible_routes() after the route is built. Returns the index
-- of the new node and the new segment for use as start/end point in the
-- pathfinder.
-- ====================================================
function add_new_taxinode_segment(in_segment_index, in_x, in_z, in_intersect_dist)
    local l_new_node, l_new_segment = 0, 0
    local l_idx = #t_taxinode + 1
    t_taxinode[l_idx] = {}
    t_taxinode[l_idx].x = in_x
    t_taxinode[l_idx].y = t_taxinode[t_segment[in_segment_index].Node2 + 1].y
    t_taxinode[l_idx].z = in_z
    t_taxinode[l_idx].Type = "New"
    t_taxinode[l_idx].Runway = ""
    t_taxinode[l_idx].Segment = ""
    t_taxinode[l_idx].f_value = nil
    t_taxinode[l_idx].g_value = nil
    t_taxinode[l_idx].h_value = nil
    t_taxinode[l_idx].parent = nil
    t_taxinode[l_idx].cost = nil
    t_taxinode[l_idx].heading = nil
    l_new_node = l_idx - 1

    if string.find(t_taxinode[t_segment[in_segment_index].Node1 + 1].Segment, ",") then
        l_new_segment = #t_segment + 1
        t_segment[l_new_segment] = {}
        t_segment[l_new_segment].ID = "ADD_NEWSEGMENT"
        t_segment[l_new_segment].Node1 = t_segment[in_segment_index].Node1
        t_segment[l_new_segment].Node2 = l_new_node
        t_segment[l_new_segment].Dir = t_segment[in_segment_index].Dir
        t_segment[l_new_segment].Type = t_segment[in_segment_index].Type
        t_segment[l_new_segment].Size = t_segment[in_segment_index].Size
        t_segment[l_new_segment].Hotzone = t_segment[in_segment_index].Hotzone
        t_segment[l_new_segment].Heading, t_segment[l_new_segment].Dist =
            heading_n_dist(
            t_taxinode[t_segment[in_segment_index].Node1 + 1].x,
            t_taxinode[t_segment[in_segment_index].Node1 + 1].z,
            in_x,
            in_z
        )
        t_taxinode[t_segment[in_segment_index].Node1 + 1].Segment =
            t_taxinode[t_segment[in_segment_index].Node1 + 1].Segment .. "," .. tostring(l_new_segment)
        t_taxinode[l_idx].Segment = tostring(l_new_segment)
    end

    if string.find(t_taxinode[t_segment[in_segment_index].Node2 + 1].Segment, ",") then
        l_new_segment = #t_segment + 1
        t_segment[l_new_segment] = {}
        t_segment[l_new_segment].ID = "ADD_NEWSEGMENT"
        t_segment[l_new_segment].Node1 = l_new_node
        t_segment[l_new_segment].Node2 = t_segment[in_segment_index].Node2
        t_segment[l_new_segment].Dir = t_segment[in_segment_index].Dir
        t_segment[l_new_segment].Type = t_segment[in_segment_index].Type
        t_segment[l_new_segment].Size = t_segment[in_segment_index].Size
        t_segment[l_new_segment].Hotzone = t_segment[in_segment_index].Hotzone
        t_segment[l_new_segment].Heading, t_segment[l_new_segment].Dist =
            heading_n_dist(
            in_x,
            in_z,
            t_taxinode[t_segment[in_segment_index].Node2 + 1].x,
            t_taxinode[t_segment[in_segment_index].Node2 + 1].z
        )
        t_taxinode[t_segment[in_segment_index].Node2 + 1].Segment =
            t_taxinode[t_segment[in_segment_index].Node2 + 1].Segment .. "," .. tostring(l_new_segment)
        if t_taxinode[l_idx].Segment == "" then
            t_taxinode[l_idx].Segment = tostring(l_new_segment)
        else
            t_taxinode[l_idx].Segment = t_taxinode[l_idx].Segment .. "," .. tostring(l_new_segment)
        end
    end
    return l_new_node, l_new_segment
end

-- ====================================================
-- Function: auto_assign_gate
-- Description:
-- Randomly selects a gate suitable for the current aircraft type. Builds
-- a list of t_gate[] entries whose Types field contains Aircraft_Type,
-- then picks one at random (randomised by os.time()). If no suitable gate
-- exists (e.g. a GA plane at an airport with only heavy gates), falls
-- back to a random gate from the full list and posts warning "-17". Also
-- used when the pilot selects "Arrival" mode with "Auto Assign" checked,
-- in which case the result is immediately shown in the gate text field.
-- Returns "2" for a type-matched gate or "1" for a fallback.
-- ====================================================
function auto_assign_gate()
    local l_index = 0
    if #t_gate == 0 then
        arrival_gate = 0
        return "-1"
    end

    t_suitable_gates = {}
    for l_index = 1, #t_gate do
        if
            string.match(t_gate[l_index].Types, Aircraft_Type) == Aircraft_Type or
                (Aircraft_Type == "0" and string.match(t_gate[l_index].Types, "7"))
         then
            t_suitable_gates[#t_suitable_gates + 1] = l_index
        end
    end
    rampstart_chg = true
    math.randomseed(os.time())
    if #t_suitable_gates > 0 then
        arrival_gate = t_suitable_gates[math.random(1, #t_suitable_gates)]
        gatetext = t_gate[arrival_gate].ID
        return "2"
    else
        arrival_gate = math.random(1, #t_gate)
        gatetext = t_gate[arrival_gate].ID
        if (Err_Msg[1].text ~= nil and string.find(Err_Msg[1].text, "No suitable gate for this plane. Lift")) then
        else
            update_msg("-17")
        end
        return "1"
    end
end

-- ====================================================
-- Function: check_gate
-- Description:
-- Detects whether the aircraft is currently parked at a known apt.dat
-- gate within 10 m. Only active when the aircraft is stationary (ground
-- speed < 0.5 m/s). Iterates t_gate[] to find the nearest entry and
-- returns its index if it is within the 10 m threshold, otherwise
-- returns 0. Called from determine_XP_route() to pre-fill depart_gate
-- so the route start point is anchored to the gate rather than the
-- aircraft nose, ensuring a clean exit heading from the stand.
-- ====================================================
function check_gate()
    local l_dist = 0
    local l_dist_min = 9999
    local l_navid_index = 9999
    if #t_gate == 0 then
        return 0
    end
    if fm_gnd_spd < 0.5 then
        for i = 1, #t_gate do
            _, l_dist = heading_n_dist(fm_plane_x, fm_plane_z, t_gate[i].x, t_gate[i].z)
            if l_dist < l_dist_min then
                l_dist_min = l_dist
                l_navid_index = i
            end
        end
        if l_dist_min <= 10 then
            rampstart_chg = true
            return l_navid_index
        else
            return 0
        end
    else
        return 0
    end
end

-- ====================================================
-- Function: closed_window
-- Description:
-- FlyWithLua callback invoked when the main FollowMe control panel is
-- closed by the user (X button). Sets window_is_open = false so
-- handle_plugin_window() knows the window is gone and the auto-arrival
-- route logic can proceed without waiting for the window to close.
-- ====================================================
function closed_window(wnd)
    window_is_open = false
end

-- ====================================================
-- Function: build_holder
-- Description:
-- imgui draw callback for the small "FM" badge floating window. Draws
-- the "FM" text and a circle: white when on the ground (service
-- available), grey with a red diagonal cross-bar when airborne (service
-- unavailable). On left-mouse release while on the ground, sets
-- toggle_window = true so handle_plugin_window() opens or closes the
-- main control panel on the next frame. Ignores the release event
-- immediately after the window was hidden (ignore_next_release counter)
-- to prevent a phantom re-open click.
-- ====================================================
function build_holder(wnd, x, y)
    local l_win_width = imgui.GetWindowWidth()
    local l_win_height = imgui.GetWindowHeight()
    local l_cx = l_win_width / 2
    local l_cy = l_win_height / 2
    local l_in_flight = (fm_gear1_gnd == 0 and fm_gear2_gnd == 0)

    -- Text and circle: grey when in flight, white on ground
    local l_color = l_in_flight and 0xFF666666 or 0xFFFFFFFF
    local l_text = "FM"
    local l_text_width, l_text_height = imgui.CalcTextSize(l_text)
    imgui.SetCursorPos(l_cx - l_text_width / 2, l_cy - l_text_height / 2)
    imgui.PushStyleColor(imgui.constant.Col.Text, l_color)
    imgui.TextUnformatted(l_text)
    imgui.PopStyleColor()
    imgui.DrawList_AddCircle(l_cx, l_cy, 12, l_color)

    -- Red diagonal bar when in flight (top-right to bottom-left, inside the circle)
    if l_in_flight then
        local l_r = 10
        imgui.DrawList_AddLine(l_cx + l_r, l_cy - l_r, l_cx - l_r, l_cy + l_r, 0xFF0000FF, 2)
    end

    -- VER1.6 : drag mechanic removed - holder position is fixed at Win_Y = 400
    -- DEBUG : log every mouse event on the holder (not every frame)
    if imgui.IsMouseReleased(0) then
        if ignore_next_release > 0 then
            ignore_next_release = ignore_next_release - 1
        elseif fm_gear1_gnd ~= 0 and fm_gear2_gnd ~= 0 then
            if holder_drag < 2 then
                toggle_window = true
            end
            holder_drag = 0
        end
    end
end

-- ====================================================
-- Function: closed_holder
-- Description:
-- FlyWithLua callback invoked when the FM badge window is closed by the
-- user or by X-Plane. Currently a no-op placeholder; the badge is never
-- expected to be closed during normal operation since it is the only
-- access point to the main window.
-- ====================================================
function closed_holder(wnd)
end

-- ====================================================
-- Function: load_config
-- Description:
-- Reads user preferences from Output/preferences/FollowMeXplane12.prf.
-- Restores vol, car_type_fmcar, speed_limiter, random_gate, show_path,
-- show_rampstart, simbrief_id, get_from_SimBrief, and all previously
-- known aircraft ICAO-to-type mappings from the t_aircraft table.
-- If the current aircraft ICAO is found in the file, Aircraft_Type is
-- restored immediately. Returns "" on success with a known aircraft,
-- "-1" if the aircraft type is not yet recorded (prompts the pilot to
-- set it), or "-2" if the file does not exist yet (first run).
-- ====================================================
function load_config()
    -- VER1.6 : new preferences file FollowMeXplane12.prf
    --          no more Win_Y (holder position is fixed)
    --          no more append logic - file is fully rewritten on save
    --          t_aircraft table keeps all known aircraft types in memory
    local l_file
    local l_line = ""
    local l_str1, l_str2 = "", ""
    local l_aircraft_type = ""

    l_file = io.open(syspath .. "Output/preferences/FollowMeXplane12.prf", "r")
    if l_file == nil then
        return "-2"
    end

    repeat
        l_line = l_file:read("*l")
        if l_line then
            l_str1, l_str2 = string.match(l_line, "([%p%a%d]+)%S-(.*)")
            l_str2 = trim_str(l_str2)
            if l_str1 == "get_from_SimBrief" then
                get_from_SimBrief = (l_str2 == "1")
            elseif l_str1 == "simbrief_id" then
                simbrief_id = l_str2 or ""
            elseif l_str1 == "show_path" then
                show_path = (l_str2 == "1")
            elseif l_str1 == "show_rampstart" then
                rampstart_chg = true
                show_rampstart = (l_str2 == "1")
            elseif l_str1 == "random_gate" then
                random_gate = (l_str2 == "1")
            elseif l_str1 == "vol" then
                vol = tonumber(l_str2)
                set_sound_vol()
            elseif l_str1 == "speed_limiter" then
                speed_limiter = (l_str2 == "1")
            elseif l_str1 == "car_type_fmcar" then
                if l_str2 == "Ferrari" then
                    car_type_fmcar = "Ferrari"
                elseif l_str2 == "Van" then
                    car_type_fmcar = "Van"
                elseif l_str2 == "Truck" then
                    car_type_fmcar = "Truck"
                elseif l_str2 == "Auto" then
                    car_type_fmcar = "Auto"
                end
            elseif l_str1 ~= nil and l_str1 ~= "" then
                -- VER1.6 : all other keys are aircraft ICAO types - store in t_aircraft table
                t_aircraft[l_str1] = l_str2
                if l_str1 == PLANE_ICAO then
                    l_aircraft_type = l_str2
                    Aircraft_Type = l_str2
                end
            end
        end
    until not l_line
    l_file:close()

    if l_aircraft_type ~= "" then
        return ""
    else
        return "-1"
    end
end

-- ====================================================
-- Function: save_config
-- Description:
-- Writes all user preferences to Output/preferences/FollowMeXplane12.prf.
-- The file is always fully rewritten (no append) so stale keys can never
-- accumulate. Saves vol, car_type_fmcar, speed_limiter, random_gate,
-- show_path, show_rampstart, simbrief_id, get_from_SimBrief, and the
-- complete t_aircraft ICAO-to-type table so previously configured
-- aircraft types are remembered across sessions. Returns "2" on success
-- or "-4" if the file cannot be opened for writing.
-- ====================================================
function save_config()
    -- VER1.6 : new preferences file FollowMeXplane12.prf
    --          full rewrite every time - no append, no accumulation possible
    --          t_aircraft table written entirely at the end
    local l_file
    local l_content = ""

    -- General preferences
    l_content = l_content .. "vol" .. "\t" .. tostring(vol) .. "\n"
    l_content = l_content .. "car_type_fmcar" .. "\t" .. car_type_fmcar .. "\n"
    l_content = l_content .. "speed_limiter" .. "\t" .. (speed_limiter and "1" or "0") .. "\n"
    l_content = l_content .. "random_gate" .. "\t" .. (random_gate and "1" or "0") .. "\n"
    l_content = l_content .. "show_path" .. "\t" .. (show_path and "1" or "0") .. "\n"
    l_content = l_content .. "show_rampstart" .. "\t" .. (show_rampstart and "1" or "0") .. "\n"

    -- SimBrief
    l_content = l_content .. "simbrief_id" .. "\t" .. simbrief_id .. "\n"
    l_content = l_content .. "get_from_SimBrief" .. "\t" .. (get_from_SimBrief and "1" or "0") .. "\n"

    -- VER1.6 : update current aircraft in t_aircraft table then write all known aircraft
    t_aircraft[PLANE_ICAO] = Aircraft_Type
    for l_icao, l_type in pairs(t_aircraft) do
        l_content = l_content .. l_icao .. "\t" .. l_type .. "\n"
    end

    l_file = io.open(syspath .. "Output/preferences/FollowMeXplane12.prf", "w")
    if l_file == nil then
        return "-4"
    end
    l_file:write(l_content)
    l_file:close()
    return "2"
end

-- ====================================================
-- Function: trim_str
-- Description:
-- Strips leading and trailing whitespace from a string. Returns an empty
-- string if in_str is nil or contains only whitespace. Used by
-- load_config() when parsing key-value pairs from the preferences file,
-- ensuring that trailing carriage-returns or spaces in the .prf file do
-- not corrupt stored values such as simbrief_id or car_type_fmcar.
-- ====================================================
function trim_str(in_str)
    local out_str = ""
    if in_str == nil then
        return ""
    end
    out_str = string.match(in_str, "%S.*")
    if out_str ~= nil then
        out_str = string.match(out_str, ".*%S")
    else
        out_str = ""
    end
    return out_str
end

-- ====================================================
-- Function: exit_plugin
-- Description:
-- Registered with do_on_exit() and called by FlyWithLua when X-Plane
-- shuts down or the script is unloaded. Calls full_reset() to destroy
-- all 3-D objects and clear plugin state, then destroys the terrain
-- probe and unregisters the custom fm/anim/sign dataref. This prevents
-- X-Plane access violations caused by dangling XPLM handles after the
-- Lua engine is torn down.
-- ====================================================
function exit_plugin()
	-- VER1.12 : clean up all 3D objects and state
    full_reset()
    -- unload_object/path/rampstart already called by full_reset if active
    unload_probe()
    XPLM.XPLMUnregisterDataAccessor(dr_sign)
    dr_sign = nil
end

-- ====================================================
-- MAIN SECTION (Initialization and flywithlua event
-- ====================================================
XPLM.XPLMGetSystemPath(char_str)
syspath = ffi.string(char_str)

register_dataref()
load_probe()

do_every_frame("handle_plugin_window()")
do_every_frame("object_physics()")
do_on_exit("exit_plugin()")
