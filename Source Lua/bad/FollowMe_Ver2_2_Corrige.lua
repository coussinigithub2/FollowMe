--    ---------------------------------------------------------------------------------
--          LICENSE
--    ---------------------------------------------------------------------------------
--    Copyright (c) 2026, Coussini
--
--    VER1.4 Coussini 2026 : Repairs the previous version to works with X-Plane 12
--    VER1.5 Coussini 2026 : Simbrief Integration using Simbrief ID
--    VER2.0 Coussini 2026 : - Major overhaul - Navigation Window, Menus, Animations, Back-taxi
--                           - Added a dedicated Navigation floating window (HUD) showing departure
--                             runway, arrival gate, ground speed, and animated turn indicators
--                           - Native X-Plane plugin menu integration (XPLM Menu API) to toggle
--                             the FollowMe and Navigation windows from the Plugins menu
--                           - Animated follow-me car tires: real-time steering angle and tire
--                             rotation DataRefs synced via XPLMSetDatavf (dr_tire_steer,
--                             dr_tire_rotate) and custom DataRef fm/anim/sign for car sign state
--                           - Back-taxi support: the car can guide the aircraft back along a
--                             runway when no forward route is available (backtaxi flag)
--                           - Add  airport edge filter (1206): vehicle-only taxiway
--                             edges are detected and excluded from routing (t_filter_1206)
--                           - Per-aircraft-type memory: the selected aircraft category is saved
--                             per ICAO aircraft code and restored on next use (t_aircraft table)
--                           - Force airport reload: user can trigger a fresh apt.dat parse
--                             without restarting the plugin (force_apt_reload flag)
--                           - Flight state tracking: we_fly flag accurately detects takeoff
--                             and suppresses spurious Follow Me triggers while airborne
--                           - Improved arrival detection: ARRIVE_DIST constant and fm_arrived
--                             state flag for reliable gate/runway arrival logic
--                           - Gate pre-waypoint: when the car spawns far from the departure gate
--                             a pre-waypoint is automatically inserted at the gate position
--                           - New audio cues: slow_down, keep_your_speed_20kts, sound_test
--                           - Global color constants (YELLOW, RED, OLIVE, GREEN, WARNING, BLUE,
--                             GRAY, DARK_GRAY, WHITE, BLACK) for consistent ImGui styling
--                           - Simplified error message system: Err_Msg is now a single string
--                             with an associated color (Err_Msg_color) instead of a table
--                           - Aircraft type combo now filters available gates by type in real time
--    VER2.2 Coussini 2026 : - Correction des erreurs de la version quarantaine
--                           - Restauration des require socket / http / ltn12 (SimBrief)
--                           - Repositionnement des dataref() apres les variables locales ffi
--                           - Correction ordre instructions dans determine_steering()
--                           - Correction placement de l_auto_sel dans load_object()
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
	,xplm_ControlCameraForever               = 2
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
			/* --- Menus --- */
			typedef void *XPLMMenuID;
			typedef void (*XPLMMenuHandler_f)(void *inMenuRef, void *inItemRef);
			XPLMMenuID XPLMFindPluginsMenu(void);
			XPLMMenuID XPLMCreateMenu(const char *inName, XPLMMenuID inParentMenu,
			                           int inParentItem, XPLMMenuHandler_f inHandler,
			                           void *inMenuRef);
			int  XPLMAppendMenuItem(XPLMMenuID inMenu, const char *inItemName,
			                        void *inItemRef, int inDeprecatedAndIgnored);
			void XPLMClearAllMenuItems(XPLMMenuID inMenuID);
			void XPLMRemoveMenuItem(XPLMMenuID inMenu, int inIndex);
			void XPLMEnableMenuItem(XPLMMenuID inMenu, int index, int enabled);
			void XPLMDestroyMenu(XPLMMenuID inMenuID);
			]]
ffi.cdef(cdefs)

-- List of color codes used in the program
YELLOW      = 0xFF00BFFF
RED         = 0xFF0000FF
OLIVE       = 0xFF00AA77
GREEN       = 0xFF45B1E6
WARNING     = 0xFFFFC107
BLUE        = 0xFFCCAA55
LIGHT_GRAY  = 0xFF888888
MEDIUM_GRAY = 0xFF444444
GRAY        = 0xFF666666
DARK_GRAY   = 0xFF333333
WHITE       = 0xFFFFFFFF
BLACK       = 0xFF000000

local char_str = ffi.new("char[256]")
local datarefs_addr = ffi.new("const char**")
local dataref_name = ffi.new("char[150]")
local dataref_array = ffi.new("const char*[7]")
local dataref_array2 = ffi.new("const char*[2]")
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
local syspath = ""
local BUFSIZE = 102400
local flight_start_cpt = 0
local we_fly = false
local followme_wnd = nil
local navigation_wnd = nil
local toggle_window = false
local followme_window_open = false
local navigation_window_open = false
local text_was_chg = false
local prepare_show_objects = false
local prepare_kill_objects = false
local kill_is_manual = false
local window_first_access = false
local fm_car_active = false
local depart_arrive = 0
local depart_gate, arrival_gate, depart_runway, gatetext = 0, 0, "", ""
local backtaxi = false
local fm_arrived = 0
local ARRIVE_DIST = 60
local curr_icao, curr_icao_name = "", ""
local force_apt_reload = false
local t_runway, t_runway_node, t_gate, t_taxinode, t_segment = {}, {}, {}, {}, {}
local t_deleted_runway = {}
local t_filter_1206 = {}
local t_possible_route = {}
local t_node = {}
local t_suitable_gates = {}
local Err_Msg = ""
local Err_Msg_color = "GREEN"
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
local aircraft_type = "8"
local t_aircraft = {}
local get_from_SimBrief = false
local show_path = false
local show_rampstart = false
local impose_restriction_chk = true
local random_gate = false
local vol = 5
local speed_limiter = false
local car_type_fmcar = "Auto"
local simbrief_id = ""
local sb_origin_icao = ""
local sb_origin_name = ""
local sb_dest_icao = ""
local sb_dest_name = ""
local sb_runway_takeoff = ""
local sb_runway_landing = ""
local sb_fetch_error_msg = ""
local sb_fetch_status = nil
local sb_airport_mismatch = false

-- Datarefs
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

-- Sounds
local snd_arrived = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/arrived.wav")
local snd_followme = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/followme.wav")
local snd_safe_flight_goodbye = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/safe_flight_goodbye.wav")
local snd_welcome = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/welcome_followme.wav")
local snd_welcome_bye = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/welcome_again_goodbye.wav")
local snd_keep_speed = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/keep_your_speed_20kts.wav")
local snd_test = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/sound_test.wav")
local snd_slow_down = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/slow_down.wav")

local path_is_shown = false
local path_chg = false
local rampstart_chg = false
local config_loaded = false
local world_alt = 0
local gravity = 9.81
local cof = 0.35
local deccel_max = -8.9
local deccel_avg = -4
local accel_avg = 2
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
local prev_new_flight = 0
local prev_plane_x = 0
local prev_plane_z = 0
local taxiway_network = ""
local speed_warn_time = 0
local slow_down_played = false
local fm_car_completed_timer = 0
local last_fm_car_completed_timer = 0
local combo_filter_list = false
local menu_handler = nil
local plugins_menu = nil
local my_menu_item = nil
local my_menu = nil

-- ====================================================
-- Function: check_SimBrief
-- Description:
-- Fetches the latest flight plan from the SimBrief API using the stored pilot ID.
-- Parses the XML response to extract origin/destination ICAO codes, airport names,
-- and planned runway assignments for both departure and arrival.
-- Sets error flags (sb_fetch_error_msg) and detects airport mismatches.
-- If get_from_SimBrief is active, automatically applies the fetched runway.
-- ====================================================
function check_SimBrief()
    sb_fetch_error_msg = ""
    sb_airport_mismatch = false

    if simbrief_id == "" then
        sb_fetch_error_msg = "NO_ID"
        return
    end
    sb_fetch_error_msg = ""
    local response_body = {}
    local url = "https://www.simbrief.com/api/xml.fetcher.php?userid=" .. simbrief_id .. "&v=xml"
    local _, code =
        http.request {
        url = url,
        sink = ltn12.sink.table(response_body),
        redirect = true
    }

    if code ~= 200 or not response_body or #response_body == 0 then
        sb_fetch_error_msg = "ERROR"
    end
    local xml_body = table.concat(response_body)
    sb_fetch_status = string.match(xml_body, "<fetch>.-<status>(.-)</status>") or ""
	if sb_fetch_error_msg == "ERROR" then
		return
    end
    sb_origin_icao = string.match(xml_body, "<origin>.-<icao_code>(.-)</icao_code>") or ""
    sb_dest_icao = string.match(xml_body, "<destination>.-<icao_code>(.-)</icao_code>") or ""
    sb_origin_name = string.match(xml_body, "<origin>.-<name>(.-)</name>") or ""
    sb_dest_name = string.match(xml_body, "<destination>.-<name>(.-)</name>") or ""
    sb_runway_takeoff = string.match(xml_body, "<origin>.-<plan_rwy>(.-)</plan_rwy>") or ""
    sb_runway_landing = string.match(xml_body, "<destination>.-<plan_rwy>(.-)</plan_rwy>") or ""

    if sb_origin_icao == "" or sb_dest_icao == "" then
        sb_fetch_error_msg = "NO_DATA"
        return
    end
    sb_fetch_error_msg = "OK"
    sb_airport_mismatch = (sb_origin_icao ~= curr_icao)

    if get_from_SimBrief then
        apply_simbrief_runway()
    end
end

-- ====================================================
-- Function: apply_simbrief_runway
-- Description:
-- Applies the departure runway obtained from SimBrief to the current session.
-- Only executes when SimBrief data has been fetched successfully and the current
-- airport matches the SimBrief origin ICAO.
-- Validates that the SimBrief runway exists in the parsed taxiway network.
-- Sets depart_runway or clears it and shows an error message if not found.
-- ====================================================
function apply_simbrief_runway()
    depart_runway = ""

    if sb_fetch_error_msg ~= "OK" then
        return
    end
    local l_rwy = ""

    if depart_arrive == 1 and sb_origin_icao == curr_icao then
        l_rwy = sb_runway_takeoff
    end

    if l_rwy == "" then
        return
    end
    local l_found = false
    for i = 1, #t_runway do
        if t_runway[i].ID == l_rwy then
            l_found = true
            break
        end
    end

    if l_found then
        depart_runway = l_rwya
    else
        depart_runway = ""
        update_msg("-30")
    end
end

-- ====================================================
-- Function: start_car
-- Description:
-- Initializes and spawns the Follow Me car at its starting position.
-- Resets all motion variables (speed, acceleration, steering, tire rotation).
-- Inserts a pre-waypoint at the departure gate if the car spawns far away and
-- the approach angle exceeds 45 degrees, ensuring a smooth entry onto the route.
-- Checks line-of-sight to the aircraft; rotates the car to face the plane if needed.
-- Plays the appropriate audio cue and displays the starting status message.
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
    fm_arrived = 0
    slow_down_played = false
    Err_Msg = ""
    Err_Msg_color = "GREEN"
    car_body_heading = t_node[1].heading
    car_x = t_node[1].x
    car_y = t_node[1].y
    car_z = t_node[1].z
    local _, l_dist_to_plane = heading_n_dist(car_x, car_z, fm_plane_x, fm_plane_z)
    logMsg(string.format(
        "FollowMe : start_car  spawnNode=t_node[1]  x=%.1f  z=%.1f  dist_to_plane=%.1fm  gate=%d",
        car_x, car_z, l_dist_to_plane, depart_gate))

    if depart_arrive == 1 and depart_gate > 0 and l_dist_to_plane > 50 then
        local l_gate_x = t_gate[depart_gate].x
        local l_gate_y = t_gate[depart_gate].y
        local l_gate_z = t_gate[depart_gate].z
        local l_hdg_gate_to_n1, l_dist_gate_to_n1 = heading_n_dist(l_gate_x, l_gate_z, t_node[1].x, t_node[1].z)
        local l_angle_diff = math.abs(l_hdg_gate_to_n1 - t_node[1].heading)
        if l_angle_diff > 180 then l_angle_diff = 360 - l_angle_diff end
        logMsg(string.format(
            "FollowMe : start_car  gate_to_node1_hdg=%.1f°  node1_leg_hdg=%.1f°  angle_diff=%.1f°",
            l_hdg_gate_to_n1, t_node[1].heading or 0, l_angle_diff))
        if l_angle_diff > 45 then
            local l_pre = {}
            l_pre.x       = l_gate_x
            l_pre.y       = l_gate_y
            l_pre.z       = l_gate_z
            l_pre.hotzone = ""
            l_pre.heading = l_hdg_gate_to_n1
            l_pre.dist    = l_dist_gate_to_n1
            table.insert(t_node, 1, l_pre)
            car_x = l_gate_x
            car_y = l_gate_y
            car_z = l_gate_z
            car_body_heading = l_hdg_gate_to_n1
            logMsg(string.format(
                "FollowMe : start_car  FIX-C/2 gate pre-waypoint inserted  gate_x=%.1f  gate_z=%.1f  hdg=%.1f°",
                l_gate_x, l_gate_z, l_hdg_gate_to_n1))
        end
    end
    local l_car_is_in_front = false
    l_car_is_in_front = chk_line_of_sight(fm_plane_head, 120, 120, fm_plane_x, fm_plane_z, car_x, car_z)

    if not l_car_is_in_front then
        local l_head_to_plane, _ = heading_n_dist(car_x, car_z, fm_plane_x, fm_plane_z)
        car_body_heading = l_head_to_plane
    end

    if depart_arrive == 1 and #t_node > 0 then
        local _, l_dist_already = heading_n_dist(fm_plane_x, fm_plane_z,
                                                  t_node[#t_node].x, t_node[#t_node].z)
        if l_dist_already <= ARRIVE_DIST then
            fm_arrived = 1
            curr_node  = #t_node
            update_msg("5")
            return
        end
    end

    if not l_car_is_in_front and depart_gate == 0 then
        update_msg("6")
    else
        update_msg("3")
    end
end

-- ====================================================
-- Function: determine_exit_angle
-- Description:
-- Calculates the optimal exit angle for a S-curve (compound turn) maneuver.
-- Iterates over candidate exit angles starting from the given lever angle,
-- computing geometric relationships between two tangent circles.
-- Returns the exit heading that minimizes deviation, used to smooth sharp turns.
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
-- Determines whether a target point falls within a given angular arc as seen
-- from a shooter position and heading.
-- Accepts separate left and right arc angles to define an asymmetric field of view.
-- Returns a boolean visibility flag, the bearing to the target, and the distance.
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

    if l_heading_to_target >= l_left_arc and
            l_heading_to_target <= l_left_arc + shooter_left_angle + shooter_right_angle
     then
        l_within_sight = true
    elseif l_heading_to_target <= l_right_arc and
            l_heading_to_target >= l_right_arc - (shooter_left_angle + shooter_right_angle)
     then
        l_within_sight = true
    end
    return l_within_sight, l_heading_to_target, l_dist_to_target
end

-- ====================================================
-- Function: move_car
-- Description:
-- Controls the Follow Me car's acceleration based on its distance to the aircraft.
-- Applies maximum deceleration when the car is too far ahead (beyond max_dist_from_plane).
-- Matches aircraft ground speed in the mid-range zone.
-- Accelerates aggressively when the car is too close or behind the aircraft.
-- Delegates actual motion physics to manage_car_motion() after computing acceleration.
-- ====================================================
function move_car(in_dist, in_car_in_sight, in_car_is_behind)
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
    elseif in_dist < min_dist_from_plane or in_car_is_behind then
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
-- Executes the frame-by-frame physics simulation of the Follow Me car.
-- Updates car speed using the current acceleration and elapsed time.
-- Enforces speed caps at speed_max and the upcoming waypoint turn speed.
-- Computes braking distances to decelerate before turns and before the final stop.
-- Calls plot_position() to advance the car along the route and detects arrival
-- at the destination, setting fm_arrived and playing the arrival sound.
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

    if fm_arrived == 0 and depart_arrive == 1 and #t_node > 0 then
        local _, l_dist_to_target = heading_n_dist(car_x, car_z, t_node[#t_node].x, t_node[#t_node].z)
        if l_dist_to_target <= ARRIVE_DIST then
            fm_arrived = 1
            car_speed  = 0
            car_accel  = 0
            if not backtaxi then
                car_sign = 1
                if not string.find(Err_Msg or "", "We have arrived") then
                    update_msg("5")
                end
            end
            logMsg(string.format(
                "FollowMe VER2.6 : fm_arrived triggered  dist_to_target=%.1fm  curr_node=%d  #t_node=%d  car_sign=%d",
                l_dist_to_target, curr_node, #t_node, car_sign))
        end
    end

    if fm_arrived == 0 and depart_arrive == 2 then
        local _, l_dist_to_target = heading_n_dist(car_x, car_z, t_node[#t_node].x, t_node[#t_node].z)
        if l_dist_to_target < 1 then
            fm_arrived = 2
            car_speed  = 0
            car_accel  = 0
            if not backtaxi then
                car_sign = 1
                if not string.find(Err_Msg or "", "We have arrived") then
                    update_msg("5")
                end
            end
            logMsg(string.format(
                "FollowMe VER2.6 : fm_arrived triggered  dist_to_target=%.1fm  curr_node=%d  #t_node=%d  car_sign=%d",
                l_dist_to_target, curr_node, #t_node, car_sign))
        end
    end
end

-- ====================================================
-- Function: plot_position
-- Description:
-- Advances the Follow Me car along its waypoint route by the given distance.
-- Handles straight segments by moving the car forward along the current leg heading.
-- Handles turning arcs (turning_is_active == 1 and 2) by rotating the car's position
-- around the computed Centre of Rotation at the appropriate radius.
-- Advances to the next waypoint node when a leg or arc is completed.
-- Updates the car's Y position via terrain probing and animates tire rotation.
-- Sets the signboard state (straight, left turn, right turn, arrived) based on context.
-- ====================================================
function plot_position(in_act_dist)
    local l_remaining_turn_dist = 0
    local l_remaining_act_dist = 0
    local l_remaining_leg_dist = 0
    local l_remaining_rot = 0
    local l_head1 = 0
    local l_head2 = 0
    local l_dist = 0
    local l_heading_from_center = 0
    local l_goto_nextnode = false
    local l_AoR = 0
    local l_act_dist = 0

    if curr_node == #t_node or fm_arrived ~= 0 then
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

    if car_sign == 0 or curr_node >= #t_node - 2 or fm_arrived ~= 0 then
        if curr_node ~= #t_node and fm_arrived == 0 then
            if remaining_dist_leg < avg_dist_from_plane and t_node[curr_node + 1].dir ~= nil and
                    t_node[curr_node + 1].AoC < 160
             then
                if t_node[curr_node + 1].dir == -1 then
                    car_sign = 3
                elseif t_node[curr_node + 1].dir == 1 then
                    car_sign = 2
                end
            end
            if depart_arrive == 1 and curr_node >= #t_node then
                if not backtaxi then
                    car_sign = 1
                    if not string.find(Err_Msg or "", "Arrived at destination") then
                        update_msg("5")
                    end
                end
            end
        elseif fm_arrived == 0 then
            if depart_arrive == 2 then
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
                flight_start_cpt = 0
                we_fly = false
                if not string.find(Err_Msg or "", "Arrived at destination") then
                    update_msg("5")
                end
            else
                car_sign = 1
                if not string.find(Err_Msg or "", "Arrived at destination") then
                    update_msg("5")
                end
            end
        end
    end
end

-- ====================================================
-- Function: CoR_coordinates_using_car_ref
-- Description:
-- Computes the Centre of Rotation (CoR) coordinates for a turn, expressed in
-- world-space X/Z coordinates, given the car's current position, heading,
-- turn radius, and turn direction.
-- Accounts for the offset between the car's reference point and the rear axle.
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
-- Determines the direction (left/right), radius, entry speed, and geometry of
-- the turn at the next waypoint node.
-- Uses physics (friction coefficient, gravity) to compute the safe cornering speed.
-- Handles sharp turns (< 90 degree angle of curvature) with minimum radius and
-- compound S-curves, and adjusts the pre-turn distance to fit within the leg length.
-- ====================================================
function determine_dir_of_turn(in_head1, in_head2, in_dist)
    local l_AoC = 0
    local l_AoR = 0
    local l_speed_skid = math.sqrt(cof * gravity * min_rot_radius)

    l_AoR, t_node[curr_node + 1].dir = compute_angle_diff(in_head1, in_head2)

    if l_AoR == nil then
        l_AoR = 0
    end

    if t_node[curr_node + 1].dir ~= 0 then
        l_AoC = 180 - l_AoR
        if l_AoC < 90 then
            t_node[curr_node + 1].radius = min_rot_radius
            t_node[curr_node + 1].speed = math.sqrt(cof * gravity * t_node[curr_node + 1].radius)
            t_node[curr_node + 1].dist_b4_turn = t_node[curr_node + 1].radius
            local exit_val = determine_exit_angle(90 - l_AoC)
            l_AoR = exit_val or 0
            t_node[curr_node + 1].head1_exit = add_delta_clockwise(in_head1, l_AoR, t_node[curr_node + 1].dir)
        else
            local l_speed_reduction_strength = 0
            local denominator = ((180 - 90) / (speed_max - l_speed_skid))
            local l_turn_speed = l_speed_skid + (l_AoC - 90) / denominator
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
            if t_node[curr_node + 1].dist_b4_turn + (min_rot_radius + car_rear_wheel_to_ref) > t_node[curr_node + 1].dist then
                t_node[curr_node + 1].dist_b4_turn = t_node[curr_node + 1].dist - (min_rot_radius + car_rear_wheel_to_ref)
                if t_node[curr_node + 1].dist_b4_turn < min_rot_radius * 2 + car_rear_wheel_to_ref then
                    t_node[curr_node + 1].dist_b4_turn = min_rot_radius + car_rear_wheel_to_ref
                end
                l_revised = true
            end
            if l_revised then
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
        t_node[curr_node + 1].angle_rear_to_ref = math.deg(math.atan(car_rear_wheel_to_ref / t_node[curr_node + 1].radius))
        t_node[curr_node + 1].ref_rot_radius = math.sqrt((car_rear_wheel_to_ref ^ 2) + (t_node[curr_node + 1].radius ^ 2))
        t_node[curr_node + 1].steering = math.deg(math.atan(car_front_to_back_wheel / (t_node[curr_node + 1].radius - width_btw_midtire / 2)))
        t_node[curr_node + 1].AoC = l_AoC
    end
end

-- ====================================================
-- Function: determine_steering
-- Description:
-- Smoothly ramps the steering angle in and out during a turn arc.
-- Increases steering progressively when entering the turn and decreases it
-- proportionally when approaching the exit, based on remaining arc distance
-- and time, ensuring the wheels return to neutral before the straight segment.
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
-- Computes a world-space X/Z coordinate offset from a reference point by
-- applying a local delta (in_delta_x, in_delta_z) rotated to the given heading.
-- Used to shift the car's reference point to the rear axle position or to
-- position the signboard above the car.
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
-- Calculates the compass heading and Euclidean distance from one X/Z point
-- to another in X-Plane's local coordinate system.
-- Returns heading in degrees (0-360) and distance in meters.
-- ====================================================
function heading_n_dist(in_from_x1, in_from_z1, in_to_x2, in_to_z2)
    local l_heading = math.fmod((math.deg(math.atan2(in_to_x2 - in_from_x1, -(in_to_z2 - in_from_z1))) + 360), 360)
    local l_dist = math.sqrt(((in_to_x2 - in_from_x1) ^ 2) + ((in_to_z2 - in_from_z1) ^ 2))
    return l_heading, l_dist
end

-- ====================================================
-- Function: minus_delta_clockwise
-- Description:
-- Subtracts an angular delta from a heading in a specified direction.
-- Direction 1 subtracts clockwise (heading decreases); direction -1 adds.
-- Wraps the result within the 0-360 degree range.
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
-- Adds an angular delta to a heading in a specified direction.
-- Direction 1 adds clockwise; direction -1 subtracts.
-- Wraps the result within the 0-360 degree range.
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
-- Computes the smallest angular difference between two headings and the
-- turn direction needed to reach the target heading (1 = right, -1 = left).
-- Used throughout the routing and physics code to determine turn angles.
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
-- Computes the angular difference between two headings when the turn direction
-- is already known (in_dir > 0 for clockwise, < 0 for counter-clockwise).
-- Returns the swept angle in degrees without re-computing the optimal direction.
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
-- Converts X-Plane local OpenGL coordinates (X, Y, Z) to geographic
-- latitude, longitude, and altitude using the XPLM WorldToLocal API.
-- Returns the converted values overwriting the input buffers.
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
-- Converts geographic latitude, longitude, and altitude to X-Plane local
-- OpenGL coordinates (X, Y, Z) using the XPLM LocalToWorld API.
-- Returns the resulting local X, Y, Z values.
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
-- Converts a geographic coordinate (lat/lon/alt) to local X/Z coordinates
-- with accurate terrain-snapped Y elevation using the X-Plane terrain probe.
-- If altitude is 0, performs a first probe pass to resolve the actual ground level,
-- then re-probes to get the precise surface-snapped position.
-- Caches the world altitude to speed up subsequent conversions.
-- ====================================================
function get_local_coordinates(in_lat, in_lon, in_alt)
    local l_x, l_y, l_z = 0, 0, 0

    if in_alt == 0 then
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
-- Returns the terrain-snapped Y elevation for a given local X/Z position
-- by converting to lat/lon, probing the terrain, and converting back.
-- Used each frame to keep the Follow Me car flush with the ground surface.
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
-- Updates the 3D position, heading, and pitch of the Follow Me car instance
-- in the X-Plane scenery engine using XPLMInstanceSetPosition.
-- Syncs tire steering and rotation datarefs via sync_anim_datarefs().
-- Applies a small pitch tilt (0.3 degrees) during acceleration or braking.
-- Also positions and updates the signboard instance above the car.
-- ====================================================
function draw_object(in_x, in_y, in_z, in_heading)
    dataref_float_value[0] = steering
    dataref_float_value[1] = steering
    dataref_float_value[2] = tire_rotate
    dataref_float_value[3] = tire_rotate
    dataref_float_value[4] = tire_rotate
    dataref_float_value[5] = tire_rotate
    dataref_float_addr = dataref_float_value
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
-- Main per-frame callback registered with do_every_frame().
-- Computes the elapsed time delta and skips frames with invalid deltas.
-- Manages loading/unloading of the path pins and ramp-start marker when flags change.
-- Drives the Follow Me car by calling move_car() when active and not yet arrived.
-- Renders the car 3D model each frame via draw_object().
-- ====================================================
function object_physics()
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

    if path_instance[0] ~= nil and not path_is_shown then
        draw_path()
    end

    if path_chg then
        if show_path then
            load_path()
        else
            unload_path()
        end
        path_chg = false
    end

    if rampstart_chg then
        if show_rampstart then
            load_rampstart()
        else
            unload_rampstart()
        end
        if rampstart_instance[0] ~= nil then
            draw_rampstart()
        end
    end

    if obj_instance[0] ~= nil and #t_node > 0 and curr_node ~= #t_node and fm_arrived == 0 then
        local l_dist = 0
        local l_car_in_sight, l_car_is_behind = false, false
        l_car_in_sight, _, l_dist = chk_line_of_sight(fm_plane_head, 80, 80, fm_plane_x, fm_plane_z, car_x, car_z)
        if not l_car_in_sight and l_dist <= 200 then
            l_car_is_behind, _, _ =
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
-- Initializes the X-Plane terrain probe object (XPLMCreateProbe) used to
-- snap objects and coordinates to the ground surface elevation.
-- Must be called once at plugin startup before any probe_y or get_local_coordinates calls.
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
-- Asynchronously loads the 3D OBJ model for the selected Follow Me car type
-- (Ferrari, Van, Truck, or a random Auto selection) and creates its scene instance.
-- Sets vehicle-specific physical parameters (tire diameter, wheelbase, speed, etc.).
-- Also asynchronously loads the signboard OBJ and creates its instance,
-- registering the fm/anim/sign custom dataref for animation state.
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
            inRefcon)
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
            inRefcon)
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
            inRefcon)
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
        inRefcon)
end

-- ====================================================
-- Function: load_path
-- Description:
-- Asynchronously loads the pushpin yellow marker OBJ and creates one instance
-- per waypoint node in t_node to visually display the planned taxi route.
-- Only runs when the car is active, show_path is enabled, and the route exists.
-- ====================================================
function load_path()

    if fm_car_active and show_path and #t_node > 0 then
        XPLM.XPLMLoadObjectAsync(
            SCRIPT_DIRECTORY .. "follow_me/objects/pushpin_yellow.obj",
            function(inObject, inRefcon)
                for i = 0, #t_node - 1 do
                    path_instance[i] = XPLM.XPLMCreateInstance(inObject, NULL)
                end
                pathref = inObject
            end,
            inRefcon)
    end

end

-- ====================================================
-- Function: load_rampstart
-- Description:
-- Asynchronously loads the diamond marker OBJ and creates a single instance
-- to display the departure gate or arrival gate position on the airport surface.
-- Skips loading if an instance already exists.
-- ====================================================
function load_rampstart()

    if rampstart_instance[0] == nil then
        XPLM.XPLMLoadObjectAsync(
            SCRIPT_DIRECTORY .. "follow_me/objects/diamond_marker.obj",
            function(inObject, inRefcon)
                rampstart_instance[0] = XPLM.XPLMCreateInstance(inObject, NULL)
                rampstartref = inObject
            end,
            inRefcon)
    end

end

-- ====================================================
-- Function: draw_path
-- Description:
-- Positions each path pin instance at the corresponding waypoint node location
-- along the planned taxi route, at the current aircraft's Y elevation.
-- Sets path_is_shown to true to avoid redundant repositioning.
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
-- Positions the diamond ramp-start marker at the active departure or arrival gate.
-- Uses the gate's stored X/Y/Z coordinates and heading from the t_gate table.
-- Hides the marker at Y=-9999 if no valid gate index is found.
-- Resets rampstart_chg to false after placement.
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
-- Destroys the terrain probe object to free X-Plane resources.
-- Called on plugin exit via exit_plugin().
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
-- Destroys the Follow Me car and signboard scene instances and unloads their
-- OBJ assets from X-Plane memory using XPLMDestroyInstance and XPLMUnloadObject.
-- Resets all instance and object reference variables to nil.
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
-- Destroys all path pin instances and unloads the pushpin OBJ asset.
-- Resets path_instance[0], pathref, and path_is_shown.
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
-- Hides the ramp-start marker by moving it to Y=-9999, then destroys its instance
-- and unloads the diamond marker OBJ asset.
-- Resets rampstart_instance[0], rampstartref, and rampstart_chg.
-- ====================================================
function unload_rampstart()

    if rampstart_instance[0] ~= nil then
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

-- ====================================================
-- Function: register_dataref
-- Description:
-- Looks up the tire steering and tire rotation datarefs by name and stores their
-- handles for use in sync_anim_datarefs().
-- Registers the custom read-only float dataref fm/anim/sign via
-- XPLMRegisterDataAccessor, backed by the car_sign variable,
-- so the signboard OBJ can animate its state via the dataref.
-- ====================================================
function register_dataref()

    dr_tire_steer = XPLM.XPLMFindDataRef("sim/graphics/animation/ground_traffic/tire_steer_deg")
    dr_tire_rotate = XPLM.XPLMFindDataRef("sim/graphics/animation/ground_traffic/tire_rotation_angle_deg")
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
        NULL)
end

-- ====================================================
-- Function: sync_anim_datarefs
-- Description:
-- Writes the current steering angle and tire rotation values to the shared
-- ground-traffic animation datarefs using XPLMSetDatavf.
-- Also pushes the car_sign value to the fm/anim/sign dataref via XPLMSetDataf.
-- Called every frame from draw_object() to keep the car animation in sync.
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
-- Detects the nearest airport to the aircraft using XPLMFindNavAid and parses
-- its apt.dat data when the airport changes or a forced reload is requested.
-- Calls read_apt_file() to populate runways, gates, taxinodes, and segments.
-- After parsing, determines the departure/arrival mode, checks whether the plane
-- is already at a gate, and applies any active SimBrief runway assignment.
-- ====================================================
function get_airport_elements()

    if force_apt_reload then
        force_apt_reload = false
        world_alt = 0
        taxiway_network = read_apt_file(curr_icao)
        if taxiway_network ~= "" then
            XPLMSpeakString("Follow Me Service is not available at this airport")
            return
        end
        if sb_fetch_error_msg == "OK" then
            sb_airport_mismatch = (sb_origin_icao ~= curr_icao)
        end
    else

    local l_airport_index = XPLMFindNavAid(nil, nil, LATITUDE, LONGITUDE, nil, xplm_Nav_Airport)
    local l_new_ICAO, l_new_ICAO_name = "", ""

    _, _, _, _, _, _, l_new_ICAO, l_new_ICAO_name = XPLMGetNavAidInfo(l_airport_index)

    if curr_icao ~= l_new_ICAO then
        world_alt = 0
        taxiway_network = read_apt_file(l_new_ICAO)
        curr_icao = l_new_ICAO
        curr_icao_name = l_new_ICAO_name
        if taxiway_network ~= "" then
            XPLMSpeakString("Follow Me Service is not available at this airport")
            return
        end
        if sb_fetch_error_msg == "OK" then
            sb_airport_mismatch = (sb_origin_icao ~= curr_icao)
        end
    end

    if #t_deleted_runway > 0 then
        update_msg("-18")
    end

    depart_gate = check_gate()

    if depart_arrive == 0 then
        if flight_start_cpt == 9999 then
            if depart_gate == 0 then
                depart_arrive = 2
            else
                flight_start_cpt = 0
                we_fly = false
                depart_arrive = 1
            end
        else
            depart_arrive = 1
        end
    end

    if get_from_SimBrief then
        apply_simbrief_runway()
    end
end

-- ====================================================
-- Function: read_apt_file
-- Description:
-- Parses the apt.dat file(s) from the X-Plane scenery pack list to extract all
-- airport data for the given ICAO code.
-- Reads runway records (code 100), ramp/gate records (code 1300/1301),
-- taxiway nodes (1201), segments (1202), hotzones (1204), and vehicle-only
-- edges (1206) by calling the corresponding decipher_* functions.
-- Processes scenery packs in priority order and stops at the first airport match.
-- Returns an error code string if data is missing, or empty string on success.
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
                        if not l_airport_found then
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
                                        if string.match(l_line2, "^%d+ %s*[^%s]+%s*[^%s]+%s*[^%s]+%s*([^%s]+)%s*.*") ==
                                                in_ICAO
                                         then
                                            l_airport_found = true
                                            l_not_first_line = false
                                            initialise_airport()
                                        end
                                        break
                                    end
                                end
                                if l_airport_found then
                                    break
                                end
                            end
                        end
                        if l_airport_found then
                            for l_line2 in l_new_lines:gmatch("[^\r\n]+") do
                                if not l_not_first_line then
                                    l_not_first_line = true
                                else
                                    if string.find(l_line2, "^1%s") or l_line2 == "99" then
                                        table.sort(
                                            t_gate,
                                            function(a, b)
                                                return a.ID < b.ID
                                            end
                                        )
                                        local l_id_count1 = {}
                                        for _, g in ipairs(t_gate) do
                                            l_id_count1[g.ID] = (l_id_count1[g.ID] or 0) + 1
                                        end
                                        for _, g in ipairs(t_gate) do
                                            if l_id_count1[g.ID] > 1 and g.Terminal and g.Terminal ~= "" then
                                                g.ID = g.ID .. " " .. g.Terminal
                                            end
                                        end
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
                                        apply_1206_filter()
                                        l_terminate_loop = true
                                        break
                                    end
                                end
                                if string.match(l_line2, "^100%s") then
                                    decipher_runway(l_line2, t_runway)
                                end
                                if string.match(l_line2, "^1301%s") and l_processed_1300 then
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
                                    if not l_processed_1204 then
                                        decipher_taxisegment_hotzone(l_line2)
                                        l_processed_1204 = true
                                    end
                                end
                            end
                            if l_terminate_loop then
                                break
                            end
                        end
                    end
                    l_file2:close()
                    if l_terminate_loop then
                        break
                    end
                end
            end
        end
    until not l_line1

    l_file1:close()

    if not l_airport_found and not l_terminate_loop then
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
-- Parses a single apt.dat runway record (code 100) and adds two opposing
-- runway threshold entries to the t_runway table.
-- Extracts runway IDs, threshold lat/lon coordinates, and converts them to
-- local X/Z coordinates using get_local_coordinates().
-- Links each runway end to its paired opposite end via the Pair index.
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
                    "([^%s]+)%s*([^%s]+)%s*([^%s]+)%s*" .. "(.*)")
    t_runway[i].Lat = tonumber(l_str1)
    t_runway[i].Lon = tonumber(l_str2)
    t_runway[i].Node = -1
    t_runway[i + 1].Lat = tonumber(l_str3)
    t_runway[i + 1].Lon = tonumber(l_str4)
    t_runway[i + 1].Node = -1
    t_runway[i].Pair = i + 1
    t_runway[i + 1].Pair = i
    t_runway[i].x, _, t_runway[i].z, world_alt = get_local_coordinates(t_runway[i].Lat, t_runway[i].Lon, world_alt)
    t_runway[i + 1].x, _, t_runway[i + 1].z, world_alt =
        get_local_coordinates(t_runway[i + 1].Lat, t_runway[i + 1].Lon, world_alt)
end

-- ====================================================
-- Function: decipher_ramp
-- Description:
-- Parses a single apt.dat ramp start record (code 1300) and adds one entry
-- to the t_gate table with its position, heading, ramp type, and aircraft types.
-- Normalizes the aircraft type string from X-Plane categories (heavy, jets,
-- turboprops, props, all) to internal numeric type codes.
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
-- Parses the optional apt.dat ramp operation record (code 1301) that follows
-- a ramp start (1300) record.
-- Appends the terminal letter and operation type (cargo, military) to the
-- most recently added gate entry in t_gate.
-- Removes A380-class aircraft (type 1) from gates tagged as E-category terminal.
-- Adds military type (0) to gates flagged as military operations.
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
-- Parses a single apt.dat taxi node record (code 1201) and appends a new entry
-- to the t_taxinode table.
-- Stores the node's geographic coordinates, converts them to local X/Y/Z,
-- and initializes all pathfinding fields (f_value, g_value, parent, etc.) to nil.
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
-- Parses a single apt.dat taxi segment record (code 1202) and appends it to
-- the t_segment table.
-- Identifies the segment type (runway or taxiway), extracts its size restriction
-- code, direction (oneway/twoway), and taxiway ID.
-- Computes the segment heading and distance using heading_n_dist(),
-- and registers the segment index in the Segment field of both endpoint nodes.
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
        t_taxinode[t_segment[i].Node2 + 1].z)

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
-- Parses the apt.dat hotzone annotation record (code 1204) that follows a
-- taxi segment (1202) and attaches the hotzone label to the most recent segment.
-- Also marks the endpoint taxi nodes as type 'hotzone' unless they are runway nodes.
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

-- ====================================================
-- Function: decipher_vehicle_edge
-- Description:
-- Parses an apt.dat vehicle-only edge record (code 1206) and stores the
-- pair of node indices in the t_filter_1206 table.
-- These edges are later used by apply_1206_filter() to remove vehicle-only
-- taxiway edges from the routing network.
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

-- ====================================================
-- Function: apply_1206_filter
-- Description:
-- Removes taxiway segments and their associated nodes that are flagged as
-- vehicle-only edges (code 1206) and are not shared with normal taxiway segments.
-- After removing segments, rebuilds the Segment connection string for all
-- affected taxi nodes to keep the pathfinding graph consistent.
-- ====================================================
function apply_1206_filter()
    if #t_filter_1206 == 0 then
        return
    end

    local l_used = {}

    for l_s = 1, #t_segment do
        l_used[t_segment[l_s].Node1] = true
        l_used[t_segment[l_s].Node2] = true
    end

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

    local l_seg_count = 0
    local l_new_segments = {}

    for l_s = 1, #t_segment do
        if l_remove[t_segment[l_s].Node1] or l_remove[t_segment[l_s].Node2] then
            l_seg_count = l_seg_count + 1
        else
            l_new_segments[#l_new_segments + 1] = t_segment[l_s]
        end
    end

    for l_s = 1, #l_new_segments do
        t_segment[l_s] = l_new_segments[l_s]
    end

    for l_s = #l_new_segments + 1, #t_segment + l_seg_count do
        t_segment[l_s] = nil
    end

    for l_idx = 1, #t_taxinode do
        if t_taxinode[l_idx] ~= nil then
            t_taxinode[l_idx].Segment = ""
        end
    end

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
-- Iterates over all runways in t_runway and calls match_runway() for each one
-- to find the closest taxiway network node at each runway threshold.
-- Removes any runway entries where no matching node was found (Node == -1),
-- logging them in t_deleted_runway for the user-facing warning message.
-- ====================================================
function determine_runway_node()
    local l_idx = 0

    for l_idx = 1, #t_runway do
        match_runway(l_idx)
    end

    local l_rows = #t_runway
    l_idx = 1

    while l_idx <= l_rows do
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
-- Finds the best taxiway network node to associate with a given runway threshold.
-- Prefers nodes that have both a runway segment and a connecting taxiway segment
-- (i.e., hold-short nodes), falling back to any runway node if none has a taxiway.
-- Stores the chosen node index in the runway entry and marks that node with
-- the runway ID for back-reference.
-- ====================================================
function match_runway(in_runway_idx)
    local l_idx = 0
    local l_curr_dist = 0
    local l_min_dist_twy = 99999
    local l_min_dist_any = 99999
    local l_best_twy = -1
    local l_best_any = -1
    local l_rwy_x = t_runway[in_runway_idx].x
    local l_rwy_z = t_runway[in_runway_idx].z
    local l_rwy_id = t_runway[in_runway_idx].ID
    local l_rwy_nodes = {}

    for l_seg = 1, #t_segment do
        if t_segment[l_seg].Type == "runway" then
            local l_seg_id = t_segment[l_seg].ID or ""
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

    local l_has_own_segs = false

    for _ in pairs(l_rwy_nodes) do
        l_has_own_segs = true
        break
    end

    for l_idx = 1, #t_taxinode do
        if t_taxinode[l_idx].Type == "runway" then
            local l_node_id = l_idx - 1
            if l_has_own_segs and not l_rwy_nodes[l_node_id] then
            else
                _, l_curr_dist = heading_n_dist(t_taxinode[l_idx].x, t_taxinode[l_idx].z, l_rwy_x, l_rwy_z)
                if l_curr_dist < l_min_dist_any then
                    l_min_dist_any = l_curr_dist
                    l_best_any = l_node_id
                end
                local l_has_twy = false
                for l_seg = 1, #t_segment do
                    if (t_segment[l_seg].Node1 == l_node_id or t_segment[l_seg].Node2 == l_node_id) and
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

-- ====================================================
-- Function: full_reset
-- Description:
-- Performs a complete plugin state reset: unloads all 3D objects, clears route
-- and airport data, resets flight-state flags and counters, and re-initializes
-- both the airport and route data structures.
-- Called on new flight detection, large teleport, or plugin exit.
-- Updates the plugin menu state after reset.
-- ====================================================
function full_reset()

    if fm_car_active then
        unload_object()
        unload_path()
        unload_rampstart()
    end

    fm_car_active = false
    prepare_kill_objects = false
    prepare_show_objects = false
    kill_is_manual = false
    flight_start_cpt = 0
    we_fly = false
    config_loaded = false
    t_deleted_runway = {}
    curr_icao = ""
    force_apt_reload = false
    initialise_airport()
    initialise_routes()
    logMsg("FollowMe : full_reset() completed")
    update_menu_state()
end

-- ====================================================
-- Function: initialise_airport
-- Description:
-- Clears all airport-specific data tables (runways, gates, taxinodes, segments,
-- 1206 filter) and resets departure/arrival mode and gate selection variables.
-- Called when a new airport is detected or a forced reload is triggered.
-- ====================================================
function initialise_airport()
    t_runway, t_runway_node, t_gate, t_taxinode, t_segment = {}, {}, {}, {}, {}
    t_filter_1206 = {}
    depart_arrive = 0
    depart_gate, arrival_gate, depart_runway, gatetext = 0, 0, "", ""
end

-- ====================================================
-- Function: initialise_routes
-- Description:
-- Clears the current route waypoint table and resets all route-level state:
-- error message, departure/arrival mode, back-taxi flag, arrival flag,
-- and marks the window as needing a first-access refresh.
-- ====================================================
function initialise_routes()
    t_node = {}
    Err_Msg = ""
    Err_Msg_color = "GREEN"
    depart_arrive = 0
    backtaxi = false
    fm_arrived = 0
    window_first_access = true
end

-- ====================================================
-- Function: handle_plugin_window
-- Description:
-- Main per-frame plugin controller registered with do_every_frame().
-- Detects new-flight resets and large aircraft teleports, triggering full_reset().
-- Monitors gear-off-ground state to detect takeoff and suppress the Follow Me
-- interface while airborne; resets flight state on touchdown.
-- Plays speed-warning and slow-down audio cues at the appropriate distances.
-- Processes the prepare_show_objects and prepare_kill_objects deferred flags
-- to safely load/unload 3D objects outside of the imgui draw callback.
-- Handles the toggle_window flag to switch between the Follow Me and Navigation windows.
-- ====================================================
function handle_plugin_window()

    if fm_new_flight < prev_new_flight and prev_new_flight > 5 then
        full_reset()
    end

    prev_new_flight = fm_new_flight

    if prev_plane_x ~= 0 then
        local _, l_teleport_dist = heading_n_dist(prev_plane_x, prev_plane_z, fm_plane_x, fm_plane_z)
        if l_teleport_dist >= 1000 then
            full_reset()
        end
    end

    prev_plane_x = fm_plane_x
    prev_plane_z = fm_plane_z

    if speed_limiter and fm_car_active and
    	fm_gear1_gnd == 1 and fm_gear2_gnd == 1 and
    	fm_gnd_spd > 10.288 and fm_run_time > speed_warn_time
     then
        play_sound(snd_keep_speed)
        speed_warn_time = fm_run_time + 15
    end

    if fm_car_active and fm_arrived == 0 and not slow_down_played and #t_node > 0 then
        local l_dist_slow = 0
        if depart_arrive == 1 then
            _, l_dist_slow = heading_n_dist(fm_plane_x, fm_plane_z, t_node[#t_node].x, t_node[#t_node].z)
        elseif depart_arrive == 2 and arrival_gate > 0 then
            _, l_dist_slow = heading_n_dist(fm_plane_x, fm_plane_z, t_gate[arrival_gate].x, t_gate[arrival_gate].z)
        end
        if l_dist_slow > 0 and l_dist_slow <= 300 then
            play_sound(snd_slow_down)
            slow_down_played = true
        end
    end

    if (fm_gear1_gnd == 0 and fm_gear2_gnd == 0) and fm_new_flight > 1 then
        if flight_start_cpt == 0 then
            flight_start_cpt = fm_run_time + 5
        end
        if fm_run_time > flight_start_cpt and flight_start_cpt ~= 9999 then
        	we_fly = true
        	update_menu_state()
            prepare_kill_objects = true
            if followme_wnd ~= nil then
	            arrival_gate = 0
	            flight_start_cpt = 9999
                hide_followme_window()
            end
            if navigation_wnd ~= nil then
	            hide_navigation_window()
            end
        end
    else
    	flight_start_cpt = 0
        we_fly = false
        update_menu_state()
    end

    if prepare_show_objects then
        fm_car_active = true
        update_menu_state()
        load_object()
        load_path()
        start_car()
        rampstart_chg = true
        prepare_show_objects = false
    end

    if prepare_kill_objects then
        fm_car_active = false
        update_menu_state()
        unload_object()
        unload_path()
        if kill_is_manual then
            unload_rampstart()
            rampstart_chg = false
            kill_is_manual = false
			fm_car_completed_timer = 0
			last_fm_car_completed_timer = 0
			fm_arrived = 0
            update_msg("7")
            force_apt_reload = true
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
            if show_rampstart and (arrival_gate > 0 or depart_gate > 0) then
                rampstart_chg = true
            end
        else
            if flight_start_cpt == 9999 then
                unload_rampstart()
            end
            initialise_routes()
        end
        prepare_kill_objects = false
    end

    if toggle_window then
        toggle_window = false
        if followme_wnd ~= nil then
            followme_window_open = false
            navigation_window_open = true
            hide_followme_window()
            show_navigation_window()
        else
            followme_window_open = true
            navigation_window_open = false
            show_followme_window()
            hide_navigation_window()
        end
    end
end

-- ====================================================
-- Function: show_followme_window
-- Description:
-- Creates and displays the main Follow Me floating window centered on screen.
-- Sets the imgui builder callback to build_followme_window and the close
-- callback to closed_followme_window.
-- Marks window_first_access so the window refreshes airport data on first draw.
-- Updates the plugin menu state to disable the menu item while the window is open.
-- ====================================================
function show_followme_window()
	local followme_title = "Follow Me Window"
    local wnd_width  = 420
    local wnd_height = 515
    local pos_x = (SCREEN_WIDTH  - wnd_width)  / 2
    local pos_y = (SCREEN_HIGHT - wnd_height) / 2

    window_first_access = true
    followme_wnd = float_wnd_create(420, 515, 1, true)
    float_wnd_set_title(followme_wnd, followme_title)
	float_wnd_set_position(followme_wnd, pos_x, pos_y)
    float_wnd_set_imgui_builder(followme_wnd, "build_followme_window")
    float_wnd_set_onclose(followme_wnd, "closed_followme_window")
    update_menu_state()
end

-- ====================================================
-- Function: hide_followme_window
-- Description:
-- Closes and destroys the Follow Me window if it is currently open,
-- by delegating to closed_followme_window().
-- ====================================================
function hide_followme_window()
    if followme_wnd ~= nil then
        closed_followme_window()
    end
end

-- ====================================================
-- Function: build_followme_window
-- Description:
-- Imgui draw callback that renders all controls of the Follow Me main window.
-- Displays departure/arrival airport info (from SimBrief or current location),
-- mode radio buttons (Departure / Arrival), runway and gate selectors,
-- aircraft type combo, car model radio buttons, path and ramp-start toggles,
-- speed limiter, volume slider, SimBrief ID input, and Save Preferences button.
-- Handles the Request / Cancel Follow Me Car action button.
-- Shows SimBrief status messages and general error/status messages at the bottom.
-- On first access, loads the config file and refreshes airport elements.
-- ====================================================
function build_followme_window(wnd, x, y)
    local l_err = "0" -- No error
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
        imgui.constant.WindowFlags.NoSavedSettings)

    if window_first_access then
        if not config_loaded then
            load_config()
            config_loaded = true
        end
        get_airport_elements()
        window_first_access = false
    end

------------------------------

    imgui.SetWindowFontScale(1.1)
    imgui.SetCursorPosY(5)
    imgui.SetCursorPosX(10)
    imgui.PushStyleColor(imgui.constant.Col.Text, LIGHT_GRAY)

    if get_from_SimBrief then
        imgui.TextUnformatted("Simbrief Departure Airport")
    else
        imgui.TextUnformatted("Departure Airport")
    end

------------------------------

    imgui.PopStyleColor()
    imgui.SetCursorPosY(18)
    imgui.SetCursorPosX(10)
    imgui.SetWindowFontScale(1.2)

    if get_from_SimBrief and sb_fetch_error_msg == "OK" and sb_origin_icao ~= "" then
        imgui.TextUnformatted(sb_origin_icao .. " " .. string.format("%-3s", sb_runway_takeoff) .. " " .. sb_origin_name)
    else
        imgui.TextUnformatted(curr_icao .. " " .. curr_icao_name)
    end

    imgui.SetWindowFontScale(1.1)
    imgui.SetCursorPosY(35)
    imgui.SetCursorPosX(10)
    imgui.PushStyleColor(imgui.constant.Col.Text, LIGHT_GRAY)

------------------------------

    if get_from_SimBrief then
        imgui.TextUnformatted("Simbrief Arrival Airport")
    else
        imgui.TextUnformatted("Arrival Airport")
    end

------------------------------

    imgui.PopStyleColor()
    imgui.SetCursorPosY(48)
    imgui.SetCursorPosX(10)
    imgui.SetWindowFontScale(1.2)

    if get_from_SimBrief and sb_fetch_error_msg == "OK" and sb_dest_icao ~= "" then
        imgui.TextUnformatted(sb_dest_icao .. " " .. string.format("%-3s", sb_runway_landing) .. " " .. sb_dest_name)
    else
        imgui.PushStyleColor(imgui.constant.Col.Text, GRAY)
        imgui.TextUnformatted("---")
        imgui.PopStyleColor()
    end

    imgui.SetWindowFontScale(1.0)

------------------------------

    -----------------------------------------------------------------------------
    -- Begining : Disable the SETTING to avoid selection when FM car is active --
    -----------------------------------------------------------------------------
	if fm_car_active or #t_runway == 0 then
	    imgui.BeginDisabled()
	end

------------------------------

    imgui.SetCursorPosY(65)
    imgui.SetCursorPosX(4)
    imgui.PushStyleColor(0, DARK_GRAY)
    imgui.TextUnformatted("_________________________       ________________________")
    imgui.PopStyleColor()
    imgui.SetCursorPosY(68)
    imgui.SetCursorPosX(4)
    imgui.PushStyleColor(0, BLUE)
    imgui.TextUnformatted("                         SETTING")
    imgui.PopStyleColor()

------------------------------

    imgui.SetCursorPosY(82)
    imgui.SetCursorPosX(18)

    if imgui.RadioButton(" Departure", depart_arrive == 1, false) then
        depart_arrive = 1
        flight_start_cpt = 0
        we_fly = false
        rampstart_chg = true
        if get_from_SimBrief then
            apply_simbrief_runway()
        end
    end

    if depart_gate > 0 then
        imgui.SameLine()
        imgui.SetCursorPosX(134)
        imgui.TextUnformatted(t_gate[depart_gate].ID)
    end

------------------------------

    imgui.SetCursorPosY(104)
    imgui.SetCursorPosX(48)
    imgui.TextUnformatted("To Runway : ")
    imgui.SameLine()
    imgui.SetCursorPosX(130)
    imgui.PushItemWidth(55)

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
                    flight_start_cpt = 0
	                we_fly = false
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

    l_changed, l_newval = imgui.Checkbox("##Use SimBrief data", get_from_SimBrief)

    if l_changed then
        get_from_SimBrief = l_newval
        if get_from_SimBrief then
            if simbrief_id ~= "" then
                check_SimBrief()
            end
        else
            depart_runway = ""
        end
    end

    imgui.SameLine()
    imgui.TextUnformatted("Use SimBrief data")

------------------------------

    imgui.SetCursorPosY(131)
    imgui.SetCursorPosX(18)

    if imgui.RadioButton(" Arrival", depart_arrive == 2, false) then
        depart_arrive = 2
        rampstart_chg = true
        if get_from_SimBrief then
            apply_simbrief_runway()
        end
    end

    if depart_arrive == 2 and random_gate and arrival_gate == 0 then
        auto_assign_gate()
    end

    imgui.SameLine()
    imgui.SetCursorPosX(230)

    l_changed, l_newval = imgui.Checkbox("##Auto Assign", random_gate)

    if l_changed then
        random_gate = l_newval
        if random_gate and arrival_gate == 0 then
            auto_assign_gate()
        end
    end

    imgui.SameLine()
    imgui.TextUnformatted("Auto Assign")

------------------------------

    imgui.SetCursorPosY(156)
    imgui.SetCursorPosX(48)
    imgui.TextUnformatted("To Gate/Ramp : ")
    imgui.SetCursorPosY(156)
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

    if combo_filter_list then
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
                if string.match(t_gate[i].Types, aircraft_type) == aircraft_type or
                        (aircraft_type == "0" and string.match(t_gate[i].Types, "7"))
                 then
                    t_suitable_gates[#t_suitable_gates + 1] = i
                end
            end
            if #t_suitable_gates == 0 then
                update_msg("-17")
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
                if l_into_list then
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

    imgui.SameLine()

    if imgui.Button("X##clear_filter", 15, 15) then
        arrival_gate = 0
        gatetext = ""
        combo_filter_list = false
        text_was_chg = false
        rampstart_chg = true
    end

------------------------------

    ---------------------------------------------------------------------------
    -- Ending : Disable the SETTING to avoid selection when FM car is active --
    ---------------------------------------------------------------------------
	if fm_car_active or #t_runway == 0 then
	    imgui.EndDisabled()
	end

------------------------------

    if fm_car_active then
	    imgui.PushStyleColor(imgui.constant.Col.Button, 0xFF27275A)
	    imgui.PushStyleColor(imgui.constant.Col.ButtonHovered, RED)
	    imgui.PushStyleColor(imgui.constant.Col.ButtonActive, 0xFF43439A)
	    imgui.SetCursorPosY(187)
	    imgui.SetCursorPosX(96)
	    if imgui.Button("N", 22, 25) then
	    	toggle_window = true
	    end
	    imgui.PopStyleColor(3)
    end

------------------------------

    imgui.SetCursorPosY(187)
    imgui.SetCursorPosX(126)

    if not fm_car_active then
		if imgui.Button("Request Follow Me Car", 170, 25) then
		    combo_filter_list = false
		    l_err = determine_XP_route()
		    if l_err == "" or (l_err ~= "" and tonumber(l_err) > 0) then
		        prepare_show_objects = true
				toggle_window = true
		    end
		end
    else
        if imgui.Button("Cancel Follow Me Car", 170, 25) then
            prepare_kill_objects = true
            kill_is_manual = true
        end
    end

------------------------------

    imgui.SetCursorPosY(221)
    imgui.SetCursorPosX(4)
    imgui.PushStyleColor(0, DARK_GRAY)
    imgui.TextUnformatted("__________________                    __________________")
    imgui.PopStyleColor()
    imgui.SetCursorPosY(224)
    imgui.SetCursorPosX(4)
    imgui.PushStyleColor(0, BLUE)
    imgui.TextUnformatted("                  OPTIONS & PREFERENCES")
    imgui.PopStyleColor()


------------------------------

    ------------------------------------------------------------------
    -- Begining : Disable the Aircraft Type and Follow Me Car model --
    --            to avoid selection when FM car is active          --
    ------------------------------------------------------------------
	if fm_car_active or #t_runway == 0 then
	    imgui.BeginDisabled()
	end

------------------------------

    imgui.SetCursorPosY(244)
    imgui.SetCursorPosX(18)
    imgui.PushStyleColor(0, BLUE)
    imgui.TextUnformatted("Aircraft Type")
    imgui.PopStyleColor()

------------------------------

	imgui.SetCursorPosY(268)
	imgui.SetCursorPosX(18)
	imgui.TextUnformatted(PLANE_ICAO)
	imgui.SetCursorPosY(266)
	imgui.SetCursorPosX(90)

    local l_ac_types = ""

    if aircraft_type == "" then
        l_ac_types = ""
    else
        l_ac_types = ac_types[tonumber(aircraft_type) + 1]
    end

    if fm_car_active and depart_arrive == 2 then
        imgui.InputText("##text_actype", l_ac_types, 250, imgui.constant.InputTextFlags.ReadOnly)
    else
        imgui.PushItemWidth(250)
        if imgui.BeginCombo("##select_type", l_ac_types) then
            for i = 1, #ac_types do
                l_is_selected = (l_ac_types == ac_types[i])
                if imgui.Selectable(ac_types[i], l_is_selected) then
                    l_ac_types = ac_types[i]
                    aircraft_type = tostring(i - 1)
                    gatetext = ""
                    arrival_gate = 0
                    rampstart_chg = true
                    Err_Msg = ""
                    Err_Msg_color = "GREEN"
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
            aircraft_type = ""
            gatetext = ""
            arrival_gate = 0
            rampstart_chg = true
            Err_Msg = ""
            Err_Msg_color = "GREEN"
            depart_gate = check_gate()
        end
    end

------------------------------

	imgui.SetCursorPosY(298)
    imgui.SetCursorPosX(18)
    imgui.PushStyleColor(0, BLUE)
    imgui.TextUnformatted("Follow Me car model")
    imgui.PopStyleColor()

------------------------------

    imgui.SetCursorPosY(315)
    imgui.SetCursorPosX(18)

    if imgui.RadioButton(" Ferrari", car_type_fmcar == "Ferrari", false) then
        car_type_fmcar = "Ferrari"
    end

    imgui.SetCursorPosY(315)
    imgui.SetCursorPosX(110)

    if imgui.RadioButton(" FM Van", car_type_fmcar == "Van", false) then
        car_type_fmcar = "Van"
    end

    imgui.SetCursorPosY(315)
    imgui.SetCursorPosX(195)

    if imgui.RadioButton(" FM Truck", car_type_fmcar == "Truck", false) then
        car_type_fmcar = "Truck"
    end

    imgui.SetCursorPosY(315)
    imgui.SetCursorPosX(290)

    if imgui.RadioButton(" Auto Select", car_type_fmcar == "Auto", false) then
        car_type_fmcar = "Auto"
    end

------------------------------

    ----------------------------------------------------------------
    -- Ending : Disable the Aircraft Type and Follow Me Car model --
    --          to avoid selection when FM car is active          --
    ----------------------------------------------------------------
	if fm_car_active or #t_runway == 0 then
	    imgui.EndDisabled()
	end

------------------------------

	imgui.SetCursorPosY(341)
    imgui.SetCursorPosX(18)
    imgui.PushStyleColor(0, BLUE)
    imgui.TextUnformatted("Follow Me requirements")
    imgui.PopStyleColor()

------------------------------

    imgui.SetCursorPosY(362)
    imgui.SetCursorPosX(18)

    l_changed, l_newval = imgui.Checkbox(" Show Path", show_path)

    if l_changed then
        show_path = l_newval
        path_chg = true
    end

------------------------------

    imgui.SetCursorPosY(362)
    imgui.SetCursorPosX(200)

    l_changed, l_newval = imgui.Checkbox(" Show Ramp Start", show_rampstart)

    if l_changed then
        show_rampstart = l_newval
        if show_rampstart then
            rampstart_chg = true
        else
            unload_rampstart()
        end
    end

------------------------------

    imgui.SetCursorPosY(387)
    imgui.SetCursorPosX(18)

    l_changed, l_newval = imgui.Checkbox("##Limit Car Speed", speed_limiter)

    if l_changed then
        speed_limiter = l_newval
        if speed_limiter then
            speed_max = 10.288
        else
            speed_max = car_default_speed
        end
    end

    imgui.SameLine()
    imgui.TextUnformatted("Limit speed to 20kts")

------------------------------

    imgui.SetCursorPosY(387)
    imgui.SetCursorPosX(200)
    imgui.PushItemWidth(150)

    l_changed, l_newval = imgui.SliderFloat("##Vol", vol, 1, 10, "%.0f")

    imgui.PopItemWidth()

    if l_changed then
        vol = l_newval
        set_sound_vol()
    end

    imgui.SetCursorPosY(385)
    imgui.SetCursorPosX(357)

    if imgui.Button("Volume", 50, 25) then
        play_sound(snd_test)
    end

    ---------------------------------------------------------------------------------
    -- Begining : Disable the Simbrief ID to avoid selection when FM car is active --
    ---------------------------------------------------------------------------------
	if fm_car_active or #t_runway == 0 then
	    imgui.BeginDisabled()
	end

------------------------------

	imgui.SetCursorPosY(416)
    imgui.SetCursorPosX(18)
    imgui.PushStyleColor(0, BLUE)
    imgui.TextUnformatted("Simbrief")
    imgui.PopStyleColor()

------------------------------

    imgui.SetCursorPosY(436)
    imgui.SetCursorPosX(18)
    imgui.TextUnformatted("SimBrief ID")
    imgui.SetCursorPosY(433)
    imgui.SetCursorPosX(105)
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

------------------------------

	imgui.PushStyleColor(imgui.constant.Col.Button, 0xFF2D5A27)
	imgui.PushStyleColor(imgui.constant.Col.ButtonHovered, 0xFF3D7A35)
	imgui.PushStyleColor(imgui.constant.Col.ButtonActive, 0xFF4D9A43)
	imgui.SetCursorPosY(429)
	imgui.SetCursorPosX(240)

	if imgui.Button("Save Preferences", 130, 25) then
			l_err = save_config()
	end

	-- VERY IMPORTANT: Pop styles to avoid affecting other buttons
	imgui.PopStyleColor(3)

------------------------------

    imgui.SetCursorPosY(460)
    imgui.SetCursorPosX(4)
    imgui.PushStyleColor(0, DARK_GRAY)
    imgui.TextUnformatted("_______________________        _________________________")
    imgui.PopStyleColor()
    imgui.SetCursorPosY(463)
    imgui.SetCursorPosX(4)
    imgui.PushStyleColor(0, BLUE)
    imgui.TextUnformatted("                       MESSAGES")
    imgui.PopStyleColor()

------------------------------

    -----------------------------------------------------------------------------------------------------
    -- Ending : Disable the Simbrief ID  and Save Preferences to avoid selection when FM car is active --
    -----------------------------------------------------------------------------------------------------
	if fm_car_active or #t_runway == 0 then
	    imgui.EndDisabled()
	end

------------------------------

    -- Affichage du message Simbrief
    imgui.SetCursorPosY(478)
    imgui.SetCursorPosX(18)

    if simbrief_id == "" then
	          imgui.PushStyleColor(imgui.constant.Col.Text, RED)
	          imgui.TextUnformatted("Enter your SimBrief ID and click Save Preferences")
	          imgui.PopStyleColor()
    elseif sb_fetch_error_msg == "OK" then
        if get_from_SimBrief then
            if sb_airport_mismatch then
	          imgui.PushStyleColor(imgui.constant.Col.Text, RED)
	          imgui.TextUnformatted("The current airport does not match the Simbrief data")
	          imgui.PopStyleColor()
            else
	          imgui.PushStyleColor(imgui.constant.Col.Text, GREEN)
	          imgui.TextUnformatted("SimBrief data fetched successfully")
	          imgui.PopStyleColor()
            end
        end
    elseif sb_fetch_error_msg == "ERROR" then
	          imgui.PushStyleColor(imgui.constant.Col.Text, RED)
	          imgui.TextUnformatted(sb_fetch_status)
	          imgui.PopStyleColor()
    end

------------------------------

    -- Affichage du message general
    if Err_Msg ~= "" and Err_Msg ~= nil then
	    imgui.SetCursorPosY(494)
	    imgui.SetCursorPosX(18)
	    if Err_Msg_color == "RED" then
	        imgui.PushStyleColor(imgui.constant.Col.Text, RED)
	    else
	        imgui.PushStyleColor(imgui.constant.Col.Text, GREEN)
	    end
	    imgui.TextUnformatted(Err_Msg)
	    imgui.PopStyleColor()
    end

------------------------------

	if l_err ~= "" then
	    update_msg(l_err)
	end
end

-- ====================================================
-- Function: closed_followme_window
-- Description:
-- Callback invoked when the Follow Me window is closed by the user or by code.
-- Destroys the floating window handle and clears followme_wnd and
-- followme_window_open, then updates the plugin menu state.
-- ====================================================
function closed_followme_window(wnd)
    if followme_wnd ~= nil then
        float_wnd_destroy(followme_wnd)
        followme_wnd = nil
        followme_window_open = false
        update_menu_state()
    end
end

-- ====================================================
-- Function: show_navigation_window
-- Description:
-- Creates and displays the narrow Navigation HUD floating window positioned
-- horizontally centered in the upper half of the screen.
-- Sets the imgui builder to build_navigation_window and the close callback
-- to closed_navigation_window.
-- ====================================================
function show_navigation_window()
	local pos_x = (SCREEN_WIDTH - 495) / 2
	local pos_y = SCREEN_HIGHT / 2
    navigation_wnd = float_wnd_create(495, 35, 1, true)
	float_wnd_set_position(navigation_wnd, pos_x, pos_y)
    float_wnd_set_imgui_builder(navigation_wnd, "build_navigation_window")
    float_wnd_set_onclose(navigation_wnd, "closed_navigation_window")
end

-- ====================================================
-- Function: hide_navigation_window
-- Description:
-- Closes and destroys the Navigation window if it is currently open,
-- by delegating to closed_navigation_window().
-- ====================================================
function hide_navigation_window()
    if navigation_wnd ~= nil then
    	closed_navigation_window()
    end
end

-- ====================================================
-- Function: build_navigation_window
-- Description:
-- Imgui draw callback for the Navigation HUD strip shown during active guidance.
-- Displays the window title (runway or gate name), left and right animated
-- turn-indicator triangles (red when a turn is signaled, dark when straight),
-- aircraft ground speed, Follow Me car speed, a compass rose arrow pointing
-- toward the car or destination, and the distance to the target.
-- Shows an 'F' button to toggle back to the Follow Me main window.
-- After arrival, flashes the destination indicator and removes the car after 7 seconds.
-- ====================================================
function build_navigation_window(wnd, x, y)
	local navigation_title= "Navigation Window"

	if fm_car_active then
		if depart_arrive == 1 then
	        navigation_title = "Depart Rwy : "..depart_runway
		end
		if depart_arrive == 2 then
		    if gatetext ~= "" then
	        	navigation_title = "Gate : "..gatetext
	        end
		end
	end

    float_wnd_set_title(navigation_wnd, navigation_title)

    -- Timer for the completed ride
    if fm_arrived ~= 0 then
	    if fm_car_completed_timer < 7 then
	        if fm_run_time >= last_fm_car_completed_timer + 1 then
		        fm_car_completed_timer = fm_car_completed_timer + 1
		        last_fm_car_completed_timer = fm_run_time
		    end
		end
	end

	local fm_ti = fm_car_completed_timer
	local nav_color_left, nav_color_right

	if car_sign == 1 or car_sign == 0 or not fm_car_active then
		nav_color_left  = DARK_GRAY
	    nav_color_right = DARK_GRAY
	elseif car_sign == 2 then
	    nav_color_left  = DARK_GRAY
	    nav_color_right = RED
	elseif car_sign == 3 then
	    nav_color_left  = RED
	    nav_color_right = DARK_GRAY
	else
	    nav_color_left  = DARK_GRAY
	    nav_color_right = DARK_GRAY
	end

	if math.floor(fm_run_time) % 2 == 0 then
	    nav_color_left  = DARK_GRAY
	    nav_color_right = DARK_GRAY
	end

	local nav_side = 20
	local nav_h    = nav_side * math.sqrt(3) / 2
	local nav_half = nav_side / 2
	local nav_mid_y = 17

--------------------

	local pos_x_begin_at = 8

    if fm_car_active then
	    imgui.PushStyleColor(imgui.constant.Col.Button, 0xFF27275A)
	    imgui.PushStyleColor(imgui.constant.Col.ButtonHovered, RED)
	    imgui.PushStyleColor(imgui.constant.Col.ButtonActive, 0xFF43439A)
	    imgui.SetCursorPosY(7)
	    imgui.SetCursorPosX(pos_x_begin_at)
	    if imgui.Button("F", 22, 20) then
	    	toggle_window = true
	    end
	    imgui.PopStyleColor(3)
    end

---------------------

	local nav_lc_x   = pos_x_begin_at + 48 + nav_h / 2
	local nav_lg_tip_x = nav_lc_x - nav_h / 2
	local nav_lg_top_x = nav_lc_x + nav_h / 2
	local nav_lg_bot_x = nav_lc_x + nav_h / 2

	imgui.DrawList_AddTriangleFilled(
	    nav_lg_tip_x, nav_mid_y,
	    nav_lg_top_x, nav_mid_y - nav_half,
	    nav_lg_bot_x, nav_mid_y + nav_half,
	    nav_color_left)

---------------------

    imgui.SetCursorPosY(11)
    imgui.SetCursorPosX(pos_x_begin_at + 78)

    if fm_car_active and depart_arrive ~= 0 and fm_arrived == 0 then
    	imgui.PushStyleColor(imgui.constant.Col.Text, OLIVE)
	else
    	imgui.PushStyleColor(imgui.constant.Col.Text, GRAY)
	end

    imgui.TextUnformatted("GND SPD")
    imgui.PopStyleColor()

---------------------

    imgui.SameLine()
    imgui.SetCursorPosX(pos_x_begin_at + 135)

    if fm_car_active and depart_arrive ~= 0 and fm_arrived == 0 then
        local l_plane_kts = fm_gnd_spd * 1.94384
        local l_spd_color = (speed_limiter and l_plane_kts > 20) and RED or OLIVE
        imgui.PushStyleColor(imgui.constant.Col.Text, l_spd_color)
        imgui.TextUnformatted(string.format("%.1f kts", l_plane_kts))
        imgui.PopStyleColor()
    else
        imgui.PushStyleColor(imgui.constant.Col.Text, GRAY)
        imgui.TextUnformatted("---")
        imgui.PopStyleColor()
    end

---------------------

    imgui.SameLine()
    imgui.SetCursorPosX(pos_x_begin_at + 201)

    if fm_car_active and  depart_arrive ~= 0 and fm_arrived == 0 then
    	imgui.PushStyleColor(imgui.constant.Col.Text, YELLOW)
	else
    	imgui.PushStyleColor(imgui.constant.Col.Text, GRAY)
	end

    imgui.TextUnformatted("FM Car")
    imgui.PopStyleColor()

---------------------

    imgui.SameLine()
    imgui.SetCursorPosX(pos_x_begin_at + 249)

    if fm_car_active and depart_arrive ~= 0 and fm_arrived == 0 then
        local l_car_kts = car_speed * 1.94384
        imgui.PushStyleColor(imgui.constant.Col.Text, YELLOW)
        imgui.TextUnformatted(string.format("%.1f kts", l_car_kts))
        imgui.PopStyleColor()
    else
        imgui.PushStyleColor(imgui.constant.Col.Text, GRAY)
        imgui.TextUnformatted("---")
        imgui.PopStyleColor()
    end

---------------------

	if (fm_car_active and car_x ~= 0) or fm_arrived ~= 0 then
	    local target_x, target_z, triangle_color
	    if fm_arrived ~= 0 then
		    if fm_ti > 6 then
		        unload_object()
			end
			triangle_color = (fm_ti == 0 or fm_ti == 2 or fm_ti == 4 or fm_ti == 6) and BLACK or BLUE
			if fm_arrived == 1 then
		        target_x = t_node[#t_node].x
		        target_z = t_node[#t_node].z
			else
			    target_x = t_gate[arrival_gate].x
			    target_z = t_gate[arrival_gate].z
			end
	    else
	        target_x = car_x
	        target_z = car_z
	        triangle_color = YELLOW
	    end
	    local l_bearing, _ = heading_n_dist(fm_plane_x, fm_plane_z, target_x, target_z)
	    local l_rel_angle = l_bearing - fm_plane_head
	    while l_rel_angle < 0   do l_rel_angle = l_rel_angle + 360 end
	    while l_rel_angle >= 360 do l_rel_angle = l_rel_angle - 360 end
	    local cx, cy    = (pos_x_begin_at + 320), 18
	    local tip_len   = 13
	    local wing_len  = 8
	    local tail_len  = 6
	    local tip_rad   = math.rad(l_rel_angle - 90)
	    local base_rad  = math.rad(l_rel_angle + 90)
	    local left_rad  = math.rad(l_rel_angle - 90 + 90)
	    local right_rad = math.rad(l_rel_angle - 90 - 90)
	    local tip_x  = cx + math.cos(tip_rad)   * tip_len
	    local tip_y  = cy + math.sin(tip_rad)   * tip_len
	    local base_x = cx + math.cos(base_rad)  * tail_len
	    local base_y = cy + math.sin(base_rad)  * tail_len
	    local lw_x = base_x + math.cos(left_rad)  * wing_len
	    local lw_y = base_y + math.sin(left_rad)  * wing_len
	    local rw_x = base_x + math.cos(right_rad) * wing_len
	    local rw_y = base_y + math.sin(right_rad) * wing_len
	    imgui.DrawList_AddTriangleFilled(tip_x, tip_y, lw_x, lw_y, rw_x, rw_y, triangle_color)
	end

---------------------

    if #t_node > 0 then
    	local l_dist_dest = 0
    	if depart_arrive == 1 then
        	_, l_dist_dest = heading_n_dist(fm_plane_x, fm_plane_z, t_node[#t_node].x, t_node[#t_node].z)
        end
    	if depart_arrive == 2 then
        	_, l_dist_dest = heading_n_dist(fm_plane_x, fm_plane_z, t_gate[arrival_gate].x, t_gate[arrival_gate].z)
        end
        imgui.SameLine()
        imgui.SetCursorPosX(pos_x_begin_at + 342)
		local text_color = (fm_ti == 0 or fm_ti == 2 or fm_ti == 4 or fm_ti == 6) and BLACK or BLUE
        if depart_arrive ~= 0 and fm_arrived == 0 then
        	imgui.PushStyleColor(imgui.constant.Col.Text, BLUE)
    	else
        	imgui.PushStyleColor(imgui.constant.Col.Text, text_color)
    	end
        imgui.TextUnformatted("Target")
        imgui.PopStyleColor()
        imgui.SameLine()
        imgui.SetCursorPosX(pos_x_begin_at + 391)
        if depart_arrive ~= 0 and fm_arrived == 0 then
        	imgui.PushStyleColor(imgui.constant.Col.Text, BLUE)
    	else
        	imgui.PushStyleColor(imgui.constant.Col.Text, text_color)
    	end
        if l_dist_dest >= 1000 then
            imgui.TextUnformatted(string.format("%.2f km", l_dist_dest / 1000))
        else
            imgui.TextUnformatted(string.format("%d m", math.floor(l_dist_dest + 0.5)))
        end
        imgui.PopStyleColor()
    end

---------------------

	local nav_rc_x   = 495 - 8 - nav_h / 2
	local nav_rg_tip_x = nav_rc_x + nav_h / 2
	local nav_rg_top_x = nav_rc_x - nav_h / 2
	local nav_rg_bot_x = nav_rc_x - nav_h / 2

	imgui.DrawList_AddTriangleFilled(
	    nav_rg_tip_x, nav_mid_y,
	    nav_rg_top_x, nav_mid_y - nav_half,
	    nav_rg_bot_x, nav_mid_y + nav_half,
	    nav_color_right)
end

-- ====================================================
-- Function: closed_navigation_window
-- Description:
-- Callback invoked when the Navigation window is closed by the user or by code.
-- Destroys the floating window handle and clears navigation_wnd and
-- navigation_window_open, then updates the plugin menu state.
-- ====================================================
function closed_navigation_window(wnd)
    if navigation_wnd ~= nil then
	    float_wnd_destroy(navigation_wnd)
	    navigation_wnd = nil
	    navigation_window_open = false
        update_menu_state()
	end
end

-- ====================================================
-- Function: set_sound_vol
-- Description:
-- Applies the current volume setting (vol, range 1-10) to all loaded WAV sound
-- samples by calling set_sound_gain() with vol divided by 10.
-- Called whenever the volume slider value changes.
-- ====================================================
function set_sound_vol()
    set_sound_gain(snd_arrived, vol / 10)
    set_sound_gain(snd_followme, vol / 10)
    set_sound_gain(snd_safe_flight_goodbye, vol / 10)
    set_sound_gain(snd_welcome, vol / 10)
    set_sound_gain(snd_welcome_bye, vol / 10)
    set_sound_gain(snd_keep_speed, vol / 10)
    set_sound_gain(snd_test, vol / 10)
end

-- ====================================================
-- Function: update_msg
-- Description:
-- Translates a numeric message code into a human-readable status or error string
-- and stores it in Err_Msg with the appropriate color (RED for errors, GREEN for info).
-- Also triggers the corresponding audio cue (follow-me, welcome, arrived, goodbye)
-- for operational codes such as car start, arrival, and cancellation.
-- No-ops on code '0' or nil.
-- ====================================================
function update_msg(in_msg)
    local l_rwy = ""

    if in_msg == nil or in_msg == "0" then
        return
    end

    if tonumber(in_msg) ~= nil and tonumber(in_msg) < 0 then
        Err_Msg_color = "RED"
    else
        Err_Msg_color = "GREEN"
    end

    if in_msg == "-30" then
        if depart_arrive == 1 and sb_origin_icao == curr_icao then
            l_rwy = sb_runway_takeoff
        elseif depart_arrive == 2 and sb_dest_icao == curr_icao then
            l_rwy = sb_runway_landing
        end
        in_msg = "Routes not defined for Simbrief runway " .. l_rwy
        if Err_Msg == in_msg then
            return
        end
    elseif in_msg == "-18" then
        in_msg = "No routes have been defined for this airport"
        for i = 1, #t_deleted_runway do
            if i == #t_deleted_runway then
                in_msg = in_msg .. t_deleted_runway[i]
            else
                in_msg = in_msg .. t_deleted_runway[i] .. ", "
            end
        end
        if Err_Msg == in_msg then
            return
        end
    elseif in_msg == "-17" then
        in_msg = "No suitable gate for this plane. Lift restriction."
    elseif in_msg == "-16" then
        in_msg = "Can't find start pt. Get off runway and request again"
    elseif in_msg == "-15" then
        in_msg = "Taxi Routes not defined for this airport"
    elseif in_msg == "-14" then
        in_msg = "Unable to find a suitable route"
    elseif in_msg == "-13" then
        in_msg = "Unable to find a suitable end pt"
    elseif in_msg == "-12" then
        in_msg = "Unable to find a suitable start pt"
    elseif in_msg == "-11" then
        in_msg = "Unable to locate scenery_packs.ini"
    elseif in_msg == "-6" then
        in_msg = "Choose a Gate/Ramp"
    elseif in_msg == "-5" then
        if get_from_SimBrief then
            in_msg = "No runway from SimBrief data"
        else
            in_msg = "Choose a Departure Runway"
        end
    elseif in_msg == "-4" then
        in_msg = "Unable to complete Save operation"
    elseif in_msg == "-2" then
        in_msg = "Preference file not found. Apply default values."
    elseif in_msg == "-1" then
        in_msg = "Choose an Aircraft Type"
    elseif in_msg == "2" then
        in_msg = "Preference Saved"
    elseif in_msg == "3" then
        in_msg = ""
        if depart_arrive == 1 then
            play_sound(snd_followme)
        else
            play_sound(snd_welcome)
        end
    elseif in_msg == "4" then
        in_msg = "No route found. Remove taxiway limitation, trying again."
    elseif in_msg == "5" then
        in_msg = "We have arrived at destination"
        play_sound(snd_arrived)
    elseif in_msg == "6" then
        in_msg = ""
        if depart_arrive == 1 then
            play_sound(snd_followme)
        else
            play_sound(snd_welcome)
        end
    elseif in_msg == "7" then
        in_msg = ""
        if depart_arrive == 1 then
            play_sound(snd_safe_flight_goodbye)
        else
            play_sound(snd_welcome_bye)
        end
    elseif in_msg == "30" then
        in_msg = "SimBrief data fetched successfully"
    elseif in_msg == "31" then
        in_msg = "Fetching SimBrief data..."
    else
        in_msg = "MESSAGE NOT FOUND CHECK YOUR CODE"
        Err_Msg_color = "GREEN"
    end

    Err_Msg = in_msg or ""
end

-- ====================================================
-- Function: determine_XP_route
-- Description:
-- Entry-point validation function called when the user clicks Request Follow Me Car.
-- Checks that taxiway data exists, the mode is set, a gate or runway is selected,
-- and an aircraft type has been chosen.
-- Returns an error code string on validation failure, or delegates to
-- determine_possible_routes() and returns its result.
-- ====================================================
function determine_XP_route()

    if #t_taxinode == 0 then
        return "-15"
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

    if aircraft_type == "" then
        return "-1"
    end

    depart_gate = check_gate()

    return determine_possible_routes()
end

-- ====================================================
-- Function: determine_possible_routes
-- Description:
-- Finds the start and end nodes for the A* pathfinding algorithm.
-- For departure, locates the nearest taxiway node to the gate or aircraft position.
-- For arrival, locates the nearest taxiway node to the target gate.
-- Computes heuristic (h_value) distances for all nodes, then calls transverse()
-- with and without restrictions to find a valid route.
-- On success, calls process_possible_routes() to build the final waypoint list.
-- ====================================================
function determine_possible_routes()
    local l_startpt_node, l_startpt_x, l_startpt_z = 0, 0, 0, 0
    local l_endpt_node, l_endpt_x, l_endpt_z = 0, 0, 0
    local l_index = 0
    local l_found = false
    local l_rev_heading = add_delta_clockwise(fm_plane_head, 180, 1)

    if depart_gate ~= 0 then
        logMsg(string.format(
            "FollowMe : determine_possible_routes  GATE=%s  heading=%.1f  x=%.1f  z=%.1f  type=%s",
            t_gate[depart_gate].ID or "?",
            t_gate[depart_gate].Heading,
            t_gate[depart_gate].x, t_gate[depart_gate].z,
            t_gate[depart_gate].Ramptype or "?"))
        l_found, l_startpt_node, l_startpt_x, l_startpt_z =
            determine_pos_on_segment(
            t_gate[depart_gate].Heading,
            t_gate[depart_gate].x,
            t_gate[depart_gate].z,
            t_gate[depart_gate].Ramptype
        )
        if l_found then
            local _, l_dgs = heading_n_dist(
                t_gate[depart_gate].x, t_gate[depart_gate].z,
                t_taxinode[l_startpt_node + 1].x, t_taxinode[l_startpt_node + 1].z)
            logMsg(string.format(
                "FollowMe : determine_possible_routes  startNode=%d  dist_gate_to_start=%.1fm  startSeg='%s'",
                l_startpt_node, l_dgs,
                t_taxinode[l_startpt_node + 1].Segment or "(empty)"))
        else
            logMsg("FollowMe : determine_possible_routes  FAILED - no start node found from gate")
        end
    else
        local l_adj_fm_plane_x = 0
        local l_adj_fm_plane_z = 0
        l_adj_fm_plane_x, l_adj_fm_plane_z = coordinates_of_adjusted_ref(fm_plane_x, fm_plane_z, 0, 60, fm_plane_head)
        l_found, l_startpt_node, l_startpt_x, l_startpt_z =
            determine_pos_on_segment(fm_plane_head, l_adj_fm_plane_x, l_adj_fm_plane_z, "tie_down")
        if not l_found then
            l_found, l_startpt_node, l_startpt_x, l_startpt_z =
                determine_pos_on_segment(l_rev_heading, fm_plane_x, fm_plane_z, "tie_down")
        end
        if not l_found and depart_arrive == 2 then
            return "-16"
        end
    end

    if not l_found then
        return "-12"
    end

    if depart_arrive == 1 then
        for l_index = 1, #t_runway do
            if t_runway[l_index].ID == depart_runway then
                l_endpt_node = t_runway[l_index].Node
                break
            end
        end
        backtaxi = false
    else
        l_found, l_endpt_node, l_endpt_x, l_endpt_z =
            determine_pos_on_segment(
            t_gate[arrival_gate].Heading,
            t_gate[arrival_gate].x,
            t_gate[arrival_gate].z,
            "gate"
        )
        if not l_found then
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
-- Implements the A* pathfinding algorithm on the airport taxiway network.
-- Expands nodes from the open list, scoring candidates using g_value (travel cost)
-- plus h_value (heuristic distance to goal) plus a turn-angle cost penalty.
-- Uses an inner evaluate_node() closure to enforce aircraft-type size restrictions
-- and avoid revisiting closed nodes.
-- On success, back-traces the parent chain to build the raw node sequence
-- stored in t_possible_route.
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

        if l_in_open or l_in_close then
            return false
        end

        if not impose_restriction_chk then
            return true
        else
            if in_size ~= "" then
                local l_aircraft_type = tonumber(aircraft_type)
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
            if t_segment[l_segment_idx].Type ~= "runway" or
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
                    logMsg(string.format(
                        "FollowMe : A* expand  curr=%d → cand=%d  seg=%d  g=%.1f  h=%.1f  f=%.1f  cost=%d  AoC=%.0f  hdg=%.0f",
                        l_curr_node, l_node, l_segment_idx,
                        l_g_value, t_taxinode[l_node + 1].h_value, l_f_value,
                        t_taxinode[l_node + 1].cost or 0,
                        l_AoC or 0, l_curr_heading))
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
            logMsg(string.format(
                "FollowMe : A* select   next=%d  f=%.1f  cost=%d  open=%d",
                l_curr_node,
                t_open[#t_open].f_value or 0,
                t_open[#t_open].cost or 0,
                #t_open))
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
                " AIRPORT=" .. curr_icao .. " FROM=" .. l_from_label .. " TO=" .. l_to_label
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
-- Function: apply_runway_axis_filter
-- Description:
-- Post-processes the A* route to remove waypoint regressions along the runway axis.
-- Projects each on-axis node onto the runway centerline vector and checks that
-- the distance to the threshold is monotonically decreasing.
-- Removes any node that increases the distance to the threshold (a U-turn detour),
-- ensuring the car approaches the runway in a straight line.
-- ====================================================
function apply_runway_axis_filter()

    if depart_arrive == 2 or #t_node < 2 then
        return
    end

    local l_T_x, l_T_z = 0, 0
    local l_rwy_idx = 0

    for l_i = 1, #t_runway do
        if t_runway[l_i].ID == depart_runway then
            l_rwy_idx = l_i
            l_T_x = t_runway[l_i].x
            l_T_z = t_runway[l_i].z
            break
        end
    end

    if l_rwy_idx == 0 then return end

    local l_O_x, l_O_z = 0, 0
    local l_pair_idx = t_runway[l_rwy_idx].Pair

    if l_pair_idx ~= nil and t_runway[l_pair_idx] ~= nil then
        l_O_x = t_runway[l_pair_idx].x
        l_O_z = t_runway[l_pair_idx].z
    else
        for l_i = 1, #t_runway do
            if l_i ~= l_rwy_idx then
                l_O_x = t_runway[l_i].x
                l_O_z = t_runway[l_i].z
                break
            end
        end
        logMsg("FollowMe : apply_runway_axis_filter - Pair fallback used for RWY " .. depart_runway)
    end

    local l_ax = l_T_x - l_O_x
    local l_az = l_T_z - l_O_z
    local l_axLen = math.sqrt(l_ax * l_ax + l_az * l_az)

    if l_axLen < 1 then return end

    local MAX_PERP_M = 15
    local function project_node(n_x, n_z)
        local l_dTN_sq = (n_x - l_T_x)^2 + (n_z - l_T_z)^2
        local l_dON_sq = (n_x - l_O_x)^2 + (n_z - l_O_z)^2
        local l_dTN    = math.sqrt(l_dTN_sq)
        local l_dON    = math.sqrt(l_dON_sq)
        local l_proj = (l_axLen * l_axLen + l_dTN_sq - l_dON_sq) / (2 * l_axLen)
        local l_perp = math.sqrt(math.max(0, l_dTN_sq - l_proj * l_proj))
        return l_proj, l_perp, l_dTN
    end

    local l_on_axis = {}

    for l_i = 1, #t_node do
        local l_nd = t_node[l_i]
        if l_nd and l_nd.x and l_nd.z then
            local l_proj, l_perp, l_dTN = project_node(l_nd.x, l_nd.z)
            if l_perp <= MAX_PERP_M and l_proj >= -50 and l_proj <= l_axLen + 50 then
                l_on_axis[#l_on_axis + 1] = { idx = l_i, dTN = l_dTN }
            end
        end
    end

    if #l_on_axis < 2 then
        logMsg(
            "FollowMe : apply_runway_axis_filter RWY " .. depart_runway ..
            " - on-axis A* nodes=" .. #l_on_axis .. " (+ threshold=1) → no filter (total ≤ 2)")
        return
    end

    logMsg(
        "FollowMe : apply_runway_axis_filter RWY " .. depart_runway ..
        " - on-axis A* nodes=" .. #l_on_axis .. " (+ threshold=1) → applying monotone filter")

    local l_to_remove = {}
    local l_prev_dTN = math.huge

    for l_k = 1, #l_on_axis do
        local l_entry = l_on_axis[l_k]
        local l_idx   = l_entry.idx
        local l_dTN   = l_entry.dTN
        if l_idx == 1 then
            l_prev_dTN = l_dTN
        elseif l_dTN > l_prev_dTN then
            l_to_remove[l_idx] = true
            logMsg(
                "FollowMe : apply_runway_axis_filter  remove t_node[" .. l_idx .. "]" ..
                " dTN=" .. string.format("%.1f", l_dTN) ..
                "m  prev_dTN=" .. string.format("%.1f", l_prev_dTN) .. "m (regression)"
            )
        else
            l_prev_dTN = l_dTN
        end
    end

    local l_removed_count = 0

    for l_k, _ in pairs(l_to_remove) do
        l_removed_count = l_removed_count + 1
    end

    if l_removed_count == 0 then
        logMsg("FollowMe : apply_runway_axis_filter RWY " .. depart_runway .. " - all nodes monotone, nothing removed")
        return
    end

    local l_new_node = {}

    for l_i = 1, #t_node do
        if not l_to_remove[l_i] then
            l_new_node[#l_new_node + 1] = t_node[l_i]
        end
    end

    t_node = l_new_node

    logMsg("FollowMe : apply_runway_axis_filter RWY " .. depart_runway ..
        " - removed " .. l_removed_count .. " node(s)," ..
        " t_node now has " .. #t_node .. " entries")
end

-- ====================================================
-- Function: process_possible_routes
-- Description:
-- Converts the raw A* node sequence from t_possible_route into the final t_node
-- waypoint list used by the car physics.
-- Copies node coordinates and computes headings and distances between consecutive nodes.
-- Adjusts nodes that are too close together (less than min_dist_btw_nodes) to prevent
-- turn geometry failures.
-- For departure routes, snaps the last node onto the runway centerline and appends
-- the runway threshold as the final waypoint.
-- Cleans up temporary taxinodes and segments added during start-point search,
-- then calls apply_runway_axis_filter() and logs the final drive route.
-- ====================================================
function process_possible_routes()

    if #t_possible_route == 0 then
        return
    end

    local l_new_head, l_new_dist = 0, 0
    local l_index = 1
    local l_node = 0
    local l_min_dist_btw_nodes = min_rot_radius * 2 + car_rear_wheel_to_ref
    local t_route_nodes = {}
    local routenode_cnt = 1

    t_node = {}

    for l_node in t_possible_route[1].Route:gmatch("[^%s]+") do
        t_route_nodes[routenode_cnt] = l_node
        routenode_cnt = routenode_cnt + 1
    end

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

    apply_runway_axis_filter()

    if depart_arrive == 1 and #t_node > 0 then
        local l_rwy_x, l_rwy_y, l_rwy_z = 0, 0, 0
        local l_far_x, l_far_z = 0, 0
        local l_rwy_idx = 0
        for l_index = 1, #t_runway do
            if t_runway[l_index].ID == depart_runway then
                l_rwy_idx = l_index
                l_rwy_x = t_runway[l_index].x
                l_rwy_y = probe_y(t_runway[l_index].x, 0, t_runway[l_index].z)
                l_rwy_z = t_runway[l_index].z
                break
            end
        end
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
        if l_far_x == 0 and l_far_z == 0 then
            for l_index = 1, #t_runway do
                if t_runway[l_index].ID ~= depart_runway then
                    l_far_x = t_runway[l_index].x
                    l_far_z = t_runway[l_index].z
                end
            end
            logMsg("FollowMe : RWY " .. depart_runway .. " Pair fallback used (scan)")
        end
        local l_ax = l_rwy_x - l_far_x
        local l_az = l_rwy_z - l_far_z
        local l_alen = math.sqrt(l_ax * l_ax + l_az * l_az)
        if l_alen < 1 then
            l_alen = 1
        end
        local l_ux = l_ax / l_alen
        local l_uz = l_az / l_alen
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
            t_node[#t_node].hotzone = "1"
        end
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
            "FollowMe : DRIVE ROUTE" .. " AIRPORT=" .. curr_icao .. " FROM=" .. l_from_lbl .. " TO=" .. l_to_lbl
        )
        logMsg("FollowMe : DRIVE NODES : " .. l_node_list)
        logMsg("FollowMe : DRIVE NODES TOTAL=" .. #t_node)
        if #t_node >= 2 then
            local l_fin_x = t_node[#t_node].x
            local l_fin_z = t_node[#t_node].z
            local l_prev_dte = math.huge
            local l_n_regressions = 0
            for l_ri = 1, #t_node do
                local _, l_dte = heading_n_dist(t_node[l_ri].x, t_node[l_ri].z, l_fin_x, l_fin_z)
                if l_dte > l_prev_dte then
                    l_n_regressions = l_n_regressions + 1
                    logMsg(string.format(
                        "FollowMe : *** REGRESSION ***  driveNode=%d  x=%.1f  z=%.1f  dToEnd=%.1fm  prevDToEnd=%.1fm  delta=+%.1fm",
                        l_ri, t_node[l_ri].x, t_node[l_ri].z,
                        l_dte, l_prev_dte, l_dte - l_prev_dte))
                end
                l_prev_dte = l_dte
            end
            if l_n_regressions == 0 then
                logMsg("FollowMe : DRIVE NODES regression check PASSED (no U-turns / detours)")
            else
                logMsg(string.format(
                    "FollowMe : DRIVE NODES regression check FAILED  regressions=%d  → route has detour(s) causing off-pavement driving",
                    l_n_regressions))
            end
        end
    end
end

-- ====================================================
-- Function: determine_pos_on_segment
-- Description:
-- Finds the best entry point onto the taxiway network from a given position and heading.
-- For gate/misc/hangar types: checks perpendicular intersections with all segments
-- and dead-end nodes within visibility range.
-- For tie-down types: also checks tangent projection points onto segments.
-- Chooses between the intersection and dead-end candidates based on closest distance.
-- Falls back to the nearest connected taxinode if no intersection is found.
-- On success, inserts a new taxinode and segment pair at the intersection point via
-- add_new_taxinode_segment().
-- ====================================================
function determine_pos_on_segment(in_heading, in_x, in_z, in_type)
    local l_within_sight, l_intersect_x, l_intersect_z, l_dist_to_intersect = false, 0, 0, 0
    local l_ret_intersect_dist = 9999
    local l_ret_segment_index, l_ret_x, l_ret_z = 0, 0, 0
    local l_deadnode, l_dist_to_deadnode
    local l_ret_node_dist = 9999
    local l_ret_deadnode = 0
    local l_in_rev_heading = 0
    local l_goto_node = 0
    local l_min_dist = 25
    local l_max_dist = 300
    local l_max_dist_node = 130
    local l_aircraft_type = tonumber(aircraft_type)

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
                    l_ret_node_dist = l_dist_to_deadnode
                end
            end
        end
        if (in_type == "tie_down" or in_type == "hangar") then
            l_within_sight, l_deadnode, l_dist_to_deadnode = check_deadend_node(in_type, in_heading, in_x, in_z, i)
            if l_within_sight then
                if l_dist_to_deadnode <= l_max_dist_node and l_dist_to_deadnode <= l_ret_node_dist then
                    l_ret_deadnode = l_deadnode
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
-- Computes the perpendicular (tangent) projection distance from a given point
-- and heading onto a taxiway segment, used for tie-down and hangar entry points.
-- Checks whether either segment endpoint is visible from the given position,
-- then calculates the tangent foot point and verifies it lies within the segment bounds.
-- Returns visibility flag, tangent distance, tangent coordinates, and the target node.
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

    if ((l_tangent_x >= l_node1_x and l_tangent_x <= l_node2_x) or
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
-- Computes the geometric intersection point between a ray (from in_x/in_z along in_heading)
-- and a taxiway segment, handling vertical, horizontal, and diagonal segment orientations.
-- Returns false if the ray is parallel to the segment or the intersection falls outside
-- the segment extents.
-- Validates the intersection is within the line-of-sight arc before accepting it.
-- ====================================================
function compute_intersection(in_type, in_heading, in_x, in_z, in_idx)
    local l_node1_x = t_taxinode[t_segment[in_idx].Node1 + 1].x
    local l_node1_z = t_taxinode[t_segment[in_idx].Node1 + 1].z
    local l_node2_x = t_taxinode[t_segment[in_idx].Node2 + 1].x
    local l_node2_z = t_taxinode[t_segment[in_idx].Node2 + 1].z
    local l_seg_heading = t_segment[in_idx].Heading
    local l_intersect_x, l_intersect_z = 0, 0

    if l_seg_heading == in_heading or l_seg_heading == add_delta_clockwise(in_heading, 180, 1) then
        return false, 0, 0, 0
    end

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

    if ((l_intersect_x >= l_node1_x and l_intersect_x <= l_node2_x) or
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

    if not l_in_sight and (in_type == "tie_down" or in_type == "hangar") then
        l_heading = add_delta_clockwise(in_heading, 180, 1)
        l_in_sight, _, l_dist_to_intersect =
            chk_line_of_sight(l_heading, 70, 70, in_x, in_z, l_intersect_x, l_intersect_z)
    end

    return l_in_sight, l_intersect_x, l_intersect_z, l_dist_to_intersect
end

-- ====================================================
-- Function: check_deadend_node
-- Description:
-- Checks whether one of the two endpoint nodes of a taxiway segment is a dead end
-- (connected to only one segment) and whether that node is visible from the given position.
-- Returns the dead-end node index, visibility flag, and distance if found.
-- Used during start-point search to connect the car directly to isolated stub taxiways.
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
-- Inserts a synthetic taxiway node at the computed intersection or tangent point
-- and creates two new segments linking it to the original segment's endpoint nodes.
-- Updates the Segment connection strings of all affected nodes to include the new segments.
-- Skips connecting to an endpoint node if its Segment field is already empty (pruned node).
-- Returns the new node index and the last created segment index.
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

    local l_n1_seg  = t_taxinode[t_segment[in_segment_index].Node1 + 1].Segment
    local l_n2_seg  = t_taxinode[t_segment[in_segment_index].Node2 + 1].Segment
    local l_n1_dead = (l_n1_seg ~= "" and not string.find(l_n1_seg, ","))
    local l_n2_dead = (l_n2_seg ~= "" and not string.find(l_n2_seg, ","))

    logMsg(string.format(
        "FollowMe : add_new_taxinode_segment  seg=%d  N1=%d(seg='%s' deadend=%s)  N2=%d(seg='%s' deadend=%s)  newNode=%d",
        in_segment_index,
        t_segment[in_segment_index].Node1, l_n1_seg, tostring(l_n1_dead),
        t_segment[in_segment_index].Node2, l_n2_seg, tostring(l_n2_dead),
        l_new_node))

    if l_n1_seg ~= "" then
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
        logMsg(string.format("FollowMe :   N1 connected  N1(%d)→newNode(%d) newSeg=%d%s",
            t_segment[l_new_segment].Node1, l_new_node, l_new_segment,
            l_n1_dead and "  [DEAD-END FIX-A applied]" or ""))
    else
        logMsg(string.format("FollowMe :   N1(%d) SKIPPED Segment='' (pruned)",
            t_segment[in_segment_index].Node1))
    end

    if l_n2_seg ~= "" then
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
        logMsg(string.format("FollowMe :   N2 connected  newNode(%d)→N2(%d) newSeg=%d%s",
            l_new_node, t_segment[l_new_segment].Node2, l_new_segment,
            l_n2_dead and "  [DEAD-END FIX-A applied]" or ""))
    else
        logMsg(string.format("FollowMe :   N2(%d) SKIPPED Segment='' (pruned)",
            t_segment[in_segment_index].Node2))
    end

    return l_new_node, l_new_segment
end

-- ====================================================
-- Function: auto_assign_gate
-- Description:
-- Automatically selects a random arrival gate suitable for the current aircraft type.
-- Filters t_gate to build a list of suitable gates matching aircraft_type,
-- then picks one at random.
-- If no suitable gate exists, falls back to a fully random gate and shows a warning.
-- Updates arrival_gate, gatetext, and triggers rampstart_chg.
-- ====================================================
function auto_assign_gate()
    local l_index = 0

    if #t_gate == 0 then
        arrival_gate = 0
        return "-1"
    end

    t_suitable_gates = {}

    for l_index = 1, #t_gate do
        if string.match(t_gate[l_index].Types, aircraft_type) == aircraft_type or
                (aircraft_type == "0" and string.match(t_gate[l_index].Types, "7"))
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
        if (Err_Msg ~= nil and string.find(Err_Msg, "No suitable gate for this plane. Lift")) then
        else
            update_msg("-17")
        end
        return "1"
    end
end

-- ====================================================
-- Function: check_gate
-- Description:
-- Checks whether the aircraft is currently parked within 10 meters of any gate in t_gate.
-- Only runs when the aircraft is stationary (ground speed < 0.5 m/s).
-- Returns the index of the closest gate within range, or 0 if none found.
-- Sets rampstart_chg to true when a gate match is found.
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
-- Function: load_config
-- Description:
-- Reads the plugin preference file (FollowMeXplane12.prf) from the X-Plane
-- Output/preferences directory.
-- Restores settings: car type, speed limiter, random gate, show path, show rampstart,
-- SimBrief ID, volume, and per-ICAO aircraft type assignments stored in t_aircraft.
-- Returns error codes if the file is missing or the current aircraft type is not found.
-- ====================================================
function load_config()
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
            if l_str1 == "car_type_fmcar" then
                if l_str2 == "Ferrari" then
                    car_type_fmcar = "Ferrari"
                elseif l_str2 == "Van" then
                    car_type_fmcar = "Van"
                elseif l_str2 == "Truck" then
                    car_type_fmcar = "Truck"
                elseif l_str2 == "Auto" then
                    car_type_fmcar = "Auto"
                end
            elseif l_str1 == "random_gate" then
                random_gate = (l_str2 == "1")
            elseif l_str1 == "show_path" then
                show_path = (l_str2 == "1")
            elseif l_str1 == "show_rampstart" then
                rampstart_chg = true
                show_rampstart = (l_str2 == "1")
            elseif l_str1 == "simbrief_id" then
                simbrief_id = l_str2 or ""
            elseif l_str1 == "speed_limiter" then
                speed_limiter = (l_str2 == "1")
            elseif l_str1 == "vol" then
                vol = tonumber(l_str2)
                set_sound_vol()
            elseif l_str1 ~= nil and l_str1 ~= "" then
                t_aircraft[l_str1] = l_str2
                if l_str1 == PLANE_ICAO then
                    l_aircraft_type = l_str2
                    aircraft_type = l_str2
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
-- Serializes all current plugin settings to the preference file (FollowMeXplane12.prf).
-- Writes volume, car type, speed limiter, random gate, show path, show rampstart,
-- SimBrief ID, and the full t_aircraft table (per-ICAO aircraft type assignments).
-- Returns error code -4 if the file cannot be opened, or '2' on success.
-- ====================================================
function save_config()
    local l_file
    local l_content = ""

    l_content = l_content .. "vol" .. "\t" .. tostring(vol) .. "\n"
    l_content = l_content .. "car_type_fmcar" .. "\t" .. car_type_fmcar .. "\n"
    l_content = l_content .. "speed_limiter" .. "\t" .. (speed_limiter and "1" or "0") .. "\n"
    l_content = l_content .. "random_gate" .. "\t" .. (random_gate and "1" or "0") .. "\n"
    l_content = l_content .. "show_path" .. "\t" .. (show_path and "1" or "0") .. "\n"
    l_content = l_content .. "show_rampstart" .. "\t" .. (show_rampstart and "1" or "0") .. "\n"
    l_content = l_content .. "simbrief_id" .. "\t" .. simbrief_id .. "\n"
    t_aircraft[PLANE_ICAO] = aircraft_type

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
-- Removes leading and trailing whitespace from a string.
-- Returns an empty string if the input is nil or contains only whitespace.
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
-- Cleanup function called when the plugin exits (registered with do_on_exit).
-- Calls full_reset() to unload all 3D objects and clear state.
-- Unloads the terrain probe, unregisters the fm/anim/sign custom dataref,
-- and removes the FollowMe entry from the X-Plane Plugins menu.
-- ====================================================
function exit_plugin()
    full_reset()
    unload_probe()
    XPLM.XPLMUnregisterDataAccessor(dr_sign)
    dr_sign = nil

    if my_menu then
        XPLM.XPLMClearAllMenuItems(my_menu)
        XPLM.XPLMDestroyMenu(my_menu)
        my_menu = nil
    end

    if plugins_menu and my_menu_item then
        XPLM.XPLMRemoveMenuItem(plugins_menu, my_menu_item)
        my_menu_item = nil
    end
end

-- ====================================================
-- Function: update_menu_state
-- Description:
-- Enables or disables the two items in the FollowMe Plugins menu based on
-- the current plugin state.
-- The Follow Me Window item is enabled only when the aircraft is on the ground,
-- the car is not active, and neither window is open.
-- The Navigation Window item is enabled only when the car is active and
-- the navigation window is not already open.
-- ====================================================
function update_menu_state()
    if my_menu == nil then return end

    local fm_available = (not we_fly)
                     and (not fm_car_active)
                     and (followme_wnd == nil)
                     and (navigation_wnd == nil)

    XPLM.XPLMEnableMenuItem(my_menu, 0, fm_available and 1 or 0)

    local nav_available = (not we_fly)
                      and fm_car_active
                      and (navigation_wnd == nil)

    XPLM.XPLMEnableMenuItem(my_menu, 1, nav_available and 1 or 0)
end

--                  ======================================================
--                  = MAIN SECTION (Initialization and flywithlua event) =
--                  ======================================================

XPLM.XPLMGetSystemPath(char_str)
syspath = ffi.string(char_str)

-- =================
-- = Prepare menus =
-- =================
menu_handler = ffi.cast("XPLMMenuHandler_f", function(inMenuRef, inItemRef)
    local item = tonumber(ffi.cast("intptr_t", inItemRef))
    if we_fly then
    else
	    if item == 0 and followme_wnd == nil then
	        followme_window_open = true
	        navigation_window_open = false
	    	show_followme_window()
	    	hide_navigation_window()
	    elseif item == 1 and navigation_wnd == nil then
	        followme_window_open = false
	        navigation_window_open = true
	        hide_followme_window()
	        show_navigation_window()
	    end
    end
end)

plugins_menu = XPLM.XPLMFindPluginsMenu()
my_menu_item = XPLM.XPLMAppendMenuItem(plugins_menu, "FollowMe", nil, 0)
my_menu = XPLM.XPLMCreateMenu("FollowMe", plugins_menu, my_menu_item, menu_handler, nil)

XPLM.XPLMAppendMenuItem(my_menu, "Follow Me Window", ffi.cast("void*", 0), 0)
XPLM.XPLMAppendMenuItem(my_menu, "Navigation Window", ffi.cast("void*", 1), 0)

update_menu_state(0)

-- ========================
-- = Other initialization =
-- ========================
register_dataref()
load_probe()
do_every_frame("handle_plugin_window()")
do_every_frame("object_physics()")
do_on_exit("exit_plugin()")