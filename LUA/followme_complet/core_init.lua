-- =============================================================================
--  core_init.lua
--  FollowMe Ver 1.19 — Module 1 : Initialisation FFI / XPLM / Datarefs / Sons
--  Chargé UNE SEULE FOIS au démarrage par followme.lua
-- =============================================================================

-- =============================================================================
--  Système de log FollowMe → followme_log.txt à la racine de X-Plane 12
--  fm_log(msg) : écrit dans followme_log.txt (horodaté) + dans Log.txt X-Plane
--  Disponible immédiatement — fm_log_path sera nil jusqu'à l'init de syspath
-- =============================================================================
fm_log_path = nil

function fm_log(msg)
    logMsg(msg)   -- toujours dans Log.txt de X-Plane
    if fm_log_path then
        local f = io.open(fm_log_path, "a")
        if f then
            f:write(os.date("[%Y-%m-%d %H:%M:%S] ") .. msg .. "\n")
            f:close()
        end
    end
end

-- Guard : FlyWithLua doit supporter ImGui
if not SUPPORTS_FLOATING_WINDOWS then
    fm_log("FollowMe : imgui not supported by your FlyWithLua version")
    return
end

require("bit")
require("graphics")

local socket_ok, socket = pcall(require, "socket")
local http_ok,   http   = pcall(require, "socket.http")
if not socket_ok or not http_ok then
    fm_log("FollowMe ERROR - socket.lua or socket.http not found")
end

-- =============================================================================
--  FFI / XPLM
-- =============================================================================
local ffi = require("ffi")
local XPLMlib = ""

if SYSTEM == "IBM" then
    XPLMlib = (SYSTEM_ARCHITECTURE == 64) and "XPLM_64" or "XPLM"
elseif SYSTEM == "LIN" then
    XPLMlib = (SYSTEM_ARCHITECTURE == 64) and "Resources/plugins/XPLM_64.so"
                                           or  "Resources/plugins/XPLM.so"
elseif SYSTEM == "APL" then
    XPLMlib = "Resources/plugins/XPLM.framework/XPLM"
else
    return
end

XPLM = ffi.load(XPLMlib)   -- global : utilisé par tous les modules

ffi.cdef([[
enum {
     xplm_ControlCameraUntilViewChanges = 1
    ,xplm_ControlCameraForever          = 2
};
typedef struct { int structSize; float x,y,z,pitch,heading,roll;          } XPLMDrawInfo_t;
typedef struct { int structSize; float locationX,locationY,locationZ,
                                       normalX,normalY,normalZ,
                                       velocityX,velocityY,velocityZ; int is_wet; } XPLMProbeInfo_t;
typedef struct { float x,y,z,pitch,heading,roll,zoom; } XPLMCameraPosition_t;

typedef void *inRefcon;
typedef void *XPLMDataRef;
typedef void *XPLMObjectRef;
typedef void *XPLMInstanceRef;
typedef void *XPLMProbeRef;
typedef int   XPLMProbeType;
typedef int   XPLMProbeResult;
typedef int   XPLMCameraControlDuration;

typedef int   (*XPLMCameraControl_f)(XPLMCameraPosition_t*, int, void*);
typedef void  (*XPLMObjectLoaded_f)(XPLMObjectRef, void*);
typedef int   (*XPLMGetDatai_f)(void*);
typedef void  (*XPLMSetDatai_f)(void*, int);
typedef float (*XPLMGetDataf_f)(void*);
typedef void  (*XPLMSetDataf_f)(void*, float);
typedef int   (*XPLMGetDatavi_f)(void*, int*, int, int);
typedef void  (*XPLMSetDatavi_f)(void*, int*, int, int);
typedef int   (*XPLMGetDatavf_f)(void*, float*, int, int);
typedef void  (*XPLMSetDatavf_f)(void*, float*, int, int);
typedef int   (*XPLMGetDatab_f)(void*, void*, int, int);
typedef void  (*XPLMSetDatab_f)(void*, void*, int, int);

XPLMDataRef   XPLMRegisterDataAccessor(const char*, int, int,
                  XPLMGetDatai_f, XPLMSetDatai_f,
                  XPLMGetDataf_f, XPLMSetDataf_f,
                  void*, void*, void*, void*, void*, void*, void*, void*,
                  void*, void*);
XPLMObjectRef XPLMLoadObject(const char*);
void          XPLMLoadObjectAsync(const char*, XPLMObjectLoaded_f, void*);
XPLMInstanceRef XPLMCreateInstance(XPLMObjectRef, const char**);
void          XPLMInstanceSetPosition(XPLMInstanceRef, const XPLMDrawInfo_t*, const float*);
XPLMProbeRef  XPLMCreateProbe(XPLMProbeType);
XPLMProbeResult XPLMProbeTerrainXYZ(XPLMProbeRef, float, float, float, XPLMProbeInfo_t*);
void          XPLMUnregisterDataAccessor(XPLMDataRef);
void          XPLMDestroyInstance(XPLMInstanceRef);
void          XPLMUnloadObject(XPLMObjectRef);
void          XPLMDestroyProbe(XPLMProbeRef);
void          XPLMControlCamera(XPLMCameraControlDuration, XPLMCameraControl_f, void*);
void          XPLMDontControlCamera(void);
int           XPLMIsCameraBeingControlled(XPLMCameraControlDuration*);
void          XPLMWorldToLocal(double, double, double, double*, double*, double*);
void          XPLMLocalToWorld(double, double, double, double*, double*, double*);
void          XPLMGetSystemPath(char*);
XPLMDataRef   XPLMFindDataRef(const char*);
void          XPLMSetDataf(XPLMDataRef, float);
void          XPLMSetDatavf(XPLMDataRef, float*, int, int);
]])

-- Buffers FFI globaux
char_str            = ffi.new("char[256]")
datarefs_addr       = ffi.new("const char**")
dataref_name        = ffi.new("char[150]")
dataref_array       = ffi.new("const char*[7]")
dataref_array2      = ffi.new("const char*[2]")

-- VER1.17 : FindDataRef + buffers pour sync animation
dr_tire_steer       = nil
dr_tire_rotate      = nil
dr_sign             = nil
car_sign            = 0     -- valeur animation panneau (0=défaut, ne jamais laisser nil)
ffi_steer_buf       = ffi.new("float[2]")
ffi_rotate_buf      = ffi.new("float[4]")

objref              = ffi.new("XPLMObjectRef")
signboardref        = ffi.new("XPLMObjectRef")
pathref             = ffi.new("XPLMObjectRef")
rampstartref        = ffi.new("XPLMObjectRef")
proberef            = ffi.new("XPLMProbeRef")

obj_instance        = ffi.new("XPLMInstanceRef[1]")
signboard_instance  = ffi.new("XPLMInstanceRef[1]")
path_instance       = ffi.new("XPLMInstanceRef[100]")
rampstart_instance  = ffi.new("XPLMInstanceRef[1]")

objpos_addr         = ffi.new("const XPLMDrawInfo_t*")
objpos_value        = ffi.new("XPLMDrawInfo_t[1]")
float_addr          = ffi.new("const float*")
float_value         = ffi.new("float[1]")
dataref_float_addr  = ffi.new("const float*")
dataref_float_value = ffi.new("float[7]")
dataref_float_value2= ffi.new("float[2]")
probeinfo_addr      = ffi.new("XPLMProbeInfo_t*")
probeinfo_value     = ffi.new("XPLMProbeInfo_t[1]")
probetype           = ffi.new("int[1]")
x1_value            = ffi.new("double[1]")
y1_value            = ffi.new("double[1]")
z1_value            = ffi.new("double[1]")

-- =============================================================================
--  Système & Chemin
-- =============================================================================
syspath  = ""
BUFSIZE  = 102400

XPLM.XPLMGetSystemPath(char_str)
syspath = ffi.string(char_str)

-- Initialisation du fichier log FollowMe (créé / réinitialisé à chaque démarrage)
fm_log_path = syspath .. "followme_log.txt"
do
    local _lf = io.open(fm_log_path, "w")
    if _lf then
        _lf:write("============================================================\n")
        _lf:write("  FollowMe v1.19 — Log démarré le " .. os.date("%Y-%m-%d %H:%M:%S") .. "\n")
        _lf:write("  X-Plane path : " .. syspath .. "\n")
        _lf:write("============================================================\n\n")
        _lf:close()
    end
end

-- =============================================================================
--  Datarefs X-Plane (via FlyWithLua)
-- =============================================================================
fm_log("--- Enregistrement datarefs FlyWithLua ---")
dataref("viewext",           "sim/graphics/view/view_is_external")
fm_log("  viewext              : sim/graphics/view/view_is_external")
dataref("camera_z_position", "sim/graphics/view/pilots_head_z")
fm_log("  camera_z_position    : sim/graphics/view/pilots_head_z")
dataref("fm_gear1_gnd",      "sim/flightmodel2/gear/on_ground", "readonly", 0)
fm_log("  fm_gear1_gnd         : sim/flightmodel2/gear/on_ground [0]")
dataref("fm_gear2_gnd",      "sim/flightmodel2/gear/on_ground", "readonly", 1)
fm_log("  fm_gear2_gnd         : sim/flightmodel2/gear/on_ground [1]")
dataref("fm_new_flight",     "sim/time/total_flight_time_sec")
fm_log("  fm_new_flight        : sim/time/total_flight_time_sec")
dataref("fm_run_time",       "sim/time/total_running_time_sec")
fm_log("  fm_run_time          : sim/time/total_running_time_sec")
dataref("fm_sim_time",       "sim/operation/misc/frame_rate_period")
fm_log("  fm_sim_time          : sim/operation/misc/frame_rate_period")
dataref("fm_replay",         "sim/time/is_in_replay")
fm_log("  fm_replay            : sim/time/is_in_replay")
dataref("fm_plane_x",        "sim/flightmodel/position/local_x")
fm_log("  fm_plane_x           : sim/flightmodel/position/local_x")
dataref("fm_plane_y",        "sim/flightmodel/position/local_y")
fm_log("  fm_plane_y           : sim/flightmodel/position/local_y")
dataref("fm_plane_z",        "sim/flightmodel/position/local_z")
fm_log("  fm_plane_z           : sim/flightmodel/position/local_z")
dataref("fm_gnd_spd",        "sim/flightmodel/position/groundspeed",   "readonly")
fm_log("  fm_gnd_spd           : sim/flightmodel/position/groundspeed")
dataref("fm_plane_head",     "sim/flightmodel/position/psi",           "readonly")
fm_log("  fm_plane_head        : sim/flightmodel/position/psi")

-- Détection Toliss vs avion standard (VER1.4)
fm_taxi_light   = 0
fm_beacon_light = 0
if (string.lower(PLANE_AUTHOR) == "gliding kiwi") or
   (string.lower(PLANE_AUTHOR) == "glidingkiwi") then
    fm_log("  Avion Toliss détecté (PLANE_AUTHOR=" .. PLANE_AUTHOR .. ") → datarefs AirbusFBW")
    dataref("fm_taxi_light",   "AirbusFBW/OHPLightSwitches", "readonly", 3)
    dataref("fm_beacon_light", "AirbusFBW/OHPLightSwitches", "readonly", 7)
else
    fm_log("  Avion standard (PLANE_AUTHOR=" .. tostring(PLANE_AUTHOR) .. ") → datarefs sim/cockpit")
    dataref("fm_taxi_light",   "sim/cockpit/electrical/taxi_light_on",    "readonly")
    dataref("fm_beacon_light", "sim/cockpit/electrical/strobe_lights_on", "readonly")
end
fm_log("--- Datarefs FlyWithLua enregistrés ---")

-- =============================================================================
--  Sons
-- =============================================================================
fm_log("--- Chargement des sons ---")
snd_arrived        = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/arrived.wav")
fm_log("  arrived.wav            : " .. (snd_arrived        ~= nil and "OK" or "*** INTROUVABLE ***"))
snd_followme       = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/followme.wav")
fm_log("  followme.wav           : " .. (snd_followme       ~= nil and "OK" or "*** INTROUVABLE ***"))
snd_safeflight_bye = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/safeflight_goodbye.wav")
fm_log("  safeflight_goodbye.wav : " .. (snd_safeflight_bye ~= nil and "OK" or "*** INTROUVABLE ***"))
snd_welcome        = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/welcome_followme.wav")
fm_log("  welcome_followme.wav   : " .. (snd_welcome        ~= nil and "OK" or "*** INTROUVABLE ***"))
snd_welcome_bye    = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/welcomeagain_goodbye.wav")
fm_log("  welcomeagain_goodbye   : " .. (snd_welcome_bye    ~= nil and "OK" or "*** INTROUVABLE ***"))
snd_keep_speed     = load_WAV_file(SCRIPT_DIRECTORY .. "follow_me/sounds/KeepYourSpeed20Kts.wav")
fm_log("  KeepYourSpeed20Kts.wav : " .. (snd_keep_speed     ~= nil and "OK" or "*** INTROUVABLE ***"))
fm_log("--- Sons chargés ---")

function set_sound_vol()
    set_sound_gain(snd_arrived,        vol / 10)
    set_sound_gain(snd_followme,       vol / 10)
    set_sound_gain(snd_safeflight_bye, vol / 10)
    set_sound_gain(snd_welcome,        vol / 10)
    set_sound_gain(snd_welcome_bye,    vol / 10)
    set_sound_gain(snd_keep_speed,     vol / 10)
end

-- =============================================================================
--  Variables globales d'état partagées entre modules
-- =============================================================================
FM_car_active        = false
prepare_show_objects = false
prepare_kill_objects = false
kill_is_manual       = false

flightstart          = 0
ground_time          = 0
prev_new_flight      = 0
prev_plane_x         = 0
prev_plane_z         = 0
prev_taxi_light      = 0
prev_beacon_light    = 0
play_time            = 0
play_text            = ""
speed_warn_time      = 0

-- =============================================================================
--  Enregistrement dataref animation voiture (VER1.17)
--  Appelé UNE FOIS depuis followme.lua après require() des modules
-- =============================================================================
function register_dataref()
    fm_log("--- register_dataref() démarré ---")

    -- Helper : vérifie si un pointeur FFI est NULL
    local function is_null(ptr)
        return ptr == nil or tostring(ptr):find("NULL") ~= nil
    end

    dr_tire_steer  = XPLM.XPLMFindDataRef("sim/graphics/animation/ground_traffic/tire_steer_deg")
    fm_log("  dr_tire_steer  : " .. (is_null(dr_tire_steer)  and "*** INTROUVABLE (NULL) ***" or "OK"))

    dr_tire_rotate = XPLM.XPLMFindDataRef("sim/graphics/animation/ground_traffic/tire_rotation_angle_deg")
    fm_log("  dr_tire_rotate : " .. (is_null(dr_tire_rotate) and "*** INTROUVABLE (NULL) ***" or "OK"))

    local _null = ffi.cast("void*", 0)   -- NULL explicite pour FFI (NULL n'existe pas en Lua)

    if LUA_RUN == 1 then
        fm_log("  dr_sign : enregistrement via XPLMRegisterDataAccessor (LUA_RUN=1)")
        dr_sign = XPLM.XPLMRegisterDataAccessor("fm/anim/sign", 2, 0,
                      _null, _null,
                      function(inRefcon) return car_sign or 0 end,   -- jamais nil
                      _null, _null, _null, _null, _null, _null, _null, _null, _null, _null, _null)
        fm_log("  dr_sign  : " .. (is_null(dr_sign) and "*** ECHEC enregistrement ***" or "OK — enregistré"))
    else
        fm_log("  dr_sign : recherche via XPLMFindDataRef (LUA_RUN~=1, LUA_RUN=" .. tostring(LUA_RUN) .. ")")
        dr_sign = XPLM.XPLMFindDataRef("fm/anim/sign")
        fm_log("  dr_sign  : " .. (is_null(dr_sign) and "*** INTROUVABLE (NULL) ***" or "OK"))
    end

    fm_log("--- register_dataref() terminé ---")
end

-- =============================================================================
--  Probe terrain
-- =============================================================================
function load_probe()
    probeinfo_value[0].structSize = ffi.sizeof(probeinfo_value[0])
    probeinfo_addr = probeinfo_value
    probetype[1]   = 0
    proberef       = XPLM.XPLMCreateProbe(probetype[1])
end

function unload_probe()
    if proberef ~= nil then XPLM.XPLMDestroyProbe(proberef) end
    proberef = nil
end

-- =============================================================================
--  Utilitaires math / géo (utilisés par plusieurs modules)
-- =============================================================================
function heading_n_dist(in_from_x, in_from_z, in_to_x, in_to_z)
    local l_heading = math.fmod((math.deg(math.atan2(in_to_x - in_from_x,
                                                      -(in_to_z - in_from_z))) + 360), 360)
    local l_dist    = math.sqrt((in_to_x - in_from_x)^2 + (in_to_z - in_from_z)^2)
    return l_heading, l_dist
end

function add_delta_clockwise(in_heading, in_delta, in_direction)
    if in_direction == 1 then
        return math.fmod(in_heading + in_delta, 360)
    elseif in_direction == -1 then
        local h = in_heading - in_delta
        if h < 0 then h = h + 360 end
        return h
    else
        return in_heading
    end
end

function minus_delta_clockwise(in_heading, in_delta, in_direction)
    if in_direction == 1 then
        local h = in_heading - in_delta
        if h < 0 then h = h + 360 end
        return h
    elseif in_direction == -1 then
        return math.fmod(in_heading + in_delta, 360)
    else
        return in_heading
    end
end

function compute_angle_diff(in_from, in_to)
    if in_to == in_from then return 0, 0
    elseif in_to > in_from then
        if in_from + 180 > in_to then return (in_to - in_from), 1
        else return (in_from + (360 - in_to)), -1 end
    else
        if in_to + 180 > in_from then return (in_from - in_to), -1
        else return (in_to + (360 - in_from)), 1 end
    end
end

function compute_angle_diff_dir(in_from, in_to, in_dir)
    if in_to == in_from then return 0
    elseif in_dir > 0 then
        if in_to > in_from then return (in_to - in_from)
        else return (in_to + (360 - in_from)) end
    elseif in_dir < 0 then
        if in_to > in_from then return (in_from + (360 - in_to))
        else return (in_from - in_to) end
    end
end

function coordinates_of_adjusted_ref(in_ref_x, in_ref_z, in_delta_x, in_delta_z, in_heading)
    local l_dist    = math.sqrt(in_delta_x^2 + in_delta_z^2)
    local l_heading = math.fmod((math.deg(math.atan2(in_delta_x, in_delta_z)) + 360), 360)
    local l_shifted_x = in_ref_x - math.sin(math.rad(in_heading - l_heading)) * l_dist * -1
    local l_shifted_z = in_ref_z - math.cos(math.rad(in_heading - l_heading)) * l_dist
    return l_shifted_x, l_shifted_z
end

function local_to_latlon(l_x, l_y, l_z)
    x1_value[0] = l_x; y1_value[0] = l_y; z1_value[0] = l_z
    XPLM.XPLMLocalToWorld(x1_value[0], y1_value[0], z1_value[0], x1_value, y1_value, z1_value)
    return x1_value[0], y1_value[0], z1_value[0]
end

function latlon_to_local(in_lat, in_lon, in_alt)
    x1_value[0] = in_lat; y1_value[0] = in_lon; z1_value[0] = in_alt
    XPLM.XPLMWorldToLocal(x1_value[0], y1_value[0], z1_value[0], x1_value, y1_value, z1_value)
    return x1_value[0], y1_value[0], z1_value[0]
end

function probe_y(in_x, in_y, in_z)
    x1_value[0] = in_x; y1_value[0] = in_y; z1_value[0] = in_z
    XPLM.XPLMProbeTerrainXYZ(proberef, x1_value[0], y1_value[0], z1_value[0], probeinfo_addr)
    probeinfo_value = probeinfo_addr
    local l_lat, l_lon, l_alt = local_to_latlon(probeinfo_value[0].locationX,
                                                 probeinfo_value[0].locationY,
                                                 probeinfo_value[0].locationZ)
    local rx, ry, rz = latlon_to_local(l_lat, l_lon, l_alt)
    return ry
end

function get_local_coordinates(in_lat, in_lon, in_alt)
    local l_x, l_y, l_z = latlon_to_local(in_lat, in_lon, in_alt)
    x1_value[0] = l_x; y1_value[0] = l_y; z1_value[0] = l_z
    XPLM.XPLMProbeTerrainXYZ(proberef, x1_value[0], y1_value[0], z1_value[0], probeinfo_addr)
    probeinfo_value = probeinfo_addr
    local la, lo, al = local_to_latlon(probeinfo_value[0].locationX,
                                       probeinfo_value[0].locationY,
                                       probeinfo_value[0].locationZ)
    l_x, l_y, l_z = latlon_to_local(la, lo, al)
    return l_x, l_y, l_z, al
end

function trim_str(s)
    return s and s:match("^%s*(.-)%s*$") or ""
end

function chk_line_of_sight(shooter_heading, shooter_left_angle, shooter_right_angle,
                             shooter_x, shooter_z, target_x, target_z)
    local l_heading_to_target, l_dist_to_target = heading_n_dist(shooter_x, shooter_z, target_x, target_z)
    local l_left_arc  = add_delta_clockwise(shooter_heading, shooter_left_angle, -1)
    local l_right_arc = math.fmod(shooter_heading + shooter_right_angle, 360)
    local l_within_sight = false
    if l_heading_to_target >= l_left_arc and
       l_heading_to_target <= l_left_arc + shooter_left_angle + shooter_right_angle then
        l_within_sight = true
    elseif l_heading_to_target <= l_right_arc and
           l_heading_to_target >= l_right_arc - (shooter_left_angle + shooter_right_angle) then
        l_within_sight = true
    end
    return l_within_sight, l_heading_to_target, l_dist_to_target
end

fm_log("FollowMe core_init.lua chargé — syspath=" .. syspath)
