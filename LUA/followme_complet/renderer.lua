-- =============================================================================
--  renderer.lua
--  FollowMe Ver 1.19 — Module 6 : Rendu 3D XPLM & Canvas
--
--  Appelé UNIQUEMENT dans do_every_draw() — ZÉRO calcul ici.
--  Ne lit que les variables d'état produites par vehicle_physics.lua.
--  Gère aussi : chargement/déchargement des objets OBJ8 (3D).
-- =============================================================================

-- =============================================================================
--  Chemins des modèles 3D
-- =============================================================================
local OBJ_BASE   = SCRIPT_DIRECTORY .. "follow_me/objects/"

local CAR_OBJ = {
    Truck   = OBJ_BASE .. "fm_truck.obj",
    Van     = OBJ_BASE .. "fm_van.obj",
}
local SIGNBOARD_OBJ = OBJ_BASE .. "signboard.obj"
local PATH_OBJ      = OBJ_BASE .. "diamond_marker.obj"
local RAMPSTART_OBJ = OBJ_BASE .. "pushpin_yellow.obj"

-- =============================================================================
--  Chargement / Déchargement objets
-- =============================================================================
function load_object()
    local l_path = CAR_OBJ[car_type_fmcar] or CAR_OBJ["Auto"]
    objref = XPLM.XPLMLoadObject(l_path)
    if objref == nil then fm_log("FollowMe renderer : ERREUR chargement " .. l_path); return end

    dataref_array[0] = dataref_name
    ffi.copy(dataref_name, "fm/anim/sign\0")
    dataref_array[1] = nil
    obj_instance[0] = XPLM.XPLMCreateInstance(objref, dataref_array)

    signboardref = XPLM.XPLMLoadObject(SIGNBOARD_OBJ)
    if signboardref ~= nil then
        dataref_array2[0] = dataref_name
        dataref_array2[1] = nil
        signboard_instance[0] = XPLM.XPLMCreateInstance(signboardref, dataref_array2)
    end

    fm_log("FollowMe renderer : objets voiture chargés (" .. car_type_fmcar .. ")")
end

function unload_object()
    if obj_instance[0] ~= nil then
        XPLM.XPLMDestroyInstance(obj_instance[0]); obj_instance[0] = nil
    end
    if objref ~= nil then
        XPLM.XPLMUnloadObject(objref); objref = nil
    end
    if signboard_instance[0] ~= nil then
        XPLM.XPLMDestroyInstance(signboard_instance[0]); signboard_instance[0] = nil
    end
    if signboardref ~= nil then
        XPLM.XPLMUnloadObject(signboardref); signboardref = nil
    end
end

function load_path()
    if not show_path then return end
    pathref = XPLM.XPLMLoadObject(PATH_OBJ)
    if pathref == nil then return end
    for i=1,math.min(#t_node, 100) do
        local da = ffi.new("const char*[2]")
        da[0] = nil
        path_instance[i] = XPLM.XPLMCreateInstance(pathref, da)
    end
end

function unload_path()
    for i=1,100 do
        if path_instance[i] ~= nil then
            XPLM.XPLMDestroyInstance(path_instance[i]); path_instance[i] = nil
        end
    end
    if pathref ~= nil then XPLM.XPLMUnloadObject(pathref); pathref = nil end
end

function load_rampstart()
    if not show_rampstart then return end
    rampstartref = XPLM.XPLMLoadObject(RAMPSTART_OBJ)
    if rampstartref == nil then return end
    local da = ffi.new("const char*[2]"); da[0] = nil
    rampstart_instance[0] = XPLM.XPLMCreateInstance(rampstartref, da)
end

function unload_rampstart()
    if rampstart_instance[0] ~= nil then
        XPLM.XPLMDestroyInstance(rampstart_instance[0]); rampstart_instance[0] = nil
    end
    if rampstartref ~= nil then XPLM.XPLMUnloadObject(rampstartref); rampstartref = nil end
end

-- =============================================================================
--  Rendu principal — appelé par do_every_draw()
-- =============================================================================
function render_all()
    -- Gestion des transitions load/unload déclenchées par vehicle_physics
    if prepare_show_objects then
        prepare_show_objects = false
        load_probe()
        load_object()
        load_path()
        load_rampstart()
        FM_car_active = true
        start_car()
    end

    if prepare_kill_objects then
        prepare_kill_objects = false
        unload_object()
        unload_path()
        unload_rampstart()
        unload_probe()
        FM_car_active = false
    end

    if rampstart_chg then
        rampstart_chg = false
        unload_rampstart()
        load_rampstart()
    end

    -- Rendu voiture
    if FM_car_active and obj_instance[0] ~= nil then
        render_car()
        render_signboard()
    end

    -- Rendu chemin
    if show_path and pathref ~= nil then
        render_path()
    end

    -- Rendu ramp starts
    if show_rampstart and rampstartref ~= nil then
        render_rampstart()
    end
end

-- =============================================================================
--  Rendu voiture + enseigne
-- =============================================================================
function render_car()
    objpos_value[0].structSize = ffi.sizeof(objpos_value[0])
    objpos_value[0].x       = car_x
    objpos_value[0].y       = car_y
    objpos_value[0].z       = car_z
    objpos_value[0].heading = car_heading
    objpos_value[0].pitch   = 0
    objpos_value[0].roll    = 0
    objpos_addr = objpos_value

    float_value[0] = car_sign
    float_addr = float_value

    XPLM.XPLMInstanceSetPosition(obj_instance[0], objpos_addr, float_addr)
end

function render_signboard()
    if signboard_instance[0] == nil then return end
    -- Enseigne portée à 1.5m au-dessus de la voiture
    objpos_value[0].structSize = ffi.sizeof(objpos_value[0])
    objpos_value[0].x       = car_x
    objpos_value[0].y       = car_y + 1.5
    objpos_value[0].z       = car_z
    objpos_value[0].heading = car_heading
    objpos_value[0].pitch   = 0
    objpos_value[0].roll    = 0
    objpos_addr = objpos_value

    dataref_float_value2[0] = car_sign
    dataref_float_addr = dataref_float_value2
    XPLM.XPLMInstanceSetPosition(signboard_instance[0], objpos_addr, dataref_float_addr)
end

-- =============================================================================
--  Rendu chemin (waypoints sphères)
-- =============================================================================
function render_path()
    for i=1,math.min(#t_node, 100) do
        if path_instance[i] ~= nil then
            local lx = t_node[i].x or 0
            local ly = t_node[i].y or 0
            local lz = t_node[i].z or 0
            objpos_value[0].structSize = ffi.sizeof(objpos_value[0])
            objpos_value[0].x = lx; objpos_value[0].y = ly + 0.3; objpos_value[0].z = lz
            objpos_value[0].heading = 0; objpos_value[0].pitch = 0; objpos_value[0].roll = 0
            objpos_addr = objpos_value
            float_value[0] = (t_node[i].hotzone == "1") and 1.0 or 0.0
            float_addr = float_value
            XPLM.XPLMInstanceSetPosition(path_instance[i], objpos_addr, float_addr)
        end
    end
end

-- =============================================================================
--  Rendu ramp start (gate de départ)
-- =============================================================================
function render_rampstart()
    if rampstart_instance[0] == nil then return end
    if depart_gate == 0 or depart_gate > #t_gate then return end
    objpos_value[0].structSize = ffi.sizeof(objpos_value[0])
    objpos_value[0].x       = t_gate[depart_gate].x
    objpos_value[0].y       = t_gate[depart_gate].y
    objpos_value[0].z       = t_gate[depart_gate].z
    objpos_value[0].heading = t_gate[depart_gate].Heading or 0
    objpos_value[0].pitch   = 0
    objpos_value[0].roll    = 0
    objpos_addr = objpos_value
    float_value[0] = 0
    float_addr = float_value
    XPLM.XPLMInstanceSetPosition(rampstart_instance[0], objpos_addr, float_addr)
end

-- =============================================================================
--  Nettoyage complet à la fermeture
-- =============================================================================
function renderer_cleanup()
    unload_object()
    unload_path()
    unload_rampstart()
    unload_probe()
    fm_log("FollowMe renderer : cleanup terminé")
end

fm_log("FollowMe renderer.lua chargé")
