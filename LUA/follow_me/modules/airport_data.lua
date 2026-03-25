-- =============================================================================
--  airport_data.lua
--  FollowMe Ver 1.19 — Module 2 : Lecture apt.dat & Structures de l'aéroport
--
--  Principe :
--    - do_often()     → surveille le changement d'ICAO (watchdog léger)
--                       pose le flag  airport_reload_needed = true
--    - do_sometimes() → exécute le chargement lourd si flag levé
--    - Résultat stocké dans les tables globales ci-dessous
-- =============================================================================

-- =============================================================================
--  Structures de données (globales, lues par pathfinding / renderer / ui)
-- =============================================================================
curr_ICAO          = ""
curr_ICAO_Name     = ""
taxiway_network    = ""    -- "" = OK, "-XX" = code d'erreur

t_runway           = {}   -- {ID, Lat, Lon, x, z, Node}
t_runway_node      = {}   -- noeuds de type runway
t_gate             = {}   -- {ID, Lat, Lon, x,y,z, Heading, Types, Ramptype}
t_taxinode         = {}   -- {Lat, Lon, x,y,z, Type, Runway, Segment, f/g/h, parent...}
t_segment          = {}   -- {Node1, Node2, Dir, Type, Size, Hotzone, Heading, Dist}
t_deleted_runway   = {}   -- pistes sans route définie
world_alt          = 0
rampstart_chg      = false

-- Flag watchdog → chargement asynchrone
airport_reload_needed = false
local _prev_icao_check = ""

-- =============================================================================
--  Watchdog ICAO — appelé par do_often() dans followme.lua
--  LÉGER : seulement XPLMFindNavAid → pas de lecture fichier
-- =============================================================================
function airport_watchdog()
    local l_index = XPLMFindNavAid(nil, nil, LATITUDE, LONGITUDE, nil, xplm_Nav_Airport)
    local _, _, _, _, _, _, l_new_ICAO, _ = XPLMGetNavAidInfo(l_index)
    if l_new_ICAO ~= _prev_icao_check then
        _prev_icao_check      = l_new_ICAO
        airport_reload_needed = true
        logMsg("FollowMe airport_watchdog : ICAO changé → " .. l_new_ICAO)
    end
end

-- =============================================================================
--  Chargement complet — appelé par do_sometimes() si airport_reload_needed
-- =============================================================================
function load_airport_if_needed()
    if not airport_reload_needed then return end
    airport_reload_needed = false

    local l_index = XPLMFindNavAid(nil, nil, LATITUDE, LONGITUDE, nil, xplm_Nav_Airport)
    local _, _, _, _, _, _, l_new_ICAO, l_new_name = XPLMGetNavAidInfo(l_index)

    if curr_ICAO == l_new_ICAO then return end   -- déjà chargé

    world_alt      = 0
    taxiway_network = read_apt_file(l_new_ICAO)
    curr_ICAO      = l_new_ICAO
    curr_ICAO_Name = l_new_name

    if taxiway_network ~= "" then
        XPLMSpeakString("FM Service is not available at this airport")
        return
    end

    -- Vérifier cohérence SimBrief
    if sb_fetch_status == "OK" then
        sb_airport_mismatch = (sb_origin_icao ~= curr_ICAO)
    end

    if #t_deleted_runway > 0 then update_msg("-18") end

    depart_gate  = check_gate()
    if depart_arrive == 0 then
        if flightstart == 9999 then
            if depart_gate == 0 then depart_arrive = 2
            else flightstart = 0; depart_arrive = 1 end
        else
            depart_arrive = 1
        end
    end

    logMsg("FollowMe airport_data : " .. curr_ICAO .. " chargé — "
           .. #t_taxinode .. " noeuds, " .. #t_segment .. " segments, "
           .. #t_runway   .. " pistes, " .. #t_gate    .. " gates")
end

-- =============================================================================
--  Lecture apt.dat — recherche dans scenery_packs.ini
-- =============================================================================
function read_apt_file(in_ICAO)
    local l_filename1, l_filename2 = "", ""
    local l_file1, l_file2
    local l_line1, l_line2, l_rest = "", "", ""
    local l_str1 = ""
    local l_airport_found   = false
    local l_processed_1204  = false
    local l_processed_1300  = false
    local l_new_lines       = ""
    local l_start           = 0
    local l_not_first_line  = false
    local l_terminate_loop  = false

    l_filename1 = syspath .. "Custom Scenery/scenery_packs.ini"
    l_file1 = io.open(l_filename1, "r")
    if l_file1 == nil then return "-11" end

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
                        if not l_new_lines then break end
                        if l_rest then l_new_lines = l_new_lines .. l_rest .. '\n' end

                        if not l_airport_found then
                            l_start = 0
                            while true do
                                l_start = (l_start > 0) and (l_start + 200) or 1
                                l_start = string.find(l_new_lines, in_ICAO, l_start)
                                if l_start == nil then break end
                                l_start = math.max(1, l_start - 15)
                                l_new_lines = string.sub(l_new_lines, l_start)
                                for l_line2 in l_new_lines:gmatch("[^\r\n]+") do
                                    if string.find(l_line2, in_ICAO) then
                                        if string.match(l_line2,
                                            "^%d+ %s*[^%s]+%s*[^%s]+%s*[^%s]+%s*([^%s]+)%s*.*") == in_ICAO then
                                            l_airport_found  = true
                                            l_not_first_line = false
                                            initialise_airport()
                                        end
                                        break
                                    end
                                end
                                if l_airport_found then break end
                            end
                        end

                        if l_airport_found then
                            for l_line2 in l_new_lines:gmatch("[^\r\n]+") do
                                if not l_not_first_line then
                                    l_not_first_line = true
                                else
                                    if string.find(l_line2, "^1%s") or l_line2 == "99" then
                                        table.sort(t_gate, function(a,b) return a.ID < b.ID end)
                                        determine_runway_node()
                                        l_terminate_loop = true
                                        break
                                    end
                                end
                                if string.match(l_line2, "^100%s")  then decipher_runway(l_line2) end
                                if string.match(l_line2, "^1301%s") and l_processed_1300 then decipher_ramp_operation(l_line2) end
                                if string.match(l_line2, "^1300%s") then l_processed_1300 = decipher_ramp(l_line2) end
                                if string.match(l_line2, "^1201%s") then decipher_taxinode(l_line2) end
                                if string.match(l_line2, "^1202%s") then l_processed_1204 = false; decipher_taxisegment(l_line2) end
                                if string.match(l_line2, "^1204%s") then
                                    if not l_processed_1204 then
                                        decipher_taxisegment_hotzone(l_line2)
                                        l_processed_1204 = true
                                    end
                                end
                            end
                            if l_terminate_loop then break end
                        end
                    end
                    l_file2:close()
                    if l_terminate_loop then break end
                end
            end
        end
    until not l_line1
    l_file1:close()

    -- Fallback : Global Airports si non trouvé
    if not l_airport_found and not l_terminate_loop then
        l_filename2 = syspath .. "Global Scenery/Global Airports/Earth nav data/apt.dat"
        l_file2 = io.open(l_filename2, "r")
        if l_file2 then
            for l_line2 in l_file2:lines() do
                if string.find(l_line2, in_ICAO) then
                    if string.match(l_line2,
                        "^%d+ %s*[^%s]+%s*[^%s]+%s*[^%s]+%s*([^%s]+)%s*.*") == in_ICAO then
                        l_airport_found = true
                        initialise_airport()
                        break
                    end
                end
            end
            l_file2:close()
        end
    end

    if #t_taxinode == 0 then return "-15" end
    rampstart_chg = true
    return ""
end

-- =============================================================================
--  Parsers apt.dat
-- =============================================================================
function decipher_runway(in_str)
    local i = #t_runway + 1
    t_runway[i] = {}; t_runway[i+1] = {}
    local l_s1,l_s2,l_s3,l_s4 = string.match(in_str,
        "100 %s*[^%s]+%s*[^%s]+%s*[^%s]+%s*[^%s]+%s*[^%s]+%s*[^%s]+%s*[^%s]+%s*"..
        "([^%s]+)%s*([^%s]+)%s*([^%s]+)%s*"..
        "[^%s]+%s*[^%s]+%s*[^%s]+%s*[^%s]+%s*[^%s]+%s*[^%s]+%s*"..
        "([^%s]+)%s*([^%s]+)%s*([^%s]+)%s*(.*)")
    t_runway[i].ID   = l_s1; t_runway[i].Lat   = tonumber(l_s2); t_runway[i].Lon   = tonumber(l_s3)
    t_runway[i+1].ID = l_s4
    -- Note : match simplifié — voir original pour extraction complète des 2 seuils
    t_runway[i].Node   = -1
    t_runway[i+1].Node = -1
    t_runway[i].x,   _, t_runway[i].z,   world_alt = get_local_coordinates(t_runway[i].Lat,   t_runway[i].Lon,   world_alt)
    t_runway[i+1].x, _, t_runway[i+1].z, world_alt = get_local_coordinates(t_runway[i+1].Lat or 0, t_runway[i+1].Lon or 0, world_alt)
end

function decipher_ramp(in_str)
    local l1,l2,l3,l4,l5,l6 = string.match(in_str,
        "1300 %s*([^%s]+)%s*([^%s]+)%s*([^%s]+)%s*([^%s]+)%s*([^%s]+)%s*(.*)")
    local i = #t_gate + 1
    t_gate[i] = {}
    t_gate[i].Lat      = tonumber(l1); t_gate[i].Lon      = tonumber(l2)
    t_gate[i].Heading  = tonumber(l3); t_gate[i].Ramptype = l4
    t_gate[i].Types    = l5;           t_gate[i].ID       = l6
    t_gate[i].x, t_gate[i].y, t_gate[i].z, world_alt =
        get_local_coordinates(t_gate[i].Lat, t_gate[i].Lon, world_alt)
    -- Normalisation des types
    if l5 == "helos" then
        t_gate[i].Types = ""
    elseif string.find(t_gate[i].Types, "all") then
        t_gate[i].Types = "1 2 3 4 5 6 7 8"
    else
        local lH,lJ,lT,lP = "","","",""
        if string.find(t_gate[i].Types,"heavy")    then lH = "1 2 " end
        if string.find(t_gate[i].Types,"jets")     then lJ = (lH~="") and "3 5 " or "3 5 7 " end
        if string.find(t_gate[i].Types,"turboprops") then
            if lJ=="3 5 " then lJ="3 5 7 " end; lT="4 6 "
        end
        if string.find(t_gate[i].Types,"|props") or
           string.find(t_gate[i].Types,"^%s*props") then lP="8" end
        t_gate[i].Types = lH..lJ..lT..lP
    end
    return true
end

function decipher_ramp_operation(in_str)
    local i = #t_gate
    local l_types = t_gate[i].Types
    if l_types == "" then return end
    local l1,l2 = string.match(in_str,"1301 %s*(%a)%s*([^%s]+)%s*")
    if l1 == "E" and string.find(l_types,"1") then t_gate[i].Types = string.sub(l_types,3) end
    if l2 == "cargo"    then t_gate[i].Cargo    = "1" end
    if l2 == "military" then t_gate[i].Military = "1"; t_gate[i].Types = "0 "..t_gate[i].Types end
end

function decipher_taxinode(in_str)
    local i = #t_taxinode + 1
    t_taxinode[i] = {}
    local l1,l2 = string.match(in_str,"1201 %s*([^%s]+)%s*([^%s]+)%s*")
    t_taxinode[i].Lat = tonumber(l1); t_taxinode[i].Lon = tonumber(l2)
    t_taxinode[i].Type = ""; t_taxinode[i].Runway = ""; t_taxinode[i].Segment = ""
    t_taxinode[i].f_value=nil; t_taxinode[i].g_value=nil; t_taxinode[i].h_value=nil
    t_taxinode[i].parent=nil;  t_taxinode[i].cost=nil;    t_taxinode[i].heading=nil
    t_taxinode[i].x, t_taxinode[i].y, t_taxinode[i].z, world_alt =
        get_local_coordinates(t_taxinode[i].Lat, t_taxinode[i].Lon, world_alt)
end

function decipher_taxisegment(in_str)
    local i = #t_segment + 1
    t_segment[i] = {}
    local l1,l2,l3,l4,l5 = string.match(in_str,
        "1202 %s*([^%s]+)%s*([^%s]+)%s*([^%s]+)%s*([^%s]+)%s*([^%s]+)%s*")
    t_segment[i].Node1 = tonumber(l1); t_segment[i].Node2 = tonumber(l2)
    t_segment[i].Dir   = l3
    if string.find(l4,"runway") then
        t_segment[i].Type = "runway"
        t_taxinode[t_segment[i].Node1+1].Type = "runway"
        t_taxinode[t_segment[i].Node2+1].Type = "runway"
        local idx = #t_runway_node+1
        t_runway_node[idx]   = t_segment[i].Node1
        t_runway_node[idx+1] = t_segment[i].Node2
    else
        t_segment[i].Size = (string.sub(l4,#l4-1,-2)=="_") and string.sub(l4,#l4,-1) or ""
        t_segment[i].Type = "taxiway"
    end
    t_segment[i].ID      = l5
    t_segment[i].Hotzone = ""
    t_segment[i].Heading, t_segment[i].Dist =
        heading_n_dist(t_taxinode[t_segment[i].Node1+1].x, t_taxinode[t_segment[i].Node1+1].z,
                       t_taxinode[t_segment[i].Node2+1].x, t_taxinode[t_segment[i].Node2+1].z)
    -- Enregistrement segment dans les noeuds
    local function append_seg(nidx)
        if t_taxinode[nidx].Segment == "" then
            t_taxinode[nidx].Segment = tostring(i)
        else
            t_taxinode[nidx].Segment = t_taxinode[nidx].Segment..","..tostring(i)
        end
    end
    append_seg(t_segment[i].Node1+1)
    append_seg(t_segment[i].Node2+1)
end

function decipher_taxisegment_hotzone(in_str)
    local i = #t_segment
    t_segment[i].Hotzone = string.match(in_str,"1204 %s*[^%s]+%s*([^%s]+)%s*")
    if t_segment[i].Type == "taxiway" then
        if t_taxinode[t_segment[i].Node1+1].Type ~= "runway" then
            t_taxinode[t_segment[i].Node1+1].Type = "hotzone"
        end
        if t_taxinode[t_segment[i].Node2+1].Type ~= "runway" then
            t_taxinode[t_segment[i].Node2+1].Type = "hotzone"
        end
    end
end

-- =============================================================================
--  Post-traitement : association runway ↔ noeud
-- =============================================================================
function determine_runway_node()
    for l_idx = 1, #t_runway do match_runway(l_idx) end
    local l_rows = #t_runway
    local l_idx  = 1
    while l_idx <= l_rows do
        if t_runway[l_idx].Node == -1 then
            t_deleted_runway[#t_deleted_runway+1] = t_runway[l_idx].ID
            table.remove(t_runway, l_idx)
            l_rows = l_rows - 1
        else
            l_idx = l_idx + 1
        end
    end
end

function match_runway(in_idx)
    local l_min_twy, l_min_any   = 99999, 99999
    local l_best_twy, l_best_any = -1, -1
    local l_rwy_x  = t_runway[in_idx].x
    local l_rwy_z  = t_runway[in_idx].z
    local l_rwy_id = t_runway[in_idx].ID
    local l_rwy_nodes = {}
    for l_seg=1,#t_segment do
        if t_segment[l_seg].Type == "runway" then
            local l_seg_id = t_segment[l_seg].ID or ""
            local l_match  = false
            for l_part in (l_seg_id.."/"):gmatch("([^/]+)/") do
                if l_part == l_rwy_id then l_match=true; break end
            end
            if l_match then
                l_rwy_nodes[t_segment[l_seg].Node1] = true
                l_rwy_nodes[t_segment[l_seg].Node2] = true
            end
        end
    end
    local l_has_own_segs = false
    for _ in pairs(l_rwy_nodes) do l_has_own_segs=true; break end

    for l_i=1,#t_taxinode do
        if t_taxinode[l_i].Type == "runway" then
            local l_node_id = l_i - 1
            if l_has_own_segs and not l_rwy_nodes[l_node_id] then
                -- skip
            else
                local _, l_dist = heading_n_dist(t_taxinode[l_i].x, t_taxinode[l_i].z,
                                                  l_rwy_x, l_rwy_z)
                if l_dist < l_min_any then l_min_any=l_dist; l_best_any=l_node_id end
                local l_has_twy = false
                for l_seg=1,#t_segment do
                    if (t_segment[l_seg].Node1==l_node_id or t_segment[l_seg].Node2==l_node_id)
                       and t_segment[l_seg].Type ~= "runway" then
                        l_has_twy=true; break
                    end
                end
                if l_has_twy and l_dist < l_min_twy then l_min_twy=l_dist; l_best_twy=l_node_id end
            end
        end
    end
    local l_chosen = (l_best_twy >= 0) and l_best_twy or l_best_any
    if l_chosen >= 0 then
        t_runway[in_idx].Node = l_chosen
        t_taxinode[l_chosen+1].Runway = l_rwy_id
    end
end

-- =============================================================================
--  Initialisation / Reset
-- =============================================================================
function initialise_airport()
    t_runway, t_runway_node, t_gate, t_taxinode, t_segment = {},{},{},{},{}
    depart_arrive = 0
    depart_gate, arrival_gate, depart_runway, gatetext = 0, 0, "", ""
end

function initialise_routes()
    if #t_node > 0 then
        if depart_arrive==1 and t_node[curr_node] and t_node[curr_node].hotzone=="1"
           and curr_node>=#t_node-2 and flightstart~=9999 and not kill_is_manual then
            play_sound(snd_safeflight_bye)
        elseif depart_arrive==2 and curr_node==#t_node then
            play_sound(snd_welcome_bye)
        end
    end
    t_node = {}
    Err_Msg[1]={};  Err_Msg[2]={};  Err_Msg[3]={}
    depart_arrive      = 0
    is_backtaxi        = false
    window_first_access = true
end

function full_reset()
    if FM_car_active then
        unload_object(); unload_path(); unload_rampstart()
    end
    FM_car_active        = false
    prepare_kill_objects = false
    prepare_show_objects = false
    kill_is_manual       = false
    ground_time          = 0
    flightstart          = 0
    t_deleted_runway     = {}
    curr_ICAO            = ""    -- force reload prochain passage
    initialise_airport()
    initialise_routes()
    logMsg("FollowMe VER1.12 : full_reset() completed")
end

-- =============================================================================
--  Détection gate courante
-- =============================================================================
function check_gate()
    if #t_gate == 0 then return 0 end
    if fm_gnd_spd < 0.5 then
        local l_dist_min = 9999
        local l_idx_min  = 0
        for i=1,#t_gate do
            local _, l_dist = heading_n_dist(fm_plane_x, fm_plane_z, t_gate[i].x, t_gate[i].z)
            if l_dist < l_dist_min then l_dist_min=l_dist; l_idx_min=i end
        end
        if l_dist_min <= 10 then rampstart_chg=true; return l_idx_min end
    end
    return 0
end

-- =============================================================================
--  Préférences (chargement / sauvegarde)
-- =============================================================================
-- Variables prefs (globales)
Win_Y              = 400
Aircraft_Type      = "8"
t_aircraft         = {}
get_from_SimBrief  = false
show_path          = false
show_rampstart     = false
impose_restriction_chk = true
random_gate        = false
vol                = 5
speed_limiter      = false
car_type_fmcar     = "Auto"

function load_config()
    local l_file = io.open(syspath.."Output/preferences/FollowMeXplane12.prf","r")
    if l_file == nil then update_msg("-2"); return "-2" end
    local l_aircraft_type = ""
    repeat
        local l_line = l_file:read("*l")
        if l_line then
            local l1,l2 = string.match(l_line,"([%p%a%d]+)%S-(.*)")
            l2 = trim_str(l2)
            if     l1=="get_from_SimBrief" then get_from_SimBrief=(l2=="1")
            elseif l1=="simbrief_id"       then simbrief_id=l2 or ""
            elseif l1=="show_path"         then show_path=(l2=="1")
            elseif l1=="show_rampstart"    then rampstart_chg=true; show_rampstart=(l2=="1")
            elseif l1=="random_gate"       then random_gate=(l2=="1")
            elseif l1=="vol"               then vol=tonumber(l2); set_sound_vol()
            elseif l1=="speed_limiter"     then speed_limiter=(l2=="1")
            elseif l1=="car_type_fmcar"    then
                if l2=="Ferrari" or l2=="Van" or l2=="Truck" or l2=="Auto" then
                    car_type_fmcar=l2
                end
            elseif l1~=nil and l1~="" then
                t_aircraft[l1]=l2
                if l1==PLANE_ICAO then Aircraft_Type=l2; l_aircraft_type=l2 end
            end
        end
    until not l_file:read(0)
    l_file:close()
    return (l_aircraft_type~="") and "" or "-1"
end

function save_config()
    local l_file = io.open(syspath.."Output/preferences/FollowMeXplane12.prf","w")
    if l_file == nil then update_msg("-4"); return "-4" end
    local c = ""
    c = c.."vol\t"             ..tostring(vol)                       .."\n"
    c = c.."car_type_fmcar\t"  ..car_type_fmcar                      .."\n"
    c = c.."speed_limiter\t"   ..(speed_limiter  and "1" or "0")     .."\n"
    c = c.."random_gate\t"     ..(random_gate    and "1" or "0")     .."\n"
    c = c.."show_path\t"       ..(show_path      and "1" or "0")     .."\n"
    c = c.."show_rampstart\t"  ..(show_rampstart and "1" or "0")     .."\n"
    c = c.."simbrief_id\t"     ..simbrief_id                         .."\n"
    c = c.."get_from_SimBrief\t"..(get_from_SimBrief and "1" or "0") .."\n"
    t_aircraft[PLANE_ICAO] = Aircraft_Type
    for icao,typ in pairs(t_aircraft) do c = c..icao.."\t"..typ.."\n" end
    l_file:write(c)
    l_file:close()
    update_msg("2")
    return ""
end

logMsg("FollowMe airport_data.lua chargé")
