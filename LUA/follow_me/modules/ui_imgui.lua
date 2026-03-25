-- =============================================================================
--  ui_imgui.lua
--  FollowMe Ver 1.19 — Module 5 : Interface ImGui
--
--  Contient :
--    - Fenêtre principale (build_window)
--    - Holder FM (petit bouton cliquable)
--    - Gestion des messages d'erreur/info (Err_Msg[])
--    - Callbacks ImGui (closed_window, closed_holder)
--  N'effectue aucun calcul de physique ou de pathfinding.
-- =============================================================================

-- =============================================================================
--  Variables d'état UI
-- =============================================================================
window_is_open       = false
toggle_window        = false
window_first_access  = true
holder_drag          = 0
ignore_next_release  = 0

-- Messages — tableau de 3 slots (index 1=principal, 2=info, 3=warning)
Err_Msg = { {}, {}, {} }

-- Messages connus — traduction code → texte
local MSG_TEXT = {
    ["-1"]  = "Aircraft type not defined, please set it in the FM window.",
    ["-2"]  = "Preferences file not found — default settings applied.",
    ["-3"]  = "Cannot save preferences.",
    ["-4"]  = "Cannot write preferences file.",
    ["-5"]  = "Departure runway not set.",
    ["-6"]  = "Arrival gate not set.",
    ["-11"] = "scenery_packs.ini not found.",
    ["-12"] = "Cannot find start position on taxiway network.",
    ["-13"] = "Cannot find arrival gate on taxiway network.",
    ["-14"] = "No path found — please set manually.",
    ["-15"] = "No taxiway data found for this airport.",
    ["-16"] = "Arrival position not found — please check your position.",
    ["-17"] = "No suitable gate for this plane — random gate assigned.",
    ["-18"] = "Runway not found in taxiway data.",
    ["2"]   = "Preferences saved.",
    ["4"]   = "Route found without size restrictions.",
}

function update_msg(in_code)
    Err_Msg[1].code = in_code
    Err_Msg[1].text = MSG_TEXT[in_code] or ("FM info: "..tostring(in_code))
    fm_log("FollowMe MSG "..tostring(in_code)..": "..(Err_Msg[1].text or ""))
end

-- =============================================================================
--  Callbacks fenêtre ImGui
-- =============================================================================
function closed_window(wnd) window_is_open = false end
function closed_holder(wnd) end

-- =============================================================================
--  Holder — petit bouton FM
-- =============================================================================
function build_holder(wnd, x, y)
    local lw = imgui.GetWindowWidth()
    local lh = imgui.GetWindowHeight()
    local cx = lw / 2
    local cy = lh / 2
    local l_in_flight = (fm_gear1_gnd == 0 and fm_gear2_gnd == 0)
    local l_color = l_in_flight and 0xFF666666 or 0xFFFFFFFF

    imgui.PushStyleColor(imgui.constant.Col.Text, l_color)
    local tw, th = imgui.CalcTextSize("FM")
    imgui.SetCursorPos(cx - tw/2, cy - th/2)
    imgui.TextUnformatted("FM")
    imgui.PopStyleColor()
    imgui.DrawList_AddCircle(cx, cy, 12, l_color)

    if l_in_flight then
        local r = 10
        imgui.DrawList_AddLine(cx+r, cy-r, cx-r, cy+r, 0xFF0000FF, 2)
    end

    if imgui.IsMouseReleased(0) then
        if ignore_next_release > 0 then
            ignore_next_release = ignore_next_release - 1
        elseif fm_gear1_gnd ~= 0 and fm_gear2_gnd ~= 0 then
            if holder_drag < 2 then toggle_window = true end
            holder_drag = 0
        end
    end
end

-- =============================================================================
--  Fenêtre principale
-- =============================================================================
function build_window(wnd, x, y)
    local l_width = imgui.GetWindowWidth()

    -- ---- En-tête titre -------------------------------------------------------
    imgui.PushStyleColor(imgui.constant.Col.Text, 0xFF00BFFF)
    imgui.TextUnformatted("  FollowMe  |  " .. curr_ICAO_Name)
    imgui.PopStyleColor()
    imgui.Separator()

    -- ---- Section aéroport / type avion --------------------------------------
    imgui.TextUnformatted("Airport : " .. curr_ICAO)
    imgui.SameLine(); imgui.TextUnformatted("  |  Aircraft : " .. PLANE_ICAO)

    imgui.Spacing()
    imgui.TextUnformatted("Aircraft type :"); imgui.SameLine()
    local l_types = {"0-Special","1-Super","2-Heavy","3-Large Jet","4-Large Prop",
                     "5-Med Jet","6-Med Prop","7-Small","8-Light"}
    local l_cur   = tonumber(Aircraft_Type) or 8
    if imgui.BeginCombo("##type", l_types[l_cur+1] or "?") then
        for i,v in ipairs(l_types) do
            if imgui.Selectable(v, (i-1)==l_cur) then
                Aircraft_Type = tostring(i-1)
                save_config()
            end
        end
        imgui.EndCombo()
    end
    imgui.Separator()

    -- ---- Section départ / arrivée -------------------------------------------
    if imgui.RadioButton("Departure", depart_arrive == 1) then
        depart_arrive = 1
        initialise_routes()
    end
    imgui.SameLine()
    if imgui.RadioButton("Arrival", depart_arrive == 2) then
        depart_arrive = 2
        initialise_routes()
    end

    imgui.Spacing()

    -- Départ
    if depart_arrive == 1 then
        imgui.TextUnformatted("Runway :"); imgui.SameLine()
        if imgui.BeginCombo("##rwy", (depart_runway ~= "") and depart_runway or "Select...") then
            for i=1,#t_runway do
                if imgui.Selectable(t_runway[i].ID, t_runway[i].ID == depart_runway) then
                    depart_runway = t_runway[i].ID
                end
            end
            imgui.EndCombo()
        end
    end

    -- Arrivée
    if depart_arrive == 2 then
        imgui.TextUnformatted("Gate :"); imgui.SameLine()
        local l_gate_label = (arrival_gate > 0) and t_gate[arrival_gate].ID or "Select..."
        if imgui.BeginCombo("##gate", l_gate_label) then
            for i=1,#t_gate do
                if imgui.Selectable(t_gate[i].ID, i == arrival_gate) then
                    arrival_gate = i
                    gatetext = t_gate[i].ID
                    rampstart_chg = true
                end
            end
            imgui.EndCombo()
        end
        imgui.SameLine()
        if imgui.Button("Random") then
            auto_assign_gate()
        end
    end

    imgui.Separator()

    -- ---- Bouton FM principal -------------------------------------------------
    imgui.PushStyleColor(imgui.constant.Col.Button,        0xFF1A7A1A)
    imgui.PushStyleColor(imgui.constant.Col.ButtonHovered, 0xFF21A321)
    imgui.PushStyleColor(imgui.constant.Col.ButtonActive,  0xFF0D5C0D)
    if imgui.Button("  FM  ") then
        if FM_car_active then
            -- Annuler / relancer
            if prepare_kill_objects then
                full_reset()
            else
                unload_object(); unload_path(); unload_rampstart()
                FM_car_active        = false
                prepare_show_objects = false
                prepare_kill_objects = false
                kill_is_manual       = true
                initialise_routes()
            end
        else
            local l_res = determine_XP_route()
            if l_res ~= "" then
                update_msg(l_res)
            else
                prepare_show_objects = true
            end
        end
    end
    imgui.PopStyleColor(3)

    imgui.SameLine()
    imgui.TextUnformatted(FM_car_active and "Car is active" or "Car is stopped")

    imgui.Separator()

    -- ---- SimBrief -----------------------------------------------------------
    imgui.TextUnformatted("SimBrief ID :"); imgui.SameLine()
    local l_changed, l_val = imgui.InputText("##sbid", simbrief_id, 64)
    if l_changed then simbrief_id = l_val end

    imgui.SameLine()
    if imgui.Button("Fetch") then check_SimBrief() end

    if sb_fetch_status ~= "" then
        local l_col = (sb_fetch_status=="OK") and 0xFF00FF00 or 0xFF0000FF
        imgui.PushStyleColor(imgui.constant.Col.Text, l_col)
        imgui.TextUnformatted("SimBrief : "..sb_fetch_status)
        imgui.PopStyleColor()
        if sb_fetch_status=="OK" then
            imgui.TextUnformatted("  "..sb_origin_icao.." → "..sb_dest_icao)
            imgui.TextUnformatted("  RWY T/O:"..sb_runway_takeoff.."  LDG:"..sb_runway_landing)
            if sb_airport_mismatch then
                imgui.PushStyleColor(imgui.constant.Col.Text, 0xFF0080FF)
                imgui.TextUnformatted("  ⚠ Airport mismatch!")
                imgui.PopStyleColor()
            end
        end
    end

    imgui.Separator()

    -- ---- Options ------------------------------------------------------------
    local l_ch, l_sp = imgui.Checkbox("Show path",         show_path)
    if l_ch then show_path = l_sp; save_config() end

    imgui.SameLine()
    local l_ch2, l_sr = imgui.Checkbox("Show ramp starts", show_rampstart)
    if l_ch2 then show_rampstart = l_sr; rampstart_chg = true; save_config() end

    local l_ch3, l_sl = imgui.Checkbox("Speed limiter",    speed_limiter)
    if l_ch3 then speed_limiter = l_sl; save_config() end

    imgui.SameLine()
    local l_ch4, l_rg = imgui.Checkbox("Random gate",      random_gate)
    if l_ch4 then random_gate = l_rg; save_config() end

    local l_ch5, l_gs = imgui.Checkbox("Get runway from SimBrief", get_from_SimBrief)
    if l_ch5 then get_from_SimBrief = l_gs; save_config() end

    imgui.Separator()

    -- ---- Volume & type de voiture -------------------------------------------
    imgui.TextUnformatted("Volume :"); imgui.SameLine()
    local l_chv, l_vv = imgui.SliderInt("##vol", vol, 0, 10)
    if l_chv then vol = l_vv; set_sound_vol(); save_config() end

    imgui.TextUnformatted("Car type :"); imgui.SameLine()
    local l_cartypes = {"Auto","Ferrari","Van","Truck"}
    if imgui.BeginCombo("##car", car_type_fmcar) then
        for _,v in ipairs(l_cartypes) do
            if imgui.Selectable(v, v == car_type_fmcar) then
                car_type_fmcar = v
                if FM_car_active then
                    -- Recharger le modèle 3D à la volée
                    unload_object()
                    load_object()
                end
                save_config()
            end
        end
        imgui.EndCombo()
    end

    imgui.Separator()

    -- ---- Zone message -------------------------------------------------------
    if Err_Msg[1].text then
        local l_msg_col = (string.sub(Err_Msg[1].code or "0", 1, 1) == "-")
                          and 0xFF0000FF or 0xFF00FF00
        imgui.PushStyleColor(imgui.constant.Col.Text, l_msg_col)
        imgui.TextWrapped(Err_Msg[1].text)
        imgui.PopStyleColor()
    end

    -- ---- Footer version -----------------------------------------------------
    imgui.Spacing()
    imgui.PushStyleColor(imgui.constant.Col.Text, 0xFF888888)
    imgui.TextUnformatted("FollowMe v1.19  |  X-Plane 12  |  FlyWithLua NG")
    imgui.PopStyleColor()

    -- ---- Toggle window (depuis holder) --------------------------------------
    if toggle_window then
        toggle_window = false
        if window_is_open then
            float_wnd_set_visible(fm_window, false)
            window_is_open = false
        else
            float_wnd_set_visible(fm_window, true)
            window_is_open = true
            window_first_access = false
        end
    end
end

fm_log("FollowMe ui_imgui.lua chargé")
