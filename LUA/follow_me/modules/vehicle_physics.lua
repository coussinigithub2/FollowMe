-- =============================================================================
--  vehicle_physics.lua
--  FollowMe Ver 1.19 — Module 4 : Physique voiture & Suivi de route
--
--  Appelé dans do_every_frame() → CALCUL PUR, zéro rendu ici
--  Le résultat est stocké dans les variables d'état de la voiture (ci-dessous)
--  et consommé par renderer.lua
-- =============================================================================

-- =============================================================================
--  Constantes physiques de la voiture
-- =============================================================================
car_max_speed         = 11.0   -- m/s (~21 kts)
car_min_speed         = 2.5    -- m/s
car_accel_rate        = 1.5    -- m/s²
car_decel_rate        = 2.5    -- m/s²
min_rot_radius        = 7.0    -- m (rayon de braquage minimum)
car_rear_wheel_to_ref = 2.5    -- m (distance essieu arrière → référence)
car_wheelbase         = 3.5    -- m

-- =============================================================================
--  État courant de la voiture (lu par renderer.lua)
-- =============================================================================
car_x            = 0.0
car_y            = 0.0
car_z            = 0.0
car_heading      = 0.0
car_speed        = 0.0
car_steer_angle  = 0.0    -- degrés
car_sign         = 0      -- 0=FM, 1=STOP, 2=ARRIVED (dataref animation)
curr_node        = 1      -- index du noeud cible dans t_node[]
car_last_heard   = 0      -- anti-répétition sons

-- Variables internes (roues)
car_front_x      = 0.0
car_front_z      = 0.0
car_rear_x       = 0.0
car_rear_z       = 0.0

-- =============================================================================
--  Démarrage / arrêt de la voiture
-- =============================================================================
function start_car()
    curr_node    = 1
    car_speed    = 0.0
    car_sign     = 0
    car_steer_angle = 0.0

    -- Positionne la voiture derrière le premier noeud
    if #t_node == 0 then return end
    local l_head = t_node[1].heading or fm_plane_head
    local l_rear_h = add_delta_clockwise(l_head, 180, 1)
    car_x = t_node[1].x + math.sin(math.rad(l_rear_h)) * (car_rear_wheel_to_ref + 2)
    car_y = t_node[1].y
    car_z = t_node[1].z + math.cos(math.rad(l_rear_h)) * (car_rear_wheel_to_ref + 2) * -1
    car_heading = l_head
    update_axles()
    play_sound(snd_followme)
end

function stop_car()
    car_speed   = 0.0
    car_sign    = 1
end

-- =============================================================================
--  Boucle principale — appelée par do_every_frame()
-- =============================================================================
function update_car()
    if not FM_car_active then return end
    if #t_node == 0 then return end

    local l_dt = fm_sim_time   -- delta-time de la frame (secondes)
    if l_dt <= 0 or l_dt > 0.5 then return end  -- protection frames aberrantes

    -- -------------------------------------------------------------------------
    --  1. Déterminer la vitesse cible
    -- -------------------------------------------------------------------------
    local l_target_speed = compute_target_speed()

    -- -------------------------------------------------------------------------
    --  2. Accélération / Décélération vers vitesse cible
    -- -------------------------------------------------------------------------
    if car_speed < l_target_speed then
        car_speed = math.min(car_speed + car_accel_rate * l_dt, l_target_speed)
    elseif car_speed > l_target_speed then
        car_speed = math.max(car_speed - car_decel_rate * l_dt, l_target_speed)
    end

    -- -------------------------------------------------------------------------
    --  3. Angle de braquage (steering)  ←  on pilote les roues arrière
    -- -------------------------------------------------------------------------
    if curr_node <= #t_node then
        local l_head_to_node, l_dist_to_node =
            heading_n_dist(car_rear_x, car_rear_z,
                           t_node[curr_node].x, t_node[curr_node].z)
        local l_angle_diff, l_dir = compute_angle_diff(car_heading, l_head_to_node)
        -- Rayon de braquage géométrique
        local l_radius = 0
        if l_angle_diff > 0.5 then
            l_radius = math.max(min_rot_radius,
                                l_dist_to_node / (2 * math.sin(math.rad(l_angle_diff))))
        end
        if l_radius > 0 then
            car_steer_angle = math.deg(math.atan(car_wheelbase / l_radius)) * l_dir
        else
            car_steer_angle = 0
        end
        car_steer_angle = math.max(-35, math.min(35, car_steer_angle))
    end

    -- -------------------------------------------------------------------------
    --  4. Avance de la voiture (modèle bicycle — essieu arrière moteur)
    -- -------------------------------------------------------------------------
    local l_dist = car_speed * l_dt
    car_rear_x = car_rear_x + math.sin(math.rad(car_heading)) * l_dist
    car_rear_z = car_rear_z + math.cos(math.rad(car_heading)) * l_dist * -1

    local l_steer_rad = math.rad(car_steer_angle)
    local l_beta = l_dist * math.tan(l_steer_rad) / car_wheelbase
    car_heading  = math.fmod(car_heading + math.deg(l_beta) + 360, 360)

    car_front_x = car_rear_x + math.sin(math.rad(car_heading)) * car_wheelbase
    car_front_z = car_rear_z + math.cos(math.rad(car_heading)) * car_wheelbase * -1

    -- Référence au centre de la voiture
    car_x = car_rear_x + math.sin(math.rad(car_heading)) * car_rear_wheel_to_ref
    car_z = car_rear_z + math.cos(math.rad(car_heading)) * car_rear_wheel_to_ref * -1
    car_y = probe_y(car_x, car_y, car_z)

    -- -------------------------------------------------------------------------
    --  5. Avancement sur t_node[]
    -- -------------------------------------------------------------------------
    advance_node()

    -- -------------------------------------------------------------------------
    --  6. Synchronisation animation pneus (VER1.17)
    -- -------------------------------------------------------------------------
    sync_tire_animation()
end

-- =============================================================================
--  Calcul vitesse cible
-- =============================================================================
function compute_target_speed()
    if curr_node > #t_node then return 0 end

    local l_dist_to_node  = 0
    local l_head_to_node  = 0
    local l_angle_diff    = 0
    if curr_node <= #t_node then
        l_head_to_node, l_dist_to_node =
            heading_n_dist(car_x, car_z, t_node[curr_node].x, t_node[curr_node].z)
        l_angle_diff = compute_angle_diff(car_heading, l_head_to_node)
    end

    -- Distance de freinage nécessaire
    local l_brake_dist = (car_speed^2) / (2 * car_decel_rate)

    -- Ralentir dans les virages serrés
    local l_corner_speed = car_max_speed
    if l_angle_diff > 30 then
        l_corner_speed = car_min_speed +
            (car_max_speed - car_min_speed) * (1 - (l_angle_diff - 30) / 120)
        l_corner_speed = math.max(car_min_speed, l_corner_speed)
    end

    -- Respect du speed_limiter (vitesse avion ≥ 7 kts → suivre)
    local l_plane_speed_ms = fm_gnd_spd
    local l_follow_speed   = car_max_speed
    if speed_limiter and l_plane_speed_ms > 3.6 then
        l_follow_speed = math.min(car_max_speed, l_plane_speed_ms * 1.1)
    end

    -- Dernier noeud = arrêt progressif
    if curr_node == #t_node then
        if l_dist_to_node < l_brake_dist + 0.5 then
            return math.max(0, car_min_speed * (l_dist_to_node / (l_brake_dist + 0.5)))
        end
    end

    return math.min(l_corner_speed, l_follow_speed)
end

-- =============================================================================
--  Avancement sur la route (passage au noeud suivant)
-- =============================================================================
function advance_node()
    if curr_node > #t_node then return end

    local _, l_dist = heading_n_dist(car_x, car_z,
                                      t_node[curr_node].x, t_node[curr_node].z)

    local l_threshold = math.max(3.0, car_speed * 0.5)

    if l_dist <= l_threshold then
        if curr_node == #t_node then
            -- Arrivé au dernier noeud
            car_speed = 0
            car_sign  = 2   -- ARRIVED
            prepare_kill_objects = true
            play_arrived_sound()
        else
            curr_node = curr_node + 1
        end
    end
end

-- =============================================================================
--  Sons d'arrivée
-- =============================================================================
function play_arrived_sound()
    if car_last_heard == 2 then return end
    car_last_heard = 2
    if depart_arrive == 1 then
        play_sound(snd_safeflight_bye)
    else
        play_sound(snd_arrived)
    end
end

-- =============================================================================
--  Synchronisation animation pneus XPLM (VER1.17)
-- =============================================================================
function sync_tire_animation()
    if dr_tire_steer == nil then return end
    -- Steer
    ffi_steer_buf[0] = car_steer_angle
    ffi_steer_buf[1] = car_steer_angle
    XPLM.XPLMSetDatavf(dr_tire_steer, ffi_steer_buf, 0, 2)
    -- Rotation
    local l_circ = math.pi * 0.35 * 2   -- roue 35cm rayon
    local l_rot  = math.fmod(
                       (ffi_rotate_buf[0] or 0) + (car_speed * fm_sim_time / l_circ * 360),
                       360)
    for i=0,3 do ffi_rotate_buf[i] = l_rot end
    XPLM.XPLMSetDatavf(dr_tire_rotate, ffi_rotate_buf, 0, 4)
end

-- =============================================================================
--  Mise à jour des essieux à partir de car_x / car_z / car_heading
-- =============================================================================
function update_axles()
    car_rear_x = car_x - math.sin(math.rad(car_heading)) * car_rear_wheel_to_ref
    car_rear_z = car_z - math.cos(math.rad(car_heading)) * car_rear_wheel_to_ref * -1
    car_front_x = car_rear_x + math.sin(math.rad(car_heading)) * car_wheelbase
    car_front_z = car_rear_z + math.cos(math.rad(car_heading)) * car_wheelbase * -1
end

-- =============================================================================
--  Trigger automatique (taxi_light allumée sur la piste)
-- =============================================================================
function check_auto_trigger()
    if FM_car_active         then return end
    if #t_taxinode == 0      then return end
    if fm_gear1_gnd == 0     then return end   -- en l'air

    -- Taxi light vient d'être allumée
    if fm_taxi_light == 1 and prev_taxi_light == 0 then
        if flightstart == 9999 then
            -- Arrivée
            if depart_arrive ~= 2 then
                depart_arrive = 2
                if random_gate then
                    auto_assign_gate()
                end
            end
        else
            -- Départ
            if depart_arrive ~= 1 then depart_arrive = 1 end
        end
        local l_res = determine_XP_route()
        if l_res ~= "" then
            update_msg(l_res)
        else
            prepare_show_objects = true
        end
    end
    prev_taxi_light = fm_taxi_light

    -- Beacon éteint → reset
    if fm_beacon_light == 0 and prev_beacon_light == 1
       and flightstart ~= 9999 and ground_time > 30 then
        full_reset()
    end
    prev_beacon_light = fm_beacon_light
end

fm_log("FollowMe vehicle_physics.lua chargé")
