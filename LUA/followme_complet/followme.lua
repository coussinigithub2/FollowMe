-- =============================================================================
--  followme.lua  —  PROGRAMME PRINCIPAL
--  FollowMe Ver 1.19 — X-Plane 12 / FlyWithLua NG
--
--  Architecture modulaire :
--    modules/core_init.lua        → FFI, XPLM, Datarefs, Sons, Utilitaires
--    modules/airport_data.lua     → Lecture apt.dat, Préférences
--    modules/pathfinding.lua      → A*, Construction de route, SimBrief
--    modules/vehicle_physics.lua  → Physique voiture, Trigger automatique
--    modules/ui_imgui.lua         → Interface ImGui
--    modules/renderer.lua         → Rendu 3D XPLM
--
--  Répartition des tâches FlyWithLua :
--    do_often        → watchdog ICAO (léger, ~1/sec)
--    do_sometimes    → chargement apt.dat si ICAO changé
--    do_every_frame  → physique voiture + trigger taxi_light
--    do_every_draw   → rendu 3D XPLM
--    do_on_exit      → nettoyage global
--
--  ⚠ RÈGLE FONDAMENTALE FlyWithLua :
--    do_every_draw = RENDU SEULEMENT  (zéro calcul, zéro I/O)
--    do_every_frame = CALCUL SEULEMENT (zéro rendu)
-- =============================================================================

-- =============================================================================
--  1. CHARGEMENT DES MODULES (ordre important)
-- =============================================================================
local BASE = SCRIPT_DIRECTORY .. "follow_me/modules/"

dofile(BASE .. "core_init.lua")         -- FFI, XPLM, datarefs, utilitaires math
dofile(BASE .. "airport_data.lua")      -- Tables t_taxinode, t_segment, t_gate…
dofile(BASE .. "pathfinding.lua")       -- A*, t_node[], SimBrief
dofile(BASE .. "vehicle_physics.lua")   -- Physique voiture, start/stop
dofile(BASE .. "ui_imgui.lua")          -- ImGui fenêtre + messages
dofile(BASE .. "renderer.lua")          -- Rendu 3D XPLM

-- =============================================================================
--  2. POST-INITIALISATION (après que tous les modules sont chargés)
-- =============================================================================

-- Enregistrer le dataref animation (VER1.17) — nécessite que FFI soit prêt
register_dataref()

-- Charger le fichier de préférences
local l_cfg_status = load_config()
if l_cfg_status == "-2" then
    -- Première utilisation : pas de fichier prefs, valeurs par défaut déjà posées
    fm_log("FollowMe : Pas de fichier préférences — valeurs par défaut")
end

-- ⚠ NE PAS appeler load_airport_if_needed() ici !
-- Les datarefs X-Plane ne sont pas encore prêts au chargement du script.
-- Le flag est posé → do_sometimes() déclenchera le chargement dès la 1re frame.
airport_reload_needed = true   -- déclenche le chargement via do_sometimes()

-- =============================================================================
--  3. CRÉATION DES FENÊTRES ImGui
-- =============================================================================

-- Holder (petit bouton FM cliquable)
fm_holder = float_wnd_create(30, 30, 1, true)
float_wnd_set_title(fm_holder, "")
float_wnd_set_position(fm_holder, 20, Win_Y)
float_wnd_set_imgui_builder(fm_holder, "build_holder")
float_wnd_set_onclose(fm_holder,       "closed_holder")

-- Fenêtre principale
fm_window = float_wnd_create(320, 540, 1, true)
float_wnd_set_title(fm_window, "FollowMe")
float_wnd_set_position(fm_window, 60, Win_Y)
float_wnd_set_imgui_builder(fm_window, "build_window")
float_wnd_set_onclose(fm_window,       "closed_window")
float_wnd_set_visible(fm_window, false)   -- cachée au démarrage

-- =============================================================================
--  4. do_often — WATCHDOG ICAO (~1 fois/seconde)
--     LÉGER : seulement XPLMFindNavAid, pose un flag si changement détecté
-- =============================================================================
do_often("airport_watchdog()")

-- =============================================================================
--  5. do_sometimes — CHARGEMENT LOURD si flag levé
--     Lecture apt.dat, parsing, construction du graphe
--     Séparé de do_often pour ne jamais geler X-Plane
-- =============================================================================
do_sometimes("load_airport_if_needed()")

-- =============================================================================
--  6. do_every_frame — PHYSIQUE & TRIGGERS (exécuté chaque frame)
--     CALCUL PUR : zéro rendu, zéro I/O fichier
-- =============================================================================
do_every_frame([[

    -- Compteur de temps au sol (pour détection début de vol)
    if fm_gear1_gnd ~= 0 or fm_gear2_gnd ~= 0 then
        ground_time = ground_time + fm_sim_time
    end

    -- Détection début de vol (en l'air pour la première fois)
    if fm_new_flight ~= prev_new_flight then
        if prev_new_flight > 0 and fm_gear1_gnd == 0 and fm_gear2_gnd == 0 then
            if flightstart == 0 then
                flightstart = 9999   -- Vol en cours → prochain atterrissage = arrivée
            end
        end
        prev_new_flight = fm_new_flight
    end

    -- Physique voiture
    if FM_car_active then
        update_car()
    end

    -- Trigger automatique (taxi_light / beacon)
    check_auto_trigger()

    -- Avancement du temps de son
    if play_time > 0 then
        play_time = play_time - fm_sim_time
    end

]])

-- =============================================================================
--  7. do_every_draw — RENDU 3D (exécuté chaque frame de rendu)
--     RENDU SEULEMENT : appelle render_all() du module renderer
-- =============================================================================
do_every_draw("render_all()")

-- =============================================================================
--  8. do_on_exit — NETTOYAGE À LA FERMETURE
-- =============================================================================
do_on_exit([[

    -- Sauvegarder préférences
    save_config()

    -- Détruire tous les objets 3D et le probe terrain
    renderer_cleanup()

    -- Désenregistrer le dataref animation (VER1.17)
    if dr_sign ~= nil and LUA_RUN == 1 then
        XPLM.XPLMUnregisterDataAccessor(dr_sign)
        dr_sign = nil
    end

    fm_log("FollowMe : do_on_exit — cleanup complet")

]])

-- =============================================================================
--  9. LOG FINAL
-- =============================================================================
fm_log("FollowMe v1.19 chargé — " .. curr_ICAO ..
       "  |  " .. #t_taxinode .. " noeuds  |  " ..
       #t_segment .. " segments  |  " .. #t_gate .. " gates")
