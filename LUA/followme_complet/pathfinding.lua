-- =============================================================================
--  pathfinding.lua
--  FollowMe Ver 1.19 — Module 3 : Algorithme A* & Construction de route
--
--  Appelé sur demande (bouton FM ou trigger SimBrief/taxi_light)
--  Résultat stocké dans t_node[] — consommé par vehicle_physics.lua
-- =============================================================================

-- Variables de route
t_node           = {}   -- route finale de la voiture {x,y,z,heading,dist,hotzone}
t_possible_route = {}   -- résultat brut de l'algorithme A*
t_stack          = {}
is_backtaxi      = false

-- Variables SimBrief (globales, partagées avec ui_imgui)
simbrief_id         = ""
sb_origin_icao      = ""
sb_origin_name      = ""
sb_dest_icao        = ""
sb_dest_name        = ""
sb_runway_takeoff   = ""
sb_runway_landing   = ""
sb_fetch_status     = ""
sb_airport_mismatch = false

-- Variables de sélection route (globales, partagées)
depart_arrive   = 0    -- 0=indéfini, 1=départ, 2=arrivée
depart_gate     = 0
arrival_gate    = 0
depart_runway   = ""
gatetext        = ""
t_suitable_gates= {}

-- =============================================================================
--  Point d'entrée principal — appelé quand l'utilisateur demande la voiture
-- =============================================================================
function determine_XP_route()
    if #t_taxinode == 0        then return "-15" end
    if Aircraft_Type == ""     then return "-1"  end
    if depart_arrive == 0      then return "-5"  end
    if depart_arrive == 2 and arrival_gate == 0 then return "-6" end
    if depart_arrive == 1 then
        if depart_runway == "" then return "-5" end
        local l_found = false
        for i=1,#t_runway do
            if t_runway[i].ID == depart_runway then l_found=true; break end
        end
        if not l_found then return "-18" end
    end
    depart_gate = check_gate()
    return determine_possible_routes()
end

-- =============================================================================
--  Recherche des points de départ / arrivée sur le réseau taxiway
-- =============================================================================
function determine_possible_routes()
    local l_startpt_node, l_startpt_x, l_startpt_z = 0, 0, 0
    local l_endpt_node,   l_endpt_x,   l_endpt_z   = 0, 0, 0
    local l_found   = false
    local l_rev_heading = add_delta_clockwise(fm_plane_head, 180, 1)

    -- Point de départ
    if depart_gate ~= 0 then
        l_found, l_startpt_node, l_startpt_x, l_startpt_z =
            determine_pos_on_segment(t_gate[depart_gate].Heading,
                                     t_gate[depart_gate].x,
                                     t_gate[depart_gate].z,
                                     t_gate[depart_gate].Ramptype)
    else
        local l_adj_x, l_adj_z = coordinates_of_adjusted_ref(fm_plane_x, fm_plane_z, 0, 60, fm_plane_head)
        l_found, l_startpt_node, l_startpt_x, l_startpt_z =
            determine_pos_on_segment(fm_plane_head, l_adj_x, l_adj_z, "tie_down")
        if not l_found then
            l_found, l_startpt_node, l_startpt_x, l_startpt_z =
                determine_pos_on_segment(l_rev_heading, fm_plane_x, fm_plane_z, "tie_down")
        end
        if not l_found and depart_arrive==2 then return "-16" end
    end
    if not l_found then return "-12" end

    -- Point d'arrivée
    if depart_arrive == 1 then
        for i=1,#t_runway do
            if t_runway[i].ID == depart_runway then
                l_endpt_node = t_runway[i].Node; break
            end
        end
        is_backtaxi = false   -- VER1.17 : always false, GPS threshold handles it all
    else
        l_found, l_endpt_node, l_endpt_x, l_endpt_z =
            determine_pos_on_segment(t_gate[arrival_gate].Heading,
                                     t_gate[arrival_gate].x,
                                     t_gate[arrival_gate].z, "gate")
        if not l_found then return "-13" end
    end

    -- Calcul heuristique h (distance à la destination)
    for i=1,#t_taxinode do
        _, t_taxinode[i].h_value =
            heading_n_dist(t_taxinode[i].x, t_taxinode[i].z,
                           t_taxinode[l_endpt_node+1].x, t_taxinode[l_endpt_node+1].z)
    end

    impose_restriction_chk = true
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
        if #t_possible_route == 0 then return "-14" end
    end

    process_possible_routes()
    return ""
end

-- =============================================================================
--  A* (transverse)
-- =============================================================================
function transverse(in_startnode, in_endnode, in_heading)

    local function evaluate_node(in_node, in_size, t_open, t_close)
        for i=1,#t_open  do if t_open[i].Node  == in_node then return false end end
        for i=1,#t_close do if t_close[i].Node == in_node then return false end end
        if not impose_restriction_chk then return true end
        if in_size ~= "" then
            local at = tonumber(Aircraft_Type)
            if in_size=="A" and at>0 and at<7  then return false end
            if (in_size=="B" or in_size=="C" or in_size=="D") and at>0 and at<3 then return false end
            if in_size=="E" and at==1           then return false end
        end
        return true
    end

    local t_close, t_open = {}, {}
    local l_curr_node = in_startnode
    local l_node, l_idx, l_segment_idx = 0, 0, 0
    local l_curr_heading, l_AoC = 0, 0
    local l_g_value, l_f_value  = 0, 0

    t_possible_route = {}
    for i=1,#t_taxinode do
        t_taxinode[i].f_value=nil; t_taxinode[i].g_value=nil
        t_taxinode[i].parent=nil;  t_taxinode[i].cost=nil
        t_taxinode[i].heading=nil
    end

    t_taxinode[in_startnode+1].f_value = t_taxinode[in_startnode+1].h_value
    if in_heading ~= -1 then t_taxinode[in_startnode+1].heading = in_heading end

    t_open[1] = { Node=in_startnode, f_value=t_taxinode[in_startnode+1].f_value }

    while l_curr_node ~= in_endnode and l_curr_node ~= -1 do
        for l_string in t_taxinode[l_curr_node+1].Segment:gmatch("[^,]+") do
            l_segment_idx = tonumber(l_string)
            local l_pass = false
            if t_segment[l_segment_idx].Type ~= "runway" or not impose_restriction_chk then
                if t_segment[l_segment_idx].Node1 == l_curr_node then
                    l_node = t_segment[l_segment_idx].Node2
                    l_pass = evaluate_node(l_node, t_segment[l_segment_idx].Size or "", t_open, t_close)
                    l_curr_heading = t_segment[l_segment_idx].Heading
                elseif t_segment[l_segment_idx].Node2 == l_curr_node
                       and t_segment[l_segment_idx].Dir == "twoway" then
                    l_node = t_segment[l_segment_idx].Node1
                    l_pass = evaluate_node(l_node, t_segment[l_segment_idx].Size or "", t_open, t_close)
                    l_curr_heading = add_delta_clockwise(t_segment[l_segment_idx].Heading, 180, 1)
                end
            end
            if l_pass then
                l_idx = #t_open + 1
                t_open[l_idx] = { Node=l_node }
                l_g_value = (t_taxinode[l_curr_node+1].g_value or 0) + t_segment[l_segment_idx].Dist
                l_f_value = l_g_value + t_taxinode[l_node+1].h_value
                if t_taxinode[l_node+1].f_value == nil or l_f_value < t_taxinode[l_node+1].f_value then
                    t_taxinode[l_node+1].g_value = l_g_value
                    t_taxinode[l_node+1].f_value = l_f_value
                    t_taxinode[l_node+1].parent  = l_curr_node
                    if t_taxinode[l_curr_node+1].heading == nil then
                        t_taxinode[l_node+1].cost = 0
                    else
                        local l_prev_cost = t_taxinode[l_curr_node+1].cost or 0
                        l_AoC = 180 - compute_angle_diff(t_taxinode[l_curr_node+1].heading, l_curr_heading)
                        if     l_AoC < 20  then t_taxinode[l_node+1].cost = l_prev_cost + 150
                        elseif l_AoC <= 80 then t_taxinode[l_node+1].cost = l_prev_cost + 12
                        elseif l_AoC < 90  then t_taxinode[l_node+1].cost = l_prev_cost + 3
                        elseif l_AoC <= 140 then t_taxinode[l_node+1].cost = l_prev_cost + 1
                        elseif l_AoC <= 170 then t_taxinode[l_node+1].cost = l_prev_cost + 1
                        else                     t_taxinode[l_node+1].cost = l_prev_cost end
                    end
                    t_taxinode[l_node+1].heading = l_curr_heading
                    t_open[l_idx].f_value = l_f_value
                    t_open[l_idx].cost    = t_taxinode[l_node+1].cost
                end
            end
        end

        for i=#t_open,1,-1 do
            if t_open[i].Node == l_curr_node then table.remove(t_open,i); break end
        end
        t_close[#t_close+1] = { Node=l_curr_node }

        table.sort(t_open, function(a,b) return a.f_value > b.f_value end)
        local l_top_f = t_open[#t_open] and t_open[#t_open].f_value or 0
        for i=#t_open,1,-1 do
            t_open[i].order = (math.abs(t_open[i].f_value - l_top_f) < 300) and 1 or 2
        end
        table.sort(t_open, function(a,b)
            return a.order > b.order or (a.order==b.order and a.cost > b.cost)
        end)
        l_curr_node = (#t_open > 0) and t_open[#t_open].Node or -1
    end

    if t_taxinode[in_endnode+1].f_value ~= nil then
        t_possible_route[1] = {
            Dist  = t_taxinode[in_endnode+1].g_value,
            Cost  = t_taxinode[in_endnode+1].cost,
            Route = tostring(in_endnode)
        }
        local l_n = in_endnode
        while l_n ~= in_startnode do
            l_n = t_taxinode[l_n+1].parent
            t_possible_route[1].Route = l_n.." "..t_possible_route[1].Route
        end
    end
end

-- =============================================================================
--  Post-traitement de la route A* → t_node[]
-- =============================================================================
function process_possible_routes()
    if #t_possible_route == 0 then return end

    local l_min_dist = min_rot_radius * 2 + car_rear_wheel_to_ref
    local t_route_nodes  = {}
    local routenode_cnt  = 1
    local l_last_taxiway = 0
    local l_hotzone_cnt  = 0
    local l_index        = 1

    for l_node in t_possible_route[1].Route:gmatch("[^%s]+") do
        t_route_nodes[routenode_cnt] = l_node
        routenode_cnt = routenode_cnt + 1
    end

    -- Trouver le dernier noeud taxiway avant la piste (départ)
    if depart_arrive == 1 then
        for i=#t_route_nodes-1,1,-1 do
            local n = t_route_nodes[i]
            if t_taxinode[n+1].Type=="hotzone" or t_taxinode[n+1].Type=="runway" then
                for s_str in t_taxinode[n+1].Segment:gmatch("[^,]+") do
                    local sidx = tonumber(s_str)
                    if t_segment[sidx].Hotzone=="" and t_segment[sidx].Type~="runway" then
                        l_last_taxiway = i; break
                    end
                end
            end
            if l_last_taxiway > 0 then break end
        end
    end

    t_node = {}
    for i=1,#t_route_nodes do
        local n        = t_route_nodes[i]
        local l_bypass = false

        if depart_arrive==1 and i >= l_last_taxiway then
            if t_taxinode[n+1].Type=="hotzone" or t_taxinode[n+1].Type=="runway" then
                local l_has_exit = false
                for s_str in t_taxinode[n+1].Segment:gmatch("[^,]+") do
                    local sidx = tonumber(s_str)
                    if sidx and t_segment[sidx] and
                       t_segment[sidx].Type~="runway" and
                       (t_segment[sidx].Hotzone==nil or t_segment[sidx].Hotzone=="") then
                        l_has_exit = true; break
                    end
                end
                if not l_has_exit then l_hotzone_cnt = l_hotzone_cnt + 1 end
            end
        end
        if l_hotzone_cnt > 0 then break end

        if l_index > 1 then
            local l_h, l_d = heading_n_dist(t_node[l_index-1].x, t_node[l_index-1].z,
                                             t_taxinode[n+1].x,   t_taxinode[n+1].z)
            if l_index > 2 and l_hotzone_cnt == 0 then
                local l_aor = compute_angle_diff(t_node[l_index-2].heading or 0, l_h)
                if l_aor <= 10 then
                    t_node[l_index-1].x = t_taxinode[n+1].x
                    t_node[l_index-1].y = t_taxinode[n+1].y
                    t_node[l_index-1].z = t_taxinode[n+1].z
                    t_node[l_index-2].heading, t_node[l_index-2].dist =
                        heading_n_dist(t_node[l_index-2].x, t_node[l_index-2].z,
                                       t_node[l_index-1].x, t_node[l_index-1].z)
                    l_bypass = true
                end
            end
            if not l_bypass then
                t_node[l_index] = {
                    hotzone = (l_hotzone_cnt>0) and "1" or "",
                    x = t_taxinode[n+1].x, y = t_taxinode[n+1].y, z = t_taxinode[n+1].z
                }
                t_node[l_index-1].heading = l_h
                t_node[l_index-1].dist    = l_d
                l_index = l_index + 1
            end
        else
            t_node[l_index] = {
                hotzone = "",
                x = t_taxinode[n+1].x, y = t_taxinode[n+1].y, z = t_taxinode[n+1].z
            }
            l_index = l_index + 1
        end
    end

    -- Espacement minimum entre noeuds
    for i=1,#t_node-2 do
        if t_node[i].dist and t_node[i].dist < l_min_dist then
            t_node[i+1].x, t_node[i+1].z =
                coordinates_of_adjusted_ref(t_node[i+1].x, t_node[i+1].z,
                                             0, l_min_dist - t_node[i].dist, t_node[i].heading)
            t_node[i].dist = l_min_dist
            t_node[i+1].heading, t_node[i+1].dist =
                heading_n_dist(t_node[i+1].x, t_node[i+1].z,
                               t_node[i+2].x, t_node[i+2].z)
        end
    end

    -- Nettoyage noeuds virtuels
    while #t_taxinode > 0 and t_taxinode[#t_taxinode].Type == "New" do
        t_taxinode[#t_taxinode] = nil
    end
    while #t_segment > 0 and t_segment[#t_segment].ID == "ADD_NEWSEGMENT" do
        local s = t_segment[#t_segment]
        if t_taxinode[s.Node1+1] then
            t_taxinode[s.Node1+1].Segment =
                string.gsub(t_taxinode[s.Node1+1].Segment, ","..tostring(#t_segment), "")
        end
        if t_taxinode[s.Node2+1] then
            t_taxinode[s.Node2+1].Segment =
                string.gsub(t_taxinode[s.Node2+1].Segment, ","..tostring(#t_segment), "")
        end
        t_segment[#t_segment] = nil
    end

    -- VER1.17 : Projection sur centreline + noeud GPS seuil (tous les départs)
    if depart_arrive==1 and #t_node > 0 then
        local l_rwy_x, l_rwy_y, l_rwy_z = 0, 0, 0
        local l_far_x, l_far_z           = 0, 0
        for i=1,#t_runway do
            if t_runway[i].ID == depart_runway then
                l_rwy_x = t_runway[i].x
                l_rwy_y = probe_y(t_runway[i].x, 0, t_runway[i].z)
                l_rwy_z = t_runway[i].z
            else
                l_far_x = t_runway[i].x; l_far_z = t_runway[i].z
            end
        end
        local l_ax = l_rwy_x - l_far_x
        local l_az = l_rwy_z - l_far_z
        local l_alen = math.max(1, math.sqrt(l_ax*l_ax + l_az*l_az))
        local l_ux, l_uz = l_ax/l_alen, l_az/l_alen
        local l_ex = t_node[#t_node].x
        local l_ez = t_node[#t_node].z
        local l_vx, l_vz = l_ex-l_far_x, l_ez-l_far_z
        local l_t  = l_vx*l_ux + l_vz*l_uz
        local l_px  = l_far_x + l_t*l_ux
        local l_pz  = l_far_z + l_t*l_uz
        local _, l_offset      = heading_n_dist(l_ex, l_ez, l_px, l_pz)
        local _, l_proj_to_thr = heading_n_dist(l_px, l_pz, l_rwy_x, l_rwy_z)
        if l_offset > 2 and l_proj_to_thr > 10 then
            t_node[#t_node].x = l_px; t_node[#t_node].y = probe_y(l_px,0,l_pz)
            t_node[#t_node].z = l_pz; t_node[#t_node].hotzone = "1"
            if #t_node >= 2 then
                t_node[#t_node-1].heading, t_node[#t_node-1].dist =
                    heading_n_dist(t_node[#t_node-1].x, t_node[#t_node-1].z, l_px, l_pz)
            end
        else
            t_node[#t_node].hotzone = "1"
        end
        local l_last = #t_node
        local _, l_gap = heading_n_dist(t_node[l_last].x, t_node[l_last].z, l_rwy_x, l_rwy_z)
        if l_gap > 5 then
            t_node[l_last].heading, t_node[l_last].dist =
                heading_n_dist(t_node[l_last].x, t_node[l_last].z, l_rwy_x, l_rwy_z)
            t_node[l_last+1] = {
                x=l_rwy_x, y=l_rwy_y, z=l_rwy_z,
                hotzone="1", heading=t_node[l_last].heading, dist=0
            }
        end
    end

    fm_log("FollowMe pathfinding : route calculée — " .. #t_node .. " noeuds")
end

-- =============================================================================
--  Localisation du point de départ/arrivée sur un segment
-- =============================================================================
function determine_pos_on_segment(in_heading, in_x, in_z, in_type)
    local l_ret_intersect_dist = 9999
    local l_ret_segment_index, l_ret_x, l_ret_z = 0, 0, 0
    local l_ret_node_dist = 9999
    local l_ret_nodesegment_index, l_ret_deadnode = 0, 0
    local l_aircraft_type = tonumber(Aircraft_Type)
    local l_min_dist = 25
    if     l_aircraft_type==0 or l_aircraft_type>=7 then l_min_dist=5
    elseif l_aircraft_type>=3 and l_aircraft_type<7 then l_min_dist=15
    elseif l_aircraft_type>=1 and l_aircraft_type<=2 then l_min_dist=30 end
    local l_max_dist      = 300
    local l_max_dist_node = 130

    for i=1,#t_segment do
        if in_type=="gate" or in_type=="misc" then
            local l_ok,lx,lz,ld = compute_intersection(in_type,in_heading,in_x,in_z,i)
            if l_ok and ld<=l_max_dist and ld<=l_ret_intersect_dist then
                l_ret_intersect_dist=ld; l_ret_segment_index=i; l_ret_x=lx; l_ret_z=lz
            end
            local l_ok2,lnode,ldist = check_deadend_node(in_type,in_heading,in_x,in_z,i)
            if l_ok2 and ldist>=l_min_dist and ldist<=l_ret_node_dist then
                l_ret_deadnode=lnode; l_ret_nodesegment_index=i; l_ret_node_dist=ldist
            end
        end
        if in_type=="tie_down" or in_type=="hangar" then
            local l_ok2,lnode,ldist = check_deadend_node(in_type,in_heading,in_x,in_z,i)
            if l_ok2 and ldist<=l_max_dist_node and ldist<=l_ret_node_dist then
                l_ret_deadnode=lnode; l_ret_nodesegment_index=i; l_ret_node_dist=ldist
            end
            local l_ok,lx,lz,ld = compute_intersection(in_type,in_heading,in_x,in_z,i)
            if l_ok and ld<=l_max_dist and ld<=l_ret_intersect_dist then
                l_ret_intersect_dist=ld; l_ret_segment_index=i; l_ret_x=lx; l_ret_z=lz
            end
            local l_ok3,ltd,ltx,ltz,_ = compute_tangent_dist(in_heading,in_x,in_z,i)
            if l_ok3 and ltd<=l_ret_intersect_dist and ltd<=l_max_dist_node then
                l_ret_intersect_dist=ltd; l_ret_segment_index=i; l_ret_x=ltx; l_ret_z=ltz
            end
        end
    end

    local l_decision = 0
    if     l_ret_node_dist<9999 and l_ret_intersect_dist<9999 then
        l_decision = (l_ret_intersect_dist <= l_ret_node_dist) and 1 or 2
    elseif l_ret_intersect_dist<9999 then l_decision=1
    elseif l_ret_node_dist<9999      then l_decision=2 end

    if l_decision == 0 then
        -- VER1.15 : fallback au noeud connecté le plus proche
        local l_best_d, l_best_nid = 99999, -1
        for ni=1,#t_taxinode do
            if t_taxinode[ni] and t_taxinode[ni].Segment and t_taxinode[ni].Segment ~= "" then
                local dd = math.sqrt((in_x-t_taxinode[ni].x)^2+(in_z-t_taxinode[ni].z)^2)
                if dd < l_best_d and dd < 500 then l_best_d=dd; l_best_nid=ni-1 end
            end
        end
        if l_best_nid >= 0 then
            return true, l_best_nid, t_taxinode[l_best_nid+1].x, t_taxinode[l_best_nid+1].z, 0
        end
        return false, 0, 0, 0, 0
    elseif l_decision==1 then
        local ln, ls = add_new_taxinode_segment(l_ret_segment_index, l_ret_x, l_ret_z,
                                                 l_ret_intersect_dist)
        return true, ln, l_ret_x, l_ret_z, ls
    else
        return true, l_ret_deadnode, 0, 0, 0
    end
end

function compute_intersection(in_type, in_heading, in_x, in_z, in_idx)
    local n1x = t_taxinode[t_segment[in_idx].Node1+1].x
    local n1z = t_taxinode[t_segment[in_idx].Node1+1].z
    local n2x = t_taxinode[t_segment[in_idx].Node2+1].x
    local n2z = t_taxinode[t_segment[in_idx].Node2+1].z
    local sh  = t_segment[in_idx].Heading
    if sh==in_heading or sh==add_delta_clockwise(in_heading,180,1) then return false,0,0,0 end
    local ix, iz = 0, 0
    if in_heading==0 or in_heading==180 then
        iz = math.tan(math.rad(90-sh))*(in_x-n1x) + (n1z*-1); iz = -iz; ix = in_x
    elseif sh==0 or sh==180 then
        iz = math.tan(math.rad(90-in_heading))*(n1x-in_x)+(in_z*-1); iz=-iz; ix=n1x
    else
        local r = math.tan(math.rad(90-in_heading))/math.tan(math.rad(90-sh))
        local v1 = -r*(n1z*-1); local v2 = math.tan(math.rad(90-in_heading))*(n1x-in_x)
        local v3 = in_z*-1;     local v4 = 1-r
        iz = -((v1+v2+v3)/v4)
        local r2 = 1/r
        ix = (-r2*n1x-(n1z-in_z)/math.tan(math.rad(90-in_heading))+in_x)/(1-r2)
    end
    if not (((ix>=n1x and ix<=n2x) or (ix>=n2x and ix<=n1x)) and
            ((iz>=n1z and iz<=n2z) or (iz>=n2z and iz<=n1z))) then
        return false,0,0,0
    end
    local lh = (in_type=="gate" or in_type=="misc") and add_delta_clockwise(in_heading,180,1) or in_heading
    local ok, _, ld = chk_line_of_sight(lh,70,70,in_x,in_z,ix,iz)
    if not ok and (in_type=="tie_down" or in_type=="hangar") then
        lh = add_delta_clockwise(in_heading,180,1)
        ok, _, ld = chk_line_of_sight(lh,70,70,in_x,in_z,ix,iz)
    end
    return ok, ix, iz, ld
end

function compute_tangent_dist(in_heading, in_x, in_z, in_idx)
    local n1x = t_taxinode[t_segment[in_idx].Node1+1].x
    local n1z = t_taxinode[t_segment[in_idx].Node1+1].z
    local n2x = t_taxinode[t_segment[in_idx].Node2+1].x
    local n2z = t_taxinode[t_segment[in_idx].Node2+1].z
    local sh  = t_segment[in_idx].Heading
    local ok1 = chk_line_of_sight(in_heading,60,60,in_x,in_z,n2x,n2z)
    local ok2 = chk_line_of_sight(in_heading,60,60,in_x,in_z,n1x,n1z)
    if not ok1 and not ok2 then return false,0,0,0,0 end
    local gn = ok1 and t_segment[in_idx].Node2 or t_segment[in_idx].Node1
    local _, ld = heading_n_dist(in_x,in_z,n2x,n2z)
    local la, ldir = compute_angle_diff(sh,_)
    local td = math.abs(math.sin(math.rad(la))*ld)
    local th = add_delta_clockwise(sh,90,ldir)
    local tx = in_x + math.sin(math.rad(th))*td
    local tz = in_z + math.cos(math.rad(th))*td*-1
    if ((tx>=n1x and tx<=n2x) or (tx>=n2x and tx<=n1x)) or
       ((tz>=n1z and tz<=n2z) or (tz>=n2z and tz<=n1z)) then
        return true, td, tx, tz, gn
    end
    return false,0,0,0,0
end

function check_deadend_node(in_type, in_heading, in_x, in_z, in_idx)
    local ld = -1
    if not string.find(t_taxinode[t_segment[in_idx].Node1+1].Segment,",") then
        ld = t_segment[in_idx].Node1
    elseif not string.find(t_taxinode[t_segment[in_idx].Node2+1].Segment,",") then
        ld = t_segment[in_idx].Node2
    end
    if ld == -1 then return false,0,0 end
    local lh = (in_type=="gate" or in_type=="misc")
               and add_delta_clockwise(in_heading,180,1) or in_heading
    local ok, _, dist = chk_line_of_sight(lh,80,80,in_x,in_z,
                                           t_taxinode[ld+1].x, t_taxinode[ld+1].z)
    return ok, ld, dist
end

function add_new_taxinode_segment(in_seg_idx, in_x, in_z, in_dist)
    local li = #t_taxinode + 1
    t_taxinode[li] = {
        x=in_x, y=t_taxinode[t_segment[in_seg_idx].Node2+1].y, z=in_z,
        Type="New", Runway="", Segment="",
        f_value=nil, g_value=nil, h_value=nil,
        parent=nil, cost=nil, heading=nil
    }
    local ln = li - 1
    local ls = 0

    local function add_seg(node1, node2)
        ls = #t_segment + 1
        t_segment[ls] = {
            ID      = "ADD_NEWSEGMENT",
            Node1   = node1, Node2   = node2,
            Dir     = t_segment[in_seg_idx].Dir,
            Type    = t_segment[in_seg_idx].Type,
            Size    = t_segment[in_seg_idx].Size,
            Hotzone = t_segment[in_seg_idx].Hotzone
        }
        t_segment[ls].Heading, t_segment[ls].Dist =
            heading_n_dist(t_taxinode[node1+1].x, t_taxinode[node1+1].z,
                           t_taxinode[node2+1].x, t_taxinode[node2+1].z)
        t_taxinode[node1+1].Segment = t_taxinode[node1+1].Segment..","..tostring(ls)
        if t_taxinode[li].Segment == "" then
            t_taxinode[li].Segment = tostring(ls)
        else
            t_taxinode[li].Segment = t_taxinode[li].Segment..","..tostring(ls)
        end
    end

    if string.find(t_taxinode[t_segment[in_seg_idx].Node1+1].Segment,",") then
        add_seg(t_segment[in_seg_idx].Node1, ln)
    end
    if string.find(t_taxinode[t_segment[in_seg_idx].Node2+1].Segment,",") then
        add_seg(ln, t_segment[in_seg_idx].Node2)
    end
    return ln, ls
end

-- =============================================================================
--  SimBrief
-- =============================================================================
function check_SimBrief()
    sb_fetch_status   = ""
    sb_airport_mismatch = false
    if simbrief_id == "" then sb_fetch_status="NO_ID"; return end
    sb_fetch_status = "LOADING"
    local resp = {}
    local url  = "https://www.simbrief.com/api/xml.fetcher.php?userid="..simbrief_id.."&v=xml"
    local res, code = http.request{url=url, sink=ltn12.sink.table(resp), redirect=true}
    if code ~= 200 or #resp == 0 then sb_fetch_status="ERROR"; return end
    local body = table.concat(resp)
    sb_origin_icao    = string.match(body,"<origin>.-<icao_code>(.-)</icao_code>") or ""
    sb_dest_icao      = string.match(body,"<destination>.-<icao_code>(.-)</icao_code>") or ""
    sb_origin_name    = string.match(body,"<origin>.-<name>(.-)</name>") or ""
    sb_dest_name      = string.match(body,"<destination>.-<name>(.-)</name>") or ""
    sb_runway_takeoff = string.match(body,"<origin>.-<plan_rwy>(.-)</plan_rwy>") or ""
    sb_runway_landing = string.match(body,"<destination>.-<plan_rwy>(.-)</plan_rwy>") or ""
    if sb_origin_icao=="" or sb_dest_icao=="" then sb_fetch_status="NO_DATA"; return end
    sb_fetch_status     = "OK"
    sb_airport_mismatch = (sb_origin_icao ~= curr_ICAO)
    if get_from_SimBrief then apply_simbrief_runway() end
end

function apply_simbrief_runway()
    depart_runway = ""
    if sb_fetch_status ~= "OK" then return end
    local l_rwy = ""
    if     depart_arrive==1 and sb_origin_icao==curr_ICAO then l_rwy = sb_runway_takeoff
    elseif depart_arrive==2 and sb_dest_icao  ==curr_ICAO then l_rwy = sb_runway_landing end
    if l_rwy == "" then return end
    for i=1,#t_runway do
        if t_runway[i].ID == l_rwy then depart_runway=l_rwy; return end
    end
    update_msg("-18")
end

function auto_assign_gate()
    if #t_gate == 0 then arrival_gate=0; return "-1" end
    t_suitable_gates = {}
    for i=1,#t_gate do
        if string.match(t_gate[i].Types, Aircraft_Type)==Aircraft_Type or
           (Aircraft_Type=="0" and string.match(t_gate[i].Types,"7")) then
            t_suitable_gates[#t_suitable_gates+1] = i
        end
    end
    rampstart_chg = true
    math.randomseed(os.time())
    if #t_suitable_gates > 0 then
        arrival_gate = t_suitable_gates[math.random(1,#t_suitable_gates)]
        gatetext = t_gate[arrival_gate].ID
        return "2"
    else
        arrival_gate = math.random(1,#t_gate)
        gatetext = t_gate[arrival_gate].ID
        update_msg("-17")
        return "1"
    end
end

fm_log("FollowMe pathfinding.lua chargé")
