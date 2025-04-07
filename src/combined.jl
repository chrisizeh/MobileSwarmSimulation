function track!(sim::Simulation, flocking_robots, tracking_robots; steps::Int=500)
    avoidSpeed = 0.6
    centerSpeed = 0.4
    turnSpeed = 0.05

    t_turnSpeed = 0.8

    vals = [0., 0.]
    v = [0., 0.]

    # FIX obstacle in front
    cells = ceil(Int64, flocking_robots[1].sensor_dist / minimum(sim.grid_step))
    for i in 1:steps
        for robot in flocking_robots
            close = [0., 0.]
            neighbors_dist = [0., 0.]
            neighbors_count = 0
            close_count = 0
            neighbors = get_neighbors(sim, robot, cells)

            dist_goal = get_sensoric_data(robot, goal)
            if(dist_goal[argmin(dist_goal)] < 1)
                return
            end

            for neighbor in neighbors
                if robot != neighbor
                    dist = get_sensoric_data(robot, neighbor)

                    if (sum(dist) < robot.NO_DIST_FOUND)
                        nearest = argmin(dist)
                        vals[1] = round(cos(robot.sensor_pos[nearest][3] + robot.deg) * dist[nearest]; digits=5)
                        vals[2] = round(sin(robot.sensor_pos[nearest][3] + robot.deg) * dist[nearest]; digits=5)
                        
                        if ((dist[nearest] < robot.sensor_dist/4) & (typeof(neighbor) == Robot))
                            close[1] -= vals[2]
                            close[2] += vals[1]
                            close_count += 1
                        elseif ((dist[nearest] < robot.sensor_dist/3) & (typeof(neighbor) != Robot))
                            close[1] -= vals[2]
                            close[2] += vals[1]
                            close_count += 1
                        elseif ((dist[nearest] > robot.sensor_dist/2) & (typeof(neighbor) == Robot))
                            neighbors_dist[1] += vals[1]
                            neighbors_dist[2] += vals[2]
                            neighbors_count += 1
                        end
                    end
                end
            end

            v = avoidSpeed * close / max(1, close_count) + centerSpeed * neighbors_dist / max(1, neighbors_count)
            if (abs(sum(v)) > 0)
                if(abs(v[2]) > 0.1)
                    deg = atan(v[2], v[1])
                else
                    deg = pi
                end
            else
                deg = 0
            end

            deg = sign(rand()-0.5) * deg

            turn = robot.radius * deg / 2 * turnSpeed
            update_speed!(robot, vMax/2 - turn, vMax/2 + turn)
        end

        for i in 2:length(tracking_robots)-1
            dist1 = get_sensoric_data(tracking_robots[i], tracking_robots[i-1])
            dist2 = get_sensoric_data(tracking_robots[i], tracking_robots[i+1])

            if (i > 1 && minimum(dist1) > tracking_robots[i].sensor_dist/2)
                nearest = argmin(dist1)
                vals[1] = cos(tracking_robots[i].sensor_pos[nearest][3]) * dist1[nearest]
                vals[2] = sin(tracking_robots[i].sensor_pos[nearest][3]) * dist1[nearest]

                if (abs(vals[2]) > 0)
                    deg = atan(vals[2]/vals[1])
                else
                    deg = 0
                end

                turn = tracking_robots[i].radius * deg / 2 * t_turnSpeed
                update_speed!(tracking_robots[i], vMax/2 - turn, vMax/2 + turn)
            end
            
            if (minimum(dist2) > tracking_robots[i].sensor_dist-2 || minimum(dist1) < tracking_robots[i].sensor_dist/2)
                update_speed!(tracking_robots[i], 0, 0)
            end
        end


        update!(sim)
    end
end