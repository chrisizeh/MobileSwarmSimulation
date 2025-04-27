function track!(sim::Simulation, flocking_robots, tracking_robots; steps::Int=500, v_flocking=10, v_tracking=10, dist_flocking=20, dist_obstacle=30, avoidObstacleSpeed=0.05, centerSpeed=0.2)
    avoidSpeed = 0.05

    t_turnSpeed = 0.8

    vals = [0., 0.]
    v = [0., 0.]

    # FIX obstacle in front
    cells = ceil(Int64, flocking_robots[1].sensor_dist / minimum(sim.grid_step))
    for i in 1:steps
        for robot in flocking_robots
            neighbors_count = 0
            close_count = 0
            neighbors = get_neighbors(sim, robot, cells)

            dist_goal = get_sensoric_data(robot, goal)
            if(dist_goal[argmin(dist_goal)] < 1)
                return
            end

            centerDeg = 0
            avoidDeg = 0
            center = [0., 0.]

            for neighbor in neighbors
                if robot != neighbor
                    dist = get_sensoric_data(robot, neighbor)

                    if (sum(dist) < robot.NO_DIST_FOUND)
                        nearest = argmin(dist)

                        if ((dist[nearest] < dist_flocking) & (typeof(neighbor) == Robot))
                            # closeness = 1 - dist[nearest]/dist_flocking
                            avoidDeg -= robot.sensor_pos[nearest][3] * avoidSpeed
                            # avoidDeg -= robot.sensor_pos[nearest][3] * avoidSpeed + sign(rand()-1/2) * pi/2
                            close_count += 1
                        elseif ((dist[nearest] < dist_obstacle) & (typeof(neighbor) != Robot))
                            avoidDeg -= robot.sensor_pos[nearest][3] * avoidObstacleSpeed + sign(rand()-1/2) * pi/8
                            close_count += 1
                        elseif ((dist[nearest] > dist_flocking) & (typeof(neighbor) == Robot))
                            # closeness = (dist[nearest] - dist_flocking) / (robot.sensor_dist - dist_flocking)
                            # centerDeg -= robot.sensor_pos[nearest][3] * centerSpeed
                            center[1] += cos(robot.sensor_pos[nearest][3]) * dist[nearest]
                            center[2] += sin(robot.sensor_pos[nearest][3]) * dist[nearest]
                            neighbors_count += 1
                        end
                    end
                end
            end

            vals = vals ./ max(1, neighbors_count)
            if (abs(vals[2]) > 0)
                deg = atan(vals[2]/vals[1])
            else
                deg = 0
            end
            turnCenter = robot.radius * deg / 2 * centerSpeed

            turnAvoid = robot.radius * avoidDeg/max(1, close_count)
            distAvoid = 1 - sqrt(v[1] ^ 2 + v[2] ^ 2)/robot.sensor_dist
            update_speed!(robot, v_flocking + v_flocking/2 * distAvoid - turnAvoid - turnCenter, v_flocking + v_flocking/2 * distAvoid + turnAvoid + turnCenter)
        end

        # Leading Tracker
        robot = tracking_robots[1]
        neighbors_count = 0
        neighbors = get_neighbors(sim, robot, cells)

        vals = [0., 0.]

        for neighbor in neighbors
            if (neighbor in flocking_robots)
                dist = get_sensoric_data(robot, neighbor)

                if (minimum(dist) < robot.sensor_dist)
                    nearest = argmin(dist)
                    vals[1] += cos(robot.sensor_pos[nearest][3]) * dist[nearest]
                    vals[2] += sin(robot.sensor_pos[nearest][3]) * dist[nearest]
                end

            end
            
        end

        vals = vals ./ max(1, neighbors_count)
        if (abs(vals[2]) > 0)
            deg = atan(vals[2]/vals[1])
        else
            deg = 0
        end

        turn = robot.radius * deg / 2 * t_turnSpeed
        update_speed!(robot, v_tracking - turn, v_tracking + turn)

        # All Other Tracker
        for i in 2:length(tracking_robots)-1
            dist1 = get_sensoric_data(tracking_robots[i], tracking_robots[i-1])
            dist2 = get_sensoric_data(tracking_robots[i], tracking_robots[i+1])

            if (minimum(dist1) > (1/2 * tracking_robots[i].sensor_dist))
                nearest = argmin(dist1)
                vals[1] = cos(tracking_robots[i].sensor_pos[nearest][3]) * dist1[nearest]
                vals[2] = sin(tracking_robots[i].sensor_pos[nearest][3]) * dist1[nearest]

                if (abs(vals[2]) > 0)
                    deg = atan(vals[2]/vals[1])
                else
                    deg = 0
                end

                turn = tracking_robots[i].radius * deg / 2 * t_turnSpeed
                update_speed!(tracking_robots[i], v_tracking - turn, v_tracking + turn)
            end
            
            if (minimum(dist2) > tracking_robots[i].sensor_dist-2 || minimum(dist1) < (1/2 * tracking_robots[i].sensor_dist))
                update_speed!(tracking_robots[i], 0, 0)
            end
        end


        update!(sim)
    end
end