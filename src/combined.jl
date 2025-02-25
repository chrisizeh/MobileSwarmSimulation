include("include_me.jl")

obstacles = []
# push!(obstacles, Rectangle_Obstacle([225.0, 175.0], 450.0, 350.0, color="#f19066"))
# push!(obstacles, Rectangle_Obstacle([975.0, 175.0], 550.0, 350.0, color="#f19066"))

push!(obstacles, Round_Obstacle([225.0, 175.0], 225.0, color="#465361"))
# push!(obstacles, Round_Obstacle([975.0, 175.0], 275.0, color="#465361"))
push!(obstacles, Round_Obstacle([1500.0, 300.0], 500.0, color="#465361"))
push!(obstacles, Round_Obstacle([750.0, 720.0], 100.0, color="#465361"))
push!(obstacles, Round_Obstacle([1300.0, 1300.0], 300.0, color="#465361"))
push!(obstacles, Round_Obstacle([350.0, 900.0], 75.0, color="#465361"))
push!(obstacles, Round_Obstacle([1.0, 500.0], 230.0, color="#465361"))

goal = Round_Obstacle([100.0, 1200.0], 50.0, color="#2C68A7")

area = Area(0, 1300, 0, 1300; obstacles=obstacles, goal=goal)

vMax = 50.0
vMin = 10.0
robots = []
flocking_robots = []
tracking_robots = []

lines = 6
per_line = 12
num_robots = per_line*lines
for line in 2:lines+1
    for i in 1:per_line
        if(line < 5)
            robot = Robot(i; radius=15, 
                color="#31E0CB",
                vel=[0, 0],
                pos=[500 + 40*(per_line-1) * (line % 2) + (-1)^line * 40 * (i - 1), 210 - 40 * (line - 2)], 
                deg = pi/2,
                sensor_dist=70.0, sensor_num=6, spec=pi)
        else
            robot = Robot(i; radius=10, 
                color="#E08631",
                vel=[0, 0],
                pos=[500 + 30*(per_line-1) * (line % 2) + (-1)^line * 30 * (i - 1), 210 - 30 * (line - 2)], 
                deg = pi/2,
                sensor_dist=200.0, sensor_num=8, spec=2*pi, sensor_deg=pi/2)
        end

        

        if(line == 2 && i == per_line/2)
            push!(tracking_robots, robot)
        end

        if(line < 5)
            push!(flocking_robots, robot)            
        else
            push!(tracking_robots, robot)
        end
        push!(robots, robot)
    end
end

sim = Simulation(robots, area; open_area=false, num_grid=5, time_step=0.2)
avoidSpeed = 0.5
centerSpeed = 0.1
turnSpeed = 0.05

t_turnSpeed = 0.8

vals = [0., 0.]
v = [0., 0.]

# FIX obstacle in front
cells = ceil(Int64, flocking_robots[1].sensor_dist / minimum(sim.grid_step))
for i in 1:500
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
                    
                    if (dist[nearest] < 30)
                        close[1] -= vals[2]
                        close[2] += vals[1]
                        close_count += 1
                    elseif (typeof(neighbor) == Robot)
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

        turn = robot.radius * deg / 2 * turnSpeed
        update_speed!(robot, vMax/2 - turn, vMax/2 + turn)
    end

    for i in 2:length(tracking_robots)-1
        dist1 = get_sensoric_data(tracking_robots[i], tracking_robots[i-1])
        dist2 = get_sensoric_data(tracking_robots[i], tracking_robots[i+1])

        if (i > 1 && minimum(dist1) > 40)
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
        
        if (minimum(dist2) > 198 || minimum(dist1) < 40)
            update_speed!(tracking_robots[i], 0, 0)
        end
    end


    update!(sim)
end
plot_hist(sim; speedup=5.0, showSensor=false, showGrid=true)