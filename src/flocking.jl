include("include_me.jl")

obstacles = []
# push!(obstacles, Rectangle_Obstacle([225.0, 175.0], 450.0, 350.0, color="#f19066"))
# push!(obstacles, Rectangle_Obstacle([975.0, 175.0], 550.0, 350.0, color="#f19066"))
push!(obstacles, Round_Obstacle([225.0, 175.0], 225.0, color="#f19066"))
push!(obstacles, Round_Obstacle([975.0, 175.0], 275.0, color="#f19066"))
push!(obstacles, Round_Obstacle([1500.0, 300.0], 500.0, color="#f19066"))
push!(obstacles, Round_Obstacle([750.0, 720.0], 100.0, color="#f19066"))
push!(obstacles, Round_Obstacle([1300.0, 1300.0], 300.0, color="#f19066"))
push!(obstacles, Round_Obstacle([350.0, 900.0], 75.0, color="#f19066"))
push!(obstacles, Round_Obstacle([1.0, 500.0], 230.0, color="#f19066"))

goal = Round_Obstacle([100.0, 1200.0], 50.0, color="#26de81")

area = Area(0, 1300, 0, 1300; obstacles=obstacles)
# area = Area(0, 500, 0, 1300; obstacles=obstacles)

vMax = 50.0
vMin = 10.0
robots = []
num_robots = 7*3
for lines in 2:2
    for i in 1:1
        robot = Robot(i; radius=40, 
            color="#a55eea",
            vel=[vMin, vMin],
            pos=[490 + 180 * (lines % 2) + (-1)^lines * 30 * (i - 1), 110 - 30 * (lines - 2)], 
            deg = pi/4,
            sensor_dist=200.0, sensor_num=6, spec=pi)
        push!(robots, robot)
    end
end

sim = Simulation(robots, area; open_area=false, num_grid=5, time_step=0.1)
avoidSpeed = 0.5
centerSpeed = 0.6
turnSpeed = 0.2

vals = [0., 0.]
v = [0., 0.]

# FIX obstacle in front
cells = ceil(Int64, robots[1].sensor_dist / minimum(sim.grid_step))
for i in 1:800
    for robot in robots
        close = [0., 0.]
        neighbors_dist = [0., 0.]
        neighbors_count = 0
        neighbors = get_neighbors(sim, robot, cells)

        for neighbor in neighbors
            if robot != neighbor
                dist = get_sensoric_data(robot, neighbor)

                if (sum(dist) < robot.NO_DIST_FOUND)
                    nearest = argmin(dist)
                    vals[1] = round(cos(transform_rad(robot.sensor_pos[nearest][3] + robot.deg)) * dist[nearest]; digits=5)
                    vals[2] = round(sin(transform_rad(robot.sensor_pos[nearest][3] + robot.deg)) * dist[nearest]; digits=5)
                    
                    if (dist[nearest] < 120)
                        close[1] -= vals[2]
                        close[2] += vals[1]
                    elseif (typeof(neighbor) == Robot)
                        neighbors_dist[1] += vals[1]
                        neighbors_dist[2] += vals[2]
                        neighbors_count += 1
                    end
                end
            end
        end

        v = avoidSpeed * close + centerSpeed * neighbors_dist / max(1, neighbors_count)
        if (abs(sum(v)) > 0)
            if(sum(abs.(v)) > robot.sensor_dist/100)
                deg = atan(v[2], v[1])
            else
                deg = pi
            end
        else
            deg = 0
        end

        turn = robot.radius * deg / 2 * turnSpeed
        # update_speed!(robot, 0.3 * (vMax/2 - turn) + 0.7 * robot.vel[1], 0.3 * (vMax/2 + turn) + 0.7 * robot.vel[2])
        update_speed!(robot, vMax/2 - turn, vMax/2 + turn)
    end
    update!(sim)
end
plot_hist(sim; speedup=5.0, showSensor=true, showGrid=true)