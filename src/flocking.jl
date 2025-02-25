include("include_me.jl")

obstacles = []
# push!(obstacles, Rectangle_Obstacle([225.0, 175.0], 450.0, 350.0, color="#f19066"))
# push!(obstacles, Rectangle_Obstacle([975.0, 175.0], 550.0, 350.0, color="#f19066"))
push!(obstacles, Round_Obstacle([225.0, 175.0], 225.0, color="#465361"))
# push!(obstacles, Round_Obstacle([975.0, 175.0], 275.0, color="#f19066"))
push!(obstacles, Round_Obstacle([1500.0, 300.0], 500.0, color="#465361"))
push!(obstacles, Round_Obstacle([750.0, 720.0], 100.0, color="#465361"))
push!(obstacles, Round_Obstacle([1300.0, 1300.0], 300.0, color="#465361"))
push!(obstacles, Round_Obstacle([350.0, 900.0], 75.0, color="#465361"))
push!(obstacles, Round_Obstacle([1.0, 500.0], 230.0, color="#465361"))

goal = Round_Obstacle([100.0, 1200.0], 50.0, color="#2C68A7")

area = Area(0, 1300, 0, 1300; obstacles=obstacles, goal=goal)
# area = Area(0, 500, 0, 1300; obstacles=obstacles)

vMax = 50.0
vMin = 10.0
robots = []

lines = 2
per_line = 12
num_robots = per_line*lines
for line in 2:lines+1
    for i in 1:per_line
        robot = Robot(i; radius=10, 
            color="#31E0CB",
            vel=[0, 0],
            pos=[500 + 30*(per_line-1) * (line % 2) + (-1)^line * 30 * (i - 1), 150 - 30 * (line - 2)], 
            deg = pi/2,
            sensor_dist=70.0, sensor_num=6, spec=pi)
        push!(robots, robot)
    end
end

sim = Simulation(robots, area; open_area=false, num_grid=5, time_step=0.1)
avoidSpeed = 0.5
centerSpeed = 0.1
turnSpeed = 0.05

vals = [0., 0.]
v = [0., 0.]

# FIX obstacle in front
cells = ceil(Int64, robots[1].sensor_dist / minimum(sim.grid_step))
for i in 1:1500
    for robot in robots
        # println(robot.id)
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
                    println(dist)
                    nearest = argmin(dist)
                    vals[1] = round(cos(robot.sensor_pos[nearest][3] + robot.deg) * dist[nearest]; digits=5)
                    vals[2] = round(sin(robot.sensor_pos[nearest][3] + robot.deg) * dist[nearest]; digits=5)
                    
                    # println(vals)
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
        println(v)
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
        # println("turn")
        # println(turn)
        update_speed!(robot, vMax/2 - turn, vMax/2 + turn)
    end
    update!(sim)
end
plot_hist(sim; speedup=5.0, showSensor=false, showGrid=true)