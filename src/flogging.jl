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
push!(obstacles, Round_Obstacle([0.0, 500.0], 230.0, color="#f19066"))
push!(obstacles, Round_Obstacle([100.0, 1200.0], 50.0, color="#26de81"))

area = Area(0, 1300, 0, 1300; obstacles=obstacles)

vMax = 50.0
vMin = 10.0
robots = []
for i in 1:10
    robot = Robot(i; radius=10, 
        color="#a55eea",
        vel=[vMin, vMin],
        pos=[500 + 30 * (round(i/5) + i%5 - 1), 10 + 30 * round(i/5)], 
        deg = pi/2,
        # pos=[rand(500:700), rand(10:200)], 
        # deg=rand(0:pi/4:2*pi),
        sensor_dist=100.0, sensor_num=6, spec=3*pi/2)
    push!(robots, robot)
end

sim = Simulation(robots, area; open_area=false, num_grid=20, time_step=0.1)
avoidSpeed = 0.05
centerSpeed = 0.05
turnSpeed = 0.02

vals = [0., 0.]
v = [0., 0.]

for i in 1:1000
    update!(sim)
    for robot in robots
        close = [0., 0.]
        neighbors_dist = [0., 0.]
        neighbors_count = 0
        neighbors = get_neighbors(sim, robot, 1)

        for neighbor in neighbors
            if robot != neighbor
                dist = get_sensoric_data(robot, neighbor)

                if (sum(dist) < robot.NO_DIST_FOUND)
                    nearest = argmin(dist)
                    vals[1] = cos(robot.sensor_pos[nearest][3]) * dist[nearest]
                    vals[2] = sin(robot.sensor_pos[nearest][3]) * dist[nearest]
                    
                    if (dist[nearest] < 20)
                        close[1] -= vals[1]
                        close[2] -= vals[2]
                    elseif (typeof(neighbor) == Robot)
                        neighbors_dist[1] += vals[1]
                        neighbors_dist[2] += vals[2]
                        neighbors_count += 1
                    end
                end
            end
        end

        v = avoidSpeed * close + centerSpeed * neighbors_dist / max(1, neighbors_count)     
        speed = sqrt(v[1]^2 + v[2]^2)
        if (speed > vMax)
            v = v / speed * vMax
            speed = vMax
        end

        if (speed < vMin)
            v = v / max(speed, 0.1) * vMin
            speed = vMin
        end

        if (abs(sum(v)) > 0)
            deg = atan(v[2]/v[1])
        else
            deg = 0
        end

        turn = robot.radius * deg / 2 * turnSpeed
        update_speed!(robot, speed - turn, speed + turn)
    end
end

plot_hist(sim; speedup=5.0, showSensor=false)