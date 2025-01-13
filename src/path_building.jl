include("include_me.jl")

obstacles = []
push!(obstacles, Round_Obstacle([225.0, 175.0], 225.0, color="#f19066"))
push!(obstacles, Round_Obstacle([975.0, 175.0], 275.0, color="#f19066"))
push!(obstacles, Round_Obstacle([1500.0, 300.0], 500.0, color="#f19066"))
push!(obstacles, Round_Obstacle([750.0, 720.0], 100.0, color="#f19066"))
push!(obstacles, Round_Obstacle([1300.0, 1300.0], 300.0, color="#f19066"))
push!(obstacles, Round_Obstacle([350.0, 900.0], 75.0, color="#f19066"))
push!(obstacles, Round_Obstacle([0.0, 500.0], 230.0, color="#f19066"))
push!(obstacles, Round_Obstacle([100.0, 1200.0], 50.0, color="#26de81"))

area = Area(0, 1300, 0, 1300; obstacles=obstacles)

vMin = 50.0
# vMin = 10.0
robots = []
num_robots = 7*4
for lines in 2:5
    for i in 1:7
        robot = Robot(i; radius=10, 
            color="##2ecc71",
            vel=[0, 0],
            pos=[500 + 180 * (lines % 2) + (-1)^lines * 30 * (i - 1), 110 - 30 * (lines - 2)], 
            deg = pi/2,
            sensor_dist=100.0, sensor_num=6, spec=2*pi)
        push!(robots, robot)
    end
end


update_speed!(robots[1], vMin, vMin)

sim = Simulation(robots, area; open_area=true, num_grid=20, time_step=0.1)
centerSpeed = 0.6
turnSpeed = 0.8

vals = [0., 0.]

for j in 1:1000
    update!(sim)

    dist = get_sensoric_data(robots[1], robots[2])
    if (minimum(dist) > 98)
        update_speed!(robots[1], 0, 0)
    else
        update_speed!(robots[1], vMin, vMin)
    end

    for i in 2:num_robots-1

        dist1 = get_sensoric_data(robots[i], robots[i-1])
        dist2 = get_sensoric_data(robots[i], robots[i+1])

        if (i > 1 && minimum(dist1) > 40)
            nearest = argmin(dist1)
            vals[1] = cos(robots[i].sensor_pos[nearest][3]) * dist1[nearest]
            vals[2] = sin(robots[i].sensor_pos[nearest][3]) * dist1[nearest]

            if (abs(vals[2]) > 0)
                deg = atan(vals[2]/vals[1])
            else
                deg = 0
            end

            turn = robots[i].radius * deg / 2 * turnSpeed
            update_speed!(robots[i], vMin - turn, vMin + turn)
        end
        
        if (minimum(dist2) > 98 || minimum(dist1) < 40)
            update_speed!(robots[i], 0, 0)
        end
    end
end

plot_hist(sim; speedup=1.0, showSensor=false)