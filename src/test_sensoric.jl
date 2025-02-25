include("include_me.jl")

vMax = 1
turnSpeed = 0.1


function adapt(robot::Robot, dist::Array)
    close = [0., 0.]

    println(dist)
    if (sum(dist) < robot.NO_DIST_FOUND)
        nearest = argmin(dist)
        vals[1] = round(cos(transform_rad(robot.sensor_pos[nearest][3] + robot.deg)) * dist[nearest]; digits=9)
        vals[2] = round(sin(transform_rad(robot.sensor_pos[nearest][3] + robot.deg)) * dist[nearest]; digits=9)
        
        println(dist[nearest])
        if (dist[nearest] < 1.5)
            close[1] -= vals[2]
            close[2] += vals[1]
        end
        v = 0.2 * close
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
        println(turn)
        update_speed!(robot, vMax/2 - turn, vMax/2 + turn)
    end

end
 

robot1 = Robot(0; pos=[11, -9], vel=[1,1], deg=-pi/3, sensor_dist=2.0, sensor_num=6)
# robot2 = Robot(1; pos=[3, 3], sensor_num=4)
# robot3 = Robot(0; pos=[-7, 5], vel=[1,1], deg=-pi/4, sensor_dist=2.0, sensor_num=5)
# robot4 = Robot(0; pos=[3, -6], vel=[1,1], deg=pi/3, sensor_dist=2.0, sensor_num=5)
square_o = Rectangle_Obstacle([14.0, -1.0], 4.0, 15.0, color="#DAF7A6")
# round_o = Round_Obstacle([20.0, -5.0], 10.0)
area = Area(-2, 30, -15, 12; obstacles=[])

sim = Simulation([robot1], area, time_step=0.1)

for i in 1:300
    update!(sim)
    sensor_data1 = get_sensoric_data(sim.robots[1], area)
    # sensor_data3 = get_sensoric_data(sim.robots[3], round_o)
    # sensor_data4 = get_sensoric_data(sim.robots[4], square_o)

    println("robot1: ", sensor_data1)
    # println("robot3: ", sensor_data3)
    # println("robot4: ", sensor_data4)

    adapt(sim.robots[1], sensor_data1)
    # adapt(sim.robots[3], sensor_data3)
    # adapt(sim.robots[4], sensor_data4)
end

plot_hist(sim; speedup=2.0, showSensor=true)