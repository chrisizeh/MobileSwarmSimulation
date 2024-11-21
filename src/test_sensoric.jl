include("include_me.jl")

robot = Robot(0; pos=[-3, 3], sensor_dist=2.0, sensor_num=5)
robot2 = Robot(1; pos=[3, 2.1], sensor_num=4)
square_o = Rectangle_Obstacle([7.0, 5.0], 2.0, 4.0, color="#DAF7A6")
area = Area(-10, 10, -10, 10; obstacles=[square_o])

update_speed!(robot, 1.0, 1.0)

sim = Simulation([robot, robot2], area, time_step=0.1)

for i in 1:100
    update!(sim)
    sensor_data = get_sensoric_data(sim.robots[1], sim.robots[2])

    if (sum(sensor_data) > 0)
        update_speed!(robot, 0.5, 0.5)
        println(sensor_data)
    end

end

plot_hist(sim; speedup=2.0, showSensor=true)