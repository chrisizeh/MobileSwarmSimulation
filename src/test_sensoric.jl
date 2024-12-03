include("include_me.jl")

function adapt(robot::Robot, sensor_data::Array)
    if (sum(sensor_data) > 0)
        update_speed!(robot, 0.2, 0.2)
    else
        update_speed!(robot, 1, 1)
    end

end
 

robot1 = Robot(0; pos=[2, 2], vel=[1,1], deg=0, sensor_dist=2.0, sensor_num=5)
robot2 = Robot(1; pos=[3, 3], sensor_num=4)
robot3 = Robot(0; pos=[-7, 5], vel=[1,1], deg=-pi/4, sensor_dist=2.0, sensor_num=5)
robot4 = Robot(0; pos=[3, -6], vel=[1,1], deg=pi/3, sensor_dist=2.0, sensor_num=5)
square_o = Rectangle_Obstacle([7.0, -5.0], 4.0, 6.0, color="#DAF7A6")
round_o = Round_Obstacle([-5.0, 5.0], 3.0)
area = Area(-12, 12, -12, 12; obstacles=[round_o, square_o])

sim = Simulation([robot1, robot2, robot3, robot4], area, time_step=0.1)

for i in 1:160
    update!(sim)
    sensor_data1 = get_sensoric_data(sim.robots[1], sim.robots[2])
    sensor_data3 = get_sensoric_data(sim.robots[3], round_o)
    sensor_data4 = get_sensoric_data(sim.robots[4], square_o)

    println("robot1: ", sensor_data1)
    println("robot3: ", sensor_data3)
    println("robot4: ", sensor_data4)

    adapt(sim.robots[1], sensor_data1)
    adapt(sim.robots[3], sensor_data3)
    adapt(sim.robots[4], sensor_data4)
end

plot_hist(sim; speedup=2.0, showSensor=true)