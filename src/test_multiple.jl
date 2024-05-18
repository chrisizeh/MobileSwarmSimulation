include("include_me.jl")

border = Border(-50, 50, -50, 50)

robot1 = Robot(0, 10, 0, 0, 0)
robot2 = Robot(1, 15, 5, 0, pi/2)
update_speed!(robot1, 1, 1)
update_speed!(robot2, 1, 1)

robots = [robot1, robot2]

sim = Simulation(robots, border, false)

for i in 1:60
    update!(sim)
end

plot_hist(sim)


robot1 = Robot(0, 5, -2, 0, -pi/7, 1, 1)
robot2 = Robot(1, 5, 2, 0, pi/8, 1, 1)

robots = [robot1, robot2]
sim2 = Simulation(robots, border, true, 10, 0.05, 5)

for i in 1:200
    update!(sim2)
end

plot_hist(sim2)