include("include_me.jl")

border = Border(-50, 50, -50, 50)

robot1 = Robot(0, 5, 0, 0, 0)
robot2 = Robot(1, 8, 5, 0, pi/2)
update_speed!(robot1, 5, 5)
update_speed!(robot2, 5, 5)

robots = [robot1, robot2]

sim = Simulation(robots, border; check=true, time_step=0.1)

for i in 1:600
    update!(sim)
end

plot_hist(sim)


robot1 = Robot(0, 5, -2, 0, -pi/7, 5, 5)
robot2 = Robot(1, 5, 2, 0, pi/8, 5, 5)

robots = [robot1, robot2]
sim2 = Simulation(robots, border; check=true, dist=10, time_step=0.05, num_grid=5)

for i in 1:200
    update!(sim2)
end

plot_hist(sim2; speedup=1.5)