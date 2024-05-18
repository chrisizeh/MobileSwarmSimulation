include("include_me.jl")

border = Border(-50, 50, -50, 50)

# 2 robots in straight lines; no border check
robot1 = Robot(0, 5, -6, 0, 0)
robot2 = Robot(1, 8, 7, 0, pi/2)
update_speed!(robot1, 5, 5)
update_speed!(robot2, 5, 5)

robots = [robot1, robot2]

sim = Simulation(robots, border; check=false, time_step=0.1)

for i in 1:100
    update!(sim)
end

plot_hist(sim)


# 2 robots in diagonal lines hitting each other
robot1 = Robot(0, 5, -5, 5, -pi/7, 5, 5)
robot2 = Robot(1, 5, 3, -5, pi/8, 5, 5)

robots = [robot1, robot2]
sim2 = Simulation(robots, border; check=true, dist=10, time_step=0.05, num_grid=5)

for i in 1:200
    update!(sim2)
end

plot_hist(sim2; speedup=1.5)



# 10 robots together with random velocities
robots = []
for i in 1:10
    push!(robots, Robot(i, 5, rand(-45.:45), rand(-45.:45), rand(0.:pi/8:2pi), rand(2.:5), rand(2.:5)))
end

sim = Simulation(robots, border; check=true, time_step=0.1)

for i in 1:400
    update!(sim)
end

plot_hist(sim; speedup=1.5)