include("include_me.jl")

# Two robots both having an obstacle in front
robot1 = Robot(0; pos=[2, 3], vel=[1, 1])
robot2 = Robot(0; pos=[-5, -3], vel=[1, 1])

round_o = Round_Obstacle([-1, -1], 3)
square_o = Rectangle_Obstacle([7, 5], 2, 4)

area = Area(-10, 10, -10, 10; obstacles=[round_o, square_o])

sim = Simulation([robot1, robot2], area; open_area=true, time_step=0.05)

for i in 1:100
    update!(sim)
end

plot_hist(sim)


# Random robots moving on area with one big obstacle
round_o = Round_Obstacle([0, 0], 20)
area = Area(-50, 50, -50, 50; obstacles=[round_o])

robots = []
for i in 1:10
    push!(robots, Robot(i; radius=5, pos=[rand(-45.:45), rand(-45.:45)], deg=rand(0.:pi/8:2pi), vel=[rand(2.:5), rand(2.:5)]))
end

sim = Simulation(robots, area; open_area=false, time_step=0.1, num_grid=5)

for i in 1:400
    update!(sim)
end

plot_hist(sim; speedup=1.5)


# Random robot moving on area with multiple obstacles

obstacles = []
for i in -2:2
    for j in -2:2
        if (i * j % 2 == 0)
            push!(obstacles, Rectangle_Obstacle([40 * i, 40 * j], 10, 15))
        else
            push!(obstacles, Round_Obstacle([40 * i, 40 * j], 10))
        end
    end
end

area = Area(-100, 100, -100, 100; obstacles=obstacles)

robots = []
for i in 1:10
    push!(robots, Robot(i; radius=5, pos=[rand(-45.:45), rand(-45.:45)], deg=rand(0.:pi/8:2pi), vel=[rand(2.:5), rand(2.:5)]))
end

sim = Simulation(robots, area; open_area=false, time_step=0.1, num_grid=5)

for i in 1:400
    update!(sim)
end

plot_hist(sim; speedup=1.5)