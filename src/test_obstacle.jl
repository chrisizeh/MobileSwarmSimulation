include("include_me.jl")

# Two robots both having an obstacle in front
robot1 = Robot(0; pos=[2, 3], vel=[1, 1])
robot2 = Robot(0; pos=[-5, -3], vel=[1, 1])

round_o = Round_Obstacle([-1, -1], 3)
square_o = Rectangle_Obstacle([7, 5], 2, 4)

border = Area(-10, 10, -10, 10; obstacles=[round_o, square_o])

sim = Simulation([robot1, robot2], border)

for i in 1:10
    update!(sim)
end

plot_hist(sim)