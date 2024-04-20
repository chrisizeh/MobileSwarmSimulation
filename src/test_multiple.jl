include("include_me.jl")

border = Border(-50, 50, -50, 50)

robot1 = Robot(0, 2, 3, 0, 0, pi/2)
robot2 = Robot(1, 2, 3, 5, 0, pi/2)
update_speed!(robot1, 1, 1)
update_speed!(robot2, 1, 1)

for i in 1:10
    move!(robot1, 1, border)
    move!(robot2, 1, border)
end

plot_mult_robot_hist([robot1, robot2], border)


robot1 = Robot(0, 2, 3, -2, 0, -pi/7)
robot2 = Robot(1, 2, 3, 2, 0, pi/8)
update_speed!(robot1, 1, 1)
update_speed!(robot2, 1, 1)

robots = [robot1, robot2]

move!(robot1, 0.05, border, robots)

for i in 1:100
    move!(robot1, 0.05, border, robots)
    move!(robot2, 0.05, border, robots)
end

plot_mult_robot_hist([robot1, robot2], border)