include("include_me.jl")

border = Border(-20, 20, -20, 20)


robot1 = Robot(0, 2, 3, 0, 0, pi/2)
robot2 = Robot(1, 2, 3, 5, 0, pi/2)
update_speed!(robot1, 1, 1)
update_speed!(robot2, 1, 1)

for i in 1:10
    move!(robot1, 1, border)
    move!(robot2, 1, border)
end

plot_mult_robot_hist([robot1, robot2], border)