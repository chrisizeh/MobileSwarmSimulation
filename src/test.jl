include("include_me.jl")

robot = Robot(0, 2, 3)
border = Border(-10, 10, -10, 10)
update_speed!(robot, 1, 1)

for i in 1:5
    move!(robot, 1, border)
end

update_speed!(robot, 0, 1)

for i in 1:3
    move!(robot, 1, border)
end

update_speed!(robot, 1, 1)

for i in 1:5
    move!(robot, 1, border)
end

update_speed!(robot, -0.5, 1.)

for i in 1:3
    move!(robot, 1, border)
end

update_speed!(robot, 1, 1)

for i in 1:5
    move!(robot, 1, border)
end

plot_single_robot_hist(robot, border)