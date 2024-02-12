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

border = Border(-5, 5, -5, 5)

robot1 = Robot(0, 1, 2, -3, -3.5, 3 * pi/8)
robot2 = Robot(0, 1, 2, -3, -3.5, -7 * pi / 8)
robot3 = Robot(0, 1, 2, -3, 3.5, 5 * pi / 8)
robot4 = Robot(0, 1, 2, -3, 3.5, -pi / 8)
robot5 = Robot(0, 1, 2, 3, -3.5, - 3 * pi / 8)
robot6 = Robot(0, 1, 2, 3, -3.5, 7 * pi / 8)
robot7 = Robot(0, 1, 2, 3, 3.5, pi / 8)
robot8 = Robot(0, 1, 2, 3, 3.5, -5pi/8)

robots = [robot1, robot2, robot3, robot4, robot5, robot6, robot7, robot8]

for robot in robots
    update_speed!(robot, 1, 1)

    for i in 1:100
        move!(robot, 0.05, border)
    end
end

plot_mult_robot_hist(robots, border)
