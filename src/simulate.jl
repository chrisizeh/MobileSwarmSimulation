using Plots

include("robot.jl")
include("area.jl")

function rectangle(x, y, deg, width, length)
    return Shape(corner_points(x, y, deg, width, length))
end

function plot_single_robot_hist(robot::Robot, border::Border)
    anim = @animate for i=1:length(robot.history)
        plot(rectangle(robot.history[i][1], robot.history[i][2], robot.history[i][3], robot.width, robot.length), 
            color="orange",
            xlim = x_axis(border),
            ylim = y_axis(border),
            legend = false)
    end
    gif(anim, fps=10)
end

function plot_mult_robot_hist(robots::Vector{Robot}, border::Border)
    min_length = minimum([length(robot.history) for robot in robots])

    anim = @animate for i=1:min_length
        plot(rectangle(robots[1].history[i][1], robots[1].history[i][2], robots[1].history[i][3], robots[1].width, robots[1].length), 
            color="orange",
            xlim = x_axis(border),
            ylim = y_axis(border),
            legend = false)

        for j in 2:length(robots)
            plot!(rectangle(robots[j].history[i][1], robots[j].history[i][2], robots[j].history[i][3], robots[j].width, robots[j].length))
        end
    end
    gif(anim, fps=10)
end