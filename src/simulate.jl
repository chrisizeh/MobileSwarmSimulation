using Plots

include("robot.jl")
include("area.jl")

function rectangle(x, y, deg, width, length)
    corners_x = [length/2 , (-1) * length/2]
    corners_y = [width/2,(-1) * width/2]
    points = []

    trans_matrix = [cos(deg), -sin(deg), sin(deg), cos(deg)]
    push!(points, (x+ trans_matrix[1]*corners_x[1] + trans_matrix[2]*corners_y[1],
        y + trans_matrix[3]*corners_x[1] + trans_matrix[4]*corners_y[1]))

    push!(points, (x + trans_matrix[1]*corners_x[1] + trans_matrix[2]*corners_y[2],
        y + trans_matrix[3]*corners_x[1] + trans_matrix[4]*corners_y[2]))

    push!(points, (x + trans_matrix[1]*corners_x[2] + trans_matrix[2]*corners_y[2],
        y + trans_matrix[3]*corners_x[2] + trans_matrix[4]*corners_y[2]))

    push!(points, (x + trans_matrix[1]*corners_x[2] + trans_matrix[2]*corners_y[1],
        y + trans_matrix[3]*corners_x[2] + trans_matrix[4]*corners_y[1]))

    return Shape(points)
end

function plot_single_robot_hist(robot::Robot, border::Border)
    @gif for i=1:length(robot.history)
        plot(rectangle(robot.history[i][1], robot.history[i][2], robot.history[i][3], robot.width, robot.length), 
            color="orange",
            xlim = x_axis(border),
            ylim = y_axis(border),
            legend = false)
    end every 1
end