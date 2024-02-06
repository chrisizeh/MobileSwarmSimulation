using Plots
include("include_me.jl")

robot = Robot(0, 2, 3)
border = Border(-30, 30, -30, 30)

update_speed!(robot, 3, -3)
move!(robot, 2, border)
update_speed!(robot, 3, 3)
move!(robot, 2, border)



plt = plot(
    1,
    xlim = x_axis(border),
    ylim = y_axis(border),
    legend = false,
    marker = 2,
)

# build an animated gif by pushing new points to the plot, saving every 10th frame
@gif for i=1:10
    move!(robot, 1, border)
    push!(plt, robot.pos_x, robot.pos_y)
end every 1