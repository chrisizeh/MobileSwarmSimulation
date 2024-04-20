using Plots

include("robot.jl")
include("area.jl")


mutable struct Simulation 
    robots::Array{Robot}
    border::Border
    check_border::Bool

    sensoric_distance::Float64
    time_step::Float64
    num_grid::Int64

    grid::Array{Array{Robot}}
    grid_step_x::Float64
    grid_step_y::Float64

    function Simulation(robots, border, check=false, dist=nothing, time=1, num_grid=5)
        if isnothing(dist)
            dist = (border.right - border.left)/10
        end

        grid_step_x = (border.right - border.left)/num_grid
        grid_step_y = (border.top - border.bottom)/num_grid
        grid = Array{Vector{Robot}}(undef, num_grid, num_grid)

        for i in eachindex(grid)
            grid[i] = []
        end

        for robot in robots
            idX = Int64(ceil((robot.pos_x - border.left) /  grid_step_x))
            idY = Int64(ceil((robot.pos_y - border.bottom) /  grid_step_y))
            push!(grid[idX, idY], robot)
        end

        new(robots, border, check, dist, time, num_grid, grid, grid_step_x, grid_step_y)
    end

end


function update!(sim::Simulation)
    for robot in sim.robots
        idX = Int64(ceil((robot.pos_x - sim.border.left) /  sim.grid_step_x))
        idY = Int64(ceil((robot.pos_y - sim.border.bottom) /  sim.grid_step_y))

        move!(robot, sim.time_step, sim.check_border, sim.border)

        newIdX = Int64(ceil((robot.pos_x - sim.border.left) /  sim.grid_step_x))
        newIdY = Int64(ceil((robot.pos_y - sim.border.bottom) /  sim.grid_step_y))

        if(idX != newIdX || idY != newIdY)
            @info "move $(robot.id) from $(idX) $(idY) to $(newIdX) $(newIdY)"
            deleteat!(sim.grid[idX, idY], findfirst(==(robot), sim.grid[idX, idY]))
            push!(sim.grid[min(sim.num_grid, max(1, newIdX)), min(sim.num_grid, max(1, newIdY))], robot)
        end

        for i in eachindex(sim.grid)
            row, col = fldmod1(i, sim.num_grid)
            if(length(sim.grid[row, col]) > 1)
                robs = sim.grid[row, col]
                append!(robs, sim.grid[min(sim.num_grid, max(1, row - 1)), min(sim.num_grid, max(1, col - 1))])
                append!(robs, sim.grid[min(sim.num_grid, max(1, row - 1)), min(sim.num_grid, max(1, col + 1))])
                append!(robs, sim.grid[min(sim.num_grid, max(1, row + 1)), min(sim.num_grid, max(1, col - 1))])
                append!(robs, sim.grid[min(sim.num_grid, max(1, row + 1)), min(sim.num_grid, max(1, col + 1))])
                unique!(robs)

                for robot in sim.grid[row, col]
                    move_intersection!(robot, robs)
                end
            end
        end
    end
end


function Rectangle(x, y, deg, width, length)
    return Shape(corner_points(x, y, deg, width, length))
end


function plot_hist(sim::Simulation)
    if(length(sim.robots) == 0)
        return
    end
    
    min_length = minimum([length(robot.history) for robot in sim.robots])
    anim = @animate for i=1:min_length
        plot(Rectangle(sim.robots[1].history[i][1], sim.robots[1].history[i][2], sim.robots[1].history[i][3], sim.robots[1].width, sim.robots[1].length), 
            color="orange",
            xlim = x_axis(border),
            ylim = y_axis(border),
            legend = false)

        if (length(sim.robots) > 1)
            for j in 2:length(sim.robots)
                plot!(Rectangle(sim.robots[j].history[i][1], sim.robots[j].history[i][2], sim.robots[j].history[i][3], sim.robots[j].width, sim.robots[j].length))
            end
        end
    end
    gif(anim, fps=10)
end