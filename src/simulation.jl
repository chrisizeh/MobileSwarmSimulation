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


function Circle(pos, radius)
    ang = range(0, 2π, length = 25)
    front = range(-pi/4 + pos[3], pi/4 + pos[3], length = 5)
    return [Shape(radius * cos.(ang) .+ pos[1], radius * sin.(ang) .+ pos[2]), Shape(radius * cos.(front) .+ pos[1], radius * sin.(front) .+ pos[2])]
end


function plot_hist(sim::Simulation)
    if(length(sim.robots) == 0)
        return
    end
    
    min_length = minimum([size(robot.history)[2] for robot in sim.robots])
    anim = @animate for i=1:min_length
        plot(Circle(sim.robots[1].history[:, i], sim.robots[1].radius), 
            xlim = x_axis(border),
            ylim = y_axis(border),
            color = :orange,
            legend = false,
            size = (width(border), height(border)))

    if (length(sim.robots) > 1)
        for j in 2:length(sim.robots)
            plot!(Circle(sim.robots[j].history[:, i], sim.robots[j].radius), color=j)
        end
    end
    end
    gif(anim, fps=10)
end