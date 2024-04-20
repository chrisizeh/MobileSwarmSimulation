using Plots

include("robot.jl")
include("area.jl")


mutable struct Simulation 
    robots::Array{Robot}
    border::Border
    checkBorder::Bool

    sensoricDistance::Float64
    timeStep::Float64
    numGrid::Int64

    grid::Array{Array{Robot}}
    gridStepX::Float64
    gridStepY::Float64

    function Simulation(robots, border, check=false, dist=nothing, time=1, num_grid=5)
        if isnothing(dist)
            dist = (border.right - border.left)/10
        end

        gridStepX = (border.right - border.left)/num_grid
        gridStepY = (border.top - border.bottom)/num_grid
        grid = Array{Vector{Robot}}(undef, num_grid, num_grid)

        for i in eachindex(grid)
            grid[i] = []
        end

        for robot in robots
            idX = Int64(ceil((robot.pos_x - border.left) /  gridStepX))
            idY = Int64(ceil((robot.pos_y - border.bottom) /  gridStepY))
            push!(grid[idX, idY], robot)
        end

        new(robots, border, check, dist, time, num_grid, grid, gridStepX, gridStepY)
    end

end


function update!(sim::Simulation)
    for robot in sim.robots
        idX = Int64(ceil((robot.pos_x - sim.border.left) /  sim.gridStepX))
        idY = Int64(ceil((robot.pos_y - sim.border.bottom) /  sim.gridStepY))

        move!(robot, sim.timeStep, sim.checkBorder, sim.border)

        newIdX = Int64(ceil((robot.pos_x - sim.border.left) /  sim.gridStepX))
        newIdY = Int64(ceil((robot.pos_y - sim.border.bottom) /  sim.gridStepY))

        if(idX != newIdX || idY != newIdY)
            @info "move $(robot.id) from $(idX) $(idY) to $(newIdX) $(newIdY)"
            deleteat!(sim.grid[idX, idY], findfirst(==(robot), sim.grid[idX, idY]))
            push!(sim.grid[min(sim.numGrid, max(1, newIdX)), min(sim.numGrid, max(1, newIdY))], robot)
        end

        for i in eachindex(sim.grid)
            row, col = fldmod1(i, sim.numGrid)
            if(length(sim.grid[row, col]) > 1)
                robs = sim.grid[row, col]
                append!(robs, sim.grid[min(sim.numGrid, max(1, row - 1)), min(sim.numGrid, max(1, col - 1))])
                append!(robs, sim.grid[min(sim.numGrid, max(1, row - 1)), min(sim.numGrid, max(1, col + 1))])
                append!(robs, sim.grid[min(sim.numGrid, max(1, row + 1)), min(sim.numGrid, max(1, col - 1))])
                append!(robs, sim.grid[min(sim.numGrid, max(1, row + 1)), min(sim.numGrid, max(1, col + 1))])
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