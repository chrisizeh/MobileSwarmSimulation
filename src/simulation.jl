using Plots

include("robot.jl")
include("area.jl")


mutable struct Simulation 
    robots::Array{Robot}
    border::Border
    open_area::Bool

    sensoric_distance::Float64
    time_step::Float64
    num_grid::Int64

    grid::Array{Array{Robot}}
    grid_step::Array{Float64}

    function Simulation(robots, border; open_area=false, dist=nothing, time_step=1, num_grid=5)
        if isnothing(dist)
            dist = (border.right - border.left)/10
        end

        grid_step_x = (border.right - border.left)/num_grid
        grid_step_y = (border.top - border.bottom)/num_grid
        grid_step = [grid_step_x, grid_step_y]

        grid = Array{Vector{Robot}}(undef, num_grid, num_grid)

        for i in eachindex(grid)
            grid[i] = []
        end

        for robot in robots
            id_x = Int64(ceil((robot.pos[1] - border.left) /  grid_step[1]))
            id_y = Int64(ceil((robot.pos[2] - border.bottom) /  grid_step[2]))
            push!(grid[id_x, id_y], robot)
        end

        new(robots, border, open_area, dist, time_step, num_grid, grid, grid_step)
    end

end


function update!(sim::Simulation)
    for robot in sim.robots
        id_x = min(sim.num_grid, max(1, Int64(ceil((robot.pos[1] - sim.border.left) /  sim.grid_step[1]))))
        id_y = min(sim.num_grid, max(1, Int64(ceil((robot.pos[2] - sim.border.bottom) /  sim.grid_step[2]))))

        move!(robot, sim.time_step; checkBorder=!sim.open_area, border=sim.border)

        new_id_x = min(sim.num_grid, max(1, Int64(ceil((robot.pos[1] - sim.border.left) /  sim.grid_step[1]))))
        new_id_y = min(sim.num_grid, max(1, Int64(ceil((robot.pos[2] - sim.border.bottom) /  sim.grid_step[2]))))

        if(id_x != new_id_x || id_y != new_id_y)
            deleteat!(sim.grid[id_x, id_y], findfirst(==(robot), sim.grid[id_x, id_y]))
            push!(sim.grid[min(sim.num_grid, max(1, new_id_x)), min(sim.num_grid, max(1, new_id_y))], robot)
        end
    end

    for i in eachindex(sim.grid)
        row, col = fldmod1(i, sim.num_grid)
        if(length(sim.grid[row, col]) >= 1)
            robs = sim.grid[row, col]

            for x in -1:1
                for y in -1:1
                    append!(robs, sim.grid[min(sim.num_grid, max(1, row + x)), min(sim.num_grid, max(1, col + y))])
                end
            end
            unique!(robs)

            if(length(robs) > 1)
                for robot in sim.grid[row, col]
                    id_x = min(sim.num_grid, max(1, Int64(ceil((robot.pos[1] - sim.border.left) /  sim.grid_step[1]))))
                    id_y = min(sim.num_grid, max(1, Int64(ceil((robot.pos[2] - sim.border.bottom) /  sim.grid_step[2]))))

                    move_intersection!(robot, robs)

                    new_id_x = min(sim.num_grid, max(1, Int64(ceil((robot.pos[1] - sim.border.left) /  sim.grid_step[1]))))
                    new_id_y = min(sim.num_grid, max(1, Int64(ceil((robot.pos[2] - sim.border.bottom) /  sim.grid_step[2]))))

                    if(id_x != new_id_x || id_y != new_id_y)
                        deleteat!(sim.grid[id_x, id_y], findfirst(==(robot), sim.grid[id_x, id_y]))
                        push!(sim.grid[min(sim.num_grid, max(1, new_id_x)), min(sim.num_grid, max(1, new_id_y))], robot)
                    end
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


# Speedup in percentage (1 = 100%)
function plot_hist(sim::Simulation; speedup=1)
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
    gif(anim, fps=speedup/sim.time_step)
end