using Plots

include("robot.jl")
include("area.jl")

"""
   Simulation

   Simulation of multiple robots on area with borders.
   The area is split into a grid of neighborhoods to speed up the intersection checking.
   Both robots and obstacles are placed in a cell according to its center point.

# Fields
- `robots::Array{Robot}`: List of robots active in simulation
- `area::Area`: The area of the simulation area
- `open_area::Bool`: Flag, if the robots can leave the area
- `time_step::Float64`: Current time step of the simulation
- `num_grid::Int64`: Number of grids per axis to split the simulation to
- `grid::Array{Array{Robot}}`: Grid of the simulation with robots in the appropriate cell
- `grid_step::Tuple{Float64, Float64}`: Size of each cell per axis
"""
mutable struct Simulation 
    robots::Array{Robot}
    area::Area
    open_area::Bool

    time_step::Float64
    num_grid::Int64

    grid::Array{Array{Union{Robot, Obstacle}}}
    grid_step::Tuple{Float64, Float64}

    function Simulation(robots, area; open_area=false, time_step=1, num_grid=5)
        grid_step_x = (area.right - area.left)/num_grid
        grid_step_y = (area.top - area.bottom)/num_grid
        grid_step = (grid_step_x, grid_step_y)

        grid = Array{Vector{Union{Robot, Obstacle}}}(undef, num_grid, num_grid)

        for i in eachindex(grid)
            grid[i] = []
        end

        for robot in robots
            id_x = Int64(ceil((robot.pos[1] - area.left) /  grid_step[1]))
            id_y = Int64(ceil((robot.pos[2] - area.bottom) /  grid_step[2]))
            push!(grid[id_x, id_y], robot)
        end

        for obstacle in area.obstacles
            id_x = min(num_grid, max(1, Int64(ceil((obstacle.center[1] - area.left) /  grid_step[1]))))
            id_y = min(num_grid, max(1, Int64(ceil((obstacle.center[2] - area.bottom) /  grid_step[2]))))
            # push!(grid[id_x, id_y], obstacle)

            # if (width(obstacle) > grid_step[1])
            id_x1 = min(num_grid, max(1, Int64(ceil(((obstacle.center[1] - width(obstacle)/2) - area.left) / grid_step[1]))))
            id_x2 = min(num_grid, max(1, Int64(ceil(((obstacle.center[1] + width(obstacle)/2) - area.left) / grid_step[1]))))

            # if (id_x1 != id_x || id_x2 != id_x)
            # for i in id_x1:1:id_x2
            #     push!(grid[i, id_y], obstacle)
            # end
            # end
            # end

            # if (height(obstacle) > grid_step[2])
            id_y1 = min(num_grid, max(1, Int64(ceil(((obstacle.center[2] - height(obstacle)/2) - area.bottom) / grid_step[2]))))
            id_y2 = min(num_grid, max(1, Int64(ceil(((obstacle.center[2] + height(obstacle)/2) - area.bottom) / grid_step[2]))))

            # if (id_y1 != id_y || id_y2 != id_y)
            for x in id_x1:id_x2
                for y in id_y1:id_y2
                    push!(grid[x, y], obstacle)
                end
            end
                # end
            # end
            
            
        end

        new(robots, area, open_area, time_step, num_grid, grid, grid_step)
    end

end


"""
update!(sim::Simulation) -> None

Perform one time step of the simulation.
Each robots moves for one timestep ignoring other robots and only checking for the border, if it is not an open area.
Afterwards moving the robots location on the grid is updated.
In the next step the intersections between robots are checked and fixed. Again, the position on the grid for the robots are updated.

# Arguments
- `sim::Simulation`: Simulation to perform the update on
"""
function update!(sim::Simulation)
    for robot in sim.robots
        id_x = min(sim.num_grid, max(1, Int64(ceil((robot.pos[1] - sim.area.left) /  sim.grid_step[1]))))
        id_y = min(sim.num_grid, max(1, Int64(ceil((robot.pos[2] - sim.area.bottom) /  sim.grid_step[2]))))

        move!(robot, sim.time_step; checkBorder=!sim.open_area, border=sim.area)

        new_id_x = min(sim.num_grid, max(1, Int64(ceil((robot.pos[1] - sim.area.left) /  sim.grid_step[1]))))
        new_id_y = min(sim.num_grid, max(1, Int64(ceil((robot.pos[2] - sim.area.bottom) /  sim.grid_step[2]))))

        if(id_x != new_id_x || id_y != new_id_y)
            deleteat!(sim.grid[id_x, id_y], findfirst(==(robot), sim.grid[id_x, id_y]))
            push!(sim.grid[min(sim.num_grid, max(1, new_id_x)), min(sim.num_grid, max(1, new_id_y))], robot)
        end
    end

    for i in eachindex(sim.grid)
        row, col = fldmod1(i, sim.num_grid)
        if(length(sim.grid[row, col]) >= 1)
            robs = deepcopy(sim.grid[row, col])

            for x in -1:1
                for y in -1:1
                    append!(robs, sim.grid[min(sim.num_grid, max(1, row + x)), min(sim.num_grid, max(1, col + y))])
                end
            end
            unique!(robs)
            if(length(robs) > 1)
                for robot in sim.grid[row, col]
                    if (typeof(robot) == Robot)
                        id_x = min(sim.num_grid, max(1, Int64(ceil((robot.pos[1] - sim.area.left) /  sim.grid_step[1]))))
                        id_y = min(sim.num_grid, max(1, Int64(ceil((robot.pos[2] - sim.area.bottom) /  sim.grid_step[2]))))

                        move_intersection!(robot, robs)

                        new_id_x = min(sim.num_grid, max(1, Int64(ceil((robot.pos[1] - sim.area.left) /  sim.grid_step[1]))))
                        new_id_y = min(sim.num_grid, max(1, Int64(ceil((robot.pos[2] - sim.area.bottom) /  sim.grid_step[2]))))

                        if(id_x != new_id_x || id_y != new_id_y)
                            deleteat!(sim.grid[id_x, id_y], findfirst(==(robot), sim.grid[id_x, id_y]))
                            push!(sim.grid[min(sim.num_grid, max(1, new_id_x)), min(sim.num_grid, max(1, new_id_y))], robot)
                        end
                    end
                end
            end
        end
    end
end


function get_neighbors(sim::Simulation, robot::Robot, cells::Int64)
    id_x = min(sim.num_grid, max(1, Int64(ceil((robot.pos[1] - sim.area.left) /  sim.grid_step[1]))))
    id_y = min(sim.num_grid, max(1, Int64(ceil((robot.pos[2] - sim.area.bottom) /  sim.grid_step[2]))))
    robs = []

    for x in -cells:cells
        for y in -cells:cells
            if(!sim.open_area && ((id_x + x) <= 1 || (id_x + x)  >= sim.num_grid || (id_y + y )<= 1 || (id_y + y)  >= sim.num_grid))
                push!(robs, sim.area)
            end
            append!(robs, sim.grid[min(sim.num_grid, max(1, id_x + x)), min(sim.num_grid, max(1, id_y + y))])
        end
    end
    unique!(robs)

    return robs
end

function get_neighbors(sim::Simulation, pos_x, pos_y, cells::Int64)
    id_x = min(sim.num_grid, max(1, Int64(ceil((pos_x - sim.area.left) /  sim.grid_step[1]))))
    id_y = min(sim.num_grid, max(1, Int64(ceil((pos_y - sim.area.bottom) /  sim.grid_step[2]))))
    robs = []

    for x in -cells:cells
        for y in -cells:cells
            if(!sim.open_area && ((id_x + x) <= 1 || (id_x + x)  >= sim.num_grid || (id_y + y )<= 1 || (id_y + y)  >= sim.num_grid))
                push!(robs, sim.area)
            end
            append!(robs, sim.grid[min(sim.num_grid, max(1, id_x + x)), min(sim.num_grid, max(1, id_y + y))])
        end
    end
    unique!(robs)

    return robs
end




function robot_Shape(robot::Robot, pos::Array{Float64}, radius::Float32; showSensor::Bool=false)
    ang = range(0, 2π, length = 25)
    front = range(-pi/4 + pos[3], pi/4 + pos[3], length = 5)

    shapes = [Shape(radius * cos.(ang) .+ pos[1], radius * sin.(ang) .+ pos[2]), 
            Shape(radius * cos.(front) .+ pos[1], radius * sin.(front) .+ pos[2])]

    if(showSensor)
        for sensor in robot.sensor_pos
            sensor_deg = range(pos[3] + sensor[3] - robot.sensor_deg/2, 
                            pos[3] + sensor[3] + robot.sensor_deg/2, length = 5)
            
            sensor_x = pos[1] .+ sensor[1] * cos(pos[3]) .- sensor[2] * sin(pos[3])
            sensor_y = pos[2] .+ sensor[1] * sin(pos[3]) .+ sensor[2] * cos(pos[3])

            x = robot.sensor_dist * cos.(sensor_deg) .+ sensor_x
            y = robot.sensor_dist * sin.(sensor_deg) .+ sensor_y

            append!(x, [sensor_x])
            append!(y, [sensor_y])

            append!(shapes, [Shape(radius/10 * cos.(ang) .+ sensor_x, radius/10 * sin.(ang) .+ sensor_y),
            Shape(x, y)
            ])
        end
    end

    return shapes
end


"""
plot_hist(sim::Simulation; speedup:Float64) -> None

Plot the movements of all robots over the entire timespan.

# Arguments
- `sim::Simulation`: Simulation to plot the simulation from
- `speedup::Float64`: Defining speedup for GIF
- `showSensor::Bool`: Flag, indicating if the sensor range should be displayed

# Keywords
- `speedup:Float64=1.0`: Speed up or Slow down framerate by percentage
"""
function plot_hist(sim::Simulation; speedup::Float64=1.0, showSensor::Bool=false, showGrid::Bool=false, showNeighbors::Bool=false)
    if(length(sim.robots) == 0)
        return
    end

    x = x_axis(area)
    y = y_axis(area)
    w = 500
    h = ratio(area) * 500
    
    min_length = minimum([size(robot.history)[2] for robot in sim.robots])
    anim = @animate for i=1:min_length
        plot(robot_Shape(sim.robots[1], Array(sim.robots[1].history[:, i]), sim.robots[1].radius; showSensor=showSensor), 
            xlim = x,
            ylim = y,
            color = sim.robots[1].color,
            legend = false,
            size = (w, h))

        if (length(sim.robots) > 1)
            for j in 2:length(sim.robots)
                plot!(robot_Shape(sim.robots[j], Array(sim.robots[j].history[:, i]), sim.robots[j].radius; showSensor=showSensor), color=sim.robots[j].color)
            end
        end

        if (showNeighbors)
            obstacles = get_neighbors(sim, sim.robots[1].history[:, i][1], sim.robots[1].history[:, i][2], 1)
        else
            obstacles = sim.area.obstacles
        end

        for obstacle in obstacles
            if (typeof(obstacle) == Round_Obstacle)
                plot!(obstacle)
            end
        end

        if showGrid
            for i in 1:sim.num_grid-1
                plot!([sim.area.left, sim.area.right], fill(sim.area.bottom + i * sim.grid_step[1], 2), color="#cccccc")
                plot!(fill(sim.area.left + i * sim.grid_step[2], 2), [sim.area.bottom, sim.area.top], color="#cccccc")
            end
        end

        
    end
    gif(anim, fps=speedup/sim.time_step)
end