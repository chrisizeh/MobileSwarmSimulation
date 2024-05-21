import Plots.plot!

abstract type Obstacle end

"""
   Round Obstacle

    Obstacle with center point and radius.
"""
struct Round_Obstacle <: Obstacle
    center::Array{Float64}
    radius::Float64
end


"""
   Rectangle Obstacle

    Obstacle with center point and width and height.
"""
struct Rectangle_Obstacle <: Obstacle
    center::Array{Float64}
    width::Float64
    height::Float64
end


"""
   Border

   Structure for storing the borders of an area in a two dimensional euclidean space in.
"""
struct Border
    left::Int64
    right::Int64
    bottom::Int64
    top::Int64

    obstacles::Array{Obstacle}

    function Border(left, right, bottom, top; obstacles=[]) 
        if (left >= right || bottom >= top)
            error("incorrect border")
        else
            new(left, right, bottom, top, obstacles)
        end
    end
end


"""
ratio(border::Border) -> Float64

Calculates the ratio of width to height of the area.

# Arguments
- `border::Border`: Border to calculate the ration of

# Returns
- `Float64`: Ratio
"""
function ratio(border::Border)
    return (border.top - border.bottom)/(border.right - border.left)
end

"""
x_axis(border::Border) -> Tuple{Int64, Int64}

Returns the borders on the x-axis as Tuple

# Arguments
- `border::Border`: Border to return the values from
"""
function x_axis(border::Border)
    return (border.left, border.right)
end

"""
y_axis(border::Border) -> Tuple{Int64, Int64}

Returns the borders on the y-axis as Tuple

# Arguments
- `border::Border`: Border to return the values from
"""
function y_axis(border::Border)
    return (border.bottom, border.top)
end


function Circle(center::Array{Float64}, radius::Float64)
    ang = range(0, 2π, length = 25)
    return Shape(radius * cos.(ang) .+ center[1], radius * sin.(ang) .+ center[2])
end


function Rectangle(center::Array{Float64}, width::Float64, height::Float64)
    return Shape(center[1] .+ [-width/2, width/2, width/2, -width/2], center[2] .+ [-height/2, -height/2, height/2, height/2])
end


"""
plot!(obstacle::Obstacle) -> None

Plotting obstacle according to its shape

# Arguments
- `obstacle::Obstacle`: Obstacle to plot
"""
function plot!(obstacle::Round_Obstacle)
    plot!(Circle(obstacle.center, obstacle.radius), color=1)
end


function plot!(obstacle::Rectangle_Obstacle)
    plot!(Rectangle(obstacle.center, obstacle.width, obstacle.height), color=1)
end