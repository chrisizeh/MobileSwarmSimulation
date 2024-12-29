import Plots.plot!
using Statistics

abstract type Obstacle end

"""
   Round Obstacle

    Obstacle with center point and radius.
"""
struct Round_Obstacle <: Obstacle
    center::Array{Float64}
    radius::Float64

    color::String

    function Round_Obstacle(center::Array{Float64}, radius::Float64; color::String="#FFC300")
        new(center, radius, color)
    end
end


"""
    Rectangle Obstacle

    Obstacle with center point and width and height.
"""
struct Rectangle_Obstacle <: Obstacle
    center::Array{Float64}
    width::Float64
    height::Float64

    color::String

    function Rectangle_Obstacle(center::Array{Float64}, width::Float64, height::Float64; color::String="#DAF7A6")
        new(center, width, height, color)
    end
end


"""
    Area

   Area with borders in a two dimensional euclidean space and none to multiple obstacles.
"""
struct Area

    # Border
    left::Int64
    right::Int64
    bottom::Int64
    top::Int64

    obstacles::Array{Obstacle}

    function Area(left, right, bottom, top; obstacles=[]) 
        if (left >= right || bottom >= top)
            error("incorrect border")
        else
            new(left, right, bottom, top, obstacles)
        end
    end
end


"""
ratio(area::Area) -> Float64

Calculates the ratio of width to height of the area.

# Arguments
- `area::Area`: Area to calculate the ration of

# Returns
- `Float64`: Ratio
"""
function ratio(area::Area)
    return (area.top - area.bottom)/(area.right - area.left)
end

"""
x_axis(area::Area) -> Tuple{Int64, Int64}

Returns the areas on the x-axis as Tuple

# Arguments
- `area::Area`: Area to return the values from
"""
function x_axis(area::Area)
    return (area.left, area.right)
end

"""
y_axis(area::Area) -> Tuple{Int64, Int64}

Returns the areas on the y-axis as Tuple

# Arguments
- `area::Area`: Area to return the values from
"""
function y_axis(area::Area)
    return (area.bottom, area.top)
end


"""
width(obstacle::Obstacle) -> Float64

Calculates the width of the obstacle

# Arguments
- `obstacle::Obstacle`: Obstacle to calculate the width of

# Returns
- `Float64`: width
"""
function width(obstacle::Round_Obstacle)
    return obstacle.radius * 2
end

function width(obstacle::Rectangle_Obstacle)
    return obstacle.width
end


"""
height(obstacle::Obstacle) -> Float64

Calculates the height of the obstacle

# Arguments
- `obstacle::Obstacle`: Obstacle to calculate the height of

# Returns
- `Float64`: height
"""
function height(obstacle::Round_Obstacle)
    return obstacle.radius * 2
end

function height(obstacle::Rectangle_Obstacle)
    return obstacle.height
end


"""
intersect(obstacle::Obstacle, center::Array{Float64}, radius::Float64) -> Array{Float64}

Calculate if a circle is intersecting with the obstacle.
    If yes, the intersecting vector is returned.
    If not, a zero vector is returned.

# Arguments
- `obstacle::Obstacle`: Obstacle to check for intersection
- `center::Array{Float64}`: Center point of circle
- `radius::Float32`: radius of circle

# Returns
- `Array{Float64}`: Vector with length and direction of intersection
"""
function intersect(obstacle::Round_Obstacle, center::Array{Float64}, radius::Float32)
    dist = sqrt(abs(obstacle.center[1] - center[1])^2 + abs(obstacle.center[2] - center[2])^2)
	r = obstacle.radius + radius

	if (dist < r)
        x = obstacle.center[1] - center[1]
		y = obstacle.center[2] - center[2]

        deg = acos(x / dist)
		if (y < 0)
			deg = -deg
		end

		return [cos(deg) * (r - dist), sin(deg) * (r - dist)]
    else
        return [0., 0.]
    end
end


function intersect(obstacle::Rectangle_Obstacle, center::Array{Float64}, radius::Float32)
    rw = obstacle.width/2 + radius
    rh = obstacle.height/2 + radius

    x = obstacle.center[1] - center[1]
    y = obstacle.center[2] - center[2]

    """
      edge case | border case
             ---|-------- 
    border case | obstacle
    """

    # Check if point in interesting area around obstacle
    if (abs(x) < rw && abs(y) < rh)

        # Check if point in border case
        if(abs(x) < obstacle.width/2 || abs(y) < obstacle.height/2)
            if ((rh - abs(y)) < (rw - abs(x)))
                return [0., sign(y) * (rh - abs(y))]
            else
                return [sign(x) * (rw - abs(x)), 0.]
            end     
        else
            edge = [obstacle.center[1] - sign(x) * obstacle.width/2, obstacle.center[2] - sign(y) * obstacle.height/2]
            dist = sqrt((edge[1] - center[1])^2 + (edge[2] - center[2])^2)

            # Check if point in edge case
            if (dist < radius)
                deg = acos((edge[1] - center[1]) / dist)
                if ((edge[2] - center[2]) < 0)
                    deg = -deg
                end
                return [cos(deg) * (radius - dist), sin(deg) * (radius - dist)]
            end
        end
    end
    return [0., 0.]
end


"""
degree_to_border(obstacle::Round_Obstacle, center::Array{Float64}) -> Array{Float64}

Calculate the degree to the border of the obstacle 90 degrees to the distance line.
For the circle obstacle it is the same degree (+-).
For the square obstacle the degree have to calculated differently (TODO).

# Arguments
- `obstacle::Obstacle`: Obstacle to check for intersection
- `center::Array{Float64}`: Center point of circle

# Returns
- `Array{Float64}`: Degree to border in both directions
"""
function degree_to_border(obstacle::Round_Obstacle, center::Array{Float64})
    dist = sqrt((obstacle.center[1] - center[1])^2 + (obstacle.center[2] - center[2])^2)

    to_border = asin(min(obstacle.radius/dist, 1.))
    return [to_border, -to_border]
end

# TODO: Implement and use for Sensoring!
function degree_to_border(obstacle::Rectangle_Obstacle, center::Array{Float64})
    dist = sqrt((obstacle.center[1] - center[1])^2 + (obstacle.center[2] - center[2])^2)
    deg = [0., 0., 0., 0.]

    for i in 1:2
        for j in 1:2
            x =   obstacle.center[1] + (-1)^i * obstacle.width/2 - center[1]
            y =   obstacle.center[2] + (-1)^j * obstacle.height/2 - center[2]   
            deg[(i-1)*i + j] = atan(y/x)
        end
    end
    return [pi, -pi]
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
    plot!(Circle(obstacle.center, obstacle.radius), color=obstacle.color)
end


function plot!(obstacle::Rectangle_Obstacle)
    plot!(Rectangle(obstacle.center, obstacle.width, obstacle.height), color=obstacle.color)
end