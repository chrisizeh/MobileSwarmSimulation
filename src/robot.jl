include("area.jl")

using ElasticArrays

"""
   Robot

   Structure for storing all necessary variable for a round robot.

# Fields
- `id::Int32`: unique value to identify robot
- `radius::Float32`: the radius of the robot in meter
- `history::ElasticArray{Float64}`: all previous positions since instantiation
- `pos::Array{Float64}`: current center point of robot
- `deg::Float32`: current degree of robot in rad, x-axis equals 0 degrees.
- `vel::Array{Float32}`: current velocity of left tire, measured in meter per seconds.
"""
mutable struct Robot

	id::Int32
	radius::Float32      # m

	history::ElasticArray{Float64}	#ElasticArray, can expand on last dim
	pos::Array{Float64}      # m [x, y]
	deg::Float32        # degree from x-axis
	vel::Array{Float32}   # m/s [left, right]

	function Robot(id; radius=1, pos=[0, 0], deg=0, vel=[0,0]) 
		history = ElasticArray{Float64}(undef, 3, 0)

		new(id, radius, history, pos, deg, vel)
	end
end

Base.show(io::IO, rob::Robot) = print(io, "{id: ", rob.id, " pos: ", rob.pos, "}")


"""
update_speed!(robot::Robot, left::Float64, right::Float64) -> None

Update the Speed of the robot

# Arguments
- `robot::Robot`: Robot to update the value from
- `left::Float64`: Speed to set for the left tire in m/s
- `right::Float64`: Speed to set for the right tire in m/s
"""
function update_speed!(robot::Robot, left::Float64, right::Float64)
	robot.vel = [left, right]
end

function update_speed!(robot::Robot, left::Int64, right::Int64)
	robot.vel = [left * 1.0, right * 1.0]
end


"""
stop(robot::Robot) -> None

Reset the speed of both tires to 0.

# Arguments
- `robot::Robot`: Robot to update the value from
"""
function stop(robot::Robot)
	robot.update_speed(0, 0)
end


"""
move!(robot::Robot, sec::Float64, checkBorder::Bool, border::Border) -> None

Move the robot for a specified amount of time. If checkBorder is False, the robot can exit the area.
	The movement with different velocities for both tires is calculated using the paper
	https://rossum.sourceforge.net/papers/DiffSteer/.

# Arguments
- `robot::Robot`: Robot to update the value from
- `sec::Float64`: Timeframe to move the robot
- `checkBorder::Bool`: Flag, indicating if the border can be crossed
- `border::Border`: Border of movement area
"""
function move!(robot::Robot, sec::Float64; checkBorder::Bool=false, border::Border=nothing)
	append!(robot.history, [robot.pos[1], robot.pos[2], robot.deg])

	mean_vel = sum(robot.vel) / 2

	new_deg = 0.
	new_pos = [0., 0.]

	if robot.vel[1] != robot.vel[2]
		diff = robot.vel[2] - robot.vel[1]
		new_deg = (diff * sec / robot.radius) + robot.deg

		new_pos[1] = robot.pos[1] + robot.radius * mean_vel / diff * (sin(new_deg) - sin(robot.deg))
		new_pos[2] = robot.pos[2] - robot.radius * mean_vel / diff * (cos(new_deg) - cos(robot.deg))
	else
		new_pos[1] = robot.pos[1] + mean_vel * cos(robot.deg) * sec
		new_pos[2] = robot.pos[2] + mean_vel * sin(robot.deg) * sec
		new_deg = robot.deg
	end

	if checkBorder & !isnothing(border)
		new_pos = check_border(robot, border, new_pos)
	end

	robot.pos = new_pos
	robot.deg = new_deg
end


"""
check_border(robot::Robot, border::Border, new_pos::Array{Float64}) -> Array{Float64}

To prevent crossing the border, intersections are solved by repositioning the robot
	in the corresponding axis directly on the border, pos = border - robot radius.

# Arguments
- `robot::Robot`: Robot to check for border intersections
- `border::Border`: Border to check for intersection
- `new_pos::Array{Float64}`: Current position of robot after movement update

# Returns
- `Array{Float64}`: New position of robot after fixing intersections
"""
function check_border(robot::Robot, border::Border, new_pos::Array{Float64})
	if (new_pos[1] + robot.radius > border.right)
		new_pos[1] = border.right - robot.radius
	elseif (new_pos[1] - robot.radius < border.left)
		new_pos[1] = border.left + robot.radius
	end

	if (new_pos[2] + robot.radius > border.top)
		new_pos[2] = border.top - robot.radius
	elseif (new_pos[2] - robot.radius < border.bottom)
		new_pos[2] = border.bottom + robot.radius
	end

	return new_pos
end


"""
move_intersection!(robot::Robot, robots::Array{Robot}=[]) -> None

Robots cannot move on top of each other. For each robot, the distant to each other is checked.
	If the distance iis less than the radii combined, robot is moved.

# Arguments
- `robot::Robot`: Robot to move to prevent intersection
- `robots::Array{Robot}`: List of robots to check for intersection
"""
function move_intersection!(robot::Robot, robots::Array{Robot}=[])
    for other in robots
        if(robot.id != other.id)
			dist = sqrt(abs(other.pos[1] - robot.pos[1])^2 + abs(other.pos[2] - robot.pos[2])^2)

			if(dist < robot.radius + other.radius)
				robot.pos = check_intersection(robot, other)
			end
        end
    end
end


"""
check_intersection(robot::Robot, other_robot::Robot) -> None

To solve the intersection, the robot is moved away from the other robot on the vector
	combining both center points.

# Arguments
- `robot::Robot`: Robot to move to prevent intersection
- `other_robot::Robot`: Other Robot, which is part of the intersection

# Returns
- `Array{Float64}`: New position of robot after fixing intersection
"""
function check_intersection(robot::Robot, other_robot::Robot)
	x = other_robot.pos[1] - robot.pos[1]
	y = other_robot.pos[2] - robot.pos[2]

	dist = sqrt(abs(other_robot.pos[1] - robot.pos[1])^2 + abs(other_robot.pos[2] - robot.pos[2])^2)
	r = other_robot.radius + robot.radius

	if (y >= 0)
		deg = acos(x / dist)
	else
		deg = -acos(x / dist)
	end

	new_pos = [0., 0.]
	new_pos[1] = robot.pos[1] - cos(deg) * (r - dist)
	new_pos[2] = robot.pos[2] - sin(deg) * (r - dist)

	return new_pos
end