include("area.jl")

using ElasticArrays

mutable struct Robot
    #=
    Structure for storing all necessary variable for a round robot.

    :id: unique value to identify robot
    :radius: the radius of the robot in meter
    :history: all previous positions since instantiation
    :pos_x: current center point of robot on x-axis
    :pos_y: current center point of robot on y-axis
    :deg: current degree of robot in rad, x-axis equals 0 degrees.
    :vel_left: current velocity of left tire, measured in meter per seconds.
    :vel_right: current velocity of right tire, measured in meter per seconds.
    =#

	id::Int32
	radius::Float32      # m

	history::ElasticArray{Float64}	#ElasticArray, can expand on last dim
	pos_x::Float64      # m
	pos_y::Float64      # m
	deg::Float32        # degree from y-axis
	vel_left::Float32   # m/s
	vel_right::Float32  # m/s

	function Robot(id, radius=5, pos_x=0, pos_y=0, deg=0, vel_left=0, vel_right=0) 
		history = ElasticArray{Float64}(undef, 3, 0)

		new(id, radius, history, pos_x, pos_y, deg, vel_left, vel_right)
	end
end

Base.show(io::IO, rob::Robot) = print(io, "{id: ", rob.id, " x: ", rob.pos_x, " y:", rob.pos_y, "}")

function update_speed!(robot::Robot, left::Float64, right::Float64)
	robot.vel_left = left
	robot.vel_right = right
end

function update_speed!(robot::Robot, left::Int64, right::Int64)
	robot.vel_left = left * 1.0
	robot.vel_right = right * 1.0
end

function stop(robot::Robot)
	robot.update_speed(0, 0)
end

function move_intersection!(robot::Robot, robots::Array{Robot}=[])
    for other in robots
        if(robot.id != other.id)
			dist = sqrt(abs(other.pos_x - robot.pos_x)^2 + abs(other.pos_y - robot.pos_y)^2)

			if(dist < robot.radius + other.radius)
				fix_intersection(robot, other)
			end
        end
    end
end


# Paper for calculation: https://rossum.sourceforge.net/papers/DiffSteer/
function move!(robot::Robot, sec::Float64, checkBorder::Bool, border::Border)
	append!(robot.history, [robot.pos_x, robot.pos_y, robot.deg])

	vel = (robot.vel_left + robot.vel_right) / 2

	if robot.vel_right != robot.vel_left
		diff = robot.vel_right - robot.vel_left
		new_deg = (diff * sec / robot.radius) + robot.deg

		new_pos_x = robot.pos_x + robot.radius * vel / diff * (sin(new_deg) - sin(robot.deg))
		new_pos_y = robot.pos_y - robot.radius * vel / diff * (cos(new_deg) - cos(robot.deg))
	else
		new_pos_x = robot.pos_x + vel * cos(robot.deg) * sec
		new_pos_y = robot.pos_y + vel * sin(robot.deg) * sec
		new_deg = robot.deg
	end

	if checkBorder
		new_pos_x, new_pos_y = check_border(robot, border, new_pos_x, new_pos_y)
	end

	robot.pos_x = new_pos_x
	robot.pos_y = new_pos_y
	robot.deg = new_deg
end


function check_border(robot::Robot, border::Border, new_pos_x::Float64, new_pos_y::Float64)
	if (new_pos_x + robot.radius > border.right)
		new_pos_x = border.right - robot.radius
	elseif (new_pos_x - robot.radius < border.left)
		new_pos_x = border.left + robot.radius
	end

	if (new_pos_y + robot.radius > border.top)
		new_pos_y = border.top - robot.radius
	elseif (new_pos_y - robot.radius < border.bottom)
		new_pos_y = border.bottom + robot.radius
	end

	return new_pos_x, new_pos_y
end


function fix_intersection(robot::Robot, other_robot::Robot)
	x = other_robot.pos_x - robot.pos_x
	y = other_robot.pos_y - robot.pos_y

	dist = sqrt(abs(other_robot.pos_x - robot.pos_x)^2 + abs(other_robot.pos_y - robot.pos_y)^2)
	r = other_robot.radius + robot.radius

	if (y >= 0)
		deg = acos(x / dist)
	else
		deg = -acos(x / dist)
	end

	robot.pos_x = robot.pos_x - cos(deg) * (r - dist)
	robot.pos_y = robot.pos_y - sin(deg) * (r - dist)
end