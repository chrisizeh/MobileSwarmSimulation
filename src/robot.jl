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
	pos::Array{Float64}      # m [x, y]
	deg::Float32        # degree from x-axis
	vel::Array{Float32}   # m/s [left, right]

	function Robot(id; radius=1, pos=[0, 0], deg=0, vel=[0,0]) 
		history = ElasticArray{Float64}(undef, 3, 0)

		new(id, radius, history, pos, deg, vel)
	end
end

Base.show(io::IO, rob::Robot) = print(io, "{id: ", rob.id, " pos: ", rob.pos, "}")

function update_speed!(robot::Robot, left::Float64, right::Float64)
	robot.vel = [left, right]
end

function update_speed!(robot::Robot, left::Int64, right::Int64)
	robot.vel = [left * 1.0, right * 1.0]
end

function stop(robot::Robot)
	robot.update_speed(0, 0)
end


# Paper for calculation: https://rossum.sourceforge.net/papers/DiffSteer/
function move!(robot::Robot, sec::Float64, checkBorder::Bool, border::Border)
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

	if checkBorder
		new_pos = check_border(robot, border, new_pos)
	end

	robot.pos = new_pos
	robot.deg = new_deg
end


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


function move_intersection!(robot::Robot, robots::Array{Robot}=[])
    for other in robots
        if(robot.id != other.id)
			dist = sqrt(abs(other.pos[1] - robot.pos[1])^2 + abs(other.pos[2] - robot.pos[2])^2)

			if(dist < robot.radius + other.radius)
				fix_intersection(robot, other)
			end
        end
    end
end


function fix_intersection(robot::Robot, other_robot::Robot)
	x = other_robot.pos[1] - robot.pos[1]
	y = other_robot.pos[2] - robot.pos[2]

	dist = sqrt(abs(other_robot.pos[1] - robot.pos[1])^2 + abs(other_robot.pos[2] - robot.pos[2])^2)
	r = other_robot.radius + robot.radius

	if (y >= 0)
		deg = acos(x / dist)
	else
		deg = -acos(x / dist)
	end

	robot.pos[1] = robot.pos[1] - cos(deg) * (r - dist)
	robot.pos[2] = robot.pos[2] - sin(deg) * (r - dist)
end