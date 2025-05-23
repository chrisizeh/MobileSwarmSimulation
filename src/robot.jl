include("area.jl")

using ElasticArrays

"""
Constructor:
	Calculate position relative to center and store in list
	calculate degree range

Function get_sensoric_data:
	for all sensors
		get robot to point 
		if dist < max dist
			calc radius
			if in range of sensor
				value is dist
			else
				value is sensoric_dist
"""


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
	color::String

	history::ElasticArray{Float64}	#ElasticArray, can expand on last dim
	pos::Array{Float64}      # m [x, y]
	deg::Float32        # degree from x-axis
	vel::Array{Float32}   # m/s [left, right]

	sensor_dist::Float32	# m
	sensor_deg::Float64
	sensor_num::Int64

	sensor_pos::Array{Vector{Float64}}
	NO_DIST_FOUND::Float32

	function Robot(id; radius=1, pos=[0, 0], deg=0, vel=[0,0], color="#1abc9c", sensor_dist=3.0, sensor_deg=pi/4, sensor_num=3, spec=pi) 
		history = ElasticArray{Float64}(undef, 3, 0)

		sensor_pos = []
		dist = spec / (sensor_num + 1)
		diff = pi/2 - spec/2
		for i in 1:sensor_num
			push!(sensor_pos, [radius * sin(diff + i * dist), radius * cos(diff + i * dist), spec/2 - i * dist])
		end
		NO_DIST_FOUND = sensor_num * sensor_dist

		new(id, radius, color, history, pos, deg, vel, sensor_dist, sensor_deg, sensor_num, sensor_pos, NO_DIST_FOUND)
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
move!(robot::Robot, sec::Float64, checkBorder::Bool, border::Area) -> None

Move the robot for a specified amount of time. If checkBorder is False, the robot can exit the area.
	The movement with different velocities for both tires is calculated using the paper
	https://rossum.sourceforge.net/papers/DiffSteer/.

# Arguments
- `robot::Robot`: Robot to update the value from
- `sec::Float64`: Timeframe to move the robot
- `checkBorder::Bool`: Flag, indicating if the border can be crossed
- `border::Area`: Border of movement area
"""
function move!(robot::Robot, sec::Float64; checkBorder::Bool=false, border::Area=nothing)
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
	robot.deg = transform_rad(new_deg)
end


"""
check_border(robot::Robot, border::Area, new_pos::Array{Float64}) -> Array{Float64}

To prevent crossing the border, intersections are solved by repositioning the robot
	in the corresponding axis directly on the border, pos = border - robot radius.

# Arguments
- `robot::Robot`: Robot to check for border intersections
- `border::Area`: Border of area to check for intersection
- `new_pos::Array{Float64}`: Current position of robot after movement update

# Returns
- `Array{Float64}`: New position of robot after fixing intersections
"""
function check_border(robot::Robot, border::Area, new_pos::Array{Float64})
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
	If the distance is less than the radii combined, robot is moved.

# Arguments
- `robot::Robot`: Robot to move to prevent intersection
- `robots::Array{Robot}`: List of robots to check for intersection
"""
function move_intersection!(robot::Robot, robots::Array{Union{Robot, Obstacle}}=[])
    for other in robots
        robot.pos = check_intersection(robot, other)
    end
end


"""
check_intersection(robot::Robot, other_robot::Robot) -> Array{Float64}

To solve the intersection, the robot is moved away from the other robot on the vector
	combining both center points.

# Arguments
- `robot::Robot`: Robot to move to prevent intersection
- `other_robot::Robot`: Other Robot, which is part of the intersection

# Returns
- `Array{Float64}`: New position of robot after fixing intersection
"""
function check_intersection(robot::Robot, other_robot::Robot)
	dist = sqrt(abs(other_robot.pos[1] - robot.pos[1])^2 + abs(other_robot.pos[2] - robot.pos[2])^2)
	r = other_robot.radius + robot.radius

	if((robot.id != other_robot.id) && (dist < r))
		x = other_robot.pos[1] - robot.pos[1]
		y = other_robot.pos[2] - robot.pos[2]

		if (y >= 0)
			deg = acos(x / dist)
		else
			deg = -acos(x / dist)
		end

		# Needed for weird glitch where the robots end on top of each other
		if (isnan(deg))
			deg = pi
		end

		new_pos = [0., 0.]
		new_pos[1] = robot.pos[1] - cos(deg) * (r - dist)
		new_pos[2] = robot.pos[2] - sin(deg) * (r - dist)

		return new_pos
	end

	return robot.pos
end


"""
check_intersection(robot::Robot, obstacle::Obstacle) -> Array{Float64}

Check if the robots intersect.
To solve the intersection, the robot is moved away from the other robot on the vector
	combining both center points.

# Arguments
- `robot::Robot`: Robot to move to prevent intersection
- `obstacle::Obstacle`: Obstacle, which is part of the intersection

# Returns
- `Array{Float64}`: New position of robot after fixing intersection
"""
function check_intersection(robot::Robot, obstacle::Obstacle)
	vec = intersect(obstacle, robot.pos, robot.radius)

	if (isnan(vec[1]) || isnan(vec[2]))
		println("vec nan")
	end

	if(abs(vec[1]) > 0 || abs(vec[2]) > 0)
		new_pos = [0., 0.]
		new_pos[1] = robot.pos[1] - vec[1]
		new_pos[2] = robot.pos[2] - vec[2]

		return new_pos
	end

	return robot.pos
end


function transform_rad(angle)
    p1 = (angle .+ sign.(angle) .* pi) .% (2 * pi)
    p2 = (sign.(sign.(angle) .+ 2 .* (sign.(abs.(((angle .+ pi) .% (2 * pi)) ./ (2 * pi))) .- 1))) .* pi
    return p1 .- p2
end


"""
get_sensoric_data(robot::Robot, other_robot::Robot) -> Array{Float64}

Calculate for another robot if it's in the range of a sensors.
If yes, return the distance to the sensor.
If not, return 0 for this sensor.

# Arguments
- `robot::Robot`: Robot to get the sensor data from
- `other_robot::Robot`: Other Robot, which is to be checked

# Returns
- `Array{Float64}`: Array of distance for each sensor
"""
function get_sensoric_data(robot::Robot, other_robot::Robot)
	sensor_data = Array{Float64}(undef, robot.sensor_num)
	fill!(sensor_data, robot.sensor_dist);

	if(robot.id == other_robot.id)
		return sensor_data
	end

	for i in 1:robot.sensor_num
		sensor = robot.sensor_pos[i]

		sensor_x = robot.pos[1] .+ sensor[1] * cos(robot.deg) .- sensor[2] * sin(robot.deg)
		sensor_y = robot.pos[2] .+ sensor[1] * sin(robot.deg) .+ sensor[2] * cos(robot.deg)

		x = other_robot.pos[1] - sensor_x
		y = other_robot.pos[2] - sensor_y
		dist = sqrt(x^2 + y^2)

		if (dist < robot.sensor_dist + other_robot.radius)

			if (y >= 0)
				deg = acos(x / dist)
			else
				deg = -acos(x / dist)
			end

			to_border = asin(min(1., other_robot.radius/dist))
			diff = deg - (robot.deg + sensor[3])

			opts = [diff, diff - to_border, diff + to_border]
			opts = transform_rad.(opts)

			if(minimum(abs.(opts)) < robot.sensor_deg/2)
				sensor_data[i] = dist - other_robot.radius
			end

		end
	end
	return sensor_data
end


"""
get_sensoric_data(robot::Robot, obstacle::Obstacle) -> Array{Float64}

Calculate for another robot if it's in the range of a sensors.
If yes, return the distance to the sensor.
If not, return 0 for this sensor.

TODO: Improve rectangular obstacle by including border checking.

# Arguments
- `robot::Robot`: Robot to get the sensor data from
- `obstacle::Obstacle`: Obstacle, which is to be checked

# Returns
- `Array{Float64}`: Array of distance for each sensor
"""
function get_sensoric_data(robot::Robot, obstacle::Obstacle)
	sensor_data = Array{Float64}(undef, robot.sensor_num)
	fill!(sensor_data, robot.sensor_dist);

	for i in 1:robot.sensor_num
		sensor = robot.sensor_pos[i]

		sensor_x = robot.pos[1] .+ sensor[1] * cos(robot.deg) .- sensor[2] * sin(robot.deg)
		sensor_y = robot.pos[2] .+ sensor[1] * sin(robot.deg) .+ sensor[2] * cos(robot.deg)

		vec = intersect(obstacle, [sensor_x, sensor_y], robot.sensor_dist)
		if (sum(abs.(vec)) > 0)
			x = obstacle.center[1] - sensor_x
			y = obstacle.center[2] - sensor_y
			dist = sqrt(x^2 + y^2)

			deg = atan(y, x)
			opts = degree_to_border(obstacle, [sensor_x, sensor_y])
			push!(opts, deg)
			push!(opts, atan(vec[2], vec[1]))

			opts = transform_rad.(opts .- transform_rad.(robot.deg + sensor[3]))
			if(minimum(abs.(opts)) < robot.sensor_deg/2)
				sensor_data[i] = robot.sensor_dist - sqrt(vec[1]^2 + vec[2]^2)
			end
		end
	end
	return sensor_data
end


# TODO: Improve
function get_sensoric_data(robot::Robot, border::Area)
	sensor_data = Array{Float64}(undef, robot.sensor_num)
	fill!(sensor_data, robot.sensor_dist);
	pos = [0., 0.]

	for i in 1:robot.sensor_num
		sensor = robot.sensor_pos[i]

		sensor_x = robot.pos[1] .+ sensor[1] * cos(robot.deg) .- sensor[2] * sin(robot.deg)
		sensor_y = robot.pos[2] .+ sensor[1] * sin(robot.deg) .+ sensor[2] * cos(robot.deg)
		deg = 0
		dist = 200

		if (sensor_x + robot.sensor_dist > border.right)
			if(abs(robot.deg + sensor[3]) < (robot.sensor_deg/2) * robot.sensor_dist / (border.right - sensor_x))
				sensor_data[i] = border.right - sensor_x
			end
		end
		if (sensor_x - robot.sensor_dist < border.left)
			if(abs(pi - robot.deg - sensor[3]) < (robot.sensor_deg/2) * robot.sensor_dist / (sensor_x - border.left))
				dist = sensor_x - border.left

				if(dist < sensor_data[i])
					sensor_data[i] = dist
				end
			end
		end
		if (sensor_y + robot.sensor_dist > border.top)
			if(abs(pi/2 - robot.deg - sensor[3]) < (robot.sensor_deg/2) * robot.sensor_dist / (border.top - sensor_y))
				dist = border.top - sensor_y

				if(dist < sensor_data[i])
					sensor_data[i] = dist
				end
			end
		end
		if (sensor_y - robot.sensor_dist < border.bottom)
			if(abs(3*pi/2 - robot.deg - sensor[3]) < (robot.sensor_deg/2) * robot.sensor_dist / (sensor_y - border.bottom))
				dist = sensor_y - border.bottom

				if(dist < sensor_data[i])
					sensor_data[i] = dist
				end
			end
		end
	end
	return sensor_data
end