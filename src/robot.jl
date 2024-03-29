include("area.jl")

mutable struct Robot
    #=
    Structure for storing all necessary variable for a robot.

    :id: unique value to identify robot
    :width: the width of the robot in meter
    :height: the height of the robot in meter
    :history: all previous positions since instantiation
    :pos_x: current center point of robot on x-axis
    :pos_y: current center point of robot on y-axis
    :deg: current degree of robot in rad, x-axis equals 0 degrees.
    :vel_left: current velocity of left tire, measured in meter per seconds.
    :vel_right: current velocity of right tire, measured in meter per seconds.
    =#

	id::Int32
	width::Float32      # m
	length::Float32     # m

	history::Array{Tuple{Float64, Float64, Float32}}
	pos_x::Float64      # m
	pos_y::Float64      # m
	deg::Float32        # degree from y-axis
	vel_left::Float32   # m/s
	vel_right::Float32  # m/s

	Robot(id, width, length) = new(id, width, length, [], 0, 0, pi / 2, 0, 0)
	Robot(id, width, length, pos_x, pos_y) = new(id, width, length, [], pos_x, pos_y, pi / 2, 0, 0)
	Robot(id, width, length, pos_x, pos_y, deg) = new(id, width, length, [], pos_x, pos_y, deg + pi / 2, 0, 0)
end

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

# Paper for calculation: https://rossum.sourceforge.net/papers/DiffSteer/
function move!(robot::Robot, sec::Float64, border::Border)
	vel = (robot.vel_left + robot.vel_right) / 2

	if robot.vel_right != robot.vel_left
		diff = robot.vel_right - robot.vel_left
		new_deg = (diff * sec / robot.width) + robot.deg

		new_pos_x = robot.pos_x + robot.width * vel / diff * (sin(new_deg) - sin(robot.deg))
		new_pos_y = robot.pos_y - robot.width * vel / diff * (cos(new_deg) - cos(robot.deg))
	else
		new_pos_x = robot.pos_x + vel * cos(robot.deg) * sec
		new_pos_y = robot.pos_y + vel * sin(robot.deg) * sec
		new_deg = robot.deg
	end

	new_pos_x, new_pos_y, new_deg = check_border!(robot, border, new_pos_x, new_pos_y, new_deg)
	robot.pos_x = new_pos_x
	robot.pos_y = new_pos_y
	robot.deg = new_deg

	push!(robot.history, (robot.pos_x, robot.pos_y, robot.deg))

end

function corner_points(x::Float64, y::Float64, deg::Float32, width::Float32, length::Float32)
	corners_x = [length / 2, (-1) * length / 2]
	corners_y = [width / 2, (-1) * width / 2]
	points = []

	trans_matrix = [cos(deg), -sin(deg), sin(deg), cos(deg)]

	# Left Front
	push!(points, (x + trans_matrix[1] * corners_x[1] + trans_matrix[2] * corners_y[1],
		y + trans_matrix[3] * corners_x[1] + trans_matrix[4] * corners_y[1]))

	# Right Front
	push!(points, (x + trans_matrix[1] * corners_x[1] + trans_matrix[2] * corners_y[2],
		y + trans_matrix[3] * corners_x[1] + trans_matrix[4] * corners_y[2]))

	# Right Back
	push!(points, (x + trans_matrix[1] * corners_x[2] + trans_matrix[2] * corners_y[2],
		y + trans_matrix[3] * corners_x[2] + trans_matrix[4] * corners_y[2]))

	# Left Back
	push!(points, (x + trans_matrix[1] * corners_x[2] + trans_matrix[2] * corners_y[1],
		y + trans_matrix[3] * corners_x[2] + trans_matrix[4] * corners_y[1]))

	return points
end

function robot_corner_points(robot::Robot)
	return corner_points(robot.pos_x, robot.pos_y, robot.deg, robot.width, robot.length)
end

function get_intersections(border::Border, points)
	sides = []

	if (points[1][1] >= border.right || points[2][1] >= border.right)
		diff_1 = maximum([0, points[1][1] - border.right])
		diff_2 = maximum([0, points[2][1] - border.right])
	elseif (points[1][1] <= border.left || points[2][1] <= border.left)
		diff_1 = minimum([0, points[1][1] - border.left])
		diff_2 = minimum([0, points[2][1] - border.left])
	else
		diff_1 = 0
		diff_2 = 0
	end
	push!(sides, [diff_1, diff_2])

	if (points[1][2] >= border.top || points[2][2] >= border.top)
		diff_1 = maximum([0, points[1][2] - border.top])
		diff_2 = maximum([0, points[2][2] - border.top])
	elseif (points[1][2] <= border.bottom || points[2][2] <= border.bottom)
		diff_1 = minimum([0, points[1][2] - border.bottom])
		diff_2 = minimum([0, points[2][2] - border.bottom])
	else
		diff_1 = 0
		diff_2 = 0
	end
	push!(sides, [diff_1, diff_2])

	return sides
end

function check_border!(robot::Robot, border::Border, new_pos_x::Float64, new_pos_y::Float64, new_deg::Float32)
	points = corner_points(new_pos_x, new_pos_y, new_deg, robot.width, robot.length)
	sides = get_intersections(border, points)

	for i in 1:2
		if (sides[i][1] != 0 && sides[i][2] != 0)
			new_pos_y -= (i - 1) * (sides[i][1] + sides[i][2]) / 2
			new_pos_x -= (2 - i) * (sides[i][1] + sides[i][2]) / 2
			new_deg = sign(sides[i][1]) * pi / 2 + -(pi / 2) * (2 - i)
			break
		end

		new_pos_y -= (sides[i][1] + sides[i][2]) / 2
		new_pos_x += sides[i][1] / 2 - sides[i][2] / 2
		new_deg += sign(sides[i][1]) * atan(sides[i][1] / abs(points[1][3-i] - points[2][3-i])) -sign(sides[i][2]) * atan(sides[i][2] / abs(points[1][3-i] - points[2][3-i]))
	end

	return new_pos_x, new_pos_y, new_deg
end
