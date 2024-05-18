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
            new_pos_x, new_pos_y, new_deg = check_intersection(robot, other, robot.pos_x, robot.pos_y, Float32(robot.deg))     
			robot.pos_x = new_pos_x
			robot.pos_y = new_pos_y
			robot.deg = new_deg
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

function corner_points(x::Float64, y::Float64, deg::Float32, radius::Float32)
	corners_x = [radius / 2, (-1) * radius / 2]
	corners_y = [radius / 2, (-1) * radius / 2]
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

	@info points
	return points
end

function get_intersections(points, other_points)
    ks = []
    ds = []

    points_1 = []
    points_2 = []
    for i in 1:4
        j = i < 4 ? i + 1 : 1

        k = (other_points[i][2] - other_points[j][2]) / (other_points[i][1] - other_points[j][1])
        d = other_points[i][2] - k * other_points[i][1]
        push!(ks, k)
        push!(ds, d)
        
        push!(points_1, points[1][2] - (k * points[1][1] + d))
        push!(points_2, points[2][2] - (k * points[2][1] + d))
    end

    s_points_1 = [sign(i) for i in points_1]
    s_points_2 = [sign(i) for i in points_2]

    a = (points[1][2] - points[2][2]) / (points[1][1] - points[2][1])
    b = points[2][2] - a * points[2][1]

    if (s_points_1 == [1, -1, -1, 1] || s_points_1 == [-1, 1, 1, -1] || s_points_1 == [-1, -1, 1, 1] || s_points_1 == [1, 1, -1, -1])
            @info "point 1 inside"

            # @info [points[1][1] * a + b for i in 1:4]
            # @info points[2][2]
            # res = [(ks[i] * points[2][1] + ds[i] - b)/a for i in 1:4]
            # # @info res
            # min_dist = [sqrt((points[1][1] - res[i])^2 +  (points[1][2] - ks[i] * points[1][1] + ds[i])^2) for i in 1:4]
            # min_i = argmin(min_dist)
            # mini = [res[min_i], ks[min_i] * points[2][1] + ds[min_i]]
            # @info "mindist $(min_dist)"
            # length = sqrt((mini[1] - points[1][1])^2 + (mini[2] - points[1][2])^2)
            # @info "length $(length)"
            # @info "deg $(atan(length/3) / pi * 180)"
            # return -atan(length/3)
    else
        @info "point 1 outside"
    end

    if (s_points_2 == [1, -1, -1, 1] || s_points_2 == [-1, 1, 1, -1] || s_points_2 == [-1, -1, 1, 1] || s_points_2 == [1, 1, -1, -1])
        @info "point 2 inside"

        # x = [(ks[i] * points[2][1] + ds[i] - b)/a for i in 1:4]
        # y = [(ks[i] * points[2][1] + ds[i]) for i in 1:4]

        
        # plt = scatter(x, y)
        # x = [points[i][1] for i in 1:4]
        # y = [points[i][2] for i in 1:4]
        # plot!(plt, x, y)

        # plot!(plt, 1:10, [a * i + b for i in 1:10])

        # display(plt)

        # res = [(ks[i] * points[2][1] + ds[i] - b)/a for i in 1:4]
        # min_dist = [sqrt((points[2][1] - res[i])^2 +  (points[2][2] - ks[i] * points[2][1] + ds[i])^2) for i in 1:4]
        # min_i = argmin(min_dist)
        # mini = [res[min_i], ks[min_i] * points[2][1] + ds[min_i]]
        # @info "mindist $(min_dist)"
        # length = sqrt((mini[1] - points[2][1])^2 + (mini[2] - points[2][2])^2)
        # @info "length $(length)"
        # @info "deg $(atan(length/3) / pi * 180)"
        # return atan(length/3)
    else
        @info "point 2 outside"
    end
    
	return 0
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


function check_intersection(robot::Robot, other_robot::Robot, new_pos_x::Float64, new_pos_y::Float64, new_deg::Float32)
	points_rob = corner_points(new_pos_x, new_pos_y, new_deg, robot.radius)
	points_other_rob = corner_points(other_robot.pos_x, other_robot.pos_y, other_robot.deg, other_robot.radius)
	sides = get_intersections(points_rob, points_other_rob)
    new_deg += sides

	return new_pos_x, new_pos_y, new_deg
end