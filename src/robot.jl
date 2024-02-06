include("area.jl")

mutable struct Robot
    id::Int32
    width::Float16      # m
    length::Float16     # m

    pos_x::Float32      # m
    pos_y::Float32      # m
    deg::Float16        # degree from y-axis
    vel_left::Float16   # m/s
    vel_right::Float16  # m/s

    Robot(id, width, length) = new(id, width, length, 0, 0, 0, 0, 0)
end

function update_speed!(robot::Robot, left::Float16, right::Float16)
    robot.vel_left = left
    robot.vel_right = right
end

function update_speed!(robot::Robot, left::Int64, right::Int64)
    robot.vel_left = left * 1.
    robot.vel_right = right * 1.
end

function stop(robot::Robot)
    robot.update_speed(0, 0)
end

# Paper for calculation: https://rossum.sourceforge.net/papers/DiffSteer/
function move!(robot::Robot, sec::Int64, border::Border)
    vel = (robot.vel_left + robot.vel_right) / 2

    if robot.vel_right != robot.vel_left
        diff = robot.vel_left - robot.vel_right
        new_deg = (diff * sec / robot.width) + robot.deg

        println("degree $(new_deg * 180/pi)")

        robot.pos_x += robot.width * vel / diff * (sin(new_deg) - sin(robot.deg) )
        robot.pos_y += robot.width * vel / diff * (cos(new_deg) - cos(robot.deg) )
        robot.deg = new_deg
    else
        robot.pos_x += vel * sin(robot.deg) * sec
        robot.pos_y += vel * cos(robot.deg) * sec
    end

    check_border!(robot, border)
    
end

function check_border!(robot::Robot, border::Border)
    if (robot.pos_y >= border.top)
        robot.pos_y = border.top - 1
    elseif (robot.pos_y <= border.bottom)
        robot.pos_y = border.bottom + 1
    end

    if (robot.pos_x <= border.left)
        robot.pos_x = (border.left + 1)
    elseif (robot.pos_x >= border.right)
        robot.pos_x = (border.right - 1)
    end
end