include("include_me.jl")

obstacles = []
# push!(obstacles, Rectangle_Obstacle([225.0, 175.0], 450.0, 350.0, color="#465361"))
# push!(obstacles, Rectangle_Obstacle([975.0, 175.0], 550.0, 350.0, color="#465361"))

push!(obstacles, Round_Obstacle([225.0, 175.0], 225.0, color="#465361"))
# push!(obstacles, Round_Obstacle([975.0, 175.0], 275.0, color="#465361"))
push!(obstacles, Round_Obstacle([1500.0, 300.0], 500.0, color="#465361"))
push!(obstacles, Round_Obstacle([750.0, 720.0], 100.0, color="#465361"))
push!(obstacles, Round_Obstacle([1300.0, 1300.0], 300.0, color="#465361"))
push!(obstacles, Round_Obstacle([350.0, 900.0], 75.0, color="#465361"))
push!(obstacles, Round_Obstacle([1.0, 500.0], 230.0, color="#465361"))

goal = Round_Obstacle([100.0, 1200.0], 50.0, color="#2C68A7")

area = Area(0, 1300, 0, 1300; obstacles=obstacles, goal=[goal])

vFlocking = 20.0
vTracking = 25.0

for times in 1:10
    robots = []
    flocking_robots = []
    tracking_robots = []

    lines = 7
    per_line = 12
    between_robots = 20
    tracking_start = 6
    num_robots = per_line * lines

    for line in 2:lines+1
        for i in 1:per_line
            if(line < tracking_start)
                robot = Robot(i; radius=5, 
                    color="#31E0CB",
                    vel=[0, 0],
                    pos=[500 + between_robots*(per_line-1) * (line % 2) + (-1)^line * between_robots * (i - 1), 10 + lines * between_robots - between_robots * (line - 2)], 
                    deg = pi/2,
                    sensor_dist=70.0, sensor_num=6, spec=pi)
            else    
                robot = Robot(i; radius=3, 
                    color="#E08631",
                    vel=[0, 0],
                    pos=[500 + between_robots*(per_line-1) * (line % 2) + (-1)^line * between_robots * (i - 1), 10 + lines * between_robots - between_robots * (line - 2)], 
                    deg = pi/2,
                    sensor_dist=200.0, sensor_num=8, spec=2*pi, sensor_deg=pi/2)
            end

            if (line < tracking_start)
                push!(flocking_robots, robot)
            else
                push!(tracking_robots, robot)
            end
            push!(robots, robot)
        end
    end

    sim = Simulation(robots, area; open_area=false, num_grid=5, time_step=0.1)
    track!(sim, flocking_robots, tracking_robots; steps=1000, v_flocking=vFlocking, v_tracking=vTracking, dist_flocking=40, dist_obstacle=25)
    plot_hist(sim; speedup=5.0, showSensor=false, showGrid=true, title="Flocking and Tracking in Open Area", name="open_$(times).gif")
end