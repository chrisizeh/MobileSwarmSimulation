include("include_me.jl")

obstacles = []
push!(obstacles, Rectangle_Obstacle([250.0, 500.0], 50.0, 1000.0, color="#465361"))
push!(obstacles, Rectangle_Obstacle([550.0, 500.0], 50.0, 1000.0, color="#465361"))
push!(obstacles, Rectangle_Obstacle([125.0, 1000.0], 300.0, 100.0, color="#465361"))
push!(obstacles, Rectangle_Obstacle([675.0, 1000.0], 300.0, 100.0, color="#465361"))

goal = Round_Obstacle([50.0, 1200.0], 10.0, color="#2C68A7")

area = Area(0, 800, 0, 1300; obstacles=obstacles, goal=[goal])

vFlocking = 10.0
vTracking = 15.0

for times in 1:10
    println(times)

    robots = []
    flocking_robots = []
    tracking_robots = []

    # Start from 300 -> 500

    lines = 12
    per_line = 8
    between_robots = 25
    tracking_start = 8
    num_robots = per_line * lines

    for line in 2:lines+1
        for i in 1:per_line
            if (line < tracking_start)
                robot = Robot(i; radius=5,
                    color="#31E0CB",
                    vel=[0, 0],
                    pos=[315 + between_robots * (per_line - 1) * (line % 2) + (-1)^line * between_robots * (i - 1), 10 + lines * between_robots - between_robots * (line - 2)],
                    deg=pi / 2,
                    sensor_dist=30.0, sensor_num=6, spec=pi)
            else
                robot = Robot(i; radius=3,
                    color="#E08631",
                    vel=[0, 0],
                    pos=[315 + between_robots * (per_line - 1) * (line % 2) + (-1)^line * between_robots * (i - 1), 10 + lines * between_robots - between_robots * (line - 2)],
                    deg=pi / 2,
                    sensor_dist=60.0, sensor_num=8, spec=2 * pi, sensor_deg=pi / 2)
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
    track!(sim, flocking_robots, tracking_robots; steps=1000, v_flocking=vFlocking, v_tracking=vTracking, dist_flocking=15, dist_obstacle=15)
    plot_hist(sim; speedup=5.0, showSensor=false, showGrid=true, title="Flocking and Tracking in Confined Area", name="tunnel_$(times).gif")
end