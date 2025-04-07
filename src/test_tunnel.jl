include("include_me.jl")

obstacles = []
push!(obstacles, Rectangle_Obstacle([250.0, 500.0], 50.0, 1000.0, color="#f19066"))
push!(obstacles, Rectangle_Obstacle([550.0, 500.0], 50.0, 1000.0, color="#f19066"))
push!(obstacles, Rectangle_Obstacle([700.0, 1000.0], 300.0, 100.0, color="#f19066"))
push!(obstacles, Rectangle_Obstacle([100.0, 1000.0], 200.0, 100.0, color="#f19066"))

goal = Round_Obstacle([50.0, 1200.0], 10.0, color="#2C68A7")

area = Area(0, 1000, 0, 1300; obstacles=obstacles, goal=[goal])

vMax = 10.0
vMin = 5.0
robots = []
flocking_robots = []
tracking_robots = []

# Start from 300 -> 500

lines = 12
per_line = 15
tracking_start = 8
num_robots = per_line*lines
for line in 2:lines+1
    for i in 1:per_line
        if(line < tracking_start)
            robot = Robot(i; radius=5, 
                color="#31E0CB",
                vel=[0, 0],
                pos=[330 + 10*(per_line-1) * (line % 2) + (-1)^line * 10 * (i - 1), 150 - 10 * (line - 2)], 
                deg = pi/2,
                sensor_dist=20.0, sensor_num=6, spec=pi)
        else
            robot = Robot(i; radius=3, 
                color="#E08631",
                vel=[0, 0],
                pos=[330 + 10*(per_line-1) * (line % 2) + (-1)^line * 10 * (i - 1), 150 - 10 * (line - 2)], 
                deg = pi/2,
                sensor_dist=60.0, sensor_num=8, spec=2*pi, sensor_deg=pi/2)
        end

        

        if(line == tracking_start-2 && i == 5)
            push!(tracking_robots, robot)
        end

        if(line < tracking_start)
            push!(flocking_robots, robot)            
        else
            push!(tracking_robots, robot)
        end
        push!(robots, robot)
    end
end

sim = Simulation(robots, area; open_area=false, num_grid=5, time_step=0.2)
track!(sim, flocking_robots, tracking_robots; steps=500)
plot_hist(sim; speedup=5.0, showSensor=false, showGrid=true, title="Flocking and Tracking in Open Area")