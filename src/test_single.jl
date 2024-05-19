include("include_me.jl")

robot = Robot(0; pos=[2, 3])
border = Border(-10, 10, -10, 10)
update_speed!(robot, 1, 1)

sim = Simulation([robot], border)

for i in 1:5
    move!(robot, 1.0; checkBorder=true, border=border)
end

update_speed!(robot, 0, 1)

for i in 1:3
    move!(robot, 1.0; checkBorder=true, border=border)
end

update_speed!(robot, 1, 1)

for i in 1:5
    move!(robot, 1.0; checkBorder=true, border=border)
end

update_speed!(robot, -0.5, 1.)

for i in 1:3
    move!(robot, 1.0; checkBorder=true, border=border)
end

update_speed!(robot, 1, 1)

for i in 1:5
    move!(robot, 1.0; checkBorder=true, border=border)
end

plot_hist(sim)

border = Border(-10, 10, -10, 10)

robot1 = Robot(0; radius=1, pos=[-8, -8], deg=-3*pi/8, vel=[1, 1])
robot2 = Robot(1; radius=1, pos=[-6, -6], deg=7*pi/8, vel=[1, 1])
robot3 = Robot(2; radius=1, pos=[-8, 8], deg=3*pi/8, vel=[1, 1])
robot4 = Robot(3; radius=1, pos=[-6, 6], deg=-7*pi/8, vel=[1, 1])
robot5 = Robot(4; radius=1, pos=[8, -8], deg=-5*pi/8, vel=[1, 1])
robot6 = Robot(5; radius=1, pos=[6, -6], deg=pi/8, vel=[1, 1])
robot7 = Robot(6; radius=1, pos=[8, 8], deg=-pi/8, vel=[1, 1])
robot8 = Robot(7; radius=1, pos=[6, 6], deg=5*pi/8, vel=[1, 1])

robots = [robot1, robot2, robot3, robot4, robot5, robot6, robot7, robot8]

sim = Simulation(robots, border; time_step=0.05)

for i in 1:100
    update!(sim)
end

plot_hist(sim, speedup=2)
