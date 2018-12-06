# activate project environment
# include these lines of code in any future scripts/notebooks
import Pkg
using Random

if !haskey(Pkg.installed(), "AA228FinalProject")
    jenv = joinpath(dirname(@__FILE__()), ".") # this assumes the notebook is in the same dir
    # as the Project.toml file, which should be in top level dir of the project.
    # Change accordingly if this is not the case.
    Pkg.activate(jenv)
end

# import necessary packages
using AA228FinalProject
using POMDPs
using POMDPPolicies
using BeliefUpdaters
using ParticleFilters
using POMDPSimulators
using Cairo
using Gtk
using Random
using Printf

# %% -----------------------------------------------------------------------
sensor = Bumper() # or Bumper() for the bumper version of the environment
config = 1 # 1,2, or 3
m = RoombaPOMDP(sensor=sensor, mdp=RoombaMDP(config=config));

# %% -----------------------------------------------------------------------
num_particles = 10000
resampler = BumperResampler(num_particles)
# for the bumper environments
# resampler = BumperResampler(num_particles)

spf = SimpleParticleFilter(m, resampler)

v_noise_coefficient = 2.0
om_noise_coefficient = 0.5

belief_updater = RoombaParticleFilter(spf, v_noise_coefficient, om_noise_coefficient);

# %% -----------------------------------------------------------------------
# Define the policy to test
mutable struct ToEnd <: Policy
    ts::Int64 # to track the current time-step.
end

# extract goal for heuristic controller
goal_xy = get_goal_xy(m)

#flag for our wall hit policy
previousBumpState = false
spinStep =

# define a new function that takes in the policy struct and current belief,
# and returns the desired action
function POMDPs.action(p::ToEnd, b::ParticleCollection{RoombaState})

    #Seed the environment
    Random.seed!()

    #Naive approach: Bump the wall, spin in a random direction, and then drive
    #again.
    global velSteps
    global previousBumpState
    global spinSteps
    global L

    #Fixed Velocity and spin rate maximums
    velMax = m.mdp.v_max
    omegaMax = m.mdp.om_max

    #Normal driving speed
    vel = 2

    #Set max and min number of time-steps to spin
    maxSpinCount = 7
    minSpinCount = 3

    #Increase time step
    p.ts += 1

    #If the wall has been bumped, then all particles are on the wall. If so,
    #then any particle will do for determining wall contact
    s = particle(b,1)

    #Call the wall_contact function to determine if we are in wall contact
    #(returns true or false)
    currentBumpState = AA228FinalProject.wall_contact(m,s)
    initialBumpState = AA228FinalProject.wall_contact(m,s)
    OmegaNinetyDegrees=0.7853981634
    if (p.ts<2 && currentBumpState==true)
        L=1
        velSteps=0
    end
    if (p.ts<2 && currentBumpState==false)
        L=2
        velSteps=0
    end
    if (p.ts>400)
        L=3
    end
    if (p.ts>420)
        L=5
    end
    if (L==1)
        if (currentBumpState==true)
            if (previousBumpState==false)
                spinSteps=4
                previousBumpState=currentBumpState
            elseif (previousBumpState=true)
                if (spinSteps!=0)
                    spinSteps-=1
                    p.ts+=1
                    return RoombaAct(0.0,OmegaNinetyDegrees)
                else
                    previousBumpState=false
                    p.ts+=1
                    L=2
                    return RoombaAct(vel,0.0)
                end
            end
        end
    elseif (L==2)
        while (currentBumpState==false)
            velSteps+=1
            previousBumpState=currentBumpState
            p.ts+=1
            return RoombaAct(vel,0.0)
        end
        while (currentBumpState==true)
            while (previousBumpState==false)
                spinSteps=7
                previousBumpState=currentBumpState
                p.ts+=1
                return RoombaAct(0.0,OmegaNinetyDegrees)
            end
            while (previousBumpState==true)
                if (spinSteps!=0)
                    spinSteps-=1
                    p.ts+=1
                    return RoombaAct(0.0,OmegaNinetyDegrees)
                else
                    velSteps-=1
                    p.ts+=1
                    L=4
                    return RoombaAct(vel,0.0)
                end
            end
        end
    elseif (L==4)
        if (velSteps>0)
            velSteps-=1
            p.ts+=1
            spinSteps=4
            return RoombaAct(vel,0.0)
        else
            if (spinSteps>0)
                spinSteps-=1
                p.ts+=1
                return RoombaAct(0.0,OmegaNinetyDegrees)
            else
                L=2
                velSteps+=1
                p.ts+=1
                return RoombaAct(vel,0.0)
            end
        end
    elseif (L==3)
        s = mean(b)
        goal_x, goal_y = goal_xy
        x,y,th = s[1:3]
        ang_to_goal = atan(goal_y - y, goal_x - x)
        del_angle = wrap_to_pi(ang_to_goal - th)
        Kprop = 1.0
        om = Kprop * del_angle
        v = 5.0
        p.ts+=1
        return RoombaAct(0.0, om)
    elseif (L==5)
        s = mean(b)
        goal_x, goal_y = goal_xy
        x,y,th = s[1:3]
        ang_to_goal = atan(goal_y - y, goal_x - x)
        del_angle = wrap_to_pi(ang_to_goal - th)
        Kprop = 1.0
        om = Kprop * del_angle
        v = 5.0
        p.ts+=1
        return RoombaAct(v,om)
    end
end
#tpharris
# %% ----------------------------------------------stanf-------------------------
# first seed the environment
Random.seed!()

# reset the policy
p = ToEnd(0) # here, the argument sets the time-steps elapsed to 0

#RUN THE SIMULATION
c = @GtkCanvas()
win = GtkWindow(c, "Roomba Environment", 600, 600)
for (t, step) in enumerate(stepthrough(m, p, belief_updater, max_steps=500))
    @guarded draw(c) do widget

        # the following lines render the room, the particles, and the roomba
        ctx = getgc(c)
        set_source_rgb(ctx,1,1,1)
        paint(ctx)
        render(ctx, m, step)

        # render some information that can help with debugging
        # here, we render the time-step, the state, and the observation
        move_to(ctx,300,400)
        show_text(ctx, @sprintf("t=%d, state=%s, o=%.3f",t,string(step.s),step.o))
    end
    show(c)
    sleep(0.1) # to slow down the simulation
end

# %% -----------------------------------------------------------------------
using Statistics

total_rewards = []

for exp = 1:1
    println(string(exp))

    Random.seed!(exp)

    p = ToEnd(0)
    traj_rewards = sum([step.r for step in stepthrough(m,p,belief_updater, max_steps=100)])

    push!(total_rewards, traj_rewards)
end

@printf("Mean Total Reward: %.3f, StdErr Total Reward: %.3f", mean(total_rewards), std(total_rewards)/sqrt(5))
