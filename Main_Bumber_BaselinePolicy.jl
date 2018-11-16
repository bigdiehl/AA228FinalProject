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
config = 3 # 1,2, or 3
m = RoombaPOMDP(sensor=sensor, mdp=RoombaMDP(config=config));

# %% -----------------------------------------------------------------------
num_particles = 2000
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
    global previousBumpState
    global spinSteps

    #Fixed Velocity and spin rate maximums
    velMax = m.mdp.v_max
    omegaMax = m.mdp.om_max

    #Normal driving speed
    vel = 1.0

    #Set max and min number of time-steps to spin
    maxSpinCount = 8
    minSpinCount = 2

    #Increase time step
    p.ts += 1

    #If the wall has been bumped, then all particles are on the wall. If so,
    #then any particle will do for determining wall contact
    s = particle(b,1)

    #Call the wall_contact function to determine if we are in wall contact
    #(returns true or false)
    currentBumpState = AA228FinalProject.wall_contact(m,s)

    #The bump sensor tells us we are in contact with the wall
    if (currentBumpState == true)
        #Our memory variable tells us we weren't in contact the previous timestep
        if (previousBumpState == false)
            #Set a random number of time steps to spin
            spinSteps = floor(minSpinCount+rand()*(maxSpinCount-minSpinCount))
            #Update the previous state
            previousBumpState = currentBumpState
            #Return our trajectory
            return RoombaAct(0.0, omegaMax)
        #Our memory variable tells us we were in contact with the wall in the
        #last time-step as well
        elseif (previousBumpState == true)
            #If we are still spinning
            if (spinSteps != 0)
                #decrement spinSteps
                spinSteps -= 1
                print(spinSteps)
                print("\n")
                #Return our trajectory
                return RoombaAct(0.0, omegaMax)
            #Otherwise we are ready to test to see if we can drive forward
            else
                #Assume that we won't aren't pointed into a wall
                previousBumpState = false
                #Attempt to drive forward. If we can't them the logic flow will
                #reset the spinSteps variable and the Roomba will spin a random
                #number of time-steps again
                return RoombaAct(vel, 0.0)
            end
        end

    #If we aren't in contact with the wall
    elseif (currentBumpState == false)
        #Update previous state
        previousBumpState = currentBumpState
        #Drive forward at a fixed velocity
        return RoombaAct(vel, 0.0)
    end

end

# %% -----------------------------------------------------------------------
# first seed the environment
Random.seed!(2)

# reset the policy
p = ToEnd(0) # here, the argument sets the time-steps elapsed to 0

#RUN THE SIMULATION
c = @GtkCanvas()
win = GtkWindow(c, "Roomba Environment", 600, 600)
for (t, step) in enumerate(stepthrough(m, p, belief_updater, max_steps=100))
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
"""using Statistics

total_rewards = []

for exp = 1:5
    println(string(exp))

    Random.seed!(exp)

    p = ToEnd(0)
    traj_rewards = sum([step.r for step in stepthrough(m,p,belief_updater, max_steps=100)])

    push!(total_rewards, traj_rewards)
end

@printf("Mean Total Reward: %.3f, StdErr Total Reward: %.3f", mean(total_rewards), std(total_rewards)/sqrt(5))
"""
