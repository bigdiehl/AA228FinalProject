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

using QMDP
using POMDPModelTools # for ordered_actions
using LinearAlgebra
using JLD2, FileIO


# %% -----------------------------------------------------------------------
#Define our sensor
sensor = Bumper()

#Room configuration. Choose from 1,2, or 3
config = 1

#Discretize state space

num_x_pts = 50
num_y_pts = 50
num_th_pts = 20
sspace = DiscreteRoombaStateSpace(num_x_pts,num_y_pts,num_th_pts)

#Discretize action space
vlist = collect(0:1.0:10.0)
omlist = collect(0:0.2:1.0)
aspace = vec(collect(RoombaAct(v, om) for v in vlist, om in omlist))

#Get rid of first action which is (0,0). Doesn't do us any good
#aspace = aspace(2:length(aspace))

#Construct the POMPDP for the QMDP solver (Discrete state/action space)
m_discrete = RoombaPOMDP(sensor=sensor, mdp=RoombaMDP(config=config, sspace = sspace,
                      aspace = aspace));

#Try increasing the rewards
m_discrete.mdp.goal_reward = 10
m_discrete.mdp.stairs_penalty = -10


#Construct the POMDP for the simulator (Continuous state/action space)
m = RoombaPOMDP(sensor=sensor, mdp=RoombaMDP(config=config));

# %% -----------------------------------------------------------------------
#Create the particle filter
num_particles = 1000 #2000
resampler = BumperResampler(num_particles)
spf = SimpleParticleFilter(m, resampler)

#Create our belief updater using our simple particle filter
v_noise_coefficient = 2.0
om_noise_coefficient = 0.5
belief_updater = RoombaParticleFilter(spf, v_noise_coefficient, om_noise_coefficient);

# %% -----------------------------------------------------------------------
#Define our solver
#solver = FIBSolver()
solver = QMDPSolver(max_iterations=20,
                    tolerance=1e-3,
                    verbose=true)


#If we need to compute our policy for the first time
if (1 == 0)
    #Use our solver and our POMDP model to find a policy
    policy = solve(solver,m_discrete)
    #Save our policy so we don't have to recompute
    using JLD2, FileIO
    @JLD2.save "my_policy.jld" policy


#Otherwise use the saved policy we computed previously
else
    @JLD2.load "my_policy2.jld" policy
end

# %% -----------------------------------------------------------------------

# Define the policy to test
mutable struct ToEnd <: Policy
    ts::Int64 # to track the current time-step.
    policy::AlphaVectorPolicy
end

# %% -----------------------------------------------------------------------
#flag for our wall hit policy
previousBumpState = false

states = POMDPs.states(m_discrete)

# define a new function that takes in the policy struct and current belief,
# and returns the desired action
function POMDPs.action(p::ToEnd, b::ParticleCollection{RoombaState})

    global previousBumpState

    #Drive straight into wall to localize belief state
    if previousBumpState == false
        #If the wall has been bumped, then all particles are on the wall. If so,
        #then any particle will do for determining wall contact
        s = particle(b,1)

        #Call the wall_contact function to determine if we are in wall contact
        #(returns true or false)
        currentBumpState = AA228FinalProject.wall_contact(m,s)

        if currentBumpState == false
            return RoombaAct(5.0, 0.0)
        else
            previousBumpState = true
        end
    end

    #Use alpha vectors once first wall contact is made
    if previousBumpState == true
        #Extract our policy from struct p
        policy = p.policy
        #Extract our set of alpha vectors from our policy (one for each action)
        alphas = policy.alphas

        greatestUtilityIndex = 6
        greatestUtility = -Inf

        numStates = POMDPs.n_states(m_discrete)
        numActions = length(alphas)


        #Create our belief vector from our particle filter
        belief = zeros(numStates)
        for i = 1:num_particles
            s = particle(b,i)
            index = POMDPs.stateindex(m_discrete,s)
            belief[index] += 1
        end
        belief = belief/num_particles

        #print(belief)
        #print("\n")

        # see which action gives the highest util value
        for i = 1:numActions
            bp1=POMDPs.update(belief_updater, b,RoombaAct(policy.action_map[i]),1)
            #bp0=POMDPs.update(belief_updater, b,RoombaAct(policy.action_map[i]),0)
            beliefnew1 = zeros(numStates)
            #beliefnew0 = zeros(numStates)
            for j = 1:num_particles
                s1 = particle(bp1,j)
                #s0 = particle(bp0,j)
                index1 = POMDPs.stateindex(m_discrete,s1)
                #index0 = POMDPs.stateindex(m_discrete,s0)
                beliefnew1[index1] += 1
                #beliefnew0[index0] += 1
            end
            beliefnew1 = beliefnew1/num_particles
            #beliefnew0 = beliefnew0/num_particles
            utility = dot(alphas[i], belief)#+obs_weight(m_discrete, RoombaAct(policy.action_map[i]), bp1, 1)*dot(alphas[i], beliefnew1)#+obs_weight(RoombaPOMDP, RoombaAct(policy.action_map[i]), 0)*dot(alphas[i], beliefnew0)
            if utility > greatestUtility
                greatestUtility = utility
                greatestUtilityIndex = i
                #print(greatestUtility)
                #print("\n")
                #print(greatestUtilityIndex)

            end
        end
        #print("\n\n\n")

        # map the index to action
        a = policy.action_map[greatestUtilityIndex]

        if greatestUtilityIndex == 1
            a = policy.action_map[6]
        end
        return RoombaAct(a[1], a[2])
    end

end


#%%--------------------------------------------------------------------------

#%%--------------------------------------------------------------------------

#%%**************************************************************************
#----------------------SIMULATION-------------------------------------------
#****************************************************************************

# first seed the environment
Random.seed!()

# reset the policy
p = ToEnd(0,policy) # here, the argument sets the time-steps elapsed to 0

#reset previousBumpState
previousBumpState = false

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
    sleep(0.01) # to slow down the simulation
end

#%%**************************************************************************
#----------------------TESTING-------------------------------------------
#****************************************************************************

if (1==0)
    using Statistics

    total_rewards = []

    for exp = 1:50
        println(string(exp))

        Random.seed!(exp)

        p = ToEnd(0,policy)
        traj_rewards = sum([step.r for step in stepthrough(m,p,belief_updater, max_steps=100)])

        push!(total_rewards, traj_rewards)
    end

    @printf("Mean Total Reward: %.3f, StdErr Total Reward: %.3f", mean(total_rewards), std(total_rewards)/sqrt(5))
end
