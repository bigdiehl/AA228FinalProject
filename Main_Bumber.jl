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

# %% -----------------------------------------------------------------------
#Define our sensor
sensor = Bumper()

#Room configuration. Choose from 1,2, or 3
config = 3

#Discretize state space
num_x_pts = 50
num_y_pts = 50
num_th_pts = 20
sspace = DiscreteRoombaStateSpace(num_x_pts,num_y_pts,num_th_pts)

#Discretize action space
vlist = collect(0:1.0:10.0)
omlist = collect(0:0.2:1.0)
aspace = vec(collect(RoombaAct(v, om) for v in vlist, om in omlist))

#Construct the POMPDP for the QMDP solver (Discrete state/action space)
m_discrete = RoombaPOMDP(sensor=sensor, mdp=RoombaMDP(config=config, sspace = sspace,
                      aspace = aspace));

#Construct the POMDP for the simulator (Continuous state/action space)
m = RoombaPOMDP(sensor=sensor, mdp=RoombaMDP(config=config));

# %% -----------------------------------------------------------------------
#Create the particle filter
num_particles = 2000 #2000
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
    using JLD
    save("my_policy.jld", "policy", policy)
#Otherwise use the saved policy we computed previously
else
    policy = load("my_policy.jld","policy")
end

# %% -----------------------------------------------------------------------

# Define the policy to test
mutable struct ToEnd <: Policy
    ts::Int64 # to track the current time-step.
    policy::AlphaVectorPolicy
end

# %% -----------------------------------------------------------------------
states = POMDPs.states(m_discrete)

# define a new function that takes in the policy struct and current belief,
# and returns the desired action
function POMDPs.action(p::ToEnd, b::ParticleCollection{RoombaState})

    #Extract our policy from struct p
    policy = p.policy
    #Extract our set of alpha vectors from our policy (one for each action)
    alphas = policy.alphas

    ihi = 0
    vhi = -Inf

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

    # see which action gives the highest util value
    for ai = 1:numActions
        util = dot(alphas[ai], belief)
        if util > vhi
            vhi = util
            ihi = ai
        end
    end

    # map the index to action
    a = policy.action_map[ihi]

    return RoombaAct(a[1], a[2])

end

#%%**************************************************************************
#----------------------SIMULATION-------------------------------------------
#****************************************************************************

# first seed the environment
Random.seed!(5)

# reset the policy
p = ToEnd(0,policy) # here, the argument sets the time-steps elapsed to 0

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

#%%**************************************************************************
#----------------------TESTING-------------------------------------------
#****************************************************************************

if (1==0)
    using Statistics

    total_rewards = []

    for exp = 1:5
        println(string(exp))

        Random.seed!(exp)

        p = ToEnd(0)
        traj_rewards = sum([step.r for step in stepthrough(m,p,belief_updater, max_steps=100)])

        push!(total_rewards, traj_rewards)
    end

    @printf("Mean Total Reward: %.3f, StdErr Total Reward: %.3f", mean(total_rewards), std(total_rewards)/sqrt(5))
end
