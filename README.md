## Reinforcement Learning - PID auto-tuning

#Auto tuning of PID parameters of a quad-rotor using Q-learning

**Project abstract:**
 - Auto-tuning of PID paramters using Q-learning is a project that was an attempt in controlling a quadrotor by tuning the 
PID paramters using a reinforcement learning technique. 
 - The quadrotor modelling, system dynamics and control theory to be used and implemented were self designed for our 
requirements in the project. 
 - Q-learning is the reinforcement tool used in tuning the PID parameters of the quadrotor. The controlling involved position
controland attitudinal control using PID tuning specific to each instance of controlling variable over the entire system
state inputs. 
 - Prior to the implementation of the reinforcement learning for tuning the parameters, the PID paramterswere manually tuned
and the desired trajectory was achieved. The system model and dynamics were accordinglyvalidated for capability and performance
of the quadrotor under specified system input states. 
 - In addition to the implementation of the RL technique, the performance of the RL was validated by setting disturbances to
the system in-course of the travel from the origin to the designated goal state. 
 - Effectively, the implementation witnessed thesuccessful traversing of the quadrotor to the designated target in the desired
trajectory even when the system was perturbed with disturbances enroute.
