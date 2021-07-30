# Examples

This folder contains videos of policy performing in the test simulations for tasks
asc-inc-cog, asc-uni-cog, des-inc-ang and des-uni-ang.  
Additionally, configurations files used to produce these policies are provided.

## Description of configuration files 

Let's look at every parameter of the "asc_inc_cog.json" file. 

`"alg":"SAC"`: stands for the employed algorithm where we use SAC. 
You can add something your own here through modification of the file `stables3_launch.py` (or your own).

`"angular":false`: we define if we want to control angular velocity of the robot or not.  

`"arm":true`: do we use arm or not?    

`"complexity":"full"` a future point for the extension for the incremental robot control, currently ignored.

`"env_type":"vect"`: `rand` or `vect`, it defines whether the env. is incremental or uniform.  

`"experiment":"asc_inc_cog"`: unique experiment name, it will be a collection in the database.  

`"experiment_series":"exp_paper"`: name of experiment sequence, it has to start with "exp_".   

`"load_path":"data/models/server/nothing"`: a path to a loaded policy, if you do not load anything, 
then end this with "nothing"  

`"log_path":"data/boards/server"`: where stable-baselines3 tensorboards are stored.  

`"model_parameters":{"ppo_cliprange":"0.2","ppo_ent_coef":"0.005","sac_ent_coef":"auto_0.5","sac_tau":0.05,"sac_train_freq":1}`:
a dictionary of parameters for loaded algorithms, the key starts with the algorithm.  

`"penalty_angular":false`: it indicates if we apply the angular velocity based penalty.  

`"penalty_deviation":true`: it indicates if we apply the center of gravity deviation based penalty.

`"policy":"default"`:  information about the employed policy.  

`"rand":false`: if the goal and robot spawn randomly.

`"save_path":"data/models/server"`: a path where a trained policy will be saved.

`"sigma":0`: state noising coefficient, currently ignored.  

`"task":"ascent"`: `flat`, `ascent` or `descent`, it indicates the env. type.  

`"time_step_limit":50`: the number of time steps per episode.  

`"total_timesteps":20000`:  total number of time steps.  


