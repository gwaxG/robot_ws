# Backend package

#### Database
This package contains `database_app` which is a node written in Go that subscribes to the topic `/rollout/analytics` and puts coming data to the database (MongoDB).
It looks at the field "exp_series" of the message which indicates a MongoDB database and appends this message to a collection indicated in the field "experiment".

#### Backend
The application `master_app` provides API for experiment management and works with the GUI application.
It can be launched with the flag `-psize=x` where `x` is the number of simulations.  

You can solely launch a simulation with included database_app.

#### Important:
* Experiment series names should always start with "exp"
* `master_app` may contain bugs, feel free to open an issue.  It was tested for basic functionalities with psize=1.
* Deleteing of a working environment can take a while, keep patience and do not abuse the "delete" button, it won't work.
* The robot Absolem is supported only for manual simulation start due to unstable launching of Gazebo with this model.

#### Launching
##### Automated
1. `roscd backend`
2. `./bin/master_app`
3. `./bin/database_app -psize=1`
##### Manual
1. Launching of simulation, the argument `robot` accepts either `jaguar` (default) or `absolem`. 
   `roslaunch backend learning.launch robot:=jaguar`
3. `roscd backend/scripts/learning_scripts`
4. Modify your configuration file (for example stables3_launch.json)
5. Launch the corresponding python scrpit, for exampel `python stables3_launch.py`

#### Description of configuration files 

To have an idea about possible values of the configuration file.  
Let's look at every parameter of [an example file](https://github.com/gwaxG/robot_ws/blob/main/examples/asc-inc-cog.json).  

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

#### TODO
1. Cover `master_app` with unit tests.

