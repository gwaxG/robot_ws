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

#### Launch
##### Automated
1. `roscd backend`
2. `./bin/master_app`
3. `./bin/database_app -psize=1`
##### Manual
1. `roslaunch backend learning.launch`
2. `roscd backend/scripts/learning_scripts`
3. Modify your configuration file (for example stables3_launch.json)
4. Launch the corresponding python scrpit, for exampel `python stables3_launch.py`

#### TODO
1. Cover `master_app` with unit tests.

