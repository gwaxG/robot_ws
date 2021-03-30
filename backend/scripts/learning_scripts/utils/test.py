import os
prms = {
    "experiment_series":"exp",
    "experiment": "1",
    "log_path": "data/boards",
    "save_path": "data/models",
}

postfix = prms["experiment_series"] + "_" + prms['experiment']
# paths
p = os.path.abspath(__file__)
print(p)
for i in range(5):
    p = os.path.split(p)[0]
log_path = os.path.join(p, prms["log_path"], postfix)
save_path = os.path.join(p, prms["log_path"], postfix)
print(log_path, save_path)