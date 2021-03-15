import requests as rq
import termcolor as clr
import json, time, sys

def test_confg():
    # check /configs
    r = rq.get('http://localhost:10000/configs')
    want = {"configs":[{"alg":"PPO2sb1","angular":True,"arm":True,"experiment":"test_exp","log_path":"/home/r1d1/dev/catkin_ws/src/robot_ws/backend/data/boards","model_parameters":{"cliprange":"0.2","ent_coef":"0.005"},"policy_type":"CustomMLPsb1","rand":False,"save_path":"/home/r1d1/dev/catkin_ws/src/robot_ws/backend/data/models","sigma":0,"task":"ascent","time_step_limit":50,"total_timesteps":10000}],"launch_files":["/home/r1d1/dev/catkin_ws/src/robot_ws/backend/scripts/learning_scripts/stables1_launch.py"],"msg":""}
    if want["configs"] in r.json()["configs"]:
        print(clr.colored("configs", "red"))
    else:
        print(clr.colored("configs", "green"))


def test_create_task():
    # check create task
    oks = []
    for i in range(5):
        data = {
            "config": {"alg": "Pepe"+str(i), "time_steps": i},
            "launch_file": "kek",
            "msg": "",
        }
        r = rq.get('http://localhost:10000/task/create', data=json.dumps(data, indent=4))
        oks.append(r.ok)

    if all(oks):
        print(clr.colored("crete task", "green"))
    else:
        print(clr.colored("create task", "red"))


def test_read_task():
    oks = []
    for i in range(3):
        r = rq.get('http://localhost:10000/task/read')
        oks.append(r.ok)
    if all(oks):
        print(clr.colored("update task", "green"))
    else:
        print(clr.colored("update task", "red"))


def test_queue():
    r = rq.get('http://localhost:10000/queue')
    if r.json()["queue"] is not None:
        print("Q", [alg["alg"] for alg in r.json()["queue"]])


def test_pool():
    r = rq.get('http://localhost:10000/pool')
    if r.json()["pool"] is not None:
        print("P",[alg["alg"] for alg in r.json()["pool"]])


def call_pool_queue_case():
    test_create_task()
    test_queue()
    test_pool()
    print("******")
    time.sleep(0.95)
    test_queue()
    test_pool()
    print("******")
    time.sleep(0.95)
    test_queue()
    test_pool()
    print("******")
    time.sleep(1)
    test_queue()
    test_pool()
    print("******")
    time.sleep(0.95)
    test_queue()
    test_pool()
    print("******")


def test_update():
    data = {
        "config": {"alg": "Pepe XXX", "time_steps": 10},
        "task_id": 2,
    }
    r = rq.get('http://localhost:10000/task/update', data=json.dumps(data))
    print(r.json())

def test_delete():
    data = {
        "task_id": 11,
    }
    r = rq.get('http://localhost:10000/task/delete', data=json.dumps(data))
    print(r.json())


if __name__ == "__main__":
    # test_config()
    # test_create_task()
    test_delete()



