#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from sshtunnel import SSHTunnelForwarder
import pymongo
import pandas as pd

# Configs
database = "exp_22_06"
colls = ["flat1", "flat2", "flat3"]
fields = ["reward"]
data = {field: [] for field in fields}

# Data retrieval
MONGO_HOST = "gazebo1.enstb.org"
MONGO_DB = database
MONGO_USER = "amitriakov"
MONGO_PASS = "enstaensta"

server = SSHTunnelForwarder(
    MONGO_HOST,
    ssh_username=MONGO_USER,
    ssh_password=MONGO_PASS,
    remote_bind_address=('127.0.0.1', 27017)
)

server.start()

client = pymongo.MongoClient('127.0.0.1', server.local_bind_port)  # server.local_bind_port is assigned local port
db = client[MONGO_DB]



for name in db.collection_names():
    if name not in colls:
        continue
    df = pd.DataFrame(list(getattr(db, name).find()))
    for field in fields:
        data[field].append(df["reward"].to_numpy())

server.stop()

# Drawing
# ...