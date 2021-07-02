#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from sshtunnel import SSHTunnelForwarder
import pymongo
import pandas as pd


def restore_consistency(data, colls, insterted):
    ret_data = {}
    indexes = [insterted.index(coll) for coll in colls]
    for field, arrays in data.items():
        ret_data[field] = []
        for i, coll in enumerate(colls):
            ret_data[field].append(arrays[indexes[i]])
    return ret_data


def retrieve_local(database, colls, fields):
    client = pymongo.MongoClient('127.0.0.1', 27017)  # server.local_bind_port is assigned local port
    db = client[database]
    inserted = []
    data = {field: [] for field in fields}
    for name in db.collection_names():
        if name not in colls:
            continue
        inserted.append(name)
        df = pd.DataFrame(list(getattr(db, name).find()))
        for field in fields:
            data[field].append(df[field].to_numpy())

    data = restore_consistency(data, colls, inserted)
    return data

def retrieve(database, colls, fields):
    """

    :param database: database name
    :param colls: database collections
    :param fields: fields to retrieve
    :return data: dictionary where keys represent fields
    """
    # Example
    # database = "exp_22_06"
    # colls = ["flat1", "flat2", "flat3"]
    # fields = ["reward"]

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
    inserted = []
    for name in db.collection_names():
        if name not in colls:
            continue
        inserted.append(name)
        df = pd.DataFrame(list(getattr(db, name).find()))
        for field in fields:
            data[field].append(df[field].to_numpy())

    server.stop()
    data = restore_consistency(data, colls, inserted)
    return data
