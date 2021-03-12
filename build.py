#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import subprocess
from termcolor import colored

"""
Go packages build tool.
It looks for all files under cmd dir and builds them.
Built files are placed in bin directory.
Important! 
One file per cmd folder.
Files are placed in bin, therefore their names should be different inside of one package to avoid collisions.
"""

os.environ["GO111MODULE"] = "auto"

def separate(path, num):
    end = ""
    for i in range(num):
        path, e = os.path.split(path)
        end = os.path.join(e, end)
    return path, end
    

if __name__ == "__main__":
    current = os.getcwd()
	
    # TODO rewrite next line to rid off of check_output
    r = subprocess.check_output('go list ./... | grep /cmd/',shell=True) 
    folders = []
    relatives = []
    build_paths = []
    files = []
    fnames = []
    paths = [p.replace("\n", "")[1:] for p in r.decode("utf-8").split("\n") if p != ""]
    for p in paths:
        files.append(os.listdir(p)[0])
        fnames.append(files[-1].strip(".go"))
        
        path, end = separate(p, 2)
        
        folders.append(path)
        build_paths.append(os.path.join("bin", fnames[-1]))
        relatives.append(os.path.join(end, files[-1]))

    cnt = 0
    for i, folder in enumerate(folders):
        pkg = folder.split("/")[-1]
        print(colored('Building', 'blue'), f'{pkg+"/"+relatives[i]}', end="\r") 
        os.chdir(folder)
        try: 
            r = subprocess.check_output(f"go build -o {build_paths[i]} {relatives[i]}", stderr=subprocess.STDOUT, shell=True).decode("UTF-8")
            import time
            time.sleep(1)
            print(" "*100, end="\r")
            print(colored('Built', 'green'), f'{pkg+"/"+build_paths[i]}')
            cnt += 1
            
        except Exception as e:
            print(colored(f"Failed", "red"), pkg+"/"+relatives[i])
            print(e.output.decode("UTF-8").strip())

    if float(cnt)/len(folders) == 1:
        print("Status:", colored(f"sucess", "green"))
    else: 
        print("Status:", colored(f"fail", "red"))
    print(f"Done: {cnt}/{len(folders)}")


