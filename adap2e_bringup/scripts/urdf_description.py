#!/usr/bin/env python3
import adap2e_bringup
import sys

if __name__ == "__main__":

  argv = sys.argv

  parameters = {}
  for argument in argv[1:]:
       name, value = argument.split(':')
       parameters[name] = value

  mode=parameters["mode"]
  robot_model=parameters["robot_model"]

  if not parameters["robot_namespace"] :
    prefix="";
  else:
    prefix=parameters["robot_namespace"]+"_";

  print(adap2e_bringup.urdf_description(prefix,mode,robot_model))
