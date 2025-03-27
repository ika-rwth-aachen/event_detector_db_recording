#!/usr/bin/env python

import argparse
import os


# parse arguments
parser = argparse.ArgumentParser(description="Generates C++ code for converting ROS messages to MongoDB BSON.")
parser.add_argument("namespace", type=str, help="ROS message namespace")
parser.add_argument("file", type=str, help="ROS .msg file")
parser.add_argument("-m", "--mode", type=str, choices=("cpp", "h", "include", "external"), default="cpp", help="Mode: 'cpp': function implementation; 'h': function declaration; 'include': includes; 'external': external namespaces")
parser.add_argument("-rv", "--ros-version", required=False, default=2, type=int, choices=(1, 2), help="ROS version (1 or 2)")
args = parser.parse_args()
namespace = args.namespace
msgFile = args.file
msgType = os.path.splitext(os.path.basename(msgFile))[0]
ros_version = args.ros_version

# convert camelCase format to underscore format (relevant for ROS 2)
def camel_case_to_underscore(input_str):
  output_str = ""
  for i, char in enumerate(input_str):
    is_upper = char.isupper()
    prev_is_lower = (i != 0 and input_str[i - 1].islower())
    prev_is_upper = (i != 0 and input_str[i - 1].isupper())
    next_is_lower = (i != len(input_str) - 1 and input_str[i + 1].islower())
    prev_is_digit = (i != 0 and input_str[i - 1].isdigit())
    if is_upper and (prev_is_lower or (prev_is_upper and next_is_lower) or prev_is_digit):
      output_str += "_"
    output_str += char.lower()
  return output_str

# parse attribute types and names from .msg file
attributes = []
externalNamespaces = []
with open(msgFile, "r") as f:
  lines = f.readlines()
for line in lines:
  line = line.strip()
  if not line or line.startswith("#") or "=" in line:
    continue # empty or comment lines or definition lines
  type, name = line.split()[0], line.split()[1]
  if type == "Header":
    type = f"std_msgs/{type}" # Header is sometimes not prefixed by std_msgs
  if not "/" in type and type[0].isupper():
    type = f"{namespace}/{type}" # implicit namespace
  elif "/" in type and type[0].islower() and not type.startswith(namespace):
    externalNamespaces.append(type.split("/")[0]) # external namespace
  attributes.append((type, name))

# format output
if args.mode == "cpp":

  # determine max attribute name length for formatting
  maxNameLen = -1
  for _, name in attributes:
    maxNameLen = max(maxNameLen, len(name))

  # generate function implementation
  if ros_version == 1:
    output  = f"value toBson(const {namespace}::{msgType}& msg) {{\n"
    output += f"  return document{{}}\n"
  else:
    output  = f"value toBson(const {namespace}::msg::{msgType}& msg) {{\n"
    output += f"  return document{{}}\n"
  for type, name in attributes:
    sp = (maxNameLen - len(name)) * " "
    output += f"    << \"{name}\"{sp} << toBson(msg.{name}){sp} // {type}\n"
  output += f"  << finalize;\n"
  output += f"}}"

elif args.mode == "h":

  if ros_version == 1:
    output = f"bsoncxx::document::value toBson(const {namespace}::{msgType}& msg);"
  else:
    output = f"bsoncxx::document::value toBson(const {namespace}::msg::{msgType}& msg);"

elif args.mode == "include":
  if ros_version == 1:
    output = f"#include <{namespace}/{msgType}.h>"
  else:
    msgType_underscore_separated = camel_case_to_underscore(msgType)
    output = f"#include <{namespace}/msg/{msgType_underscore_separated}.hpp>"

elif args.mode == "external":

  if len(externalNamespaces) > 0:
    output = f"{namespace}/{msgType} relies on external namespaces: {externalNamespaces}"
  else:
    output = ""

if len(output) > 0:
  print(output)