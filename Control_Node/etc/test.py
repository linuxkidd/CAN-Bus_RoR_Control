#!/usr/bin/env python3

import json
import ruamel.yaml as yaml
from pprint import pprint

with open('ascomroof.yml','r') as specfile:
  try:
    spec=yaml.round_trip_load(specfile)
  except yaml.YAMLError as err:
    print(err)
    exit(1)

print(json.dumps(spec,indent=4))
