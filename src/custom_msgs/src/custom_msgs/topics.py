import yaml
import os

THIS_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(THIS_DIRECTORY, '../../config', 'topics.yaml')

yamldir = None
with open(CONFIG_FILE, 'r') as file:
	yamldir = yaml.safe_load(file)

for key in yamldir:
	val = yamldir[key]
	command = key + '=val'
	exec(command)
	# print(command)
