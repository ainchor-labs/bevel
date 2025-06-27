import yaml

def load_game_config(file='config.yaml'):
    with open(file, "r") as config:
        return yaml.safe_load(config)