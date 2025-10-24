#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import yaml
from ament_index_python.packages import get_package_share_directory

def extract(config_path: str) -> dict:
    """Extracts information from a YAML file.

    Args:
        config_path (str): Path to the YAML file.

    Returns:
        dict: A dictionary containing the extracted information.
    """
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

def extract_configuration() -> dict:
    """Extracts configuration from the g1pilot package's config.yaml file.

    Returns:
        dict: A dictionary containing the extracted configuration.
    """
    package_share_directory = get_package_share_directory('g1pilot')
    config_file_path = f"{package_share_directory}/config/config.yaml"
    return extract(config_file_path)
