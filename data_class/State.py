from dataclasses import dataclass
from data_class import PathProblem
from typing import Optional, Set
from path_finding.common_types import *

@dataclass
class State:
    """Data class that contains information about a process state"""
    connections: Set[Connection] # A set of Connections -> represents current build state
    state_grid : any # state grid with build layout
    part_stock: dict  # parts left
    definite_path: DefinitePath # only used when deviation detected to create partial solutions
    layouts: Layouts
