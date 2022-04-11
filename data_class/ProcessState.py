from dataclasses import dataclass, field
from data_class.PathProblem import PathProblem
from data_class.BuildingInstruction import BuildingInstruction
from data_class.ConstructionState import ConstructionState
from data_class.EventInfo import EventInfo
from typing import Optional, Set, Union

from path_finding import path_math
from path_finding.common_types import *
from data_class.Solution import Solution
import numpy as np




# todo: documentation
from path_finding.path_math import get_direction, diff_pos, manhattan_distance
from pp_utilities import get_optimal_attachment_pos, get_deviation_trail


class ProcessState:
    """Data class that contains information about a process state"""

    def __init__(self, solution:Solution):
        self.state_grid = solution.path_problem.state_grid
        self.aimed_solution = solution
        self.last_event_layout = solution.layouts[0]

        self.part_stock = solution.path_problem.part_stock
        self.picked_parts: list[int] = []
        self.building_instructions = get_building_instruction(solution)

        self.motion_dict: dict[tuple[Pos,int]:ConstructionState] = {}

        self.last_event_info = None

        self.event_history = []

    def get_construction_state(self, pos:Union[Pos,Trail], event_code) -> Optional[ConstructionState]:
        if event_code == 2:
            for trail, construction_state in self.motion_dict.items():
                    if pos in trail and construction_state.event_code == 2:
                        return construction_state

            else:
                return None
        else:
            return self.motion_dict.get(pos)

    #events
    def pick_part(self, part_id: int):
        self.picked_parts.append(part_id)
        self.part_stock[part_id] -= 1
        message = str.format(f"ProcessPlanner: Picked part with id {part_id}")
        return message

    def evaluate_placement(self, worker_event: tuple[Pos, int], check_for_deviation_events: bool = True,
                              ignore_errors: bool = False, allow_stacking: bool = False):

        worker_event_pos = worker_event[0]
        worker_event_code = worker_event[1]

        part_id = None
        current_layout = None


        # possible event outcomes:
        removal = False
        completed = False
        obstructed_obstacle = False
        obstructed_part = False
        deviated = False
        misplaced = False
        unnecessary = False
        error = False
        part_not_picked = False
        rerouting_event = {}

        event_info = EventInfo(event_pos=worker_event_pos, event_code=worker_event_code, part_id=part_id,
                                    obstructed_part=obstructed_part, obstructed_obstacle=obstructed_obstacle, deviated=deviated, misplaced=misplaced, unnecessary=unnecessary, completed=completed,
                               current_layout=current_layout, removal=removal, part_not_picked=part_not_picked, rerouting_event= rerouting_event, error=error)

        construction_state = ConstructionState(event_code=worker_event_code, part_id=part_id, deviated=deviated, misplaced=misplaced, unnecessary=unnecessary)

        # event evaluation

        current_layout = self.last_event_layout
        building_instruction = self.building_instructions[current_layout]

        if self.part_removed(worker_event_pos,worker_event_code, event_info):
            return event_info

        if self.obstructed_obstacle(worker_event_pos):
            event_info.obstructed_obstacle = True
            return event_info

        event_info.obstructed_part = self.obstructed_part(worker_event_pos, worker_event_code)
        if obstructed_part:
            return event_info

        # get the current layout and building instruction, if event inside solution
        if worker_event_pos in self.aimed_solution.total_definite_trail.keys():

            # get information about layout where event occurred
            if worker_event_pos in self.last_event_layout:
                current_layout = self.last_event_layout
            else:
                trail = None
                # find the current layout
                if worker_event_code == 1:
                    for trail in self.building_instructions.keys():
                        # current layout can be ambiguous
                        if worker_event_pos in self.building_instructions[trail].required_fit_positions:
                            break
                    else:
                        for trail in self.building_instructions.keys():
                            if worker_event_pos in trail:
                                break
                else:
                    for trail in self.building_instructions.keys():
                        if worker_event_pos in trail:
                            break
                current_layout = trail
                self.last_event_layout = current_layout

            layout_changed = current_layout != self.last_event_layout

            building_instruction = self.building_instructions[current_layout]

            # todo: check recommended att pos

        if worker_event_code == 3:
            self.attachment_placed(event_pos=worker_event_pos, event_code=worker_event_code, event_info=event_info, building_instruction=building_instruction)

        elif worker_event_code == 2:
            self.pipe_placed(event_pos=worker_event_pos, event_code=worker_event_code, event_info=event_info, building_instruction=building_instruction)

        elif worker_event_code == 1:
            self.fitting_placed(event_pos=worker_event_pos, building_instruction=building_instruction, event_info=event_info)

        if not event_info.error:

            if not event_info.deviated:
                # check completion state of this and neighboring layouts
                event_info.completed = self.set_completion_state(current_layout, event_info)
            else:
                if worker_event_code == 1 and check_for_deviation_events:
                    # check for deviation events
                    deviation = self.rerouting_event(event_pos=worker_event_pos, event_code=worker_event_code)
                    if deviation:
                        self.building_instructions.update(deviation)
                        # update state grid
                        for pos in list(deviation.keys())[0]:
                            self.state_grid[pos] = 2

            # register placement
            if not event_info.part_not_picked:
                #make construction state
                construction_state.event_code = worker_event_code
                construction_state.misplaced = event_info.misplaced
                construction_state.deviated = event_info.deviated
                construction_state.unnecessary = event_info.unnecessary
                construction_state.part_id = event_info.part_id
                if worker_event_code == 2 and not construction_state.deviated:
                    self.motion_dict[building_instruction.possible_att_pipe_positions] = ConstructionState
                else:
                    self.motion_dict[worker_event_pos] = ConstructionState

        return event_info

    def part_removed(self, event_pos, event_code, event_info) -> bool:

        construction_state = self.get_construction_state(pos=event_pos, event_code=event_code)

        if construction_state:
            # removal confirmed
            event_info.deviated = construction_state.deviated
            event_info.unnecessary = construction_state.unnecessary
            event_info.misplaced = construction_state.misplaced
            event_info.removal = True

        return True

    def attachment_placed(self, event_pos, event_code, building_instruction, event_info):
        #todo: check recommended att_pos
        if event_pos in self.aimed_solution.total_definite_trail.keys():
            if event_pos not in building_instruction.required_fit_positions:
                if event_pos in building_instruction.possible_att_pipe_positions:
                    for pos in building_instruction.possible_att_pipe_positions:
                        if self.get_construction_state(pos, event_code).event_code == 3:
                            event_info.unnecessary = True

            else:
                event_info.misplaced = True
        else:
            event_info.deviated = True



    def pipe_placed(self, event_pos, event_code,  building_instruction, event_info):

        if event_pos in self.aimed_solution.total_definite_trail.keys():
            # if pipe placement occurred somewhere inside the trail of building instruction, then it's valid
            part_id = building_instruction.part_id
            # check if part was actually picked
            # todo: if part not in picked_parts, check if other pipes available like on else route
            if part_id not in self.picked_parts:
                # ERROR : part was not picked!
                event_info.part_not_picked = True
                event_info.error = True
                event_info.part_id = part_id
        else:
                event_info.deviated = True
                picked_parts = self.picked_parts
                # find the part id
                picked_pipes = [i for i in picked_parts if i != 0]
                if len(set(picked_pipes)) == 1:
                    # if there is only one of a kind of pipe in picked_parts, then we know which id was placed
                    event_info.part_id = picked_pipes[0]
                    event_info.deviated = True
                elif len(set(picked_parts)) > 1:
                    # if there are multiple kinds of pipe picked
                    event_info.part_id = -2
                    event_info.deviated = True
                else:
                    event_info.part_id = -99
                    event_info.deviated = True
                    event_info.part_not_picked = True


    def fitting_placed(self, event_pos, building_instruction, event_info):
        if 0 in self.picked_parts:
            event_info.part_id = 0
            if event_pos in self.aimed_solution.total_definite_trail.keys():
                    if event_pos not in building_instruction.required_fit_positions:

                        event_info.misplaced = True
            else:
                event_info.deviated = True
        else:
            #ERROR: Part not picked!
            event_info.part_not_picked = True
            event_info.error = True


    def rerouting_event(self, event_pos, event_code) -> dict:

        """Checks for deviating layouts on captured motion events and updates tentative state."""

        check_fit_set = self.get_all_fit_positions()

        if event_code == 1:
            for pos in check_fit_set:

                # check if conditions for a deviation event are met
                fit_diff = path_math.get_length_same_axis(pos,
                                                          event_pos)  # distance between fittings
                pipe_id = fit_diff - 1  # length of needed part

                if pipe_id == 0:
                    # fittings are directly next to each other
                    break

                fittings_in_proximity = pipe_id in self.part_stock.keys()  # fittings are connectable by available parts

                if not fittings_in_proximity:
                    continue

                fit_dir = get_direction(diff_pos(pos, event_pos))
                fit_tup = (pos, event_pos)
                deviation_trail = get_deviation_trail(length=fit_diff, direction=fit_dir, fit_pos=fit_tup)

                attachment_is_between = False

                att_set = set()
                pipe_set = set()

                pipe_trail = [i for i in deviation_trail if i not in fit_tup]
                for att_pos in tentative_state.deviated_motion_set_attachment:
                    if att_pos in pipe_trail:
                        att_set.add(att_pos)

                if not att_set:
                    # no attachments in between
                    continue

                for pipe_pos in pipe_trail:
                    if tentative_state.deviated_motion_dict_pipe.get(pipe_pos) == pipe_id:
                        pipe_set.add(pipe_pos)
                    elif tentative_state.deviated_motion_dict_pipe.get(pipe_pos) == -1:
                        pipe_set.add(pipe_pos)

                if not pipe_set:
                    # no pipes in between
                    continue

                print("Deviation confirmed")

                first_pipe_pos = ()
                # get_updated_state_on_construction_event(tentative_state, fit_diff, fit_dir, event_pos, pos)

                deviation_state = get_deviation_state(length=fit_diff, att_set=att_set, pipe_set=pipe_set,
                                                      fit_tup=fit_tup,
                                                      state_grid=self._initial_path_problem.state_grid)

                tentative_state.fc_set.add(fit_tup)

                return {deviation_trail: deviation_state}


    #restriction checks

    def obstructed_part(self,pos:Pos, event_code:int) -> Optional[int]:
        construction_state = self.get_construction_state(pos=pos, event_code=2)
        if construction_state and event_code != 2:
            # obstructed pipe
            return construction_state.event_code
        else:
            construction_state = self.get_construction_state(pos=pos, event_code=1)
            if construction_state and construction_state.event_code != event_code:
                if construction_state.event_code == 3 and event_code == 2:
                    # pipes can obstruct attachments
                    return None
                # there is a part here that can't be obstructed for current event code
                return construction_state.event_code

            else:
                #nothing here
                return None

    def obstructed_obstacle(self, pos:Pos):
        """Check state grid if position obstructs obstacle"""
        if self.state_grid[pos] == 1:
            return True
        else:
            return False

    # utilities
    def set_completion_state(self, current_layout, event_info):
        """sets completion state of current layout and neighboring layouts. Updates state grid."""
        neighboring_layouts = get_neighboring_layouts(current_layout, self.aimed_solution.layouts)
        neighboring_layouts.append(current_layout)

        for layout in neighboring_layouts:
            layout_state = self.building_instructions.get(layout)
            if not layout_state.completed:
                # check if required fittings placed
                for fit_pos in layout_state.required_fit_positions:
                    if not self.get_construction_state(fit_pos, event_code=1).event_code == 1:
                        return False

                # check if required pipe placed
                if not self.get_construction_state(layout_state.possible_att_pipe_positions, event_code=2) == 2:
                    return False


                layout_state.completed = True

                for pos in layout:
                    self.state_grid[pos] = 2
                return True

            else:
                if event_info.removal:
                    layout_state.completed = False
                    for pos in layout:
                        self.state_grid[pos] = 0


    def get_all_fit_positions(self):
        fit_set = set()
        for pos, construction_state in self.motion_dict.items():
            if construction_state.event_code == 1:
                fit_set.add(pos)
        return fit_set

    def get_all_pipe_positions(self):
        pipe_set = set()
        for pos, construction_state in self.motion_dict.items():
            if construction_state.event_code == 2:
                pipe_set.add(pos)
        return pipe_set


def get_building_instruction(solution):
    construction_layout = {}
    start = solution.path_problem.start_pos
    goal = solution.path_problem.goal_pos


    for layout_trail in solution.layouts:
        add_fit = set()
        pipe_id = solution.total_definite_trail[layout_trail[1]]
        rec_att_pos = get_optimal_attachment_pos(state_grid=solution.path_problem.state_grid,
                                                 direction=get_direction(diff_pos(layout_trail[0],layout_trail[1])),
                                                 part_id=manhattan_distance(layout_trail[1], layout_trail[-1]),
                                                 pos=layout_trail[1])
        if layout_trail[0] == start:
            add_fit.add(start)
        elif layout_trail[-1] == goal:
            add_fit.add(goal)

        construction_layout[tuple(layout_trail)] = BuildingInstruction(att_set=set(), pipe_set=set(),
                                                                       fit_set=add_fit, pipe_id=pipe_id,
                                                                       required_fit_positions=(layout_trail[0],layout_trail[-1]),
                                                                       recommended_attachment_pos=rec_att_pos)


    return construction_layout

def get_neighboring_layouts(current_layout: Trail, layouts: Layouts) -> list[Trail]:
    neighboring_layouts = []
    idx = layouts.index(current_layout)

    if idx + 1 < len(layouts):
        neighboring_layouts.append(layouts[idx + 1])
    if idx - 1 >= 0:
        neighboring_layouts.append(layouts[idx - 1])

    return neighboring_layouts