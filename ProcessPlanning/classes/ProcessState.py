from copy import deepcopy
from datetime import datetime
from typing import Optional, Union

from ProcessPlanning.classes.data_class.BuildingInstruction import BuildingInstruction
from ProcessPlanning.classes.data_class.ConstructionState import ConstructionState
from ProcessPlanning.classes.data_class.EventInfo import EventInfo
from PathFinding.data_class.Solution import Solution
from PathFinding.util import path_math
from PathFinding.util.path_math import get_direction, diff_pos
from ProcessPlanning.classes.util.ps_util import get_neighboring_layouts, get_detour_trail, \
    get_detour_state, get_building_instructions_from_solution
# todo: documentation
from type_dictionary.common_types import Pos, Trail


def get_completion_proportion(building_instructions):
    count = 0
    for layout, instruction in building_instructions.items():
        if instruction.layout_completed:
            count += 1
    proportion = count / len(building_instructions.keys())
    return proportion



class ProcessState:
    """Data class that contains information about a process state

    Attributes:

        state_grid: Numpy array that shows where obstacles and completed layouts occupy nodes
        aimed_solution: Solution that contains information of the desired construction./
        Is recalculated after a detour event.
        last_event_trail: Layout trail at which the last non deviated event occurred.

        part_stock: Dictionary containing the number of available parts in stock.
        picked_parts: List containing parts picked from stock. Parts will be removed upon usage.

        building_instructions: Dictionary containing information about the required placements and IDs of parts to
        complete the construction process. Is recalculated after a detour event.

        motion_dict: Dictionary containing all registered motion events that are valid (non-error) that point to
        a ConstructionState. The keys for pipe events are of the type Trail, while other events are of the type Pos.

        last_event_info: Dictionary containing information of the last event that occurred.

        event_history: #todo: consider removing if no usage

        """

    def __init__(self, solution: Solution):
        self.state_grid = solution.path_problem.state_grid
        self.aimed_solution = solution
        self.last_event_trail = solution.ordered_trails[0]

        self.part_stock = solution.path_problem.part_stock
        self.picked_parts: list[int] = []
        self.building_instructions = get_building_instructions_from_solution(solution)



        self.motion_dict: dict[tuple[Pos, int]:ConstructionState] = {}

        self.last_event_info = None

        self.detour_trails = []

        self.event_history = []

        self.completion = 0

    def get_construction_state(self, pos: Union[Pos, Trail], event_codes: list) -> Union[
        Union[Union[Pos, Trail], ConstructionState], tuple[None, None]]:
        """returns construction state and pos/trail if pos and event_code specified are in motion_dict."""
        if 2 in event_codes:
            for trail, construction_state in self.motion_dict.items():
                if (pos in trail or pos == trail) and construction_state.event_code == 2:
                    return trail, construction_state

        else:
            construction_state = self.motion_dict.get(pos)
            if construction_state:
                if construction_state.event_code in event_codes:
                    return pos, construction_state

        return None, None

    # events
    def pick_part(self, part_id: int):
        """Handling of a pick event."""
        self.picked_parts.append(part_id)
        self.part_stock[part_id] -= 1
        message = str.format(f"ProcessPlanner: Picked part with id {part_id}")
        return message

    def evaluate_placement(self, worker_event: tuple[Pos, int],
                           ignore_part_check: bool = False, ignore_obstructions: bool = False, assume_pipe_id_from_solution:bool = False) -> EventInfo:
        """Evaluates a placement event and registers changes to the motion dict according to building_instructions./
         Also registers if a instruction has been completed."""
        worker_event_pos = worker_event[0]
        worker_event_code = worker_event[1]

        part_id = None
        current_layout = None

        # possible event outcomes:
        removal = False
        completed_layouts = set()
        obstructed_obstacle = False
        obstructed_part = False
        deviated = False
        misplaced = False
        unnecessary = False
        error = False
        part_not_picked = False
        layout_changed = False
        detour_event = {}
        next_recommended_action = ()

        time_registered = datetime.now()

        event_info = EventInfo(event_pos=worker_event_pos, event_code=worker_event_code, part_id=part_id,
                               obstructed_part=obstructed_part, obstructed_obstacle=obstructed_obstacle,
                               deviated=deviated, misplaced=misplaced, unnecessary=unnecessary,
                               completed_layouts=completed_layouts,
                               layout=current_layout, removal=removal, part_not_picked=part_not_picked,
                               detour_event=detour_event, error=error, time_registered=time_registered,
                               layout_changed=layout_changed)

        construction_state = ConstructionState(event_pos=worker_event_pos, event_code=worker_event_code,
                                               part_id=part_id, deviated=deviated,
                                               misplaced=misplaced, unnecessary=unnecessary,
                                               time_registered=time_registered)

        # get current layout and building instruction
        building_instruction, current_layout, layout_changed = self.get_current_layout_and_building_instruction(
            worker_event_code, worker_event_pos, )
        event_info.layout = current_layout

        # event evaluation

        if self.part_removed(worker_event_pos, worker_event_code, event_info):
            if not event_info.deviated:
                event_info.completed_layouts.update(self.set_completion_state(current_layout, event_info))
            # return part to picked_parts
            if event_info.part_id in self.part_stock:
                self.picked_parts.append(event_info.part_id)
            return event_info
        if not ignore_obstructions:
            if self.obstructed_obstacle(worker_event_pos):
                event_info.obstructed_obstacle = True
                event_info.error = True
                return event_info

            event_info.obstructed_part = self.obstructed_part(worker_event_pos, worker_event_code)
            if event_info.obstructed_part:
                event_info.obstructed_part = True
                event_info.error = True
                return event_info

        # evaluate placement and modify event info accordingly

        if worker_event_code == 3:
            self.attachment_placed(event_pos=worker_event_pos, event_code=worker_event_code, event_info=event_info,
                                   building_instruction=building_instruction)

        elif worker_event_code == 2:
            self.pipe_placed(event_pos=worker_event_pos, event_info=event_info,
                             building_instruction=building_instruction, ignore_part_check=ignore_part_check, assume_pipe_id_from_solution=assume_pipe_id_from_solution)

        elif worker_event_code == 1:
            self.fitting_placed(event_pos=worker_event_pos, building_instruction=building_instruction,
                                event_info=event_info, ignore_part_check=ignore_part_check, assume_pipe_id_from_solution=assume_pipe_id_from_solution,
                                current_layout=current_layout)

        if not event_info.error:
            # register placement
            construction_state.part_id = event_info.part_id
            construction_state.deviated = event_info.deviated
            construction_state.unnecessary = event_info.unnecessary
            construction_state.misplaced = event_info.misplaced
            self.register_placement(building_instruction, construction_state, worker_event_code,
                                    worker_event_pos)


            if not event_info.deviated:

                if not assume_pipe_id_from_solution:
                    # check if pipe ids can be assumed from fittings
                    self.check_assignment_pipe_id(current_layout, event_info)
                # check completion state of this and neighboring layouts
                event_info.completed_layouts.update(self.set_completion_state(current_layout, event_info))
                self.completion = get_completion_proportion(self.building_instructions)


            else:
                if worker_event_code == 1:
                    # check for detour events
                    detour_event = self.check_detour_event(event_pos=worker_event_pos, event_code=worker_event_code)
                    if detour_event:
                        self.building_instructions.update(detour_event)
                        event_info.detour_event = detour_event
                        # update state grid
                        for pos in list(detour_event.keys())[0]:
                            self.state_grid[pos] = 2

            # modify parts_picked
            if event_info.part_id in self.part_stock.keys():
                self.picked_parts.remove(event_info.part_id)

        return event_info

    def get_current_layout_and_building_instruction(self, event_code, event_pos):
        """Returns the current layout and building instruction, if event occurred inside solution."""

        current_layout = self.last_event_trail
        building_instruction = self.building_instructions.get(current_layout)
        layout_changed = False

        if event_pos in self.aimed_solution.absolute_trail.keys():

            # get information about layout where event occurred
            if event_pos not in self.last_event_trail:
                trail = None
                # find the current layout
                if event_code == 1:
                    for trail in self.building_instructions.keys():
                        # current layout can be ambiguous
                        if event_pos in self.building_instructions[trail].required_fit_positions:
                            break
                    else:
                        for trail in self.building_instructions.keys():
                            if event_pos in trail:
                                break
                else:
                    for trail in self.building_instructions.keys():
                        if event_pos in trail:
                            break
                current_layout = trail

                layout_changed = current_layout != self.last_event_trail

                self.last_event_trail = current_layout

            building_instruction = self.building_instructions[current_layout]
        else:
            current_layout = None
            building_instruction = None

            # todo: check recommended att pos
        return building_instruction, current_layout, layout_changed

    def register_placement(self, building_instruction, construction_state, worker_event_code,
                           worker_event_pos):
        """Registers placement event into motion_dict."""
        # make construction state
        if worker_event_code == 2:
            if not construction_state.deviated and construction_state.part_id != -2:
                self.motion_dict[building_instruction.possible_att_pipe_positions] = construction_state
            else:
                self.motion_dict[(worker_event_pos,)] = construction_state
        else:
            self.motion_dict[worker_event_pos] = construction_state

    def part_removed(self, event_pos, event_code, event_info) -> Optional[tuple[Pos, ConstructionState]]:
        """Checks if a part was removed and registers it to motion_dict."""
        pos, construction_state = self.get_construction_state(pos=event_pos, event_codes=[event_code])

        if construction_state:
            # removal confirmed
            event_info.deviated = construction_state.deviated
            event_info.unnecessary = construction_state.unnecessary
            event_info.misplaced = construction_state.misplaced
            event_info.part_id = construction_state.part_id
            event_info.removal = True
            self.motion_dict.pop(pos)

            return pos, construction_state

        return None

    def attachment_placed(self, event_pos, event_code, building_instruction, event_info):
        """Evaluates the placement of an attachment. Notes findings into event_info."""
        # todo: check recommended att_pos
        if event_pos in self.aimed_solution.absolute_trail.keys():
            if event_pos not in building_instruction.required_fit_positions:
                if event_pos in building_instruction.possible_att_pipe_positions:
                    for pos in building_instruction.possible_att_pipe_positions:
                        _, construction_state = self.get_construction_state(pos=pos, event_codes=[event_code])
                        if construction_state:
                            event_info.unnecessary = True
                            event_info.deviated = True

            else:
                event_info.misplaced = True
                event_info.deviated = True
        else:
            event_info.deviated = True
        event_info.part_id = -1

    def pipe_placed(self, event_pos, building_instruction: BuildingInstruction, event_info, ignore_part_check, assume_pipe_id_from_solution):
        """Evaluates the placement of a pipe. Notes findings in event_info."""
        if event_pos in self.aimed_solution.absolute_trail.keys():
            part_id = building_instruction.pipe_id
            if not assume_pipe_id_from_solution:
                pipe_id = self.pipe_placed_deviated_route(event_info, ignore_part_check)#
                if pipe_id != part_id:
                    event_info.deviated = True
            else:
                # if pipe placement occurred somewhere inside the trail of building instruction, then it's valid
                event_info.part_id = part_id
                # check if part was actually picked
                if part_id not in self.picked_parts and not ignore_part_check:
                    event_info.deviated = True
                    self.pipe_placed_deviated_route(event_info, ignore_part_check)
        else:
            event_info.deviated = True
            self.pipe_placed_deviated_route(event_info, ignore_part_check)

    def pipe_placed_deviated_route(self, event_info, ignore_part_check):
        picked_parts = self.picked_parts
        # find the part id
        picked_pipes = [i for i in picked_parts if i != 0]
        if len(set(picked_pipes)) == 1:
            # if there is only one of a kind of pipe in picked_parts, then we know which id was placed
            event_info.part_id = picked_pipes[0]
        elif len(set(picked_parts)) > 1 or ignore_part_check:
            if self.get_num_unknown_pipes() < len(picked_pipes) or ignore_part_check:
                event_info.part_id = -2
            # multiple kinds of pipe picked
            else:
                # number of unknown pipes can't be greater than pipes picked
                event_info.part_id = -99
                event_info.error = True
                event_info.part_not_picked = True


        else:
            event_info.part_id = -99
            event_info.error = True
            event_info.part_not_picked = True

        return event_info.part_id

    def fitting_placed(self, event_pos, building_instruction, event_info,assume_pipe_id_from_solution, ignore_part_check, current_layout):
        """Evaluates the placement of a fitting. Notes findings in event_info. If picked_parts is set to None, part restrictions will be ignored"""
        if 0 in self.picked_parts or ignore_part_check:
            event_info.part_id = 0
            if event_pos in self.aimed_solution.absolute_trail.keys():

                if event_pos not in building_instruction.required_fit_positions:
                    event_info.misplaced = True
                    event_info.deviated = True
            else:
                event_info.deviated = True
        else:
            # ERROR: Part not picked!
            event_info.part_not_picked = True
            event_info.error = True
            event_info.part_id = -99

    def check_assignment_pipe_id(self, current_layout, event_info):
        neighboring_layouts = get_neighboring_layouts(current_layout, self.aimed_solution.ordered_trails)
        neighboring_layouts.append(current_layout)


        for layout in neighboring_layouts:
            check_building_instruction = self.building_instructions.get(layout)
            # check if both fittings are placed
            fit_set = set()
            for fit_pos in check_building_instruction.required_fit_positions:
                if None not in self.get_construction_state(fit_pos, event_codes=[1]):
                    fit_set.add(fit_pos)

            if len(fit_set) != 2:
                continue

            # check if a pipe id needs to be assigned
            pipe_set = set()
            for pos in check_building_instruction.possible_att_pipe_positions:
                if None not in self.get_construction_state(pos=pos, event_codes=[2]):
                    pipe_set.add(pos)

            if len(pipe_set) == 1:
                pipe_pos = pipe_set.pop()
                _, pipe_state = self.get_construction_state(pos=pipe_pos, event_codes=[2])
                if self.assign_pipe_id(pipe_state, check_building_instruction.pipe_id):
                    # reassign pipe id
                    if check_building_instruction.pipe_id in self.picked_parts:
                        self.picked_parts.remove(check_building_instruction.pipe_id)
                    else:
                        continue
                    self.motion_dict.pop((pipe_pos,))
                    self.register_placement(building_instruction=check_building_instruction, construction_state=pipe_state,
                                             worker_event_pos=pipe_pos, worker_event_code=2)

    def assign_pipe_id(self, pipe_state, pipe_id):

            if pipe_state.part_id == -2:
                # check picked parts
                if pipe_id in self.picked_parts:
                    pipe_state.part_id = pipe_id

                    return pipe_id



    def check_detour_event(self, event_pos, event_code) -> dict:

        """Checks if conditions for a detour event are met (after placement of a fitting).
        Detour event occurrs if:
        - 2 fittings in proximity (connectable by available parts)
        - 1 attachment in between those 2 fittings
        - 1 pipe (with correct id) in between those fittings"""

        check_fit_set = self.get_all_fit_positions()
        check_fit_set.remove(event_pos)

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
                detour_trail = get_detour_trail(length=fit_diff, direction=fit_dir, fit_pos=fit_tup)

                attachment_is_between = False

                att_set = set()
                pipe_set = set()

                pipe_trail = [i for i in detour_trail if i not in fit_tup]
                for att_pos in self.get_all_att_positions():
                    if att_pos in pipe_trail:
                        att_set.add(att_pos)

                if not att_set:
                    # no attachments in between
                    continue

                for pipe_pos in pipe_trail:
                    if None not in self.get_construction_state(pos=pipe_pos, event_codes=[2]):
                        pipe_set.add(pipe_pos)

                if len(pipe_set) != 1:
                    continue

                # get_updated_state_on_construction_event(tentative_state, fit_diff, fit_dir, event_pos, pos)

                detour_instruction = get_detour_state(length=fit_diff, att_set=att_set, pipe_set=pipe_set,
                                                fit_tup=fit_tup,
                                                state_grid=self.aimed_solution.path_problem.state_grid,
                                                possible_att_pipe_positions=tuple(pipe_trail))

                pipe_pos = pipe_set.pop()
                _, pipe_state = self.get_construction_state(pos=pipe_pos, event_codes=[2])
                if pipe_state.part_id == -2:
                    # assign part id
                    if pipe_id in self.picked_parts:
                        pipe_state.part_id = pipe_id
                        self.picked_parts.remove(pipe_id)
                    else:
                        continue

                    self.motion_dict.pop((pipe_pos,))
                    self.register_placement(building_instruction=detour_instruction, construction_state=pipe_state,
                                             worker_event_pos=pipe_pos, worker_event_code=2)
                elif pipe_state.part_id != pipe_id:
                    continue



                return {detour_trail: detour_instruction}

    def handle_detour_event(self, solution: Solution, detour_event: dict = None):

        self.aimed_solution = solution
        self.building_instructions = get_building_instructions_from_solution(solution)
        # copy motion_dict
        motion_dict = deepcopy(self.motion_dict)
        self.motion_dict = {}

        # copy picked parts
        picked_parts = deepcopy(self.picked_parts)

        # reevaluate motion dict according to new building instructions
        self.last_event_trail = self.aimed_solution.ordered_trails[0]
        for pos, construction_state in motion_dict.items():
            building_instruction, current_layout, _ = self.get_current_layout_and_building_instruction(
                event_pos=construction_state.event_pos, event_code=construction_state.event_code)

            # possible event outcomes:
            completed_layouts = set()
            deviated = False
            misplaced = False
            unnecessary = False
            # error = False

            event_info = EventInfo(event_pos=construction_state.event_pos, event_code=construction_state.event_code,
                                   removal=False, layout=None,
                                   part_id=None, deviated=deviated, detour_event={}, misplaced=misplaced,
                                   unnecessary=unnecessary,
                                   obstructed_part=False, completed_layouts=completed_layouts, part_not_picked=False,
                                   error=False,
                                   obstructed_obstacle=False, time_registered=construction_state.time_registered,
                                   layout_changed=False)

            new_construction_state = ConstructionState(event_pos=construction_state.event_pos,
                                                       event_code=construction_state.event_code,
                                                       part_id=construction_state.part_id, deviated=deviated,
                                                       unnecessary=unnecessary, misplaced=misplaced,
                                                       time_registered=construction_state.time_registered)

            if construction_state.event_code == 3:
                self.attachment_placed(event_pos=construction_state.event_pos, event_code=construction_state.event_code,
                                       event_info=event_info,
                                       building_instruction=building_instruction)

            elif construction_state.event_code == 2:
                self.pipe_placed_detour_event(event_pos=construction_state.event_pos, event_info=event_info,
                                              building_instruction=building_instruction,
                                              pipe_id=construction_state.part_id)

            elif construction_state.event_code == 1:
                self.fitting_placed(event_pos=construction_state.event_pos, building_instruction=building_instruction,
                                    event_info=event_info, ignore_part_check=True, assume_pipe_id_from_solution=False, current_layout=current_layout)


            new_construction_state.part_id = event_info.part_id
            new_construction_state.deviated = event_info.deviated
            new_construction_state.unnecessary = event_info.unnecessary
            new_construction_state.misplaced = event_info.misplaced
            self.register_placement(building_instruction=building_instruction,
                                    construction_state=new_construction_state,
                                    worker_event_code=construction_state.event_code,
                                    worker_event_pos=construction_state.event_pos)
            if not event_info.deviated:
                event_info.completed_layouts.update(self.set_completion_state(current_layout, event_info))

        if detour_event:
            self.last_event_trail = list(detour_event.keys())[0]
        else:
            self.last_event_trail = self.aimed_solution.ordered_trails[0]

    # restriction checks

    def obstructed_part(self, event_pos: Pos, event_code: int) -> Optional[int]:
        pos, construction_state = self.get_construction_state(pos=event_pos, event_codes=[2])
        if construction_state and event_code != 2:
            # obstructed pipe
            return construction_state.event_code
        else:
            pos, construction_state = self.get_construction_state(pos=event_pos, event_codes=[1, 3])
            if construction_state:
                if construction_state.event_code != event_code:
                    if construction_state.event_code == 3 and event_code == 2:
                        # pipes can obstruct attachments
                        return None
                    # there is a part here that can't be obstructed for current event code
                    return construction_state.event_code

            else:
                # nothing here
                return None

    def obstructed_obstacle(self, pos: Pos):
        """Check state grid if position obstructs obstacle"""
        if self.state_grid[pos] == 1 or self.state_grid[pos] == 3:
            return True
        else:
            return False

    # utilities
    def set_completion_state(self, current_layout, event_info) -> set:
        """sets completion state of current layout and neighboring layouts. Updates state grid on completed layouts."""
        neighboring_layouts = get_neighboring_layouts(current_layout, self.aimed_solution.ordered_trails)
        neighboring_layouts.append(current_layout)
        completed_layouts = set()
        for layout in neighboring_layouts:

            building_instruction = self.building_instructions.get(layout)
            if not building_instruction.layout_completed:
                # check if instruction was completed
                if self.completed_instruction(building_instruction)[0]:
                    building_instruction.layout_completed = True
                    completed_layouts.add(layout)
                    for pos in layout:
                        self.state_grid[pos] = 2

            else:
                if event_info.removal:
                    building_instruction.layout_completed = False
                    for pos in layout:
                        self.state_grid[pos] = 0

        return completed_layouts

    def completed_instruction(self, building_instruction) -> tuple[bool, int]:
        """Checks if the building instruction was completed.
        and returns event code corresponding to its completion state."""

        # check if required pipe placed
        pipe_positions = building_instruction.possible_att_pipe_positions

        att_pos_set = set()
        for pos in building_instruction.possible_att_pipe_positions:
            att_pos, att_state = self.get_construction_state(pos, event_codes=[3])
            if att_state:
                if not att_state.deviated:
                    att_pos_set.add(att_pos)

        if not att_pos_set:
            return False, 3

        if None in self.get_construction_state(pipe_positions, event_codes=[2]):
            return False, 2

        # check if required fitting placed
        for fit_pos in building_instruction.required_fit_positions:
            if None in self.get_construction_state(fit_pos, event_codes=[1]):
                return False, 1

        return True, 0

    def get_all_fit_positions(self):
        fit_set = set()
        for pos, construction_state in self.motion_dict.items():
            if construction_state.event_code == 1:
                fit_set.add(pos)
        return fit_set

    def get_all_att_positions(self):
        att_set = set()
        for pos, construction_state in self.motion_dict.items():
            if construction_state.event_code == 3:
                att_set.add(pos)
        return att_set

    def get_all_pipe_positions(self):
        pipe_set = set()
        for pos, construction_state in self.motion_dict.items():
            if construction_state.event_code == 2:
                pipe_set.add(pos)
        return pipe_set

    def pipe_placed_detour_event(self, event_pos, event_info, building_instruction, pipe_id):
        if event_pos in self.aimed_solution.absolute_trail.keys():
            # if pipe placement occurred somewhere inside the trail of building instruction, then it's valid
            part_id = building_instruction.pipe_id
            event_info.part_id = pipe_id
            # check if part was actually picked
            if pipe_id == -2:
                pass
            elif part_id != pipe_id:
                event_info.deviated = True
        else:
            event_info.part_id = pipe_id
            event_info.deviated = True

        pass

    def get_num_unknown_pipes(self):
        counter = 0
        for pos, construction_state in self.motion_dict.items():
            if construction_state.event_code == 2:
                if construction_state.part_id == -2:
                    counter += 1
        return counter
