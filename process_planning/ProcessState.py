from copy import deepcopy
from typing import Optional, Union

from data_class.BuildingInstruction import BuildingInstruction
from data_class.ConstructionState import ConstructionState
from data_class.EventInfo import EventInfo
from data_class.Solution import Solution
from path_finding import path_math
from types.type_dictionary import *
# todo: documentation
from types.type_dictionary import Pos, Trail
from path_finding.path_math import get_direction, diff_pos
from process_planning.ps_util import get_neighboring_layouts, get_detour_trail, \
    get_detour_state, get_building_instructions_from_solution


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

        self.event_history = []

    def get_construction_state(self, pos: Union[Pos, Trail], event_codes: list) -> Union[
        Union[Union[Pos, Trail], ConstructionState], tuple[None, None]]:
        """returns construction state and pos/trail if pos and event_code are specified are in motion_dict."""
        if 2 in event_codes:
            for trail, construction_state in self.motion_dict.items():
                if pos in trail and construction_state.event_code == 2:
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

    def evaluate_placement(self, worker_event: tuple[Pos, int], check_for_deviation_events: bool = True,
                           ignore_errors: bool = False, allow_stacking: bool = False) -> EventInfo:
        """Evaluates a placement event and registers changes to the motion dict according to building_instructions./
         Also registers if a instruction has been completed."""
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
        detour_event = {}

        event_info = EventInfo(event_pos=worker_event_pos, event_code=worker_event_code, part_id=part_id,
                               obstructed_part=obstructed_part, obstructed_obstacle=obstructed_obstacle,
                               deviated=deviated, misplaced=misplaced, unnecessary=unnecessary, completed=completed,
                               current_layout=current_layout, removal=removal, part_not_picked=part_not_picked,
                               detour_event=detour_event, error=error)

        construction_state = ConstructionState(event_code=worker_event_code, part_id=part_id, deviated=deviated,
                                               misplaced=misplaced, unnecessary=unnecessary)

        # event evaluation

        if self.part_removed(worker_event_pos, worker_event_code, event_info):
            # return part to picked_parts
            if event_info.part_id in self.part_stock:
                self.picked_parts.append(event_info.part_id)
            return event_info

        if self.obstructed_obstacle(worker_event_pos):
            event_info.obstructed_obstacle = True
            return event_info

        event_info.obstructed_part = self.obstructed_part(worker_event_pos, worker_event_code)
        if obstructed_part:
            return event_info

        building_instruction, current_layout = self.get_building_instruction(worker_event_code, worker_event_pos)

        # evaluate placement and modify event info accordingly

        if worker_event_code == 3:
            self.attachment_placed(event_pos=worker_event_pos, event_code=worker_event_code, event_info=event_info,
                                   building_instruction=building_instruction)

        elif worker_event_code == 2:
            self.pipe_placed(event_pos=worker_event_pos, event_info=event_info,
                             building_instruction=building_instruction)

        elif worker_event_code == 1:
            self.fitting_placed(event_pos=worker_event_pos, building_instruction=building_instruction,
                                event_info=event_info)

        if not event_info.error:

            if not event_info.deviated:
                # check completion state of this and neighboring layouts
                event_info.completed = self.set_completion_state(current_layout, event_info)
            else:
                if worker_event_code == 1 and check_for_deviation_events:
                    # check for detour events
                    detour_event = self.detour_event(event_pos=worker_event_pos, event_code=worker_event_code)
                    if detour_event:
                        self.building_instructions.update(detour_event)
                        event_info.detour_event = detour_event
                        # update state grid
                        for pos in list(detour_event.keys())[0]:
                            self.state_grid[pos] = 2

            # register placement
            if not event_info.part_not_picked:
                self.register_placement(building_instruction, construction_state, event_info, worker_event_code,
                                        worker_event_pos)
                # todo: remove function_test

            # modify parts_picked
            if event_info.part_id in self.part_stock.keys():
                self.picked_parts.remove(event_info.part_id)

        return event_info

    def get_building_instruction(self, event_code, event_pos):
        """Returns the current layout and building instruction, if event occurred inside solution. Otherwise returns the last/
        building instruction."""
        current_layout = self.last_event_trail
        building_instruction = self.building_instructions.get(current_layout)

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
                self.last_event_trail = current_layout

            layout_changed = current_layout != self.last_event_trail

            building_instruction = self.building_instructions[current_layout]

            # todo: check recommended att pos
        return building_instruction, current_layout

    def register_placement(self, building_instruction, construction_state, event_info, worker_event_code,
                           worker_event_pos):
        """Registers placement event into motion_dict."""
        # make construction state
        construction_state.event_code = worker_event_code
        construction_state.misplaced = event_info.misplaced
        construction_state.deviated = event_info.deviated
        construction_state.unnecessary = event_info.unnecessary
        construction_state.part_id = event_info.part_id
        if worker_event_code == 2:
            if not construction_state.deviated:
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

            else:
                event_info.misplaced = True
        else:
            event_info.deviated = True
        event_info.part_id = -1

    def pipe_placed(self, event_pos, building_instruction: BuildingInstruction, event_info):
        """Evaluates the placement of a pipe. Notes findings in event_info."""
        if event_pos in self.aimed_solution.absolute_trail.keys():
            # if pipe placement occurred somewhere inside the trail of building instruction, then it's valid
            part_id = building_instruction.pipe_id
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
            elif len(set(picked_parts)) > 1:
                # if there are multiple kinds of pipe picked
                event_info.part_id = -2
            else:
                event_info.part_id = -99
                event_info.part_not_picked = True

    def fitting_placed(self, event_pos, building_instruction, event_info, ignore_part_restriction=False):
        """Evaluates the placement of a fitting. Notes findings in event_info. If picked_parts is set to None, part restrictions will be ignored"""
        if 0 in self.picked_parts or ignore_part_restriction:
            event_info.part_id = 0
            if event_pos in self.aimed_solution.absolute_trail.keys():
                if event_pos not in building_instruction.required_fit_positions:
                    event_info.misplaced = True
            else:
                event_info.deviated = True
        else:
            # ERROR: Part not picked!
            event_info.part_not_picked = True
            event_info.error = True
            event_info.part_id = -99

    def detour_event(self, event_pos, event_code) -> dict:

        """Checks if conditions for a detour event are met (after placement of a fitting).
        Detour event occurrs if:
        - 2 fittings in proximity (connectable by available parts)
        - 1 attachment in between those 2 fittings
        - 1 pipe (with correct id) in between those fittings"""

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
                    _, construction_state = self.get_construction_state(pos=pipe_pos, event_codes=[2])
                    if construction_state:
                        if construction_state.part_id == pipe_id:
                            pipe_set.add(pipe_pos)
                            break

                if not pipe_set:
                    # no pipes in between
                    continue
                # get_updated_state_on_construction_event(tentative_state, fit_diff, fit_dir, event_pos, pos)

                detour_state = get_detour_state(length=fit_diff, att_set=att_set, pipe_set=pipe_set,
                                                fit_tup=fit_tup,
                                                state_grid=self.aimed_solution.path_problem.state_grid,
                                                possible_att_pipe_positions=tuple(pipe_trail))

                return {detour_trail: detour_state}

    def handle_detour_event(self, detour_event: dict, solution: Solution):

        self.aimed_solution = solution
        self.building_instructions = get_building_instructions_from_solution(solution)
        # copy motion_dict
        motion_dict = self.motion_dict

        # copy picked parts
        picked_parts = deepcopy(self.picked_parts)

        # make a fake picked parts
        self.picked_parts = None

        # reevaluate motion dict according to new building instructions
        self.last_event_trail = self.aimed_solution.ordered_trails[0]
        for pos, construction_state in motion_dict.items():
            building_instruction, current_layout = self.get_building_instruction(
                event_pos=construction_state.event_pos, event_code=construction_state.event_code)
            event_info = EventInfo(event_pos=construction_state.event_pos, event_code=construction_state.event_code,
                                   removal=False, current_layout=None,
                                   part_id=None, deviated=False, detour_event={}, misplaced=False, unnecessary=False,
                                   obstructed_part=False, completed=False, part_not_picked=False, error=False,
                                   obstructed_obstacle=False)
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
                                    event_info=event_info, ignore_part_restriction=True)

        self.last_event_trail = detour_event.popitem()[0]

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
        if self.state_grid[pos] == 1:
            return True
        else:
            return False

    # utilities
    def set_completion_state(self, current_layout, event_info):
        """sets completion state of current layout and neighboring layouts. Updates state grid."""
        neighboring_layouts = get_neighboring_layouts(current_layout, self.aimed_solution.ordered_trails)
        neighboring_layouts.append(current_layout)

        for layout in neighboring_layouts:
            layout_state = self.building_instructions.get(layout)
            if not layout_state.completed:
                # check if required fittings placed
                for fit_pos in layout_state.required_fit_positions:
                    if None in self.get_construction_state(fit_pos, event_codes=[1]):
                        return False

                # check if required pipe placed
                if None in self.get_construction_state(layout_state.possible_att_pipe_positions, event_codes=[2]):
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
            # check if part was actually picked
            # todo: if part not in picked_parts, check if other pipes available like on else route
            if part_id != pipe_id:
                event_info.deviated = True
                event_info.part_id = True
        else:
            event_info.deviated = True
            event_info.part_id = True

        pass


