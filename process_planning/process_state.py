from __future__ import annotations

from copy import deepcopy
from datetime import datetime
from typing import Optional

import numpy as np

from path_finding.path_finding_util import path_math, restrictions
from path_finding.path_finding_util.path_math import get_direction, diff_pos
from path_finding.pf_data_class.solution import Solution
from process_planning.pp_data_class.assembly_event_result import AssemblyEventResult
from process_planning.pp_data_class.building_instruction import BuildingInstruction
from process_planning.pp_data_class.construction_state import ConstructionState
from process_planning.pp_data_class.pick_event_result import PickEventResult
from process_planning.process_util.ps_util import get_neighboring_layouts, construct_trail, \
    construct_detour_building_instruction, construct_building_instructions_from_solution, get_completion_proportion
from type_dictionary import constants
from type_dictionary.class_types import MotionDict, BuildingInstructions
from type_dictionary.type_aliases import *


class ProcessState:
    """Data class that contains information about a process state.

    Args:

        solution(:class:`Solution<solution>`): Solution to the path problem that is the basis for the assembly process.

    :ivar state_grid: :obj:`~type_aliases.StateGrid` that shows where obstacles and completed layouts occupy nodes

    :ivar aimed_solution: :class:`~Solution` that contains information of the desired construction. Is recalculated
        after a detour event.

    :ivar last_event_trail: :obj:`~type_aliases.Trail` at which the last non deviated event occurred.

    :ivar part_stock: See :obj:`~type_aliases.PartStock`

    :ivar picked_parts: :obj:`list` [:obj:`int`] containing
        part IDs picked from stock. Parts will be removed upon assembly.

    :ivar building_instructions: (See :obj:`~class_types.BuildingInstructions`) Dictionary containing information
        about the required assembly and IDs of parts to complete the construction process. Is recalculated after a
        detour event.

    :ivar motion_dict: Dictionary (See :obj:`~class_types.MotionDict`) containing all registered motion events that are
        valid (non-error) that point to a ConstructionState. The keys for pipe events are of the type
        :obj:`~type_aliases.Trail`, while other
        events are of the type :obj:`~type_aliases.Pos`.

    :ivar last_event_info: Dictionary (See :obj:`~class_types.BuildingInstructions`) containing information of the last
        event that occurred."""

    def __init__(self, solution: Solution):
        self.state_grid = deepcopy(solution.path_problem.state_grid)
        self.aimed_solution = solution
        self.last_event_trail = solution.ordered_trails[0]

        self.part_stock = solution.path_problem.part_stock
        self.picked_parts: list[int] = []
        self.building_instructions = construct_building_instructions_from_solution(solution)

        self.motion_dict: MotionDict = {}

        self.last_assembly_event_result = None
        self.last_pick_event_result = None

        self.detour_trails = []

        self.event_history = []

        self.completion = 0

    def get_construction_state(self, find_me: Union[Pos, Trail], event_codes: list) -> Union[
        tuple[Pos, ConstructionState], tuple[Trail, ConstructionState], tuple[None, None]]:
        """returns construction state and pos/trail if pos and event_code specified are in motion_dict.

        Args: find_me (:obj:`Union` [:obj:`~type_aliases.Pos`, :obj:`~type_aliases.Trail`]): Position or trail to
        find in motion_dict. event_codes (:obj:`list` [:obj:`int`]): List containing event codes valid for search.

        Returns:
            If nothing was found, returns a :obj:`tuple` containing :obj:`None`, otherwise returns a :obj:`tuple`
            containing a :obj:`~type_aliases.Pos` or :obj:`~type_aliases.Trail` and its
            :class:`ConstructionState<construction_state>`"""

        if 2 in event_codes:
            for trail, construction_state in self.motion_dict.items():
                if (find_me in trail or find_me == trail) and construction_state.event_code == 2:
                    return trail, construction_state

        else:
            construction_state = self.motion_dict.get(find_me)
            if construction_state:
                if construction_state.event_code in event_codes:
                    return find_me, construction_state

        return None, None

    # events
    def pick_part(self, event_code: int, part_id: int, ignore_empty_stock: bool) -> PickEventResult:
        """Evaluates the pick event in the context of the current process state and registers changes to the
        :obj:`~type_aliases.PartStock` and other class instance variables if the action was valid.

        Args:
            event_code(:obj:`int`): See :paramref:`~evaluate_assembly.event_code`
            part_id (:obj:`int`): Part ID that was picked.
            ignore_empty_stock (:obj:`bool`): See
                :paramref:`~process_planner.ProcessPlanner.handle_motion_event.ignore_empty_stock`

        Returns:
            :class:`PickEventResult<pick_event_result>`"""
        error = False
        part_not_available = False
        stock_empty = set()
        if self.part_stock[part_id] > 0:

            self.picked_parts.append(part_id)
            self.part_stock[part_id] -= 1
        elif not ignore_empty_stock:
            error = True
            part_not_available = True

        for p_id, amount in self.part_stock.items():
            if amount < 1:
                stock_empty.add(p_id)
        return PickEventResult(part_id=part_id, time_registered=datetime.now(), stock_empty=stock_empty, error=error,
                               part_not_available=part_not_available, event_code=event_code)

    def evaluate_assembly(self, event_pos: Pos, event_code: int,
                          ignore_part_check: bool = False, ignore_obstructions: bool = False,
                          assume_pipe_id_from_solution: bool = False) -> AssemblyEventResult:
        """Evaluates the assembly event in the context of the current process state and registers changes to the
        :obj:`~class_types.MotionDict` if the action was valid.
        Also registers if a :obj:`building_instruction.BuildingInstruction` has been fulfilled.

        Args:
            event_pos (:obj:`~type_aliases.pos`): :obj:`~type_aliases.pos` where :obj:`motion event<type_aliases.MotionEvent>` occurred.
            event_code (:obj:`int`): Event code of :obj:`motion event<type_aliases.MotionEvent>` (See :ref:`Encoding Information`)
            ignore_part_check (:obj:`bool`): Option for ignoring part restrictions.
             See :paramref:`~process_planner.ProcessPlanner.handle_motion_event.ignore_part_check`
            ignore_obstructions (:obj:`bool`): Option for ignoring obstructions.
             See :paramref:`~process_planner.ProcessPlanner.handle_motion_event.ignore_obstructions`
            assume_pipe_id_from_solution (:obj:`bool`): Option for assuming a certain pipe ID has been placed, if motion
             event occured on solution and pipe ID is among the parts that have been picked.

        Returns:
            :class:`AssemblyEventResult <assembly_event_result>`
            """

        part_id = None
        current_layout = None

        # possible event outcomes:
        removal = False
        completed_layouts = set()

        deviated = False
        misplaced = False
        unnecessary = False
        error = False
        part_not_picked = False
        obstructed_obstacle = False
        obstructed_part = False

        detour_event = {}

        time_registered = datetime.now()

        event_result = AssemblyEventResult(event_pos=event_pos, event_code=event_code, part_id=part_id,
                                           obstructed_part=obstructed_part, obstructed_obstacle=obstructed_obstacle,
                                           deviated=deviated, misplaced=misplaced, unnecessary=unnecessary,
                                           completed_layouts=completed_layouts,
                                           layout=current_layout, removal=removal, part_not_picked=part_not_picked,
                                           detour_event=detour_event, error=error, time_registered=time_registered,
                                           )

        construction_state = ConstructionState(event_pos=event_pos, event_code=event_code,
                                               part_id=part_id, deviated=deviated,
                                               misplaced=misplaced, unnecessary=unnecessary,
                                               time_registered=time_registered)

        # get current layout and building instruction
        building_instruction, current_layout, layout_changed = self.get_current_layout_and_building_instruction(
            event_code, event_pos, )
        event_result.layout = current_layout

        # event evaluation

        if self.part_removed(event_pos, event_code, event_result):
            if not event_result.deviated:
                event_result.completed_layouts.update(self.set_completion_state(current_layout, event_result))
            # return part to picked_parts
            if event_result.part_id in self.part_stock:
                self.picked_parts.append(event_result.part_id)
            return event_result
        if not ignore_obstructions:
            if self.obstructed_obstacle(event_pos):
                event_result.error = event_result.obstructed_obstacle = True
                return event_result

            event_result.obstructed_part = self.obstructed_part(event_pos, event_code)
            if event_result.obstructed_part:
                event_result.error = True
                return event_result

        # evaluate assembly and modify event result accordingly

        if event_code == 3:
            self.attachment_placed(event_pos=event_pos, event_code=event_code, event_result=event_result,
                                   building_instruction=building_instruction)

        elif event_code == 2:
            self.pipe_placed(event_pos=event_pos, event_result=event_result,
                             building_instruction=building_instruction, ignore_part_check=ignore_part_check,
                             assume_pipe_id_from_solution=assume_pipe_id_from_solution)

        elif event_code == 1:
            self.fitting_placed(event_pos=event_pos, building_instruction=building_instruction,
                                event_result=event_result, ignore_part_check=ignore_part_check)

        if not event_result.error:
            # register assembly
            construction_state.part_id = event_result.part_id
            construction_state.deviated = event_result.deviated
            construction_state.unnecessary = event_result.unnecessary
            construction_state.misplaced = event_result.misplaced
            self.register_assembly(building_instruction, construction_state, event_code,
                                   event_pos)

            if not event_result.deviated:

                if not assume_pipe_id_from_solution:
                    # check if pipe ids can be assumed based on placed fittings
                    self.check_assignment_pipe_id(current_layout)
                # check completion state of this and neighboring layouts
                event_result.completed_layouts.update(self.set_completion_state(current_layout, event_result))
                self.completion = get_completion_proportion(self.building_instructions)

            # else:
            if event_code == 1:
                # check for detour events
                detour_event = self.get_detour_event(event_pos=event_pos, event_code=event_code)
                event_result.detour_event = detour_event

            # modify parts_picked
            if event_result.part_id in self.part_stock.keys():
                self.picked_parts.remove(event_result.part_id)

        return event_result

    def get_current_layout_and_building_instruction(self, event_code: int, event_pos: Pos) -> tuple[
        BuildingInstruction, Trail, bool]:
        """Returns the current layout and :class:`BuildingInstruction<building_instruction>` if event occurred inside solution.

        Args:
            event_pos(:obj:`~type_aliases.Pos`): See :paramref:`~evaluate_assembly.event_pos`
            event_code(:obj:`int`): See :paramref:`~evaluate_assembly.event_code`

        Returns:
            :obj:`tuple` [:class:`BuildingInstruction<building_instruction>`, :obj:`~type_aliases.Trail`, :obj:`bool`] containing the
            current layout (:obj:`~type_aliases.Trail`), :class:`BuildingInstruction<building_instruction>` and a value that signifies if the layout has changed from the previous
            one. """

        current_layout = self.last_event_trail
        layout_changed = False

        if event_pos in self.aimed_solution.node_trail.keys():

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

            building_instruction = self.building_instructions.get(current_layout)
        else:
            current_layout = None
            building_instruction = None

        return building_instruction, current_layout, layout_changed

    def register_assembly(self, building_instruction: BuildingInstruction, construction_state: ConstructionState,
                          event_code: int,
                          event_pos: Pos):
        """Registers assembly event to motion_dict.

        Args:

            event_pos(:obj:`~type_aliases.Pos`): See :paramref:`~evaluate_assembly.event_pos`
            event_code(:obj:`int`): See :paramref:`~evaluate_assembly.event_code`
            building_instruction(:class:`BuildingInstruction<building_instruction>`): The current building instruction.
            construction_state(:class:`ConstructionState<construction_state>`): Construction state of the current motion event.

        """
        # make construction state
        if event_code == 2:
            if not construction_state.deviated and construction_state.part_id != -2:
                self.motion_dict[building_instruction.possible_att_pipe_positions] = construction_state
            else:
                self.motion_dict[(event_pos,)] = construction_state
        else:
            self.motion_dict[event_pos] = construction_state

    def part_removed(self, event_pos: Pos, event_code: int, event_result: AssemblyEventResult) \
            -> Optional[
                tuple[Pos, ConstructionState]]:
        """Removes event_pos from the motion dict, if already in motion dict.

        Args:
            event_result(:class:`assembly_event_result.AssemblyEventResult`): Event result being modified.
            event_pos(:obj:`~type_aliases.Pos`): See :paramref:`~evaluate_assembly.event_pos`
            event_code(:obj:`int`): See :paramref:`~evaluate_assembly.event_code`

        Returns:

            :obj:`tuple` [:obj:`~type_aliases.Pos`, :class:`ConstructionState<construction_state>`] containing the :obj:`~type_aliases.Pos` and :class:`ConstructionState<construction_state>`
            of the motion event of the part that was removed, else :obj:`None`."""
        pos, construction_state = self.get_construction_state(find_me=event_pos, event_codes=[event_code])

        if construction_state:
            # removal confirmed
            event_result.deviated = construction_state.deviated
            event_result.unnecessary = construction_state.unnecessary
            event_result.misplaced = construction_state.misplaced
            event_result.part_id = construction_state.part_id
            event_result.removal = True
            self.motion_dict.pop(pos)

            return pos, construction_state

        return None

    def attachment_placed(self, event_pos: Pos, event_code: int, building_instruction: BuildingInstruction,
                          event_result: AssemblyEventResult):
        """Evaluates the assembly of an attachment. Notes findings to event_info.

        Args:
            event_result(:class:`AssemblyEventResult <assembly_event_result>`): Event info being modified.
            building_instruction(:class:`BuildingInstruction<building_instruction>`): Building instruction of the current layout.
            event_pos(:obj:`~type_aliases.Pos`): See :paramref:`~evaluate_assembly.event_pos`
            event_code(:obj:`int`): See :paramref:`~evaluate_assembly.event_code`"""
        # todo: check recommended att_pos
        if event_pos in self.aimed_solution.node_trail.keys():
            if event_pos not in building_instruction.required_fit_positions:
                if event_pos in building_instruction.possible_att_pipe_positions:
                    for pos in building_instruction.possible_att_pipe_positions:
                        _, construction_state = self.get_construction_state(find_me=pos, event_codes=[event_code])
                        if construction_state:
                            event_result.unnecessary = True
                            event_result.deviated = True

            else:
                event_result.misplaced = True
                event_result.deviated = True
        else:
            event_result.deviated = True
        event_result.part_id = -1

    def pipe_placed(self, event_pos, building_instruction: BuildingInstruction, event_result, ignore_part_check,
                    assume_pipe_id_from_solution):
        """Evaluates the assembly of a pipe. Notes findings in event_info.

        Args:
            event_pos(:obj:`~type_aliases.Pos`): See parameter :paramref:`~evaluate_assembly.event_pos`
            event_result(:class:`AssemblyEventResult <assembly_event_result>`): Event result being modified
            building_instruction(:class:`BuildingInstruction<building_instruction>`): Building instruction of the current layout.
            ignore_part_check (:obj:`bool`): Option for ignoring part restrictions.
             See :paramref:`~process_planner.ProcessPlanner.main.ignore_part_check`
            assume_pipe_id_from_solution(:obj:`bool`): See :paramref:`~process_state.ProcessState.evaluate_assembly.assume_pipe_id_from_solution`

            """
        if event_pos in self.aimed_solution.node_trail.keys():
            if event_pos in building_instruction.possible_att_pipe_positions:
                part_id = building_instruction.pipe_id
                if not assume_pipe_id_from_solution:
                    pipe_id = self.pipe_placed_deviated_route(event_result, ignore_part_check)  #
                    if pipe_id != part_id:
                        event_result.deviated = True
                else:
                    # if pipe assembly occurred somewhere inside the trail of building instruction, then it's valid
                    event_result.part_id = part_id
                    # check if part was actually picked
                    if part_id not in self.picked_parts and not ignore_part_check:
                        event_result.deviated = True
                        self.pipe_placed_deviated_route(event_result, ignore_part_check)
            else:
                event_result.deviated = True
                event_result.misplaced = True
        else:
            event_result.deviated = True
            self.pipe_placed_deviated_route(event_result, ignore_part_check)

    def pipe_placed_deviated_route(self, event_result, ignore_part_check) -> int:
        """
        Pipe assembly evaluation if either motion event occurred outside solution or if :paramref:`~process_state.ProcessState.evaluate_assembly.assume_pipe_id_from_solution` has been set to True in :meth:`~process_state.ProcessState.evaluate_assembly`.


        Args:

            event_result(:class:`AssemblyEventResult <assembly_event_result>`): Event info being modified.
            ignore_part_check (:obj:`bool`): Option for ignoring part restrictions.
                 See :paramref:`~process_planner.ProcessPlanner.main.ignore_part_check`

        Returns:
            Event part ID (:obj:`int`)



        """
        picked_parts = self.picked_parts
        # find the part id
        picked_pipes = [i for i in picked_parts if i != 0]
        if len(set(picked_pipes)) == 1:
            # if there is only one of a kind of pipe in picked_parts, then we know which id was placed
            event_result.part_id = picked_pipes[0]
        elif len(set(picked_parts)) > 1 or ignore_part_check:
            if self.get_num_open_pipes() < len(picked_pipes) or ignore_part_check:
                event_result.part_id = -2
            # multiple kinds of pipe picked
            else:
                # number of unknown pipes can't be greater than pipes picked
                event_result.part_id = -99
                event_result.error = event_result.part_not_picked = True


        else:
            event_result.part_id = -99
            event_result.error = event_result.part_not_picked = True

        return event_result.part_id

    def fitting_placed(self, event_pos, building_instruction, event_result, ignore_part_check):
        """Evaluates the assembly of a fitting. Notes findings in event_info.

        Args:
            event_pos(:obj:`~type_aliases.Pos`): See parameter :paramref:`~evaluate_assembly.event_pos`
            event_result(:class:`AssemblyEventResult <assembly_event_result>`): Event info being modified
            building_instruction(:class:`BuildingInstruction<building_instruction>`): Building instruction of the current layout.
            ignore_part_check (:obj:`bool`): Option for ignoring part restrictions.
             See :paramref:`~process_planner.ProcessPlanner.main.ignore_part_check`

         """
        if 0 in self.picked_parts or ignore_part_check:
            event_result.part_id = 0
            if event_pos in self.aimed_solution.node_trail.keys():

                if event_pos not in building_instruction.required_fit_positions:
                    event_result.misplaced = True
                    event_result.deviated = True
            else:
                event_result.deviated = True
        else:
            # ERROR: Part not picked!
            event_result.error = event_result.part_not_picked = True
            event_result.part_id = -99

    def check_assignment_pipe_id(self, current_layout: Trail):
        """Identifies and assigns IDs to motion events with unidentified pipe IDs based on placed fittings.

        Args:
            current_layout(:obj:`~type_aliases.Trail`): The current layout.

        """
        neighboring_layouts = get_neighboring_layouts(current_layout, self.aimed_solution.ordered_trails)
        neighboring_layouts.add(current_layout)

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
                if None not in self.get_construction_state(find_me=pos, event_codes=[2]):
                    pipe_set.add(pos)

            if len(pipe_set) == 1:
                pipe_pos = pipe_set.pop()
                _, pipe_state = self.get_construction_state(find_me=pipe_pos, event_codes=[2])
                if self.assign_pipe_id(pipe_state, check_building_instruction.pipe_id):
                    # reassign pipe id
                    if check_building_instruction.pipe_id in self.picked_parts:
                        self.picked_parts.remove(check_building_instruction.pipe_id)
                    else:
                        continue
                    self.motion_dict.pop((pipe_pos,))
                    pipe_state.deviated = False
                    self.register_assembly(building_instruction=check_building_instruction,
                                           construction_state=pipe_state,
                                           event_pos=pipe_pos, event_code=2)

    def assign_pipe_id(self, pipe_state, pipe_id):

        if pipe_state.part_id == -2:
            # check picked parts
            if pipe_id in self.picked_parts:
                pipe_state.part_id = pipe_id

                return pipe_id

    @staticmethod
    def transition_valid(pos, other_pos, start_pos):
        transition_direction = get_direction(diff_pos(pos, other_pos))
        check_start_pos = (abs(start_pos[0] * transition_direction[0]), abs(start_pos[1] * transition_direction[1]))
        check_t_pos = (abs(pos[0] * transition_direction[0]), abs(pos[1] * transition_direction[1]))
        if transition_direction != get_direction(diff_pos(check_t_pos, check_start_pos)):
            return True

    def get_detour_event(self, event_pos, event_code) -> BuildingInstructions:
        """Checks if conditions for a detour event are met.
        Detour event occurs if:\n
        1. 2 fittings in proximity (connectable by available parts)\n
        2. 1 attachment in between those 2 fittings\n
        3. 1 pipe (with correct or open id) in between those fittings

        Args:
            event_pos (:obj:`~type_aliases.Pos`): See :paramref:`~evaluate_assembly.event_pos`
            event_code (:obj:`int`): See :paramref:`~evaluate_assembly.event_code`

        Return: :obj:`~class_types.BuildingInstructions` containing the detour event.
        """

        check_fit_set = self.get_positions_from_motion_dict(constants.fit_event_code)
        check_fit_set.remove(event_pos)
        check_fit_set = self.remove_already_connected_fittings(check_fit_set)

        for pos in check_fit_set:

            # check if conditions for a deviation event are met
            fit_diff = path_math.get_length_same_axis(pos,
                                                      event_pos)  # distance between fittings
            # at least one fitting needs to be deviating
            fit_tup = (pos, event_pos)
            fit_deviated = []

            for fit_pos in fit_tup:
                _, fit_state = self.get_construction_state(fit_pos, event_codes=[constants.fit_event_code])
                fit_deviated.append(fit_state.deviated)

            if not any(fit_deviated):  # at least one fitting needs to be deviating
                continue

            fit_dir = get_direction(diff_pos(pos, event_pos))

            # check for transition points
            state_grid = self.aimed_solution.path_problem.state_grid
            iter_pos = deepcopy(pos)
            pos_list = []

            obstructed = False
            while iter_pos != event_pos:
                iter_pos = path_math.sum_pos(iter_pos, fit_dir)
                if state_grid[iter_pos] == constants.obstacle_state:
                    obstructed = True
                    break
                pos_list.append(iter_pos)

            if obstructed:
                continue

            start_pos = self.aimed_solution.path_problem.start_pos

            if len(pos_list) > 2:
                if state_grid[pos_list[0]] == 3:
                    if self.transition_valid(pos, pos_list[0], start_pos):
                        pipe_id = fit_diff - 2
                    else:
                        continue
                elif state_grid[pos_list[-2]] == 3:
                    if self.transition_valid(pos_list[-1],pos_list[-2], start_pos):
                        pipe_id = fit_diff - 2
                    else:
                        continue
                else:
                    pipe_id = fit_diff - 1  # length of needed part
            else:
                pipe_id = fit_diff - 1

            # for transition_point in self.aimed_solution.path_problem.transition_points:
            #     check_idx = 0
            #     if transition_point[0] < 0:
            #         check_idx = 1
            #
            #     check_pos_1 = (pos[0] + fit_dir[0], pos[1] + fit_dir[1])
            #     check_pos_2 = (event_pos[0] - fit_dir[0], event_pos[1] - fit_dir[1])
            #
            #     if check_pos_1[check_idx] == transition_point[check_idx] ^ check_pos_2[check_idx] == transition_point[check_idx]:
            #         # One of the fitting positions is directly next to a transition point

            if pipe_id == 0:
                # fittings are directly next to each other
                break

            fittings_in_proximity = pipe_id in self.part_stock.keys()  # fittings are connectable by available parts

            if not fittings_in_proximity:
                continue

            detour_trail = construct_trail(length=fit_diff, direction=fit_dir, pos=fit_tup[0])

            attachment_is_between = False

            att_set = set()
            pipe_set = set()

            pipe_trail = [i for i in detour_trail if i not in fit_tup]
            for att_pos in self.get_positions_from_motion_dict(constants.att_event_code):
                if att_pos in pipe_trail:
                    att_set.add(att_pos)

            if not att_set:
                # no attachments in between
                continue

            for pipe_pos in pipe_trail:
                construction_state = self.get_construction_state(find_me=pipe_pos, event_codes=[2])
                if None not in construction_state:
                    if construction_state[1].deviated:
                        pipe_set.add(pipe_pos)

            # pipes are blocking each other
            if len(pipe_set) != 1:
                continue

            # get_updated_state_on_construction_event(tentative_state, fit_diff, fit_dir, event_pos, pos)

            detour_instruction = construct_detour_building_instruction(pipe_id, fit_tup=fit_tup,
                                                                       state_grid=self.aimed_solution.path_problem.state_grid,
                                                                       possible_att_pipe_positions=tuple(
                                                                           pipe_trail))

            pipe_pos = pipe_set.pop()
            _, pipe_state = self.get_construction_state(find_me=pipe_pos, event_codes=[2])
            if pipe_state.part_id == -2:
                # assign part id
                if pipe_id in self.picked_parts:
                    pipe_state.part_id = pipe_id
                    self.picked_parts.remove(pipe_id)
                else:
                    continue

                self.motion_dict.pop((pipe_pos,))
                self.register_assembly(building_instruction=detour_instruction, construction_state=pipe_state,
                                       event_pos=pipe_pos, event_code=2)
            elif pipe_state.part_id != pipe_id:
                continue

            return {detour_trail: detour_instruction}

    def reevaluate_motion_dict_from_solution(self, solution: Solution, detour_event: BuildingInstructions = None):
        """Reevaluates all entries in the motion dict according to the given solution.

        Args:
            solution(:class:`Solution<solution>`): New solution.
            detour_event(:obj:`~class_types.BuildingInstructions`): Cause of the detour event.
        Returns:
            :obj:`~type_aliases.Trail` with :obj:`~type_aliases_pos` in the order according to the solution.
        """
        self.aimed_solution = solution
        self.building_instructions = construct_building_instructions_from_solution(solution)
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

            event_result = AssemblyEventResult(event_pos=construction_state.event_pos,
                                               event_code=construction_state.event_code,
                                               removal=False, layout=None,
                                               part_id=None, deviated=deviated, detour_event={}, misplaced=misplaced,
                                               unnecessary=unnecessary,
                                               obstructed_part=False, completed_layouts=completed_layouts,
                                               part_not_picked=False,
                                               error=False,
                                               obstructed_obstacle=False,
                                               time_registered=construction_state.time_registered,
                                               )

            new_construction_state = ConstructionState(event_pos=construction_state.event_pos,
                                                       event_code=construction_state.event_code,
                                                       part_id=construction_state.part_id, deviated=deviated,
                                                       unnecessary=unnecessary, misplaced=misplaced,
                                                       time_registered=construction_state.time_registered)

            if construction_state.event_code == 3:
                self.attachment_placed(event_pos=construction_state.event_pos, event_code=construction_state.event_code,
                                       event_result=event_result,
                                       building_instruction=building_instruction)

            elif construction_state.event_code == 2:
                self.pipe_placed_detour_event(event_pos=construction_state.event_pos, event_result=event_result,
                                              building_instruction=building_instruction,
                                              pipe_id=construction_state.part_id)

            elif construction_state.event_code == 1:
                self.fitting_placed(event_pos=construction_state.event_pos, building_instruction=building_instruction,
                                    event_result=event_result, ignore_part_check=True)

            new_construction_state.part_id = event_result.part_id
            new_construction_state.deviated = event_result.deviated
            new_construction_state.unnecessary = event_result.unnecessary
            new_construction_state.misplaced = event_result.misplaced

            self.register_assembly(building_instruction=building_instruction,
                                   construction_state=new_construction_state,
                                   event_code=construction_state.event_code,
                                   event_pos=construction_state.event_pos)

            if not event_result.deviated:
                event_result.completed_layouts.update(self.set_completion_state(current_layout, event_result))

        if detour_event:
            # find detour trail in ordered trails
            ordered_pos_set: OrderedPos = tuple([set(i) for i in self.aimed_solution.ordered_trails])
            idx = ordered_pos_set.index(set(tuple(detour_event.keys())[0]))
            self.last_event_trail = self.aimed_solution.ordered_trails[idx]
            # self.last_event_trail = list(detour_event.keys())[0]
        else:
            self.last_event_trail = self.aimed_solution.ordered_trails[0]

        return self.last_event_trail

    # restriction checks

    def obstructed_part(self, event_pos: Pos, event_code: int) -> Optional[int]:
        """Checks if motion event obstructed an already placed part.

        Args:

            event_pos(:obj:`~type_aliases.Pos`): See parameter :paramref:`~evaluate_assembly.event_pos`
            event_code(:obj:`int`): See parameter :paramref:`~evaluate_assembly.event_code`

        Returns:
            Event code (:obj:`int`) of part that was obstructed or :obj:`None` if no part was obstructed."""
        pos, construction_state = self.get_construction_state(find_me=event_pos, event_codes=[2])
        if construction_state and event_code != 2:
            # obstructed pipe
            return construction_state.event_code
        else:
            pos, construction_state = self.get_construction_state(find_me=event_pos, event_codes=[1, 3])
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

    def obstructed_obstacle(self, event_pos: Pos) -> bool:
        """Checks state grid if motion event obstructed obstacle.

        Args:
            event_pos(:obj:`~type_aliases.Pos`): See parameter :paramref:`~evaluate_assembly.event_pos`

        Returns:
            :obj:`bool` if motion event obstructed obstacle.
            """
        if self.state_grid[event_pos] == 1 or self.state_grid[event_pos] == 3:
            return True
        else:
            return False

    # utilities
    def set_completion_state(self, current_layout: Trail, event_result: AssemblyEventResult) -> set:
        """Sets completion state of current layout and neighboring layouts. Updates state grid on completed layouts.

        Args:
            event_result(:class:`assembly_event_result.AssemblyEventResult`):
            current_layout(:obj:`Trail`):

        """
        neighboring_layouts = get_neighboring_layouts(current_layout, self.aimed_solution.ordered_trails)
        neighboring_layouts.add(current_layout)
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
                if event_result.removal:
                    building_instruction.layout_completed = False
                    for pos in layout:
                        self.state_grid[pos] = 0

        return completed_layouts

    def completed_instruction(self, building_instruction: BuildingInstruction) -> tuple[bool, int]:
        """Checks if the building instruction was completed. Additionally, returns the next event code needed to advance
        the completion of this instruction.

        Args:
            building_instruction(:class:`BuildingInstruction<building_instruction>`): Building instruction of the current layout.

        Returns:
        :obj:`tuple` containing if building instruction was completed (:obj:`bool`) and event code (:obj:`int`)
        corresponding to the next event code needed to advance the completion of this instruction.
            """

        # check if required pipe placed
        pipe_positions = building_instruction.possible_att_pipe_positions

        att_pos_set = set()
        for pos in building_instruction.possible_att_pipe_positions:
            att_pos, att_state = self.get_construction_state(pos, event_codes=[constants.att_event_code])
            if att_state:
                if not att_state.deviated:
                    att_pos_set.add(att_pos)

        if not att_pos_set:
            return False, constants.att_event_code

        if None in self.get_construction_state(pipe_positions, event_codes=[constants.pipe_event_code]):
            pipe_set = set()
            for pos in pipe_positions:
                pipe_pos, pipe_state = self.get_construction_state(pos, event_codes=[constants.pipe_event_code])
                if pipe_state:
                    pipe_set.add(pipe_pos)
            if not pipe_set:
                return False, constants.pipe_event_code

        # check if required fitting placed
        for fit_pos in building_instruction.required_fit_positions:
            if None in self.get_construction_state(fit_pos, event_codes=[constants.fit_event_code]):
                return False, constants.fit_event_code

        return True, 0

    def get_positions_from_motion_dict(self, event_code):
        """
        Returns:
            :obj:`set` containing all positions for given event_code in the motion dictionary.
        """
        pos_set = set()
        for pos, construction_state in self.motion_dict.items():
            if construction_state.event_code == event_code:
                pos_set.add(pos)
        return pos_set

    def pipe_placed_detour_event(self, event_pos: Pos, event_result: AssemblyEventResult,
                                 building_instruction: BuildingInstruction, pipe_id: int) -> AssemblyEventResult:
        """Pipe assembly reevaluation for detour events. Similar to :meth:`pipe_placed`.

        Args:
            event_pos(:obj:`~type_aliases.Pos`): See parameter :paramref:`~evaluate_assembly.event_pos`
            event_result(:class:`AssemblyEventResult <assembly_event_result>`): Event info being modified
            building_instruction(:class:`BuildingInstruction<building_instruction>`): Building instruction of the current layout.
            pipe_id(:obj:`int`): Pipe ID of the construction state being reevaluated.
        Return:


        """
        if event_pos in self.aimed_solution.node_trail.keys():
            if event_pos in building_instruction.possible_att_pipe_positions:
                part_id = building_instruction.pipe_id
                event_result.part_id = pipe_id
                # check if part was actually picked
                if part_id != pipe_id:
                    event_result.deviated = True
            else:
                event_result.deviated = True
                event_result.misplaced = True
        else:
            event_result.part_id = pipe_id
            event_result.deviated = True
        return event_result

    def get_num_open_pipes(self) -> int:
        """

        Returns:
            Amount of pipes (:obj:`int`) with open ID in motion dictionary.

        """

        counter = 0
        for pos, construction_state in self.motion_dict.items():
            if construction_state.event_code == 2:
                if construction_state.part_id == -2:
                    counter += 1
        return counter

    def get_completed_instructions(self) -> BuildingInstructions:
        """Looks for completed instructions inside building_instructions and returns them.

        Args:
            building_instructions(:obj:`~class_types.BuildingInstructions`): See :paramref:`~process_state.ProcessState.building_instructions`.

        Return:
            All completed instructions (:obj:`~class_types.BuildingInstructions`)

        """

        completed_instructions = {}
        for layout_trail in self.building_instructions.keys():
            instruction = self.building_instructions[layout_trail]
            if instruction.layout_completed:
                completed_instructions[layout_trail] = instruction

        return completed_instructions

    def remove_already_connected_fittings(self, check_fit_set) -> set[Pos]:
        completed_instructions_dict = self.get_completed_instructions()
        start_pos = self.aimed_solution.path_problem.start_pos
        for trail, building_instruction in completed_instructions_dict.items():
            fit_set = set(building_instruction.required_fit_positions)
            fit_remove = set()
            for fit_pos in fit_set:
                counter = 0
                for direction in constants.valid_directions:
                    self.state_grid : np.ndarray
                    check_pos = path_math.sum_pos(fit_pos,direction)
                    if not restrictions.out_of_bounds(check_pos, self.state_grid):
                        if self.state_grid[path_math.sum_pos(fit_pos,direction)] == 2:
                            counter += 1
                if counter >= 2 or (fit_pos==start_pos and counter ==1):
                    fit_remove.add(fit_pos)

            check_fit_set = check_fit_set.difference(fit_remove)


        return check_fit_set



