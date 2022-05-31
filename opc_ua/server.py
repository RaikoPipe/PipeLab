import asyncio
import os
from typing import Union

from asyncua import ua, uamethod, Server

from process_planning.pp_data_class.pick_event_result import PickEventResult
from process_planning.pp_data_class.assembly_event_result import AssemblyEventResult
from process_planning.pp_data_class.process_output import ProcessOutput
from process_planning.process_planner import ProcessPlanner
from type_dictionary import constants
import json


class PipeLabServer:
    def __init__(self, endpoint, name, model_filepath, process_planner):
        self.server = Server()
        self.model_filepath = model_filepath
        self.server.set_server_name(name)
        self.server.set_endpoint(endpoint)
        self.process_planner = process_planner

    async def init(self):
        await self.server.init()
        #  This need to be imported at the start or else it will overwrite the data
        await self.server.import_xml(self.model_filepath)

        objects = self.server.nodes.objects

        freeopcua_namespace = await self.server.get_namespace_index(
            "urn:freeopcua:python:server"
        )
        hellower = await objects.get_child("0:Hellower")

        await hellower.add_method(
            freeopcua_namespace,
            "SendMotionEvent",
            self.send_motion_event,
            [ua.VariantType.Boolean],
            [ua.VariantType.Int64],
            [ua.VariantType.Int64],
            [ua.VariantType.Int64]
        )

    @uamethod
    def send_motion_event(self, parent, x, y, code):
        """send_motion_event()
        Receives a motion event from the client and sends it to the process planner.
        Returns a JSON formatted string with the returned event result.

        Args:
            x(:obj:`int`): x-coordinate of the motion event position or a part id
            y(:obj:`int`): y-coordinate of the motion event position
            code(:obj:`int`): Motion event code (See :ref:`Motion Event Codes`)
            """
        try:
            if code in {constants.pick_manual_event_code, constants.pick_robot_event_code}:
                output: ProcessOutput = self.process_planner.handle_motion_event((x, code))
            elif code in {constants.fit_event_code, constants.pipe_event_code, constants.att_event_code}:
                output = self.process_planner.handle_motion_event(((x, y), code))
            else:
                return f"Event code {code} not recognized!"

            event_result: Union[AssemblyEventResult, PickEventResult] = output.current_event_result
            # pprint.pprint(output)
            response = {"eventCode": event_result.event_code, "timeRegistered": str(event_result.time_registered),
                        "error": event_result.error}

            if isinstance(event_result, AssemblyEventResult):
                event_result: AssemblyEventResult
                response.update({"AssemblyEvent" : {"obstructedPart": event_result.obstructed_part,
                                 "obstructedObstacle": event_result.obstructed_obstacle,
                                 "deviated": event_result.deviated, "misplaced": event_result.misplaced,
                                 "unnecessary": event_result.unnecessary, "message1": output.messages[0],
                                 "message2": output.messages[1] if len(
                                     output.messages) > 1 else None}})

            elif isinstance(event_result, PickEventResult):
                event_result: PickEventResult
                response.update({"PickEvent": {"partNotAvailable": event_result.part_not_available,
                                               "partID" : event_result.part_id}})
            response = json.dumps(response)
            return response

        except BaseException as e:
            print(e)
            return str(e)

    async def __aenter__(self):
        await self.init()
        await self.server.start()
        return self.server

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.server.stop()


async def main(process_planner: ProcessPlanner):
    script_dir = os.path.dirname(__file__)
    async with PipeLabServer(
            "opc.tcp://0.0.0.0:4840/freeopcua/server/",
            "FreeOpcUa Example Server",
            os.path.join(script_dir, "test_saying.xml"),
            process_planner
    ) as server:
        while True:
            await asyncio.sleep(1)
