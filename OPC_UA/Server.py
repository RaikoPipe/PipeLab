import asyncio
import os
import pprint
import threading

from asyncua import ua, uamethod, Server

from FunctionTestGUI.GUI import FunctionTestGUI
from FunctionTestGUI.gui_util import update_button_grid
from PathFinding.pf_data_class.PathProblem import PathProblem
from ProcessPlanning.ProcessPlanner import ProcessPlanner
from ProcessPlanning.pp_data_class.PlacementEventInfo import PlacementEventInfo
from FunctionTestGUI.GUI import FunctionTestGUI


@uamethod
def say_hello_xml(parent, happy):
    print("Calling say_hello_xml")
    if happy:
        result = "I'm happy"
    else:
        result = "I'm not happy"
    print(result)
    return result


@uamethod
def say_hello_array(parent, happy):
    if happy:
        result = "I'm happy"
    else:
        result = "I'm not happy"
    print(result)
    return [result, "Actually I am"]


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
        if code == 4:
            output = self.process_planner.handle_motion_event((x, code))
        else:
            output = self.process_planner.handle_motion_event(((x, y), code))
        event_info: PlacementEventInfo = output.placement_event_info
        print('\033[92m' + "ProcessPlanner Output:")
        #pprint.pprint(output)
        if event_info.detour_event:
            return event_info.event_code, "Deviated from optimal solution!"
        return event_info.event_code

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
