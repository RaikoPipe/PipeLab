import asyncio
import logging
from pprint import pprint

from asyncua import Client

from time import sleep

import json


class HelloClient:
    def __init__(self, endpoint):
        self.client = Client(endpoint)

    async def __aenter__(self):
        await self.client.connect()
        return self.client

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.client.disconnect()


async def send_motion_events():
    async with HelloClient("opc.tcp://localhost:4840/freeopcua/server/") as client:
        root = client.nodes.root
        # print("Root node is: ", root)
        objects = client.nodes.objects
        # print("Objects node is: ", objects)

        hellower = await objects.get_child("0:Hellower")
        # print("Hellower is: ", hellower)
        for item in motion_events:
            response = await hellower.call_method("1:SendMotionEvent", item[0], item[1], item[2])
            print('\033[92m'"ProcessPlanner Output:")
            pprint(json.loads(response))
            print("\n")
            sleep(0.5)


# motion_events = [(5, 4, 3), (1, None, 4), (5, 4, 2), (0, None, 4), (5, 3, 1), (0, None, 4), (5, 5, 1)]
motion_event_error_test = [(3,3,2)]
motion_events_detour_test = [(6, 7, 3), (2, None, 4), (6, 7, 2), (0, None, 4), (6, 6, 1), (0, None, 4), (6, 9, 1)]
motion_events_completion_test = [
    (6, 7, 3), (2, None, 4), (6, 7, 2), (0, None, 4), (6, 6, 1), (0, None, 4), (6, 9, 1),
    (6, 7, 3), (6, 7, 2), (6, 6, 1), (6, 9, 1),
    (4, 0, 3), (4, 0, 2), (3, 0, 1), (6, 0, 1),
    (6, 1, 3), (3, None, 4), (6, 1, 2), (0, None, 4), (6, 4, 1),
    (4, 4, 3), (2, 6, 3), (3, 8, 3),
    (3, None, 4), (3, None, 4), (2, None, 4),
    (5, 4, 2), (2, 5, 2), (4, 8, 2),
    (0, None, 4), (0, None, 4), (0, None, 4),
    (2, 4, 1), (2, 8, 1), (5, 8, 1),
    (5,10,3), (3,None,4), (5,10,2), (0,None,4), (5,12,1),
    (7,12,3), (3,None,4), (7,12,2), (0,None,4), (0,None,4), (9,12,1),
    (9,13,3), (2,None,4), (9,14,2), (9,15,1),
    (9,18,3), (3,None,4), (0,None,4), (9,18,2), (9,20,1),
    (7,20,3), (3,None,4), (7,20,2), (0,None,4), (5,20,1),
    (3,None,4), (5,22,3), (5,22,2), (0,None,4), (5,24,1)

]

motion_events = motion_events_completion_test

# [(0, None, 4), (0, None, 4), (0, None, 4), (0, None, 4), (2, None, 4), (4, None, 4), (3, 2, 3),
#             (3, 2, 2), (3, 0, 1), (3, 5, 1), (4, 5, 3), (4, 5, 2), (6, 5, 1)]

if __name__ == "__main__":
    # logging.basicConfig(level=logging.INFO)
    asyncio.run(send_motion_events())
