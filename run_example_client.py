import asyncio
import logging

from asyncua import Client

from time import sleep


class HelloClient:
    def __init__(self, endpoint):
        self.client = Client(endpoint)

    async def __aenter__(self):
        await self.client.connect()
        return self.client

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.client.disconnect()


async def send_motion_event(x, y, code):
    async with HelloClient("opc.tcp://localhost:4840/freeopcua/server/") as client:
        root = client.nodes.root
        print("Root node is: ", root)
        objects = client.nodes.objects
        print("Objects node is: ", objects)

        hellower = await objects.get_child("0:Hellower")
        print("Hellower is: ", hellower)

        resulting_text = await hellower.call_method("1:SendMotionEvent", x, y, code)
        print(resulting_text)


motion_events = [(5, 4, 3), (1, None, 4), (5, 4, 2), (0, None, 4), (5, 3, 1), (0, None, 4), (5, 5, 1)]

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    for item in motion_events:
        asyncio.run(send_motion_event(item[0], item[1], item[2]))
        sleep(1)
