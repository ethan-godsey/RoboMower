import websockets
import asyncio

async def main():
    async with websockets.connect("ws://172.20.10.7:3000") as websocket:
        while True:
            message = await websocket.recv()
            print(message)

asyncio.run(main())
