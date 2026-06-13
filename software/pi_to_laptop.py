import asyncio
import websockets
import json

CONN = set()
IP = "0.0.0.0"

async def handler(websocket):
    # add connection from client
    CONN.add(websocket)
    print("connection made!")
    try:
        # just sit here until client disconnects, and remove this connection from q
        await websocket.wait_closed()  
    finally:
        CONN.remove(websocket)


async def broadcast(queue):
    while True:
        # create blocking call to get data from queue
        scan = await asyncio.get_event_loop().run_in_executor(None, queue.get)
        data = json.dumps(scan)
        
        # send it out to listening clients
        websockets.broadcast(CONN, data)

async def main(queue):
    # spinup server, run the handler, asynchronously sending the queue data once it comes in
    async with websockets.serve(handler, IP, 3000):
        print("started!")
        await broadcast(queue)
        print("sent!")

def main_wrap(queue):
    asyncio.run(main(queue))
