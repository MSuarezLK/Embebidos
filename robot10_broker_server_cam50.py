import asyncio
import json
import socket

import websockets

PREFIX = "UDFJC/emb1/robot10"
TCP_PORT = 5051
WS_PORT = 5052
CAMERA_URL = "http://192.168.1.1:81/stream"  # ESP32-CAM


def topic_match(pattern, topic):
    p = pattern.split("/")
    t = topic.split("/")
    for i, part in enumerate(p):
        if part == "#":
            return True
        if i >= len(t):
            return False
        if part == "+":
            continue
        if part != t[i]:
            return False
    return len(p) == len(t)


class TCPClient:
    def __init__(self, writer):
        self.writer = writer

    async def send(self, msg):
        self.writer.write((msg + "\n").encode())
        await self.writer.drain()

    def __repr__(self):
        return "TCPClient({})".format(id(self))


class WSClient:
    def __init__(self, websocket):
        self.ws = websocket

    async def send(self, msg):
        await self.ws.send(msg)

    def __repr__(self):
        return "WSClient({})".format(id(self))


class CallbackClient:
    """Cliente interno para que el broker pueda escuchar vision/control."""
    def __init__(self, callback, name="CallbackClient"):
        self.callback = callback
        self.name = name

    async def send(self, msg):
        pkt = json.loads(msg)
        if pkt.get("action") == "PUB":
            self.callback(pkt.get("topic", ""), pkt.get("data", {}))

    def __repr__(self):
        return self.name


class PubSub:
    def __init__(self):
        self.subscriptions = {}

    def subscribe(self, client, topic):
        self.subscriptions.setdefault(topic, set()).add(client)
        print("[SUB]", client, "->", topic)

    def unsubscribe_all(self, client):
        for clients in self.subscriptions.values():
            clients.discard(client)
        print("[UNSUB]", client)

    async def publish(self, topic, data, origin=None):
        msg = json.dumps({"action": "PUB", "topic": topic, "data": data})
        dead = set()
        count = 0

        for pattern, clients in list(self.subscriptions.items()):
            if not topic_match(pattern, topic):
                continue
            for client in list(clients):
                if client is origin:
                    continue
                try:
                    await client.send(msg)
                    count += 1
                except Exception as e:
                    print("[PUB] cliente muerto", client, type(e).__name__, e)
                    dead.add(client)

        for client in dead:
            self.unsubscribe_all(client)

        if "camera/frame" not in topic:
            print("[PUB]", topic, "->", count, "clientes")


class TCPServer:
    def __init__(self, pubsub, host="0.0.0.0", port=TCP_PORT):
        self.pubsub = pubsub
        self.host = host
        self.port = port

    async def handle_client(self, reader, writer):
        client = TCPClient(writer)
        print("TCP conectado:", client)
        try:
            while True:
                data = await reader.readline()
                if not data:
                    break
                try:
                    pkt = json.loads(data.decode().strip())
                except Exception as e:
                    print("JSON TCP inválido:", e, "len=", len(data))
                    continue

                action = pkt.get("action")
                if action == "SUB":
                    self.pubsub.subscribe(client, pkt.get("topic", ""))
                elif action == "PUB":
                    await self.pubsub.publish(pkt.get("topic", ""), pkt.get("data", {}), origin=client)
        except Exception as e:
            print("TCP excepción:", e)
        finally:
            print("TCP desconectado:", client)
            self.pubsub.unsubscribe_all(client)
            try:
                writer.close()
                await writer.wait_closed()
            except Exception:
                pass

    async def start(self):
        server = await asyncio.start_server(self.handle_client, self.host, self.port)
        print("TCP broker en puerto", self.port)
        return server


class WSServer:
    def __init__(self, pubsub, host="0.0.0.0", port=WS_PORT):
        self.pubsub = pubsub
        self.host = host
        self.port = port

    async def handler(self, websocket):
        client = WSClient(websocket)
        print("WS conectado:", client)
        try:
            async for message in websocket:
                try:
                    pkt = json.loads(message)
                except Exception as e:
                    print("JSON WS inválido:", e)
                    continue

                action = pkt.get("action")
                if action == "SUB":
                    self.pubsub.subscribe(client, pkt.get("topic", ""))
                elif action == "PUB":
                    await self.pubsub.publish(pkt.get("topic", ""), pkt.get("data", {}), origin=client)
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            print("WS excepción:", e)
        finally:
            print("WS desconectado:", client)
            self.pubsub.unsubscribe_all(client)

    async def start(self):
        server = await websockets.serve(
            self.handler,
            self.host,
            self.port,
            max_size=5 * 1024 * 1024,
            ping_interval=20,
            ping_timeout=60,
        )
        print("WS broker en puerto", self.port)
        return server


def probable_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return socket.gethostbyname(socket.gethostname())


async def main():
    print("IP probable del broker:", probable_ip())
    print("Prefijo:", PREFIX)

    pubsub = PubSub()
    tcp = TCPServer(pubsub)
    ws = WSServer(pubsub)

    await tcp.start()
    await ws.start()

    vision = None
    try:
        from vision_task import VisionTask
        vision = VisionTask(pubsub, PREFIX, camera_url=CAMERA_URL)
        print("[VIS] VisionTask activo")

        def on_vision_control(topic, data):
            if vision is None:
                return
            if "activo" in data:
                vision.activar_busqueda(bool(data["activo"]))

        pubsub.subscribe(CallbackClient(on_vision_control, "VisionControlClient"), PREFIX + "/vision/control")
        print("[VIS] Control por tópico:", PREFIX + "/vision/control")
    except ImportError as e:
        print("[VIS] vision_task.py no encontrado o falta dependencia:", e)
    except Exception as e:
        print("[VIS] Error iniciando VisionTask:", e)

    print("Broker listo: TCP {} | WS {}".format(TCP_PORT, WS_PORT))
    await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
