import asyncio
import json
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import websockets

HOST_IP = "172.16.10.172"   # 라즈베리파이 호스트 IP
PORT = 8765
TIMEOUT_SEC = 3.0

class WsHeartbeatListener(Node):
    def __init__(self):
        super().__init__("ws_heartbeat_listener")
        self.pub = self.create_publisher(Bool, "/link_alive", 10)

        self.last_seen = 0.0
        self.alive = False

        # ROS 타이머로 timeout 체크
        self.create_timer(0.5, self._tick)

        # asyncio WS 루프를 별도 스레드로 실행
        t = threading.Thread(target=self._ws_thread, daemon=True)
        t.start()

    def _tick(self):
        now = time.time()
        alive_now = (now - self.last_seen) <= TIMEOUT_SEC

        if alive_now != self.alive:
            self.alive = alive_now
            msg = Bool()
            msg.data = self.alive
            self.pub.publish(msg)
            self.get_logger().info(f"link_alive changed -> {self.alive}")

    def _ws_thread(self):
        asyncio.run(self._ws_loop())

    async def _ws_loop(self):
        url = f"ws://{HOST_IP}:{PORT}"
        while rclpy.ok():
            try:
                async with websockets.connect(url, ping_interval=None) as ws:
                    # role 등록
                    await ws.send(json.dumps({"role": "container"}))
                    _ = await ws.recv()  # ok:container

                    self.get_logger().info("connected to host router")

                    async for msg in ws:
                        try:
                            data = json.loads(msg)
                            if data.get("type") == "hb":
                                self.last_seen = time.time()
                        except Exception:
                            pass

            except Exception as e:
                # 연결 끊기면 last_seen 갱신이 멈춰서 timeout으로 alive=false로 내려감
                try:
                    self.get_logger().warn(f"ws connection error: {repr(e)}")
                except Exception:
                    pass
                await asyncio.sleep(1.0)

def main():
    rclpy.init()
    node = WsHeartbeatListener()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
