#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import asyncio
import websockets
from uarm.wrapper import SwiftAPI

# 连接uarm swift pro，指定设备路径
swift = SwiftAPI(port="/dev/ttyACM0")

# 定义 WebSocket 服务器的处理函数
async def websocket_server(websocket, path):
    try:
        async for message in websocket:
            if message == "reset":
                print("Received 'reset' command")
                # 调用 SwiftAPI 的 reset 方法
                swift.reset(timeout=3, x=110, y=0, z=35)
            else:
                print("Received unknown command:", message)
    except websockets.exceptions.ConnectionClosedError:
        print("WebSocket connection closed unexpectedly")

# 启动 WebSocket 服务器
async def start_server():
    server = await websockets.serve(websocket_server, "0.0.0.0", 8801)
    print(f"WebSocket server started on ws://0.0.0.0:8801")

    # 进入事件循环，等待连接和消息
    await server.wait_closed()

if __name__ == "__main__":
    try:
        asyncio.run(start_server())
    except KeyboardInterrupt:
        print("\nExiting program...")
