#!/usr/bin/env/ python
# -*- coding: utf-8 -*-
import os
from tornado import web, ioloop, websocket
from tornado.options import define, options

define("ip", default='localhost')
define("port", default=8888)

# 此物件將記錄並管理所有連線
class ChatManager(object):
    users = []
    @classmethod
    def add_user(cls, websocket):
        cls.users.append(websocket)

    @classmethod
    def remove_user(cls, websocket):
        cls.users.remove(websocket)

# 輸出聊天室 UI 畫面
class Chat(web.RequestHandler):
    def get(self):
        self.render("chat.html")

# 利用 WebSocketHandler 來與 Client 端互動
class Socket(websocket.WebSocketHandler):
    def open(self):
        # 完成連線，將此 WebSocket 儲存
        print ' [x] connected.'
        ChatManager.add_user(self)

    def on_close(self):
        # 連線結束，將此 WebSocket 移除
        print ' [x] disconnected.'
        ChatManager.remove_user(self)

    def on_message(self, message):
        # 當有訊息進來時，將此訊息發送給其他 WebSocket
        print ' [x] send message.'
        for user in ChatManager.users:
            user.write_message('{"time": "23:59", "text":"text", "source":"source"}')

# 相關設定參數
settings = dict(
    debug=True,
    autoreload=True,
    compiled_template_cache=False,
    static_path=os.path.join(os.path.dirname(__file__), "static"),
    template_path=os.path.join(os.path.dirname(__file__), "templates")
)

class Application(web.Application):
    def __init__(self):
        handlers = [
            (r"/", Chat),
            (r"/socket", Socket)
        ]
        web.Application.__init__(self, handlers, **settings)

def main():
    options.parse_command_line()
    app = Application()
    app.listen(options.port, options.ip)
    ioloop.IOLoop.current().start()

if __name__ == "__main__":
    main()
