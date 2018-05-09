#!/usr/bin/env/ python
# -*- coding: utf-8 -*-
import os
from tornado import web, ioloop, websocket
from tornado.options import define, options


define("ip", default='localhost')
define("port", default=8888)


class ChatManager(object):
    users = []
    @classmethod
    def add_user(cls, websocket):
        cls.users.append(websocket)

    @classmethod
    def remove_user(cls, websocket):
        cls.users.remove(websocket)


class Chat(web.RequestHandler):
    def get(self):
        self.render("chat.html")


class Socket(websocket.WebSocketHandler):
    def open(self):
        print ' [x] connected.'
        ChatManager.add_user(self)

    def on_close(self):
        print ' [x] disconnected.'
        ChatManager.remove_user(self)

    def on_message(self, message):
        print ' [x] send message.'
        for user in ChatManager.users:
            user.write_message('{"time": "23:59", "text":"text", "source":"source"}')


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
