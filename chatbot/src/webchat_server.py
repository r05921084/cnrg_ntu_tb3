#!/usr/bin/env/ python
# -*- coding: utf-8 -*-
import os, sys
from tornado import web, ioloop, websocket
from tornado.options import define, options
from chatbot_io import ChatBot_IO


define("ip", default=sys.argv[1] if len(sys.argv) > 1 else 'localhost')
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
    chatbot = None
    def open(self):        
        ChatManager.add_user(self)
        print ' %d connected.' % len(ChatManager.users)
        if Socket.chatbot is None:
            Socket.chatbot = ChatBot_IO(self.send_to_ws)
            print ' chatbot initialized.'

    def on_close(self):
        ChatManager.remove_user(self)
        print ' 1 disconnected, %d left.' % len(ChatManager.users)      

    def on_message(self, message):
        message = message.strip('\n')
        print ' send_to_ros: "%s"' % message
        Socket.chatbot.send_to_ros(message)

    def send_to_ws(self, message):
        print ' send_to_ws: "%s"' % message
        for user in ChatManager.users:
            user.write_message(message)


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
    # rospy.on_shutdown(ioloop.IOLoop.current().stop)
    ioloop.IOLoop.current().start()


if __name__ == "__main__":
    main()
