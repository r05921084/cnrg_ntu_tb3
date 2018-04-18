#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
from tornado import web, ioloop, websocket
from tornado.options import define, options
import random
import time
import json
import numpy as np
from gtts import gTTS
from pygame import mixer
import tempfile
import threading
import rospy
from std_msgs.msg import String


event = threading.Event()

def callback(data):
    global msg
    msg = data.data
    event.set()

msg = None

card_num = 3

game_introduction = ["XXX您好,我們來玩遊戲吧！", "這裡有{}張牌,每張牌都有不同的圖案".format(card_num),
                    "等會兒我會把牌蓋起來", "看看你能記住幾張", "記得越多就可以拿到越多獎品喔！",
                    "知道怎麼玩了嗎？", "那我們就開始囉！"]

card_describe = ["這張牌是什麼呢？", "請你描述一下這張卡片"]

praise = ["很棒", "說得很好"]

praise1 = ["很接近了", "可惜，差點就答對了"]

test = ["請問這張牌是什麼呢？"]


filepath = os.path.dirname(os.path.abspath(__file__))
cards_all = os.listdir(os.path.join(filepath, 'static', 'images'))
cards_all.remove('ques.png')
cards_all.sort()

f = open(os.path.join(filepath, 'card_property.json'), 'r')
card_property = json.load(f)
f.close()


# define("ip", default="localhost")
define("ip", default="172.16.0.130")
define("port", default=8888)


class Chat(web.RequestHandler):
    def get(self):
        self.render("index.html")

class MyWebSocket(websocket.WebSocketHandler):

    def open(self):
        print ' connected.'
        create_agent(self)

    def on_close(self):
        print ' disconnected.'
        
    def on_message(self, message):
        print message


def create_agent(ws):
    t = threading.Thread(target=agent, args=(ws,))
    t.start()


def speak(sentence):
    sentence = sentence.decode('utf8')
    fp = tempfile.NamedTemporaryFile(delete=True)
    tts = gTTS(text=sentence, lang='zh')
    tts.save('{}.mp3'.format(fp.name))
    # tts.save('hello.mp3')
    mixer.init()
    mixer.music.load('{}.mp3'.format(fp.name))
    mixer.music.play()
    fp.close()


def get_feedback():
    # feedback = raw_input().decode('utf8')
    event.wait()
    event.clear()
    if msg == 'q':
        rospy.signal_shutdown()
    else:
        feedback = msg.decode('utf8')

    return feedback


def agent(websocket):
    # TODO: check connection

    def send_command(command, n=''):
        #TODO: async IO
        if type(n)==str:
            if n=='':
                msg = command
            else:
                msg = command+','+n

        elif type(n)==int:
            msg = command+','+str(n)

        else:
            msg = command+','+','.join(n)

        websocket.write_message(msg)


    # Set level (card number)
    send_command('init', card_num)

    response_time = np.zeros(card_num)
    # Loading image
    select_card = random.sample(cards_all, card_num)
    # print select_card
    send_command('load', select_card)


    # Introducing the game
    for s in game_introduction:
        send_command('talk', s)
        speak(s)
        feedback = get_feedback()
        # print 'people: '+feedback


    # Playing
    send_command('start', s)
    
    ## Describing the card
    # TODO: check if the answer is right
    for i in range(card_num):
        s_describe = random.choice(card_describe)
        send_command('ask', [str(i), s_describe])
        speak(s_describe)
        feedback = get_feedback()

        s_praise = random.choice(praise)
        send_command('talk', s_praise)
        speak(s_praise)
        time.sleep(2)



    send_command('talk', "都記住了嗎?")
    speak("都記住了嗎")
    feedback = get_feedback()


    ## Fold?
    send_command('fold')


    ## Test?
    def check_answer(s, key_):
        for p in card_property[key_]:
            if s.find(p)>=0:
                return True
        return False



    for i in range(card_num):
        s_test = random.choice(test)
        send_command('ask', [str(i), s_test])
        speak(s_test)
        t = time.time()
        ###
        while True:
            feedback = get_feedback()

            if check_answer(feedback, select_card[i][:-4]):
                send_command('talk,答對了!')
                speak("答對了")
                time.sleep(1)
                # send_command('turn,{}'.format(i))
                send_command('turn,%d'%i)
                break

            else:
                send_command('talk,答錯了!')
                speak("答錯了")
                time.sleep(2)

                send_command('talk,再猜猜看')
                speak("再猜猜看")


        time.sleep(2)
        response_time[i] = time.time() - t

    total_time = np.sum(response_time)
    send_command('talk,共花了%d秒'%total_time)
    # send_command('talk,共花了{}秒'.format(int(total_time)))
    


settings = dict(
    debug=True,
    autoreload=True,
    compiled_template_cache=False,
    # os.path.abspath(__file__)
    static_path=os.path.join(os.path.dirname(__file__), "static"),
    template_path=os.path.join(os.path.dirname(__file__), "templates")
    # static_path=os.path.join(os.path.dirname(os.path.abspath(__file__)), "static"),
    # template_path=os.path.join(os.path.dirname(os.path.abspath(__file__)), "templates")
)

class Application(web.Application):
    def __init__(self):
        handlers = [
            (r"/", Chat),
            (r"/socket", MyWebSocket)
            # (r"/images/(.*)", web.StaticFileHandler, {'path': "./images"})
        ]
        web.Application.__init__(self, handlers, **settings)


def main():
    rospy.init_node('game_agent', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    options.parse_command_line()
    app = Application()
    app.listen(options.port, options.ip)
    ioloop.IOLoop.current().start()


if __name__ == "__main__":
    main()
