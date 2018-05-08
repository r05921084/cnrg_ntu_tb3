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

key_map = {'apple':'蘋果', 'banana':'香蕉', 'cat':'貓', 'dog':'狗'}

game_introduction = ["XXX您好,我們來玩遊戲吧！", "這裡有{}張牌,每張牌都有不同的圖案".format(card_num),
                    "等會兒我會把牌蓋起來", "看看你能記住幾張", "記得越多就可以拿到越多獎品喔！",
                    "知道怎麼玩了嗎？", "那我們就開始囉！"]

card_describe = ["這張牌是什麼呢？", "請你描述一下這張卡片"]

praise = ["很棒", "說得很好"]

praise1 = ["很接近了", "可惜，差點就答對了"]

test = ["請問這張牌是什麼呢？"]

STATIC_PATH = os.path.join(os.path.dirname(__file__), 'static')

cards_all = os.listdir(os.path.join(STATIC_PATH, 'images'))
cards_all.remove('ques.png')
cards_all.sort()

f = open(os.path.join(STATIC_PATH, 'card_property.json'), 'r')
card_property = json.load(f)
f.close()


# define("ip", default="localhost")
define("ip", default="172.16.0.105")
define("port", default=8888)


class Chat(web.RequestHandler):
    def get(self):
        self.render("cardgame.html")


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
    feedback = msg.decode('utf8')

    return feedback


def check_answer(s, key_):
    for p in card_property[key_]:
        if s.find(p)>=0:
            return True
    return False


def analysis(statement, card_key):
    # check the statement is corresponding to the card
    correct_comment = ['您說得真好', '很棒', '很厲害喔', '沒錯，就是{}'.format(key_map[card_key]), '你喜歡{}嗎'.format(key_map[card_key])]
    not_correct_comment = ['說得很好，可是好像不對喔', '差一點就講對了，還有沒有別的答案呢', '好像不太一樣耶，有沒有可能是{}呢'.format(key_map[card_key])]
    dont_know_words = ['不知道', '不明白', '不曉得', '不清楚', '不懂' , '不會', '不能理解']
    stop_words = ['不玩了', '不想玩了']
    dont_know_comment = ['再想想看，你一定可以想起來的', '這是{}'.format(key_map[card_key])]
    correct = False
    dont_know = False
    for p in card_property[card_key]:
        if statement.find(p)>=0:
            correct = True
            break

    if correct:
        comment = random.choice(correct_comment)
    else:
        for a in dont_know_words:
            comment = random.choice(not_correct_comment)
            if statement.find(a.decode('utf8'))>=0:
                comment = random.choice(dont_know_comment)
                break

    return correct, comment







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
        right = False

        s_describe = random.choice(card_describe)
        send_command('ask', [str(i), s_describe])
        speak(s_describe)

        while not right:
            feedback = get_feedback()
            right, comment = analysis(feedback, select_card[i][:-4])
            # s_praise = random.choice(praise)
            print comment
            send_command('talk', comment)
            speak(comment)
        time.sleep(2)



    send_command('talk', "都記住了嗎?")
    speak("都記住了嗎")
    feedback = get_feedback()


    ## Fold?
    send_command('fold')

    accuracy = np.zeros(card_num) - 1
    ## Test?
    for i in range(card_num):
        s_test = random.choice(test)
        send_command('ask', [str(i), s_test])
        speak(s_test)
        t = time.time()
        ###

        count = 0

        while True:
            feedback = get_feedback()
            count += 1

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

        accuracy[i] = (1. / count) * 100
        time.sleep(1)
        response_time[i] = time.time() - t

    total_time = np.sum(response_time)
    send_command('talk,共花了%d秒'%total_time)
    # send_command('talk,共花了{}秒'.format(int(total_time)))
    with open(os.path.join(STATIC_PATH, 'history.txt'), 'w') as f:
        f.write(time.strftime("%Y-%m-%d %H:%M:%S\n"))
        f.write('card, reaction time, accuracy\n')

        for i in range(card_num):
            f.write('{}, {:.1f} s, {:.2f}%\n'.format(select_card[i][:-4], response_time[i], accuracy[i]))


    rospy.signal_shutdown('Game Finished')
    


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

    rospy.on_shutdown(ioloop.IOLoop.current().stop)

    ioloop.IOLoop.current().start()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as e:
        print e
