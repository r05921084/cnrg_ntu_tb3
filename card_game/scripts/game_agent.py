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
import socket

import rospy
from std_msgs.msg import String
# from chatbot.msg import ChatterStamped


event = threading.Event()

def callback(data):
    global msg
    msg = data.data
    # msg = data.text
    event.set()

msg = None

card_num = 3

key_map = {'apple':'蘋果', 'banana':'香蕉', 'cat':'貓', 'dog':'狗', 'elephant':'大象', 'fox':'狐狸'}

game_introduction = ["湯姆您好,我們來玩遊戲吧！", "這裡有幾張牌,每張牌都有不同的圖案",
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



# define("ip", default="172.16.0.130")


def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    return s.getsockname()[0]

URL = get_ip_address()
define("ip", default = URL)
define("port", default = 8888)


class Chat(web.RequestHandler):
    def get(self):
        self.render('cardgame.html', address = URL + ':8888')


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


def speak(sentence, timeout=2.):
    sentence = sentence.decode('utf8')
    fp = tempfile.NamedTemporaryFile(delete=True)
    tts = gTTS(text=sentence, lang='zh')
    tts.save('{}.mp3'.format(fp.name))
    # tts.save('hello.mp3')
    mixer.init()
    mixer.music.load('{}.mp3'.format(fp.name))
    mixer.music.play()
    time.sleep(timeout)
    fp.close()


def play_sound(sound_type, timeout=3.):

    if sound_type=='right':
        f = os.path.join(STATIC_PATH, 'sounds', 'clapping.mp3')

    elif sound_type=='wrong':
        f = os.path.join(STATIC_PATH, 'sounds', 'failure.mp3')

    mixer.init()
    mixer.music.load(f)
    mixer.music.play()
    time.sleep(timeout)
    mixer.music.stop()


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

    comment = None
    correct = False
    # dont_know = False

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


def get_level(user_name):
    p = os.path.join(os.path.dirname(__file__), 'log', user_name)
    try:
        with open(p, 'r') as f:
            level = int(f.readline().strip('\n'))
    except:
        level = 3

    return level


def save(user_name, accuracy, reaction_time):
    p = os.path.join(os.path.dirname(__file__), 'log', user_name)
    level = len(accuracy)
    with open(p, 'a') as f:
        f.write(time.strftime("%Y-%m-%d %H:%M:%S level:{}".format(level)))
        f.write(' accuracy:%s reaction_time(sec):%s\n' % (','.join(map(str, accuracy.round(2))), ','.join(map(str, reaction_time.round(1)))))


def agent(websocket, user_name='測試員1號'):
    # TODO: check connection

    def send_command(command):
        #TODO: async IO
        # if type(n)==str:
        #     if n=='':
        #         msg = command
        #     else:
        #         msg = command+','+n
        #
        # elif type(n)==int:
        #     msg = command+','+str(n)
        #
        # else:
        #     msg = command+','+','.join(n)
        websocket.write_message(command)

    # Set level (card number)
    suggested_level = get_level(user_name)

    while True:
        send_command('init,{}'.format(suggested_level))
        card_num = suggested_level
        accuracy = np.zeros(card_num)
        reaction_time = np.zeros(card_num)

        # Loading image
        select_card = random.sample(cards_all, card_num)
        # print select_card
        send_command('load,{}'.format(','.join(select_card)))

        # Introducing the game

        for s in game_introduction:
            send_command('talk,{}'.format(s))
            speak(s)
            feedback = get_feedback()
            # print 'people: '+feedback

        # Playing
        send_command('start')

        ## Describing the card
        # TODO: check if the answer is right
        for i in range(card_num):
            right = False

            s_describe = random.choice(card_describe)
            send_command('ask,{},{}'.format(i, s_describe))
            speak(s_describe)

            while not right:
                feedback = get_feedback()
                right, comment = analysis(feedback, select_card[i][:-4])
                # s_praise = random.choice(praise)
                send_command('talk,{}'.format(comment))
                speak(comment)
            time.sleep(2)


        send_command('talk,都記住了嗎?')
        speak("都記住了嗎")
        feedback = get_feedback()


        ## Fold?
        send_command('fold')

        game_mode = 1
        # 0: keep playing when the answer is wrong
        # 1: turn next card when the answer is wrong

        ## Test?
        for i in range(card_num):
            s_test = random.choice(test)
            send_command('ask,{},{}'.format(i, s_test))
            speak(s_test)
            t = time.time()
            ###

            count = 0

            while True:
                feedback = get_feedback()
                count += 1

                if check_answer(feedback, select_card[i][:-4]):
                    send_command('talk,答對了!')
                    speak("答對了", timeout=1)
                    play_sound('right')
                    # send_command('turn,{}'.format(i))
                    send_command('turn,%d'%i)
                    break

                else:
                    send_command('talk,答錯了!')
                    speak("答錯了", timeout=1)
                    play_sound('wrong')

                    if game_mode==0:
                        send_command('talk,再猜猜看')
                        speak("再猜猜看")
                    else:
                        send_command('turn,%d' % i)
                        send_command('talk,答案是{}'.format(key_map[select_card[i][:-4]]))
                        speak('答案是{}'.format(key_map[select_card[i][:-4]]))
                        count = 0
                        break

            if game_mode==0:
                accuracy[i] = (1. / count)
            else:
                accuracy[i] = count

            time.sleep(1)
            reaction_time[i] = time.time() - t

        total_time = np.sum(reaction_time)
        send_command('talk,共花了%d秒'%total_time)

        suggested_level = level_adaptation(suggested_level, accuracy, reaction_time)
        save(user_name, accuracy, reaction_time)
        print 'avg_acc:', np.mean(accuracy)
        print 'avg_reaction_time:', np.mean(reaction_time)
        print 'adapting to level', suggested_level

        send_command('talk,還要玩嗎?')
        speak('還要玩嗎', timeout=2)
        next_game = get_feedback()
        if next_game == 'q':
            break
        else:
            pass

    rospy.signal_shutdown('Game Finished')
    

def level_adaptation(curr_level, accuracy, reaction_time):

    acc_u = 0.75
    acc_l = 0.25
    rt_u = 20
    rt_l = 10

    avg_acc = np.mean(accuracy)
    avg_rt = np.mean(reaction_time)

    if avg_acc >= acc_u and avg_rt <= rt_l:
        adapted_level = min(curr_level + 1, 5)

    elif avg_acc <= acc_l or avg_rt >= rt_u:
        adapted_level = curr_level - 1

    else:
        adapted_level = curr_level


    return adapted_level


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
    # rospy.Subscriber("chatbot/input", ChatterStamped, callback)
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
