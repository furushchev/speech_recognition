#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from __future__ import division
import os
import rospy
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.chrome.options import Options


class ChromeSpeechRecognition(object):
    def __init__(self):
        self.stats = {}
        self.latest_time = rospy.Time(0)

        # launch browser
        options = Options()
        options.add_argument("--no-first-run")
        options.add_argument("--use-fake-ui-for-media-stream")
        options.add_argument("--use-fake-device-for-media-stream")
        # options.add_argument("--headless")
        self.browser = webdriver.Chrome(chrome_options=options)
        self.browser.get("https://furushchev.ru/rwt/speech_recognition")

        # setup diag
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("SpeechRecognition")
        self.updater.add("status", self.update_status)

        self.pub_sr = rospy.Publisher("speech_to_text", SpeechRecognitionCandidates, queue_size=1)

        hz = 20.0
        self.timer = rospy.Timer(rospy.Duration(1./hz), self.timer_cb)

    def __del__(self):
        try:
            self.stop()
        except:
            pass
        try:
            self.browser.quit()
        except:
            pass

    def timer_cb(self, event):
        self.updater.update()
        if not self.stats:
            rospy.logwarn_throttle(1.0, "No stats")
            return

        # publish results
        new_stats = dict(sorted(filter(lambda e: e[0] > self.latest_time, self.stats.items())))
        for t, data in new_stats:
            stat, detail = data
            if "onresult" in stat:
                msg = SpeechRecognitionCandidates()
                for transcript, confidence in detail:
                    msg.transcript.append(transcript)
                    msg.confidence.append(confidence)
                self.pub_sr.publish(msg)

        # store latest status time
        latest = sorted(self.stats.items())[0]
        self.latest_time = latest[0]

    def update_status(self, diag):
        text = self.browser.find_element_by_id("status").text
        stats = {}
        for line in text.split(os.linesep):
            tm, st, detail = line.strip().split('|')
            sec, nsec = int(tm) // 1000, (int(tm) % 1000) * 1000 * 1000
            stats[rospy.Time(sec, nsec)] = (st, json.loads(detail))

        # diagnostics
        latest = sorted(stats.items())[0]
        diag.summary(DiagnosticStatus.OK, "Status OK")
        diag.add("Status", latest[1])
        self.stats = stats
        return diag

    def get_result(self):
        text = self.browser.find_element_by_id("result").text
        rospy.loginfo("result: " + text)
        return text

    def start(self):
        btn = self.browser.find_element_by_id("start-button")
        btn.click()
        rospy.loginfo("Started speech recognition")

    def stop(self):
        btn = self.browser.find_element_by_id("stop-button")
        btn.click()
        rospy.loginfo("Stopped speech recognition")

if __name__ == '__main__':
    rospy.init_node("chrome_speech_driver")
    d = ChromeSpeechRecognition()
    rospy.spin()
