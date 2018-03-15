#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import random
import string
import subprocess
import sys


class UniqueStringGenerator(object):
    def __init__(self, strlen=8):
        self.strlen = strlen
        self.issued = set()
    def gen(self):
        while True:
            s = ''.join([random.choice(string.ascii_letters + string.digits) for i in range(self.strlen)])
            if s not in self.issued:
                self.issued |= set(s)
                return s
