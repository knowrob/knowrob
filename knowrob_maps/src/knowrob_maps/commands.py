#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import os
from urdf_to_sem import URDF2SEM


class FileNotFoundException(Exception):
    pass
class FileAlreadyExistsException(Exception):
    pass

def urdf_to_sem(urdf_path, sem_path=None, overwrite=False, suffix=None):
    if not os.path.exists(urdf_path):
        raise FileNotFoundException(urdf_path)
    if sem_path is None:
        base_name, _ = os.path.splitext(urdf_path)
        sem_path = base_name + ".owl"

    if os.path.exists(sem_path) and overwrite is False:
        raise FileAlreadyExistsException(sem_path)

    u2s = URDF2SEM(urdf_path, sem_path, name_suffix=suffix)
    if not os.path.exists(os.path.dirname(sem_path)):
        os.makedirs(os.path.dirname(sem_path))
    with open(sem_path, "w") as f:
        f.write(u2s.to_string())
    print "saved to", sem_path

    return True

