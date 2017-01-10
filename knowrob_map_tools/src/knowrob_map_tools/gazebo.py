#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import rospkg
import subprocess


class GazeboModelResolver(object):
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.paths = []
        self.rospkgs = {}
        self.cached_path = {}
        self.load_path_from_env()
        self.load_path_from_plugin()

    def add_path(self, path):
        self.paths.insert(0, path)

    def add_ros_pkg(self, pkg, path):
        if pkg in self.rospkgs:
            self.rospkgs[pkg].insert(0, path)
        else:
            self.rospkgs[pkg] = [path]

    def load_path_from_env(self):
        env_str = os.environ.get("GAZEBO_MODEL_PATH", None)
        if env_str is None:
            return
        for path in env_str.split(':'):
            if os.path.isdir(path):
                self.add_path(path)

    def load_path_from_plugin(self):
        cmd = "rospack plugins --attrib=gazebo_model_path gazebo_ros"
        lines = subprocess.check_output(cmd, shell=True)
        if lines:
            for line in lines.split(os.linesep):
                if not line:
                    continue
                pkg, path = line.split()
                if os.path.isdir(path):
                    self.add_path(path)
                    if pkg not in self.rospkgs:
                        self.rospkgs[pkg] = self.rospack.get_path(pkg)

    def parse_model_path(self, path):
        key = None
        if path.startswith("model://"):
            spath = path[len("model://"):].split('/')
            key = spath[0]
            path = '/'.join(spath[1:])
        return key, path

    def search_path(self, key):
        for base_path in self.paths:
            path = os.path.join(base_path, key)
            if os.path.isdir(path):
                self.cached_path[key] = path

    def pack_ros_path(self, path):
        for pkg, pkg_path in self.rospkgs.items():
            if path.startswith(pkg_path):
                return "package://" + pkg + path[len(pkg_path):]
        return path

    def resolve_path(self, path):
        key, path = self.parse_model_path(path)
        if not key:
            return path
        if key not in self.cached_path:
            self.search_path(key)
        if key in self.cached_path:
            path = os.path.join(self.cached_path[key], path)
        else:
            raise Exception("path %s is not found" % path)
        return self.pack_ros_path(path)

_gazebo = GazeboModelResolver()

def resolve_model_path(path):
    return _gazebo.resolve_path(path)
