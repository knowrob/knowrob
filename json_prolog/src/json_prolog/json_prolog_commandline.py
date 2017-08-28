#!/usr/bin/env python
import os
import rospy
import readline
from json_prolog import PrologException, Prolog

HISTORY_NAME = os.path.expanduser('~/.json_prolog_commandline_history')


class PQ(object):
    def __init__(self):
        rospy.wait_for_service('/json_prolog/query')
        self.prolog = Prolog()

    def prolog_query(self, q):
        query = self.prolog.query(q)
        solutions = [x for x in query.solutions()]
        query.finish()
        self.print_all_solutions(solutions)

    def print_all_solutions(self, solutions):
        if len(solutions) == 0:
            print('false.')
        else:
            for s in solutions:
                if s == dict():
                    print('true.')
                else:
                    for k, v in s.items():
                        print('{}: {}'.format(k, v))


if __name__ == '__main__':
    if os.path.isfile(HISTORY_NAME):
        readline.read_history_file(HISTORY_NAME)
    rospy.init_node('pq')
    pq = PQ()
    cmd = ''
    try:
        while not rospy.is_shutdown():
            cmd = raw_input('?- ')
            if cmd == 'quit.':
                break
            elif cmd == '':
                continue
            else:
                try:
                    pq.prolog_query(cmd)
                except PrologException as e:
                    print(e)
    except Exception as e:
        print(e)
    finally:
        readline.write_history_file(HISTORY_NAME)