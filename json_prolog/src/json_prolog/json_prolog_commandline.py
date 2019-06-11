#!/usr/bin/env python
import re
import os
import rospy
import readline
import sys
from json_prolog import PrologException, Prolog

HISTORY_NAME = os.path.expanduser('~/.json_prolog_commandline_history')
HISTORY_LENGTH = 300
RE_SPACE = re.compile('.*\s+$', re.M)


def read_single_keypress():
    """Waits for a single keypress on stdin.

    This is a silly function to call if you need to do it a lot because it has
    to store stdin's current setup, setup stdin for reading single keystrokes
    then read the single keystroke then revert stdin back after reading the
    keystroke.

    Returns the character of the key that was pressed (zero on
    KeyboardInterrupt which can happen when a signal gets handled)

    """
    import termios, fcntl, sys, os
    fd = sys.stdin.fileno()
    # save old state
    flags_save = fcntl.fcntl(fd, fcntl.F_GETFL)
    attrs_save = termios.tcgetattr(fd)
    # make raw - the way to do this comes from the termios(3) man page.
    attrs = list(attrs_save)  # copy the stored version to update
    # iflag
    attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
                  | termios.ISTRIP | termios.INLCR | termios.IGNCR
                  | termios.ICRNL | termios.IXON)
    # oflag
    attrs[1] &= ~termios.OPOST
    # cflag
    attrs[2] &= ~(termios.CSIZE | termios.PARENB)
    attrs[2] |= termios.CS8
    # lflag
    attrs[3] &= ~(termios.ECHONL | termios.ECHO | termios.ICANON
                  | termios.ISIG | termios.IEXTEN)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    # turn off non-blocking
    fcntl.fcntl(fd, fcntl.F_SETFL, flags_save & ~os.O_NONBLOCK)
    # read a single keystroke
    try:
        ret = sys.stdin.read(1)  # returns a single character
    except KeyboardInterrupt:
        ret = 0
    finally:
        # restore old state
        termios.tcsetattr(fd, termios.TCSAFLUSH, attrs_save)
        fcntl.fcntl(fd, fcntl.F_SETFL, flags_save)
    return ret


class PQ(object):
    def __init__(self):
        rospy.wait_for_service('/json_prolog/query')
        self.prolog = Prolog()
        self.predicates = []
        self.load_namespace()
        self.load_all_predicates()

    def load_namespace(self):
        q = 'findall([_X, _Y], rdf_current_ns(_X, _Y), NS)'
        solution = self.prolog.once(q)
        print('namespaces:')
        for ns in solution['NS']:
            print('{}: {}'.format(ns[0], ns[1]))
        print('-----------')

    def load_all_predicates(self):
        q = 'findall(X, current_predicate(X/_);current_module(X), L)'
        solution = self.prolog.once(q)
        self.predicates = [str(x) for x in solution['L']]

    def start_prolog_query(self, q):
        self.q = q
        self.query = self.prolog.query(q)

    def finish_prolog_query(self):
        self.query.finish()
        if self.q.startswith('register_ros_package'):
            self.load_namespace()
            self.load_all_predicates()

    def next_solution(self):
        for solution in self.query.solutions():
            yield solution

    def print_solution(self, solution):
        if solution == dict():
            sys.stdout.write('true')
        else:
            sys.stdout.write(',\n'.join(['{}: {}'.format(k, v) for k, v in solution.items()]))

    def start_commandline(self):
        try:
            while not rospy.is_shutdown():
                cmd = raw_input('?- ')
                if cmd == 'quit.':
                    break
                elif cmd == '':
                    continue
                else:
                    try:
                        pq.start_prolog_query(cmd)
                        print_false = True
                        for solution in self.next_solution():
                            if not print_false:
                                print(' ;')
                                print('')
                            print_false = False
                            self.print_solution(solution)
                            if solution == dict():
                                break
                            cmd = read_single_keypress()
                            if cmd == '.':
                                break
                        pq.finish_prolog_query()
                        if print_false:
                            print('false.')
                        else:
                            print('.')
                    except PrologException as e:
                        print(e)
        except Exception as e:
            print(e)
        finally:
            readline.write_history_file(HISTORY_NAME)

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

    def completer(self, text, state):
        buffer = readline.get_line_buffer()
        line = readline.get_line_buffer().split()

        if not line:
            return [c + ' ' for c in self.predicates][state]

        # account for last argument ending in a space
        if RE_SPACE.match(buffer):
            line.append('')

        cmd = re.split(r',|\(|\[|\+|\=|\-', line[-1])[-1]
        results = [c for c in self.predicates if c.startswith(cmd)] + [None]

        return results[state]


if __name__ == '__main__':
    rospy.init_node('json_prolog_commandline')
    if os.path.isfile(HISTORY_NAME):
        readline.read_history_file(HISTORY_NAME)
    readline.set_history_length(HISTORY_LENGTH)
    readline.parse_and_bind("tab: complete")
    pq = PQ()
    readline.set_completer(pq.completer)
    pq.start_commandline()
