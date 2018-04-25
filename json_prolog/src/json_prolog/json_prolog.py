import json
from multiprocessing import Lock, TimeoutError

import rospy
from json_prolog_msgs import srv

# We have to lock because calling json prolog twice at the same time can lead to a deadlock
json_prolog_lock = Lock()


class PrologException(Exception):
    pass


class PrologQuery(object):
    def __init__(self, query_str, simple_query_srv, next_solution_srv, finish_srv, timeout=10, iterative=True):
        self._simple_query_srv = simple_query_srv
        self._next_solution_srv = next_solution_srv
        self._finish_query_srv = finish_srv

        self._query_id = self._makeQueryId()
        self._finished = False

        global json_prolog_lock
        if not json_prolog_lock.acquire(timeout=timeout):
            raise TimeoutError('Failed to acquire prolog lock after {}s'.format(timeout))
        result = self._simple_query_srv(id=self._query_id, query=query_str, mode=(1 if iterative else 0))
        if not result.ok:
            raise PrologException('Prolog query failed: {}'.format(result.message))

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.finish()

    def solutions(self):
        try:
            while not self._finished:
                next_solution = self._next_solution_srv(id=self._query_id)
                if next_solution.status == srv.PrologNextSolutionResponse.OK:
                    yield self._solutionToBdgs(next_solution.solution)
                elif next_solution.status == srv.PrologNextSolutionResponse.WRONG_ID:
                    raise PrologException(
                        'Query id invalid. Maybe another process terminated our query?')
                elif next_solution.status == srv.PrologNextSolutionResponse.QUERY_FAILED:
                    raise PrologException(
                        'Prolog query failed: {}'.format(next_solution.solution))
                elif next_solution.status == srv.PrologNextSolutionResponse.NO_SOLUTION:
                    break
                else:
                    raise PrologException(
                        'Unknow query status {}'.format(next_solution.status))
        finally:
            self.finish()

    def finish(self):
        if not self._finished:
            try:
                self._finish_query_srv(id=self._query_id)
            finally:
                global json_prolog_lock
                json_prolog_lock.release()
                self._finished = True

    def _makeQueryId(self):
        return 'PYTHON_QUERY_{}'.format(rospy.Time.now().to_nsec())

    def _solutionToBdgs(self, solution):
        return json.loads(solution)


class Prolog(object):
    def __init__(self, ns='json_prolog', timeout=None):
        rospy.loginfo('waiting for {} services'.format(ns))
        self._simple_query_srv = rospy.ServiceProxy('{}/simple_query'.format(ns), srv.PrologQuery)
        self._simple_query_srv.wait_for_service(timeout=timeout)
        self._next_solution_srv = rospy.ServiceProxy('{}/next_solution'.format(ns), srv.PrologNextSolution)
        self._next_solution_srv.wait_for_service(timeout=timeout)
        self._finish_query_srv = rospy.ServiceProxy('{}/finish'.format(ns), srv.PrologFinish)
        self._finish_query_srv.wait_for_service(timeout=timeout)
        rospy.loginfo('{} services ready'.format(ns))

    def query(self, query_str, timeout=10):
        return PrologQuery(query_str, timeout=timeout, simple_query_srv=self._simple_query_srv,
                           next_solution_srv=self._next_solution_srv, finish_srv=self._finish_query_srv)

    def once(self, query_str, timeout=10):
        q = None
        try:
            q = PrologQuery(query_str, timeout=timeout, simple_query_srv=self._simple_query_srv,
                            next_solution_srv=self._next_solution_srv, finish_srv=self._finish_query_srv)
            return q.solutions().next()
        except StopIteration:
            return []
        finally:
            if q is not None:
                q.finish()

    def all_solutions(self, query_str, timeout=10):
        return list(PrologQuery(query_str,
                                timeout=timeout,
                                iterative=False,
                                simple_query_srv=self._simple_query_srv,
                                next_solution_srv=self._next_solution_srv,
                                finish_srv=self._finish_query_srv).solutions())

    def wait_for_service(self, timeout=None):
        """
        Blocks until json_prolog service is available. Use this in
        initialization code if your program depends on the service
        already running.
        @param timeout: timeout time in seconds, None for no
        timeout. NOTE: timeout=0 is invalid as wait_for_service actually
        contacts the service, so non-blocking behavior is not
        possible. For timeout=0 uses cases, just call the service without
        waiting.
        @type  timeout: double
        @raise ROSException: if specified timeout is exceeded
        @raise ROSInterruptException: if shutdown interrupts wait
        """
        rospy.logwarn('json_prolog.Prolog.wait_for_service is deprecated, __init__ waits for services by default')
        self._simple_query_srv.wait_for_service(timeout=timeout)
        self._next_solution_srv.wait_for_service(timeout=timeout)
        self._finish_query_srv.wait_for_service(timeout=timeout)
