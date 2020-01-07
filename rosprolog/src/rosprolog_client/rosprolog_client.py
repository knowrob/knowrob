import json

import rospy
from json_prolog_msgs import srv


class PrologException(Exception):
    pass


class PrologQuery(object):
    def __init__(self, query_str, simple_query_srv, next_solution_srv, finish_srv, iterative=True):
        """
        This class wraps around the different rosprolog services to provide a convenient python interface.
        :type query_str: str
        :type simple_query_srv: rospy.impl.tcpros_service.ServiceProxy
        :type next_solution_srv: rospy.impl.tcpros_service.ServiceProxy
        :type finish_srv: rospy.impl.tcpros_service.ServiceProxy
        :param iterative: if False, all solutions will be calculated by rosprolog during the first service call
        :type iterative: bool
        """
        self._simple_query_srv = simple_query_srv
        self._next_solution_srv = next_solution_srv
        self._finish_query_srv = finish_srv

        self._finished = False
        self._query_id = None
        result = self._simple_query_srv(id=self.get_id(), query=query_str, mode=(1 if iterative else 0))
        if not result.ok:
            raise PrologException('Prolog query failed: {}'.format(result.message))

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.finish()

    def solutions(self):
        """
        :rtype: Iterator[dict]
        """
        try:
            while not self._finished:
                next_solution = self._next_solution_srv(id=self.get_id())
                if next_solution.status == srv.PrologNextSolutionResponse.OK:
                    yield self._json_to_dict(next_solution.solution)
                elif next_solution.status == srv.PrologNextSolutionResponse.WRONG_ID:
                    raise PrologException(
                        'Query id {} invalid. Maybe another process terminated our query?'.format(self.get_id()))
                elif next_solution.status == srv.PrologNextSolutionResponse.QUERY_FAILED:
                    raise PrologException('Prolog query failed: {}'.format(next_solution.solution))
                elif next_solution.status == srv.PrologNextSolutionResponse.NO_SOLUTION:
                    break
                else:
                    raise PrologException('Unknown query status {}'.format(next_solution.status))
        finally:
            self.finish()

    def finish(self):
        if not self._finished:
            try:
                self._finish_query_srv(id=self.get_id())
            finally:
                self._finished = True

    def get_id(self):
        """
        :rtype: str
        """
        if self._query_id is None:
            self._query_id = 'PYTHON_QUERY_{}'.format(rospy.Time.now().to_nsec())
        return self._query_id

    def _json_to_dict(self, json_text):
        """
        :type json_text: str
        :rtype: dict
        """
        return json.loads(json_text)


class Prolog(object):
    def __init__(self, name_space='rosprolog', timeout=None, wait_for_services=True):
        """
        :type name_space: str
        :param timeout: Amount of time in seconds spend waiting for rosprolog to become available.
        :type timeout: int
        """
        self._simple_query_srv = rospy.ServiceProxy('{}/query'.format(name_space), srv.PrologQuery)
        self._next_solution_srv = rospy.ServiceProxy('{}/next_solution'.format(name_space), srv.PrologNextSolution)
        self._finish_query_srv = rospy.ServiceProxy('{}/finish'.format(name_space), srv.PrologFinish)
        if wait_for_services:
            rospy.loginfo('waiting for {} services'.format(name_space))
            self._finish_query_srv.wait_for_service(timeout=timeout)
            self._simple_query_srv.wait_for_service(timeout=timeout)
            self._next_solution_srv.wait_for_service(timeout=timeout)
            rospy.loginfo('{} services ready'.format(name_space))

    def query(self, query_str):
        """
        Returns an Object which asks rosprolog for one solution at a time.
        :type query_str: str
        :rtype: PrologQuery
        """
        return PrologQuery(query_str, simple_query_srv=self._simple_query_srv,
                           next_solution_srv=self._next_solution_srv, finish_srv=self._finish_query_srv)

    def once(self, query_str):
        """
        Call rosprolog once and finished it.
        :type query_str: str
        :rtype: list
        """
        q = None
        try:
            q = PrologQuery(query_str, simple_query_srv=self._simple_query_srv,
                            next_solution_srv=self._next_solution_srv, finish_srv=self._finish_query_srv)
            return q.solutions().next()
        except StopIteration:
            return []
        finally:
            if q is not None:
                q.finish()

    def all_solutions(self, query_str):
        """
        Requests all solutions from rosprolog, this might take a long time!
        :type query_str: str
        :rtype: list
        """
        return list(PrologQuery(query_str,
                                iterative=False,
                                simple_query_srv=self._simple_query_srv,
                                next_solution_srv=self._next_solution_srv,
                                finish_srv=self._finish_query_srv).solutions())

    def wait_for_service(self, timeout=None):
        """
        Blocks until rosprolog service is available. Use this in
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
        rospy.logwarn('rosprolog.Prolog.wait_for_service is deprecated, __init__ waits for services by default')
        self._simple_query_srv.wait_for_service(timeout=timeout)
        self._next_solution_srv.wait_for_service(timeout=timeout)
        self._finish_query_srv.wait_for_service(timeout=timeout)
        
