
import json
import rospy
from json_prolog_msgs import srv

_simple_query_srv = None
_next_solution_srv = None
_finish_query_srv = None

class PrologException(Exception):
    def __init__(self, error_msg):
        self.msg = error_msg
    def __str__(self):
        return repr(self.msg)
    
class PrologQuery(object):
    def __init__(self, query_str):
        self.query_id = self._makeQueryId()
        self.finished = False
        result = _simple_query_srv(id=self.query_id, query=query_str)
        if not result.ok:
            raise PrologException('Prolog query failed: %s' % result.message)

    def solutions(self):
        try:
            while not self.finished:
                next_solution = _next_solution_srv(id=self.query_id)
                if next_solution.status == srv.PrologNextSolutionResponse.NO_SOLUTION:
                    finished = True
                    return
                elif next_solution.status == srv.PrologNextSolutionResponse.WRONG_ID:
                    finished = True
                    raise PrologException(
                        'Query id invalid. Maybe another process terminated our query?')
                elif next_solution.status == srv.PrologNextSolutionResponse.QUERY_FAILED:
                    finished = True
                    raise PrologException(
                        'Prolog query failed: %s' % next_solution.solution)
                elif next_solution.status == srv.PrologNextSolutionResponse.OK:
                    yield self._solutionToBdgs(next_solution.solution)
                else:
                    raise PrologException(
                        'Unknow query status %d', next_solution.status)
        finally:
            self.finish()
    
    def finish(self):
        _finish_query_srv(id=self.query_id)
        self.finished=True

    def _makeQueryId(self):
        return 'PYTHON_QUERY_%d' % rospy.Time.now().to_nsec()

    def _solutionToBdgs(self, solution):
        return json.loads(solution)


class Prolog(object):
    def __init__(self, ns='json_prolog'):
        global _simple_query_srv
        global _next_solution_srv
        global _finish_query_srv

        _simple_query_srv = rospy.ServiceProxy(ns + '/simple_query', srv.PrologQuery)
        _next_solution_srv = rospy.ServiceProxy(ns + '/next_solution', srv.PrologNextSolution)
        _finish_query_srv = rospy.ServiceProxy(ns + '/finish', srv.PrologFinish)

    def query(self, query_str):
        return PrologQuery(query_str)

    def once(self, query_str):
        q = None
        try:
            q = PrologQuery(query_str)
            result = q.solutions().next()
            return result
        except StopIteration:
            return []
        finally:
            if q:
                q.finish()
