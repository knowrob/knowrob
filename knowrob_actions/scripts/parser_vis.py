#!/usr/bin/env python3

import signal
import sys
import json
import time

import pyrap
from pyrap.communication import RWTSetOperation
from pyrap.layout import GridLayout, RowLayout, CellLayout, ColumnLayout
from pyrap.widgets import Shell, Label, Composite, Group, Button, DropDown
from pyrap.pwt.tree.tree import Tree
from pyrap.pwt.graph.graph import Graph
from pyrap.pwt.plot.plot import Scatterplot
from pyrap.threads import DetachedSessionThread
from pyrap.engine import PushService
from pyrap.dialogs import msg_err, options_list
from pyrap import session
from pyrap.ptypes import Color

import rospy
import rosprolog_client as rosprolog

from knowrob_actions.msg._EventToken import EventToken

class ParserPoller(DetachedSessionThread):
    """
    queries the parser state periodically.
    """
    def __init__(self,parser_page,rate=0.5):
        DetachedSessionThread.__init__(self)
        self.push_ = PushService()
        self.parser_page_ = parser_page
        self.is_running_ = False
        self.has_stopped_ = True
        self.rate_ = rate
    
    def stop_poller(self):
        self.is_running_ = False
    
    def run(self):
        self.is_running_ = True
        self.has_stopped_ = False
        self.push_.start()
        try:
            while self.is_running_:
                has_updates = self.parser_page_.update()
                if has_updates:
                    self.push_.flush()
                time.sleep(self.rate_)
        finally:
            self.is_running_ = False
            self.has_stopped_ = True
            self.push_.stop()

class ParserWidget:
    """
    a widget that displays a parse tree.
    """
    def __init__(self,parent,limit):
        self.ongoing_ = []
        self.finished_ = []
        self.limit_ = limit
    
    def updatedata(self,data_finished,data_ongoing):
        if data_finished==[] and data_ongoing == self.ongoing_:
            return False
        else:
            self.ongoing_ = data_ongoing
            # TODO: not sure how well *+* scales, finished_ list could be _huge_
            self.finished_ = data_finished+self.finished_
            rospy.loginfo('Parser num_ongoing: {}, num_finished: {}'.format(len(self.ongoing_), len(self.finished_)))
            return True
    
    def getfinished(self):
        if self.limit_==0:
            return self.finished_
        else:
            return self.finished_[:self.limit_]
    
    def cleardata(self):
        self.widget.clear()
    
    def download_pdf(self):
        self.widget.download(pdf=True)
    
    def download_png(self):
        self.widget.download(pdf=False)

    def get_episode_name(self):
        return "Episode"
    
    def get_episode_interval(self):
        begin0,end0 = (None,None)
        begin1,end1 = (None,None)
        if len(self.finished_)>0:
            begin0,end0 = self.get_interval(self.finished_)
        if len(self.ongoing_)>0:
            begin1,end1 = self.get_interval(self.ongoing_)
        begin = begin0 or begin1
        end   = end1 or end0
        return (begin,end)
    
    def get_episode_tooltip(self):
        begin,end = self.get_episode_interval()
        return "[{},{}]".format(begin,end)

    def get_item_tooltip(self,data):
        if data['type']=='action':
            begin,end = self.get_interval(data['children'])
            return "[{},{}]: '{}--{}'".format(begin,end,data['event_type'],data['description'])
        elif data['type']=='phase':
            return "[{}] {}".format(data['event_time'],', '.join(data['participants']))

    def get_interval(self,data):
        if isinstance(data, list):
            x = list(map(lambda x: self.get_interval(x), data))
            begin,_ = min(x)
            _,end   = max(x)
            return (begin,end)
        elif data['type']=='action':
            return self.get_interval(data['children'])
        elif data['type']=='phase':
            return (data['event_time'],data['event_time'])

class ParserTree(ParserWidget):
    """
    pyrap.Tree
    TODO: control what branches are collapsed.
    TODO: would be nice if Tree widget would have a proper update function.
           currently it disposes all, and re-adds
    """
    def __init__(self,parent,limit=5):
        ParserWidget.__init__(self,parent,limit=limit)
        self.widget = Tree(parent, halign='fill', valign='fill')
    
    def updatedata(self,data_finished,data_ongoing):
        if not ParserWidget.updatedata(self,data_finished,data_ongoing):
            return False
        else:
            self.widget.setdata({
                'name': self.get_episode_name(),
                'tooltip': self.get_episode_tooltip(),
                'showname': 1,
                'type': None,
                'children': self.get_tree_data_(self.ongoing_ + self.getfinished())
            })
            return True

    def get_tree_data_(self,data):
        if isinstance(data, list):
            return list(map(lambda x: self.get_tree_data_(x), data))
        else:
            children = self.get_tree_data_(data['children'])
            return {
                'name': data['event_polarization']+data['event_type'],
                'tooltip': self.get_item_tooltip(data),
                'showname': 1,
                'type': data['type'],
                'children': children
            }

class ParserGraph(ParserWidget):
    """
    pyrap.Graph
    """
    def __init__(self,parent,limit=1):
        ParserWidget.__init__(self,parent,limit=limit)
        self.widget = Graph(parent, halign='fill', valign='fill')

    def updatedata(self,data_finished,data_ongoing):
        if not ParserWidget.updatedata(self,data_finished,data_ongoing):
            return False
        else:
            graph_data=[]
            for act in self.ongoing_ + self.getfinished():
                graph_data.append({
                    'arcStyle': 'green',
                    'value': 'includes',
                    'source': 'Episode',
                    'target': self.get_graph_node_(act)
                })
            self.load_graph_data_(data,graph_data)
            self.widget.updatedata(graph_data)
            return True
    
    def load_graph_data_(self,data_in,data_out):
        if isinstance(data_in, list):
            for evt in data_in:
                self.load_graph_data_(evt,data_out)
        elif data_in['type']=='phase':
            pass
        elif data_in['type']=='action':
            self.load_action_node_(data_in,data_out)
            for child in data_in['children']:
                self.load_graph_data_(child,data_out)

    def load_action_node_(self,act_term,data_out):
        for child_term in act_term['children']:
            data_out.append({
                'arcStyle': 'green',
                'value': child_term['type'],
                'source': self.get_graph_node_(act_term),
                'target': self.get_graph_node_(child_term)
            })
    
    def get_graph_node_(self,data):
        return {'name': data['event'], 'show': 1, 'text': self.get_item_tooltip(data)}

class EventTimeline(DetachedSessionThread):
    def __init__(self,parent,timeframe=30.0,rate=0.5):
        DetachedSessionThread.__init__(self)
        self.push_ = PushService()
        self.is_running_ = False
        self.has_stopped_ = True
        self.rate_ = rate
        ##
        self.plot_data = None
        self.events = []
        self.timeframe = timeframe
        self.widget = Scatterplot(parent, halign='fill', valign='fill')
        self.widget.axeslabels(xlabel='time', ylabel='')
        self.widget.formats(xformat=['', '.0f', ''], yformat=['', '', ''])
        self.widget.setdata([])
        self.widget.timeline(timerange=10.0, rate=0.05)
    
    def stop_thread(self):
        self.is_running_ = False
    
    def run(self):
        self.is_running_ = True
        self.has_stopped_ = False
        self.push_.start()
        try:
            while self.is_running_:
                if self.plot_data!=None:
                    self.widget.setdata({"scatter": self.plot_data})
                    self.plot_data=None
                    self.push_.flush()
                time.sleep(self.rate_)
        finally:
            self.is_running_ = False
            self.has_stopped_ = True
            self.push_.stop()
    
    def add_event(self,timestamp_s,event_type,event_descr):
        event_name = event_type.split("#")[-1]
        self.events.append((timestamp_s, {'name': event_name, 'x': timestamp_s, 'y': 0.5, 'tooltip': event_descr}))
        self.events.sort()
        if len(self.events)>1:
            last_event_time=self.events[-1][0]
            while last_event_time-self.events[0][0]>self.timeframe:
                self.events=self.events[1:]
        self.plot_data=list(map(lambda x: x[1],self.events))

# a global used to shut down threads
PARSER_PAGES=[]

class ParserPage:
    """
    A shell that interacts with KnowRob's activity parser.
    """
    def main(self, **kwargs):
        self.is_parser_running_=False
        self.parser_id_=None
        self.prolog = rosprolog.Prolog(wait_for_services=False)
        shell = Shell(title='Manipulation Activity Parsing', maximized=True)
        self.shell = shell
        self.create_tree_page(shell.content)
        #
        self.token_subscriber = rospy.Subscriber('/parser/token', EventToken, self.push_token)
        #
        PARSER_PAGES.append(self)
        # 
        shell.on_resize += shell.dolayout
        shell.show(pack=True)

    def push_token(self,tok):
        timestamp_s = tok.timestamp/1000.0
        event_type  = tok.event_type
        event_descr = tok.event_type
        self.timeline.add_event(timestamp_s,event_type,event_descr)

    def get_parser_id(self):
        if self.parser_id_==None:
            self.prolog.all_solutions("parser_create(parser0)")
            self.parser_id_ = 'parser0'
        return self.parser_id_

    def start_parser(self):
        self.prolog.all_solutions("parser_start('{}')".format(self.get_parser_id()))
        self.btn_toggle.text='Stop'
        self.is_parser_running_=True
        ##
        self.parser_poller_=ParserPoller(self)
        self.parser_poller_.start()

    def stop_parser(self):
        self.parser_poller_.stop_poller()
        self.parser_poller_=None
        ##
        self.prolog.all_solutions("parser_stop('{}')".format(self.get_parser_id()))
        self.btn_toggle.text='Start'
        self.is_parser_running_=False
    
    def stop_threads(self):
        self.stop_parser()
        self.timeline.stop_thread()

    def update(self):
        """ query parser state and update widgets. """
        query = self.prolog.query("parser_pop_finalized({},_Fin), parser_intermediate_results({},_On), parser_jsonify(_Fin,JSON0), parser_jsonify(_On,JSON1)".format(self.get_parser_id(),self.get_parser_id()))
        solution = next(query.solutions())
        return self.widget.updatedata(json.loads(solution['JSON0']),json.loads(solution['JSON1']))

    def create_tree_page(self, parent):
        """ create widgets. """
        grp = Group(parent)
        grp.layout = RowLayout(halign='fill', valign='fill', flexrows=1)
        # button area
        comp_btn = Composite(grp)
        comp_btn.layout = ColumnLayout(halign='fill', valign='fill', equalwidths=True)
        # main area
        comp_body = Composite(grp)
        comp_body.layout = CellLayout(halign='fill', valign='fill')
        # buttons
        self.btn_toggle = Button(comp_btn, text='Start', halign='fill', valign='fill')
        btn_download = Button(comp_btn, text='Download', halign='fill', valign='fill')
        # TODO: add a combo to select
        #self.widget = ParserGraph(comp_body)
        self.widget = ParserTree(comp_body)
        # plot area
        comp_timeline = Composite(grp)
        comp_timeline.layout = CellLayout(halign='fill', minheight='240px')
        comp_timeline.bg = Color('#F5F5F5')
        self.timeline = EventTimeline(comp_timeline)
        self.timeline.start()
        # event handler
        def download(*_):
            selection = options_list(comp_body,["OWL","JSON","PDF","PNG"])[1]
            if selection=="OWL":
                self.download_owl()
            elif selection=="JSON":
                self.download_json()
            elif selection=="PDF":
                self.widget.download_pdf()
            elif selection=="PNG":
                self.widget.download_png()
        def toggle(*_):
            if self.is_parser_running_:
                self.stop_parser()
            else:
                self.widget.cleardata()
                self.start_parser()
        self.btn_toggle.on_select += toggle
        btn_download.on_select += download

    def download_json(self):
        # TODO JSON export
        msg_err(self.shell, 'Download failed', 'JSON export not yet implemented.')

    def download_owl(self):
        # TODO OWL export
        msg_err(self.shell, 'Download failed', 'OWL export not yet implemented.')


if __name__ == '__main__':
    rospy.init_node('parser_vis')
    pyrap.register(name='Manipulation Activity Parsing',
                   clazz=ParserPage,
                   entrypoints={'start': ParserPage.main},
                   path='parsing')
    def signal_handler(sig, frame):
        for pp in PARSER_PAGES:
            pp.stop_threads()
    signal.signal(signal.SIGINT, signal_handler)
    pyrap.run()
