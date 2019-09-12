knowrob_memory
===

An interface for the acquisition of *experiential knowledge*.
Such knowledge is generally acquired by the *monitored*
execution of some plan.
This package provides predicates that create descriptions
of the plan execution, and that are used to load previous experiences
into the knowledge base.

The main usecase is to create a library of episodic memories
that supports selection of episodes in an expressive language,
and that links descriptions of episodes to low-level data.
Such a collection of episodic memories is a valuable source
of information to e.g. find patterns in activities,
learn motor control behaviours, etc.

### Usage

Start logging by creating a new episode

    mem_episode_start(Episode).

By default, only *tf* messages will be logged.
A second clause of the rule allows to specify
a list of topics that shall be logged

    mem_episode_start(Episode,[['tf',['__recorded']]]).

Above statement causes *tf* messages to be recorded where
the *__recorded* field is indexed. 

Once the episode is running you can advance it by adding
events, objects, or sub-episodes to it.
Generally speaking, knowrob_memory creates a tree of episodes
and sub-episodes.
For example, actions can may be added to the episode

    mem_event_create(Episode,mem_test:'TestTask',TskNode).

Here, *TskNode* is the new node in the episode tree, and
*TestTask* is the task type (i.e. subclass-of *Task*)
that classifies the action that is executed.

Once the execution has finished, the acquired experiential knowledge
can be stored on harddrive which is triggered by calling

    mem_episode_stop(Episode)
