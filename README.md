KnowRob
=======

![CI](https://github.com/knowrob/knowrob/workflows/CI/badge.svg)

[KnowRob](http://www.knowrob.org) is a knowledge processing system designed for robots.
Its purpose is to equip robots with the capability to organize information in re-usable
knowledge chunks, and to perform reasoning in an expressive logic.
It further provides a set of tools for visualization and acquisition of knowledge.

## Outline

1. [Functionality]()
2. [Dependencies]()
3. [Install Instructions]()
4. [Building Knowrob]()
5. [Starting Knowrob]()
6. [Using Knowrob]()


## Functionality

Knowledge is stored in a graph using a given ontology and ... .
Users can query this graph using KnowRobs interfaces from their own applications. 

### Querying

The core of KnowRob is an extendible querying interface that
provides basic operations *ask*, *tell*, *forget*, and *remember*.
Their argument is some statement written in the [KnowRob Querying Language](src/README.md).
Language phrases are terms whose semantics is defined
in form of Prolog rules using special operators such as *?>* (the ask operator),
or *+>* (the tell operator).

One useful CLI for queries is launched with `rosrun rosprolog rosprolog_commandline.py`

### Model

KnowRob structures knowledge according to models represented using RDF.
Some models are very basic and domain-independent such as the OWL model
that e.g. distinguishes between object and datatype properties, or the
toplevel ontology SOMA which is supported by KnowRob.
[KnowRob Models](src/model/README.md) is a collection of such models
that are explicitely supported by KnowRob.
However, support for other models may be added through plugins.

### Triple Store and Data Access

Knowledge is represented in form of temporalized triples --
each subject-predicate-object triple has an additional field
that restricts the temporal scope in which the statement
represented by the triple is true.
A configurable knowledgeGraph is used to store and retrieve temporalized triples --
as a falback implementation, KnowRob provides a simple MongoDB
implementation of a temporalized triple store.

One important aspect in knowledge representation for robots is that
a lot of knowledge is *implicitly* encoded in the control structures
of the robot. Hence, one goal is to make the knowledge in robot
control structures *explicit*.
KnowRob does that through *Ontology-based Data Access* (OBDA).
So called, *semantic data accessors* are used to map data to symbols in
an ontology, often by accessing some database, or by reading from
a message queue, etc.

For more information on database backends in KnowRob, please have a look
[here](src/db/README.md).

### Reasoning

KnowRob uses an ensemble of reasoners approach where inferences
of different reasoners are combined into correlated knowledge pieces.
The reason for choosing this approach is that there is no single
formalism that is suited for every reasoning tasks.
Instead, given a problem, one should decide what the most suitable
formalism is to tackle it.
Consequently, KnowRob can be configured to solve specific problems
by loading corresponding reasoning modules that implement a common interface.
KnowRob also ships with a set of reasoning modules including
an (incomplete) OWL reasoner, a SWRL reasoner, and some specialized
reasoning modules that handle domain-specific problems
such as reasoning about time intervals using Allen's interval
calculus.
More complete information about reasoning in KnowRob can be found
[here](src/reasoning/README.md).


## Dependencies

- [SWI Prolog](https://www.swi-prolog.org/) >= 8.2.4
- [mongo DB server](https://www.mongodb.com/de-de) >= 4.4 and libmongoc
- [spdlog](https://github.com/gabime/spdlog.git)
- [Raptor2](https://librdf.org/raptor/)
- [fmt](https://github.com/fmtlib/fmt)

### ROS Version only

- [ROS](http://wiki.ros.org/noetic/Installation/Ubuntu) (*ROS noetic* for the master branch)

## Install Instructions

These instructions will get you a copy of KnowRob up and running on your local machine.

### Installing SWI Prolog

KnowRob requires SWI Prolog's latest stable version 8.2.4 or higher. 
The latest version shipped with Ubuntu 20.04 or older is 7.6.4,
so you might need to manually update the library.

```bash
# Install from apt
sudo apt install swi-prolog

# Check the version
swipl --version

# If it's under 8.2.4, add the ppa of the latest stable version and update
sudo apt-add-repository -y ppa:swi-prolog/stable
sudo apt update

# upgrade
sudo apt install swi-prolog
```

### Installing MongoDB

KnowRob requires version 4.4 or higher.
If your version is lower you will need to update.

Newer versions are **not** compatible with old DBs and won't allow the mongodb service to start.
To avoid issues later, a complete re-installation of the newer version is performed, 
which requires wiping, dumping or restoring all existing DBs.
As this will delete all previous deps, settings and DBs, 
you should store them **before** starting the update.
Instructions partly taken from [here](https://www.digitalocean.com/community/tutorials/how-to-install-mongodb-on-ubuntu-20-04).

```bash
# Install mongodb
sudo apt install libmongoc-dev libmongoc-1.0-0

# Check the mongodb version
mongod --version

# If below 4.4, an update is needed.
# ! Store your DBs before proceeding !


# Stop the service
sudo systemctl stop mongod.service
# Remove all DBs
sudo rm -r /var/log/mongodb
sudo rm -r /var/lib/mongodb
# Uninstall all mongo packages
# Be aware that this also removes unrelated packages starting with 'mongo*'
sudo apt purge mongo*

## Install version 4.4
# Add the source, should return 'OK'
curl -fsSL https://www.mongodb.org/static/pgp/server-4.4.asc | sudo apt-key add -

echo "deb [ arch=amd64,arm64 ] https://repo.mongodb.org/apt/ubuntu focal/mongodb-org/4.4 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-4.4.list

# Update references and install mongodb
sudo apt update
sudo apt install mongodb-org

# Troubleshoot: If dpkg errors occurr, the deps still refer to old versions. Force the new version
# Replace the <version> with your own new version. To this day it is '4.4.25'. 
sudo dpkg -i --force-overwrite /var/cache/apt/archives/mongodb-org-tools_4.4.<version>_amd64.deb

# Try to run the mongodb service
sudo systemctl start mongod.service
# Check if the service is running properly
sudo systemctl status mongod.service
# Refer to the mongodb error code explaination for further insight: 
# https://github.com/mongodb/mongo/blob/master/src/mongo/util/exit_code.h
# Status 62 identifies old DBs in /var/log and /var/lib, so delete them.
# To instead keep them, you'll need to downgrade mongo, dump DBs, upgrade mongo, recreate DBs.
# When it fails to open /var/log/mongodb/mongod.log the permissons for that file are incorrect.
# Either set owner and group of these two paths to mongodb, or reinstall mongodb.
```



### Installing Raptor2, spdlog and fmt

(Tested using a WSL2 and Windows 11)

```
sudo apt update
sudo apt install libspdlog-dev raptor2-utils libraptor2-dev libfmt-dev
```

### Installing ROS Noetic

Follow [these instructions](http://wiki.ros.org/noetic/Installation/Ubuntu) to install ROS Noetic on your machine.
Only required if you want to use catkin and ROS tools.


## Building Knowrob

### Standalone Package

### ROS Package

To build knowrob, you will need to add this repository to your catkin workspace.
Then proceed by executing

```bash
catkin build knowrob
```

If you encounter errors regarding SWI Prolog or MongoDB versions,
redo the install process and try again.
Remember to 

```bash
catkin clean knowrob
```

If the package still does not build, try modifying the CMakeList.txt slightly, save and build again.
Your build process might still be compromised by old errors and this restarts the build from scratch.


## Starting Knowrob

### Launch command
To launch the basic example, type

```bash
 roslaunch knowrob knowrob.launch
 ```

### Expected Output

You should see something like

```bash
# ROS Startup
...

setting /run_id to 1fa113aa-887c-11ee-8552-00155d76860c
process[rosout-1]: started with pid [19352]
started core service [/rosout]
process[knowrob_ros-2]: started with pid [19355]
[15:41:59.227] [info] Using knowledge graph `mongodb` with type `MongoDB`.
[15:41:59.620] [info] dropping graph with name "user".
[15:41:59.621] [info] Using reasoner `mongolog` with type `Mongolog`.
[15:41:59.663] [info] common foreign Prolog modules have been registered.
## The following will only appear on the first startup or when u delete the mongodb base
[15:42:00.285] [info] Loading ontology at '/home/your_user/catkin_ws/src/knowrob/owl/rdf-schema.xml' with version "Mon Nov 20 11:56:29 2023" into graph "rdf-schema".
[15:42:00.304] [info] Loading ontology at '/home/your_user/catkin_ws/src/knowrob/owl/owl.rdf' with version "Mon Nov 20 11:56:54 2023" into graph "owl".
[15:42:00.329] [info] Loading ontology at '/home/your_user/catkin_ws/src/knowrob/owl/test/swrl.owl' with version "Mon Nov 20 11:56:54 2023" into graph "swrl".
 ```

### Troubleshooting

If your output looks more like this,
the mongoDB server is not running.

```bash
[15:44:50.789] [info] Using knowledge graph `mongodb` with type `MongoDB`.
[15:45:20.791] [error] failed to load a knowledgeGraph: mng_error(create_index_failed,No suitable servers found: `serverSelectionTimeoutMS` expired: [connection refused calling ismaster on 'localhost:27017'])
[15:45:20.791] [info] Using reasoner `mongolog` with type `Mongolog`.
[15:45:20.791] [error] failed to load a reasoner: Reasoner `mongolog` refers to unknown data-backend `mongodb`.
 ```


On native linux systems, you can enable the service using systemd:

```bash
# Try to run the mongodb service
sudo systemctl start mongod.service
# Check if the service is running properly
sudo systemctl status mongod.service
```

On a WSL2 machine you will need to manually launch a mongoDB server as .systemd does not exist in WSL2.
To launch a seperate mongoDB process,  execute the following commands in a new terminal 
(taken from [here](https://github.com/michaeltreat/Windows-Subsystem-For-Linux-Setup-Guide/blob/master/readmes/installs/MongoDB.md)):

 ```bash
# create a directory for the mongoDB data
sudo mkdir -p ~/data/db

# launch mongoDB server with the created path as argument
sudo mongod --dbpath ~/data/db
 ```

If you now re-launch [this example](https://github.com/marcomasa/knowrob#launch-command),
you should see the [expected output](https://github.com/marcomasa/knowrob#expected-output).


## Using Knowrob

### Provided interfaces

Once knowrob is launched, it provides four actions to ask queries:
- askone
- askincremental
- askall
- tell

### Interface libraries

We provide both a C++ and Python library for you to include in your own project.
They implement the action client interface to access knowrob data.


## Further Information

- Sourcecode documentation is available [here](https://knowrob.github.io/knowrob/)
