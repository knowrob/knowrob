KnowRob
=======

![CI](https://github.com/knowrob/knowrob/workflows/CI/badge.svg)

KnowRob is a knowledge processing system designed for robots.
Its purpose is to equip robots with the capability to organize information in re-usable
knowledge chunks, and to perform reasoning in an expressive logic.
It further provides a set of tools for visualization and acquisition of knowledge.

# NEW 

## Architecture / Use Case

Knowledge is stored in a graph using a given ontology and ... .
Users can query this graph using KnowRobs interfaces from their own applications. 


## Dependencies

- SWI Prolog >= 8.2.4 (see [Further Information](https://github.com/artnie/knowrob/tree/update-setup#further-information))
- mongo DB server >= 4.4 and libmongoc (see [Further Information](https://github.com/artnie/knowrob/tree/update-setup#further-information))
- [spdlog](https://github.com/gabime/spdlog.git)
- [raptor2](https://librdf.org/raptor/)
- [FMT](https://github.com/fmtlib/fmt)

### ROS Version only

- [ROS](http://wiki.ros.org/noetic/Installation/Ubuntu) (*ROS noetic* for the master branch)

- [??? rosprolog](https://github.com/knowrob/rosprolog)



## Getting Started

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
sudo apt upgrade

# or use this command? upgrade seems harsh if we only want to upgrade swi prolog
sudo apt install swi-prolog
```

This installs version 9.0.4 which apparently does not provide -lswipl

### Installing MongoDB


```bash
# Install mongodb
sudo apt install libmongoc-dev
# Check the mongodb version
mongod --version
# If below 4.4, an update is needed.
# Updating the mongodb requires either wiping all existing DBs or dumping/restoring them.
# Newer versions are not compatible with old DBs and wouldn't even allow the mongodb service to start
# Therefore, if you want to keep old DBs, store them BEFORE upgading mongodb.
# The following procedure reinstalls mongodb completely with the desired version. 
# All previous deps, settings, DBs of mongodb will be lost!

# Stop the service
sudo systemctl stop mongod.service
# Remove all DBs
sudo rm -r /var/log/mongodb
sudo rm -r /var/lib/mongodb
# Uninstall all mongo packages
# Be aware that this also removes unrelated packages starting with 'mongo*'
sudo apt purge mongo*
# Fetch the latest packages of mongodb-org
wget -qO - https://www.mongodb.org/static/pgp/server-4.4.asc | sudo apt-value add -
echo "deb [ arch=amd64,arm64 ] https://repo.mongodb.org/apt/ubuntu bionic/mongodb-org/4.4 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-4.4.list
# Update references and install mongodb
sudo apt update
sudo apt install mongodb-org

# Troubleshoot: If dpkg errors occurr, the deps still refer to old versions. Force the new version
# Replace the <version> with your own new version. To this day it is '4.4.10'. 
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



### Apt Packages for Ubuntu 20.04 LTS and higher?

(Tested using a WSL2 and Windows 11)

Does not install ROS as that requires key setup before.
Follow [these instructions](http://wiki.ros.org/noetic/Installation/Ubuntu).

```
sudo apt update
sudo apt install libspdlog-dev swi-prolog raptor2-utils libraptor2-dev libmongoc-1.0-0 libmongoc-dev libfmt-dev
```



## END NEW



## Getting Started

These instructions will get you a copy of KnowRob up and running on your local machine.

### Prerequisites

- ROS (*ROS noetic* for the master branch)
- SWI Prolog >= 8.2.4 (see [Further Information](https://github.com/artnie/knowrob/tree/update-setup#further-information))
- mongo DB server >= 4.4 and libmongoc (see [Further Information](https://github.com/artnie/knowrob/tree/update-setup#further-information))
- [rosprolog](https://github.com/knowrob/rosprolog)

### Installation

KnowRob uses the [catkin](http://wiki.ros.org/catkin) buildsystem that has been the main ROS buildsystem.
We have prepared different *.rosinstall* setup files that you can add to your ROS workspace as described [here](http://www.ros.org/wiki/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

```Bash
rosdep update
cd ~/catkin_ws/src
wstool init
wstool merge https://raw.github.com/knowrob/knowrob/master/rosinstall/knowrob-base.rosinstall
wstool update
rosdep install --ignore-src --from-paths .
cd ~/catkin_ws
catkin_make
```

You may further need to set the *SWI_HOME_DIR* environment variable to the installation location of *swipl*:

```
export SWI_HOME_DIR=/usr/lib/swi-prolog
```

### Launching

In order to interact with other robot components,
the [Robot Operating System](https://www.ros.org/) (ROS)
is used via [rosprolog](https://github.com/knowrob/rosprolog).
*rosprolog* provides a ROS node that manages a pool of Prolog engines
in which KnowRob can be loaded such that its querying interface
is exposed via the node.
KnowRob provides a launch file that starts the *rosprolog* node, and initializes KnowRob:

```
roslaunch knowrob knowrob.launch
```

Please refer to the *rosprolog* documentation for how to interact with this node.
However, KnowRob can also be launched without ROS node through a script offered by *rosprolog*:

```
rosrun rosprolog rosprolog knowrob
```

Launching KnowRob without the ROS node may help debugging (at the moment the GUI tracing tool of SWI Prolog *gtrace* does not work via the rosprolog node).

## Getting Familiar

Here we provide an overview about functionality of KnowRob.

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

## Further Information

- Sourcecode documentation is available [here](https://knowrob.github.io/knowrob/)
- A blog and more wiki pages are avaiabale at [knowrob.org](http://www.knowrob.org)

### Installation of SWI-Prolog and MongoDB

- SWI Prolog latest stable version

```bash
# If swi-prolog is already installed, check the version
swipl --version
# If it's under 8.2.4, add the ppa of the latest stable version
sudo apt-add-repository -y ppa:swi-prolog/stable
# And update it
sudo apt update
sudo apt upgrade
# or install it, if not present before.
sudo apt install swi-prolog
```

- MongoDB

```bash
# Check the mongodb version
mongod --version
# If below 4.4, an update is needed.
# Updating the mongodb requires either wiping all existing DBs or dumping/restoring them.
# Newer versions are not compatible with old DBs and wouldn't even allow the mongodb service to start
# Therefore, if you want to keep old DBs, store them BEFORE upgading mongodb.
# The following procedure reinstalls mongodb completely with the desired version. 
# All previous deps, settings, DBs of mongodb will be lost!

# Stop the service
sudo systemctl stop mongod.service
# Remove all DBs
sudo rm -r /var/log/mongodb
sudo rm -r /var/lib/mongodb
# Uninstall all mongo packages
# Be aware that this also removes unrelated packages starting with 'mongo*'
sudo apt purge mongo*
# Fetch the latest packages of mongodb-org
wget -qO - https://www.mongodb.org/static/pgp/server-4.4.asc | sudo apt-value add -
echo "deb [ arch=amd64,arm64 ] https://repo.mongodb.org/apt/ubuntu bionic/mongodb-org/4.4 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-4.4.list
# Update references and install mongodb
sudo apt update
sudo apt install mongodb-org

# Troubleshoot: If dpkg errors occurr, the deps still refer to old versions. Force the new version
# Replace the <version> with your own new version. To this day it is '4.4.10'. 
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
