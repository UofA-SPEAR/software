# New Member Activity

Welcome to SPEAR!

This is a little exercise that is meant to give you a gentle introduction to ROS and set up your development environment.
It will also show you how to contribute to our github repo using a pull request.

ROS (Robot Operating System) is not actually an operating system, but a bunch of libraries and programs that are useful for building robots.
You can read more about ROS [here](https://www.ros.org/about-ros/).
We use ROS for nearly all the software on the rover.

Don't be afraid to ask lots of questions, especially if you get stuck!
Also refer to the "Further Resources" section at the bottom of this page if you want to learn more about a specific skill such as using the terminal.

## 1. Set up the development environment

You can develop either from a docker container or from a virtual machine.

### Docker

The recommended way to run all our software is inside a Docker container.
Docker containers are kinda like virtual machines except they are much more lightweight.
Docker will take care of installing ROS and all the necessary dependencies for our code.
The downside is you will need Linux in order to display GUI applications running from inside the container.
You can learn more about Docker [here](https://www.docker.com/resources/what-container)

See [the readme](https://github.com/UofA-SPEAR/software#docker-setup-and-install-instructions) for setup instructions.

### Virtual machine

If you are running Windows and don't want to dual-boot Linux (due to space constraints, for example), you can develop within a VM.

See [the readme](https://github.com/UofA-SPEAR/software#virtual-machine-setup-and-install-instructions) for setup instructions.

## 2. Write a simple ROS publisher and subscriber

Create 2 simple ROS nodes by following our tutorial [here](https://github.com/UofA-SPEAR/software/wiki/ROS-publisher-and-subscriber).

## 3. Push to github and create a pull request

Pull requests are how you can contribute code to our github repository.
In this step, you will create a pull request containing the new nodes that you just created.
Since this is just an exercise, we will not actually accept or merge your pull request, however you will go through all the steps up until your pull request would get merged.

Before we make a pull request, we will create a new branch of the git repository to store our changes before they are merged into the master branch.
(See [this](https://guides.github.com/introduction/flow/) guide for more information about the github workflow.)

On the system where you cloned the git repo, open a terminal and enter the `software` directory.

Now run this command:

```bash
git status
```

You should see something like this:

```
On branch master
Your branch is up to date with 'origin/master'.

Untracked files:
  (use "git add <file>..." to include in what will be committed)
	spear_rover/nodes/listener.py
	spear_rover/nodes/talker.py

nothing added to commit but untracked files present (use "git add" to track)
```

To create a new branch, run this command (replacing \<your name\> with your first name):

```bash
git checkout -b new-member-<your name>
```

This will create a new branch called `new-member-<your name>`.

Previously, we ran `git status` and saw that under `Untracked files`, git was notifying us that there are 2 new files which we haven't started tracking with git yet.

To get git to track these files, run the following 2 commands:

```bash
git add -A
git commit -m "Add simple publisher and subcriber"
```

The `git add -A` command will tell git which files we want to commit.
The `-A` flag adds every new or recently edited file in the repo.
The `git commit` command makes a commit which is essentially a snapshot of the changes you just made.

At this point, the local copy of the git repo will have the new commit but the remote github will not.
To get the changes you made onto github, we must make a push:

```bash
git push -u origin new-member-<your name>
```

Now your commit and new branch have been sent to github.

Next, go to the github repo in your browser: https://github.com/UofA-SPEAR/software

Click on the "Pull Requests" tab, then click on the "New Pull Request" button.

Set "base: master" and "compare: new-member-\<your name\>".
This means that you are requesting the commits from your branch to be applied to master.

Once you have set the base and compare branches, click "Create pull request".

You are now done the new member activity!

## Further Resources

### Languages

* C++ tutorial: http://www.cplusplus.com/doc/tutorial/
* Python 2 tutorial: https://docs.python.org/2/tutorial/ 
  * note: ROS uses python 2, not python 3 :(

### ROS

* Official ROS tutorials: http://wiki.ros.org/ROS/Tutorials

### Using the terminal

* Basic bash commands: https://medium.com/@jasonrigden/a-brief-tutorial-of-the-absolute-basics-of-bash-9a379c49f7a8

### Git and GitHub

* GitHub guides: https://guides.github.com/

### Tmux

* tmux cheat sheet: https://gist.github.com/MohamedAlaa/2961058
