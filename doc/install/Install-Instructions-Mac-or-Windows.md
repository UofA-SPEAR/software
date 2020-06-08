# Install Instructions for Mac or Windows

If you do not have a native installation of linux, you will have to set up a virtual machine that runs linux.


## Create a linux VM in VirtualBox

If you already have a linux VM in VirtualBox or VMWare, then you can skip this step.

Otherwise, here is a tutorial to create a Ubuntu VM in VirtualBox on Mac:
https://www.dev2qa.com/how-to-install-ubuntu-on-virtualbox-mac/:

Here is one for Windows:
https://itsfoss.com/install-linux-in-virtualbox/

Note that it doesn't really matter which linux distro or version you install, as long as it can run docker.

Make sure you give your VM adequate disk space and ram.

## Install Git in the VM

You will need to install git in order to clone and work on our repo.

On Ubuntu, git can be installed as follows:

```bash
sudo apt-get install git
```

To install git on other distros see the [git website](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git).

## Clone our repo in the VM

In your linux VM, run:

```bash
git clone https://github.com/UofA-SPEAR/software.git
```

This will create a new directory called `software`.
Enter this new directory by running `cd software`.

## Set up Docker in the VM

From this point, you should be able to follow the instructions in the main README of the sofware repo:

https://github.com/UofA-SPEAR/software

Start at "Docker setup and install instructions" and go until "Run Docker container interactively".
Run the listed commands in your linux VM.
Note that whenever the README refers to a "host" machine or system, this refers to your linux VM, not your Mac or Windows system.

Note that after you add your user to the "docker" group, you may have to restart the VM, not just log out.

