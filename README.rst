-------------------------------------------
Getting a robotpy project set up in VS Code
-------------------------------------------

Python Install
--------------

Install python 3.12 from https://www.python.org/downloads/  Open the folder the installer downloads to.  Right-click the file and select "Run as Administrator". 
In the installer, be certain to install python for "all users" and add python to the path. 

Install VS Code from https://code.visualstudio.com/download

Open either a terminal in VS Code or a command prompt in the project folder:

```pip``` is a package manager for python. It is used to install python packages.

Install robotpy:

    pip install robotpy

Install all robotpy subpackages:

    pip install robotpy[all]

If you have trouble the instructions for a robotpy install are available
at https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html

You also want to install the following packages via pip:

- numpy (Math)
- hypothesis (Testing)
- sphinx (Documentation)
- doc8 (Documentation)

Virtual Environment (Optional)
==============================

I recommend skipping this step if you are new to Python, but it is good to be aware of.

Virtual environments isolate your project from other projects on your system.
This allows you to run different versions of packages for different projects.
This is unlikely to be used on a dedicated robotics dev machine, but you may
encounter or need it in the wild.

Create a virtual environment for the project.  Use the command palette CTRL+SHIFT+P
and type "Python: Create Virtual Environment"

You may need to enable unrestricted execution policy for the current user
Don't worry about this step if you don't have problems activating the
virtual environment.  The command below needs to be run from powershell.

    Set-ExecutionPolicy -ExecutionPolicy Unrestricted -Scope CurrentUser

VS Code is not always good about activating the virtual environment when a workspace
is opened.  You may need to do this manually. It can be done from the command palate
(CTRL+SHIFT+P) by typing "Python: Select Interpreter" and selecting the virtual
environment.

Alternatively, activate the virtual environment from the terminal:

    .\venv\Scripts\Activate.ps1

Robot Project
-------------

Cloned project
==============

If you cloned this repository, you can skip this step.  Open the
cloned folder in VS Code.

Bare project
============

Go to the folder you want to create a robot project in and run:

    python -m robotpy init

This creates a bare robot project with a ```robot.py``` file and a ```pyproject.toml```
file. The pyproject.toml file is used by the robotpy build system to
determine what to build and deploy to the robot.

Open the project folder in VS Code.

VS Code Configuration
---------------------

I suggest opening VS Code Preferences (CTRL+,) and setting the following:
    Save on focus change: on

This saves files automatically and prevents you from deploying old code by accident.

You want some extension now.  Open the Extension tab (CTRL+SHIFT+X, or left edge button menu, towards the bottom.)

Search for and install the following extensions:

- python
- GitHub Copilot
- GitHub Copilot Chat
- reStructuredText (Optional, for documentation, which we all aspire to write)
- VS Code Speech (Optional, to talk to CoPilot)
- Live Share (Optional)
- GitLens (Optional)


There are other great extensions, if you find yourself wishing a task was easier
poke around the extension list to see if someone has a solution already.

Optional, but highly recommended.  Open the GitHub CoPilot chat tab (below extension library tab on left side). 
Ensure you are logged in to GitHub.  This will allow CoPilot to learn from your code and provide better 
suggestions.  This requires a GitHub Education account, which you can get for free at https://education.github.com/

(When in the code editor, Use CTRL+I to prompt CoPilot at a specific location in the code via chat for more complex requests.)

Never trust CoPilot code without review.  Think of the results as a suggestion, not a solution.

Robotpy Deploy System
---------------------

To deploy python to the robot we need RoboRIO versions of python and the python packages.
The sync command downloads and caches packages for our bot listed in the pyproject.toml file.

    python -m robotpy sync

**Note: If you add a new package to your project, update your toml file and run robotpy sync again.**
    
After that, to deploy your code to the robot, run:

    python -m robotpy deploy

VS Tasks
--------

If you cloned the project, you have a workspace file that defines some
useful tasks.  Run these tasks from the command palette (CTRL+SHIFT+P)
or the terminal menu.  These tasks deploy to the robot, debug, and
run tests.  

Workspace settings may also be accessed directly in the .vscode/settings.json file.
It is essential for auto-complete and code analysis that the python.analysis.include
and python.analysis.extraPaths settings contain the correct paths for your project. 
If you are not using a virtual environment they global python install's site-packages.
If you are using an environment they should point to site-packages folder of that 
environment.

Global install example (Your python install path may be different):

    "python.analysis.include": [ 
        "{env:PyTHONPATH}/Lib/site-packages/*", 
        "./*"
    ],
    "python.analysis.extraPaths": [
        ".",
        "{env:PyTHONPATH}/Lib/site-packages"
    ],

Virtual environment example:

    "python.analysis.include": [ 
        ".venv/Lib/site-packages/*",
        "./*"
    ],
    "python.analysis.extraPaths": [
        ".",
        ".venv/Lib/site-packages"
    ],


NetConsole
==========

The RoboRIO netconsole can be viewed in VS Code using the netconsole task.