Introduction
===================
In this read.me, we will walk you through the process of setting up a local environment called "pd-final" and installing the necessary packages. We will assume that you already have Python 3.6 or above and that your python is below version 3.10 and pip already installed.

### Installation
Open a terminal window and navigate to the directory where you want to create the pd-final environment.

You first have to downlad the repository.

``` {.sourceCode .bash}
git clone git@github.com:WillemMomma/Planning_and_decision_making.git
```

### Setting up the pd-final environment

Run the following command to create the environment:

``` {.sourceCode .bash}
python -m venv pd-final
```

Activate the enviroment with the following command:


``` {.sourceCode .bash}
# Windows
pd-final\Scripts\activate

# Linux or macOS
source pd-final/bin/activate
```

Check if your python has the requierd version:

``` {.sourceCode .bash}
python --version
```

Verify that you have the required python version, the version needs to be between 3.6 and 3.9. If you have the incorrect version please scroll down to the "Alternitive setup-way".

With the pd-final environment active, run the following command to install the cvxpy, shapely, motion-planning-scenes packages

``` {.sourceCode .bash}
pip install cvxpy shapely motion-planning-scenes
```

Finally install urdfenvs package with the following command:

``` {.sourceCode .bash}
pip install urdfenvs
```

Congratulations you have succesfully installed the URDFenviroment. To check if every thing works as it should run the following command. in the ../planning_and_decision_making>

``` {.sourceCode .bash}
python main.py
```




