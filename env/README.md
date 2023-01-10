Introduction
===================
In this read.me, we will walk you through the process of setting up a local environment called "pd-final" and installing the necessary packages. We will assume that you already have Python 3.6 or above and that your python is below version 3.9 and pip already installed.

### Installation
Open a terminal window and navigate to the directory where you want to create the pd-final environment.

1. You first have to downlad the repository.

``` {.sourceCode .bash}
git clone git@github.com:WillemMomma/Planning_and_decision_making.git
```

### Setting up the pd-final environment
The urdfenv requires Python >3.6, <3.9. Check which python version of python is currently active on your computer.

``` {.sourceCode .bash}
python --version
```

If your version is correct just start installation at step 1. Else scroll down to "Alternative setup way".

1. Run the following command to create the environment:

``` {.sourceCode .bash}
python -m venv pd-final
```

2. Activate the enviroment with the following command:


``` {.sourceCode .bash}
# Windows
pd-final\Scripts\activate

# Linux or macOS
source pd-final/bin/activate
```

3. Verify that you have the required python version, the version needs to be between <3.6 and 3.8>.

``` {.sourceCode .bash}
python --version
```


4. With the pd-final environment active, run the following command to install the cvxpy, shapely, motion-planning-scenes packages

``` {.sourceCode .bash}
pip install cvxpy shapely motion-planning-scenes
```

5. Finally install urdfenvs package with the following command:

``` {.sourceCode .bash}
pip install urdfenvs
```

6. Congratulations you have succesfully installed the URDFenviroment. To check if every thing works as it should run the following command in the "../planning_and_decision_making>" folder

``` {.sourceCode .bash}
python main.py
```

### Alternitive setup way.

1. You have a python installed which is 3.5 or lower or 3.9 or higher. By running the following command you can check if you have other python versions installed on your computer. 

``` {.sourceCode .bash}
where python
```
If you do not have any compatible python installed use this link to install python 3.8.8 https://www.python.org/downloads/release/python-388/

2. Run this command to create an enviroment with a compatible python version. For this example we use python 3.8.8.

``` {.sourceCode .bash}
conda create --name pd-final python==3.8.8  #replace 3.8.8 for your version of python
```

3. Activate the enviroment with the following command:
``` {.sourceCode .bash}
conda activate pd-final
```

4. To continue with the setup of the "pd-final" environment, please refer to step 3 in the "Setting up the pd-final environment" section of the guide.


