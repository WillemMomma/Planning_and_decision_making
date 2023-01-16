# Planning_and_decision_making
Group 21: Willem Momma, Jasper van Leuven, Godert Notten, Willem Kolff


<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
    </li>
    <li>
      <a href="#getting-started">Nodes</a>
      <ul>
        <li><a href="#prerequisites">State machine</a></li>
        <li><a href="#prerequisites">Enviroment</a></li>
        <li><a href="#prerequisites">Local path planner</a></li>
        <li><a href="#prerequisites">MPC</a></li>
        <li><a href="#prerequisites">Global path planner</a></li>
      </ul>
    </li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project
# insert GIF 

The field of warehouse automation has seen significant advancements in recent years with the introduction of robots capable of performing various tasks such as pick and place, transportation, and inventory management. However, one of the key challenges in warehouse robotics is motion planning, which involves generating safe and efficient paths for the robot to navigate through the cluttered and dynamic environment. This project proposes a motion planning pipeline for a unicycle warehouse robot. The pipeline contains a global planner, RRT*, local planner, MPC, and implements obstacle avoidance. The simulation is modelled in the URDF environment from: https://github.com/maxspahn/gym_envs_urdf/tree/master/docs. 

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- GETTING STARTED -->
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


<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

Use this space to list resources you find helpful and would like to give credit to. I've included a few of my favorites to kick things off!

* [Choose an Open Source License](https://choosealicense.com)
* [GitHub Emoji Cheat Sheet](https://www.webpagefx.com/tools/emoji-cheat-sheet)
* [Malven's Flexbox Cheatsheet](https://flexbox.malven.co/)
* [Malven's Grid Cheatsheet](https://grid.malven.co/)
* [Img Shields](https://shields.io)
* [GitHub Pages](https://pages.github.com)
* [Font Awesome](https://fontawesome.com)
* [React Icons](https://react-icons.github.io/react-icons/search)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/othneildrew/Best-README-Template.svg?style=for-the-badge
[contributors-url]: https://github.com/othneildrew/Best-README-Template/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/othneildrew/Best-README-Template.svg?style=for-the-badge
[forks-url]: https://github.com/othneildrew/Best-README-Template/network/members
[stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=for-the-badge
[stars-url]: https://github.com/othneildrew/Best-README-Template/stargazers
[issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=for-the-badge
[issues-url]: https://github.com/othneildrew/Best-README-Template/issues
[license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=for-the-badge
[license-url]: https://github.com/othneildrew/Best-README-Template/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/othneildrew
[product-screenshot]: images/screenshot.png
[Next.js]: https://img.shields.io/badge/next.js-000000?style=for-the-badge&logo=nextdotjs&logoColor=white
[Next-url]: https://nextjs.org/
[React.js]: https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=for-the-badge&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=for-the-badge&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Svelte.dev]: https://img.shields.io/badge/Svelte-4A4A55?style=for-the-badge&logo=svelte&logoColor=FF3E00
[Svelte-url]: https://svelte.dev/
[Laravel.com]: https://img.shields.io/badge/Laravel-FF2D20?style=for-the-badge&logo=laravel&logoColor=white
[Laravel-url]: https://laravel.com
[Bootstrap.com]: https://img.shields.io/badge/Bootstrap-563D7C?style=for-the-badge&logo=bootstrap&logoColor=white
[Bootstrap-url]: https://getbootstrap.com
[JQuery.com]: https://img.shields.io/badge/jQuery-0769AD?style=for-the-badge&logo=jquery&logoColor=white
[JQuery-url]: https://jquery.com 
