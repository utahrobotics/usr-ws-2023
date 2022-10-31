# usr-ws-2023
Codebase (and other dependencies) for running our candidate for the 2023 competition
This document shall contain as much information pertaining to the programming team as possible.

## Coding Style
### Overall
Top level directories in `src` should contain *significant* content. The most common type of folder would be a ROS package. Another would be global dependencies. Creating these should be a team decision. Only add code within the top level directories, and ensure that the code is used in those directories and not anywhere else. Follow the layout of a package as well; There is a rightful place for all files, and putting stuff in the wrong place can lead to confusion.

All scripts, regardless of programming language, should be kept small. Big scripts are candidates for refactoring, or reconsideration. Remember, the easiest code to read, write, and debug is no code. If your script is getting to big, you may want to do some research to see if there is a library that does at least some of what you need. Scripts should also only have one overarching goal in mind. It is perfectly fine to have multiple scripts (ideally Nodes) that all must work together to accomplish just one competition task (ie. digging).

You may notice that GitHub does not let you push to `main` directly (hopefully). This is intentional so that we can keep `main` at a functional state. There is usually always a branch ready for you to write code to. If there isn't, ask the team lead before making your own, as having two or more branches that should contain the same thing can be very confusing. To merge to main, you must make a pull request, and wait for it to get approved. Before creating a request, you should check that your branch passes all automated checks (see below sections).

Documentation is a thing we care about here. At the very least, you should have in-line comments for complex sections of code. Your team lead will carry out review sessions so you can better understand what is hard to read. Ideally, you should also have method, class, and module/script documentation, even package documentation if you are making one. Everyone is encouraged to add their names to the script documentation as an author if they feel they have modified it in a significant way.

ROS packages will also contain a `README.md`, mainly to explain what nodes are contained within, how these nodes should be used, and what to expect when using these nodes. Any script that is not a node does not have to be explained here (but they still need their own documentation).

If you copy a significant code snippet (highly subjective), or an entire script, you are recommended to cite it (refer to below sections)
If it does not contain sufficient documentation, you must add it yourself.

Do not commit files that other people do not need. The usual way to avoid this is to add the file(s) to the gitignore. However, since everyone shares this gitignore, it can cause unintended side effects with name collisions. As such, updates to gitignore are a team decision. Instead, you should use your local gitignore (refer to reading list)

Unit testing is a strong recommendation as well. Understandably, many components of our code cannot be unit tested as it involves physical interactions. However, if you follow proper separation of concerns, there should be plenty of code to unittest. Individual tests should be small, and test suites should test one class or module. All test suites should be contained within the `test` folder of the package that contains the code being tested

### Python
We are to follow all of Python's PEP Style Guides. PyCharm (and probably otthers) should do this automatically, so just follow all the advice it gives you. Ideally, code should not contain any warnings, so you are strongly encouraged to resolve any you see. If a warning was given incorrectly by your IDE, you should leave a comment as follows:

`# Resolved Warning: <explanation>`

All Python scripts shall use 4 spaces per indent, not a tab. If there's one thing to take away from all this, it is this statement. This has caused *many* issues in the past.

Pylint is automatically ran on each push, so be sure to check it after making your last push (The cross or tick that can be found next to the commit ID)

![](https://manglemix.com/usr_files/checks.png)

Pylint can be pretty strict, so if you feel that certain warning is unnecessary, you can ignore it until you make a pull request. You can bring it up inside the request discussion, and the team lead may consider loosening the restrictions.

File sizes for scripts should try to stay under 8 KB. Any higher is a sign that you should write a package instead (also refer to Overall). You are strongly encouraged to use existing python libraries to help you. 

It is very important to note that ROS applications are not run the same way as regular python applications. Installing a package to Python with pip does not mean you can use it in a script that is run with ROS. Refer to [rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)

Type hints are strongly recommended. IDEs like PyCharm can even help you debug your code if you add it. It is a good form of documentation as well.

Unit tests shall be done with `pytest`, which is an extremely simple library.

How to cite python code:

`# Taken from <web link>`


### VSCode Dev Container Setup

Required Extentions:

- Dev Containers
- Remote - SSH
- Remove Development
- WSL (for windows users)

Setup:

1) Clone Repository
2) Confirm Required Extentions are installed and DockerEngine (Or Docker Desktop) is running.
3) On the bottom left of the VSCode window there should be a little icon that has two arrow icons (kinda like: ><), click that button
4) After, a menu should pop up with loads of options. Click the options that says "Reopen In Container"
5) Visual Studio should restart and pull/build the docker image. This can take some time as it requires a decent internet connection.
6) Once the image is built, the new VSCode window should appear full inside the container.

## Reading List
Below is a list of things you should read up about
1. [ROS2 Documentation](https://docs.ros.org/en/humble/index.html)
2. [Markdown Guide](https://www.markdownguide.org/)
3. [ROS2 Simplified Installation](https://docs.google.com/document/d/1emTYbDWZH72I8Ifpqjb3O8foFpgskyILs_1xUKBiDWM/edit?usp=sharing)
4. [Python Packages Guide](https://realpython.com/python-modules-packages/)
5. [Python Documentation Guide](https://realpython.com/documenting-python-code/)
6. [Local Gitignore](https://stackoverflow.com/questions/49305201/gitignore-only-on-local)
7. [Pytest](https://docs.pytest.org/en/7.1.x/getting-started.html)
8. [ROS 2 Navigation](https://navigation.ros.org/index.html)
