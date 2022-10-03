# usr-ws-2023
Codebase (and other dependencies) for running our candidate for the 2023 competition
This document shall contain as much information pertaining to the programming team as possible.

## Coding Style
#### Overall
Top level directories in `src` should contain *significant* content. The most common type of folder would be a ROS package. Another would be global dependencies. Creating these should be a team decision. Only add code within the top level directories, and ensure that the code is used in those directories and not anywhere else. 

All scripts, regardless of programming language, should be kept small. Big scripts are candidates for refactoring, or reconsideration. Remember, the easiest code to read, write, and debug is no code. If your script is getting to big, you may want to do some research to see if there is a library that does at least some of what you need.

You may notice that GitHub does not let you push to `main` directly (hopefully). This is intentional so that we can keep `main` at a functional state. There is usually always a branch ready for you to write code to. If there isn't, ask the team lead before making your own, as having two or more branches that should contain the same thing can be very confusing. To merge to main, you must make a pull request, and wait for it to get approved. Before creating a request, you should check that your branch passes all automated checks (see below sections).

Documentation is a thing we care about here. At the very least, you should have in-line comments for complex sections of code. Your team lead will carry out review sessions so you can better understand what is hard to read. Ideally, you should also have method, class, and module/script documentation, even package documentation if you are making one. Everyone is encouraged to add their names to the script documentation as an author if they feel they have modified it in a significant way.

ROS packages will also contain a `README.md`, mainly to explain what it does, and all the ways it accesses anything outside the package. This includes messages, services, actions, wireless communication, filesystem actions (eg. reading or writing files), and GPIO. These files should ideally be written with Markdown, but this is not necessary (good skill to have though).

If you copy a significant code snippet (highly subjective), or an entire script, you are recommended to cite it (refer to below sections)
If it does not contain sufficient documentation, you must add it yourself.

#### Python
We are to follow all of Python's PEP Style Guides. PyCharm (and probably otthers) should do this automatically, so just follow all the advice it gives you. Ideally, code should not contain any warnings, so you are strongly encouraged to resolve any you see. If a warning was given incorrectly by your IDE, you should leave a comment as follows:
`# Resolved Warning: <explanation>`
Pylint is automatically ran on each push, so be sure to check it after making your last push (The cross or tick that can be found next to the commit ID)
![](https://manglemix.com/usr_files/checks.png)

File sizes for scripts should try to stay under 8 KB. Any higher is a sign that you should write a package instead (also refer to Overall). You are strongly encouraged to use existing python libraries to help you. 

It is very important to note that ROS applications are not run the same way as regular python applications. Installing a package to Python with pip does not mean you can use it in a script that is run with ROS. Refer to [rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)

Type hints are strongly recommended. IDEs like PyCharm can even help you debug your code if you add it. It is a good form of documentation as well.

How to cite python code:
`# Taken from <web link>`

## Reading List
Below is a list of things you should read up about
1. [ROS2 Documentation](https://docs.ros.org/en/humble/index.html)
2. [Markdown Guide](https://www.markdownguide.org/)
3. [ROS2 Simplified Installation](https://docs.google.com/document/d/1lkOGz_ISWwEarPJ8ZaNjJ-dXnlfeb1B63kSqP9ng6hY/edit?usp=sharing)
4. [Python Packages Guide](https://realpython.com/python-modules-packages/)
5. [Python Documentation Guide](https://realpython.com/documenting-python-code/)