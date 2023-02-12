We are to follow all of Python's PEP Style Guides. PyCharm (and probably others) should do this automatically, so just follow all the advice it gives you. Ideally, code should not contain any warnings, so you are strongly encouraged to resolve any you see. If a warning was given incorrectly by your IDE, you should leave a comment as follows:  `# Resolved Warning: <explanation>`

All Python scripts shall use 4 spaces per indent, not a tab. If there's one thing to take away from all this, it is this statement. This has caused many issues in the past.

Pylint is automatically run on each push, so be sure to check it after making your last push (The cross or tick that can be found next to the commit ID)

Pylint can be pretty strict, so if you feel that certain warning is unnecessary, you can ignore it until you make a pull request. You can bring it up inside the request discussion, and the team lead may consider loosening the restrictions.

File sizes for scripts should try to stay under 8 KB. Any higher is a sign that you should write a package instead (also refer to Overall). You are strongly encouraged to use existing python libraries to help you.

It is very important to note that ROS applications are not run the same way as regular python applications. Installing a package to Python with pip does not mean you can use it in a script that is run with ROS. Refer to rosdep

Type hints are strongly recommended. IDEs like PyCharm can even help you debug your code if you add it. It is a good form of documentation as well.

Unit tests shall be done with pytest, which is an extremely simple library.

How to cite python code:  `# Taken from <web link>`
