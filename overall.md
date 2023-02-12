## Overal

Top level directories in `src` should contain *significant* content. The most common type of folder would be a ROS package. Another type would be global dependencies. Creating these should be a team decision. Only add code within the top level directories, and ensure that the code is used in those directories and not anywhere else. Follow the layout of a package as well. There is a rightful place for all files, and putting stuff in the wrong place can lead to confusion.

All scripts, regardless of programming language, should be kept small. Big scripts are candidates for refactoring, or reconsideration. Remember, the easiest code to read, write, and debug is no code. If your script is getting to big, you may want to do some research to see if there is a library that does at least some of what you need. Scripts should also only have one overarching goal in mind. It is perfectly fine to have multiple scripts (ideally Nodes) that all must work together to accomplish just one competition task (i.e., digging).

You may notice that GitHub does not let you push to `main` directly (hopefully). This is intentional so that we can keep `main` at a functional state. There is usually always a branch ready for you to write code to. If there isn't one, ask the team lead before making your own, as having two or more branches containing the same thing can be very confusing. To merge to main, you must make a pull request, and wait for it to get approved. Before creating a request, you should check that your branch passes all automated checks.

Documentation is a thing we care about here. At the very least, you should have in-line comments for complex sections of code. Your team lead will carry out review sessions so you can better understand what is hard to read. Ideally, you should also have method, class, module/script documentation, and even package documentation if you are making one. Everyone is encouraged to add their names to the script documentation as an author if they feel they have modified it in a significant way.

ROS packages will also contain a `README.md`, mainly to explain what nodes are contained within, how these nodes should be used, and what to expect when using these nodes. Any script that is not a node does not have to be explained here (but they still need their own documentation).

If you copy a significant code snippet (highly subjective), or an entire script, you are recommended to cite it. If it does not contain sufficient documentation, you must add it yourself.

Do not commit files that other people do not need. The usual way to avoid this is to add the file(s) to the gitignore. However, since everyone shares this gitignore, it can cause unintended side effects with name collisions. As such, updates to gitignore are a team decision. Instead, you should use your local gitignore (refer to reading list).

Unit testing is a strong recommendation as well. Understandably, many components of our code cannot be unit tested as it involves physical interactions. However, if you follow proper separation of concerns, there should be plenty of code to unittest. Individual tests should be small, and test suites should test one class or module. All test suites should be contained within the `test` folder of the package that contains the code being tested
