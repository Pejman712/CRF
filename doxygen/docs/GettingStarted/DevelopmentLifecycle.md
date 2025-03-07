# Development Lifecycle {#development_lifecycle}

1. Create a new GitLab issue (or ask your leader to do it), describe and update it regularly, following the next steps:
    - If your issue is related to a project with an existing milestone, assign it to the milestone.
    - If there is a known due date, set it.
    - Assing the corresponding labels (Type, Impact and Status). If the issue is a critical bug also assign the maximum weight.
2. From the issue create a new branch.
3. On your local machine pull from the repository and check out the new branch you created.
4. Follow the general code structure explained in the Template module. Your new module should be put in one of the directories defined in \ref framework_structure according to its major purpose.
5. After each commit and push to the repository:
    - All code must compile (it is checked with every push by CI).
    - All tests must pass (also checked by CI).
    - cpplint script must pass (also checked by CI on every file that was added or changed).
6. Consequently, before every commit:

````bash
cd build
make
make test
../scripts/runcpplint.sh
````
7. Implementations of the new features without corresponding tests are considered broken (not working) by definition.
8. Please obey the \ref code_style while writing your code.
9. In case of CI pipeline failure after your push to the repository consider reverting your changes first, then take your time to correct faulty behavior and commit/push again.
10. It is encouraged to use `git commit -F commit_message_file.txt instead of git commit -m "commit message"` and to write meaningful commit messages where changes are explained in a detailed way.
    - Example of a bad commit message:
        `Some fixes`
    - Example of a good commit message:
````
Creation of IPC interface with open and close methods and implementation of MMAP and FIFO extending the new interface.

    - Created IPC abstract class (interface) with 4 methods: open, close, write and read.
    - Implemented MMAP and FIFO classes according to the new interface.
    - Corrected all classes in CRF that were using incorrect or deleted interfaces (e.g. default constructed MMAP).
    - Corrected all classes that use MMAP or FIFO in some way to call "open" before using the IPC (look for them with grep "CreateWriterPtr|CreateReaderPtr").
    - Corrected all lint errors in modified classes.
    - Disabled Carlos ScrewDetection (he told me, he needs to adjust it himself).
    - Removed TCPJAVASocket and MasterSlave from CRF.
````
11. If you realize that another module requires modifications to complete your issue, don't make the modifications in the same branch, but create a new issue and new branch, solve that one first, merge it to master and then continue with your work.
12. Before creating a merge request make sure that your code can be merged into the master without conflicts and that it will not affect other modules in a harmful way.
13. By default the test coverage will only be run in the master branch, merge requests and scheduled pipelines. Furthermore, a Valgrind check of all the test can be launched if the environment variable VALGRIND_TEST is equal to ON:
````bash
git push -o ci.variable="VALGRIND_TEST=ON"
````
