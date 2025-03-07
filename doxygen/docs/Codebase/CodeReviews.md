# Code Reviews {#code_reviews}

### Most Common Coding Mistakes
Before creating or accepting a merge request, be sure that you checked the following checklist:

 - Constant reference (const &) for big objects to avoid unnecessary copies.
 - Check all signatures carefully. Are they consistent? Are they clear?. If a signature doesn't highlight the clear behavior of the function, document it in the header file using Doxygen style
 - Every class field must be initialized in the constructor, if possible in initializer list in the same order of header file
 - Every local variable must be initialized in place
 - Never use virtual when overriding, but use the override keyword
 - Never return void for public methods
 - Never use threads if not needed, use thread/async with proper sync if necessary
 - Add logging of BEGIN/END in tests for debugging of occasional failures
 - All the public methods of a class must be tested, if a public method is not tested it's considered bugged
 - If your modification is a bug fix, a new test case must be defined that exploits that bug
 - Test cases should be written before the implementation of the class
 - Compile with -g -Wall -pedantic flags and remove the -O2 optimization flag
 - Run the tests using Valgrind, before set ulimit -c unlimited. If Valgrind fails, solve your memory leaks
 - If implement IInitializable interface, one of the test cases must be initialized/deinitialize sequence multiple times
 - Catch always the const ref to the exception and throw always an exception by value
 