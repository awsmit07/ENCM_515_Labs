Hello Marker!

The only edits were in the main.c file.
Place the main.c file in a correctly configured project such as the one provided in lab2 (that is the project this assignment is based on).

To recreate our results:

Change the value of PROCESS_BLOCK_FUNC on line 36 to a value 1-4. Preprocessor if statements automatically change the code for each of the 4 tests.

Tests:
1. Basic ProcessBlock function for a frame size of 3 - Uses ProcessBlock function
2. Basic ProcessBlock function for a frame size of 16 - Uses ProcessBlock function
3. Unrolled Loop ProcessBlock function for a frame size of 3 - Uses ProcessBlock3 function
4. Unrolled Loop ProcessBlock function for a frame size of 16 - Uses ProcessBlock4 function

Uncomment FUNCTIONAL_TEST on line 32 to run filter without the timer interrupt.