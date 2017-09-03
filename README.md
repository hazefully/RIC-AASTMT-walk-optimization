
RIC-AASTMT walk optimization code.

This is the code we used in Robocup 2017 to generate CMA-ES paramters to try to optimize the walk engine paramters for the nao robots. The code is written in C++ and was used to push the test paramters to 9 local machines where a different code that had the required fitness function would run optimization tasks to test the paramaters ability to walk in a speedy and stable manner. This depends on the existence of a bash file "start_all.sh" on a main machine that pushes the paramaters using parallel-ssh and another bash file "start_me.sh" that runs N instances of an agent according to the population size used in the CMA-ES code.

You can use this code in anyway you want.

This code is based on UTAustinVilla3D published works about their optimization tasks and how they used CMA-ES. You can find all their publications here:
http://www.cs.utexas.edu/~AustinVilla/sim/3dsimulation/publications.html

Don't hesitate to contact me:
hazem.essam@gmail.com
