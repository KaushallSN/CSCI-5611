The approach used was CEM optimization. 
I sampled 2000 inputs around the guassian with mean 1 and standard deviation 1. 
Then I ran each input through the neural network and calculated the sum of absolute values of the output vectors.
Then I created a list of tuples like so [(input1, sum_of_output1),...]
Then I sorted them, in ascending order, of their sum_of_output
Then I selected the tuples with the lowest 25 outputs and recalculated the mean and standard deviation with those inputs.
Repeated step 1-5 until either 200 interations were completed or the standard deviation was less than 1e-07
Then means obtained at the end were the input vector.

Broadly speaking, the method was effective only when the standard deviations were very small compared to the sample size. When the standard deviations was equal to the numbers sampled (n = 2000), the algoritm would fall into a local minimas in the neural network.