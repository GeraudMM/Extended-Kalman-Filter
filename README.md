[image1]: example.gif "Intro GIF"

# Extended-Kalman-Filter
In this project, we will use an extended kalman filter relying on a Radar and a Lidar sensor to predict the next position and velocity of a vehicule on a 2D map. We can see the results in the gif bellow:

![Driving Example][image1] 

### Introduction

  We will use the Udacity simulation of a car on a track to collect data on how the human drive. Then, once we have enough data we will train our Neural Network to copy the human behavior with only the images for input and the speed and angle choose by the human player to help the neural network to generalize on diffenrent images.

### Getting Started

In order to open and interact with this algorithm and the simulation you will need to follow the following steps:

**Step 1:** Set up the [CarND Term1 Starter Kit](https://github.com/udacity/CarND-Term1-Starter-Kit/blob/master/README.md) if you haven't already.

**Step 2:** Open the code in a Jupyter Notebook


Then download the unity simulation from [this](https://github.com/udacity/self-driving-car-sim) github repo. Here we trained the car with the `Term 1` repo.


### Instruction
Activate the `CarND-term1` environment on our cmd and go to the project folder.

Run the `model.py` file from your cmd to train the model by taping `python model.py`

Launch the trained model to drive the simulation from your cmd by taping `python drive.py model.h5`

Finaly, launch the `.exe` file in the simulation folder and choose autonomous mode to see the car driving itself.

Check also the `Report.md` for explaination on the algorithm.
