# particle_filter_project

## The names of your team members
Ting-Han (Timmy) Lin and Evan Wu

## Write Up

### Objectives description
In the particle filter localization project, our objective is to implement Monte Carlo localization to help a turtlebot find its position in a maze.

### High-level Description
We achieved localization by first initializing a set of particles with randomized locations and orientations on the map. As the robot moves, the particles' locations are updated relative to the robot's odometry. At the same time, based on LiDAR distances to the closest obstacle from the robot's LiDAR we calculate the likelihood that a given particle at its pose would hypothetically obtain the measurements detected. We then weight the particles based on this likelihood, and resample the cloud of particles so that they converge on the true position of the robot. The resulting mass of particles localizes the robot in relation to its environment. 

### Code Location and Functions/Code Description in Each Step
#### Initialization of Particle Cloud
Lines 136-167 initialize_particle_cloud()
This function draws the initial sample of particles from the state space of the robot's environment. We limited the particles to be initialized within the cells of the map that are unoccupied, then randomly sampled x and y from this range as well as a random angular orientation uniformly. These poses were then assigned to particles and form the whole cloud of particles that we start off with. 

#### Movement Model
Lines 383-421 update_particles_with_motion_model()
The robot has internal odometry that lets predict its own linear and angular displacement as it moves. We use these measurements to update each particle in the cloud to move in this relative manner, but using the particle's known pose instead of the robot's unknown pose. We implemented this using a rotation, translation, and final rotation algorithm. First the particle faces towards its final position, then we move it the same amount the robot moved, and finally we adjust the orientation to the final orientation if necessary. 

#### Measurement Model
Lines 343-380 update_particles_with_measurement_model() and lines 170-183 normalize_particles() 
We update the weights of each particle using a likelihood field algorithm. For each particle, we took the distance to the closest object from the robot in the cardinal and intercardinal directions (every 45 degrees), and calculated the likelihood that a particle would observe these measurements relative to its own pose. We then multiply the weight in each direction and add some noise to get the weight of each particle, an unnormalized probability that the particle is in the same pose as the robot. We then normalized these weights using the normalization function. 

#### Resampling
Lines 206-240 resample_particles() and lines 37-46 draw_random_sample(n,p)
The higher the weight of a particle, the more likely it is to be close to the robot's true location and thus we want it to represent a higher proportion of the particle cloud in the next timestep. To do this, we used the choices function to sample indexes of particles based on the weight of the particle. We then allocate a new particle with the same pose as the one in the previous cloud, and append it to the new resampled cloud. 

#### Incorporation of Noise
Found in updating the motion model and measurement model.
Random noise is added to the translation and final rotation steps when updating the particles' positions based on the relative motion model. This makes resampling more robust as you get small deviations from the previous particle's pose which might make up for insufficiencies in the initial sampling step. It also represents measurement uncertainty in the robot's odometry. We also added noise to the measurement model as we add normally distributed noise to the weight in each direction. This accounts for noise in the LiDAR measurements. 

#### Updating Estimated Robot Pose
Lines 313-340 update_estimated_robot_pose()
We defined the estimated robot pose as the average position and orientation of all the particles in the particle cloud, which over time should localize to the true robot pose. The mean location is intuitive as the average x and y locations, but we approached the average angle as arctan of the sum of sines divided by the sum of cosines for each orientation. This allows us to easily visualize whether or not the particles are converging. 

#### Optimization Of Parameters
When writing initialization, motion model, measurement model.
There were a few parameters we played around with. The initial size of the particle cloud was a big one as the script did not run well with too many particles initialized, but we wanted enough of an initial sample to be able to guarantee convergence. We also optimized noise parameters described above so that the resulting cloud showed uncertainty and not all the final particles were on top of each other.

### Challenges
* Visualizing particles: during the initial stage of our project, we had difficulty visualizing the particles on rviz. The reason was because we didn't ensure the publishers and subscribers were all set up before we called the initialize_particle_cloud function. After we added a few seconds of sleep, we were able to see the particles.
* Python debugging: in our code, we accessed a field in a Particle p by "p.weight" instead of "p.w" which didn't throw an error, making us have to debug the script by simulating only a few fixed particles and tracking the weights during updates. 
* Working with the map: we had difficulties in the beginning understanding the data available in the map, particularly the OccupancyGrid data which helped us do better at initializing the cloud
* Computational models: the algorithms for the measurement and motion models took us some time to fully understand the geometric transformations that were taking place, and to debug in general.

### Future Work 
In the future, we can think more about how to initialize the particle cloud strictly within the boundaries of the maze as in our current implementation, some of the particles are initialized a little bit outside of the wall of the maze. We can also try to fine tune the noise parameters a bit more, and try to incorporate z_hit and z_random which we mostly ignored. Finally, we can overall try and optimize the computational steps to be able to run the script using more particles, as our final run was limited to 2500 particles, although this may be limited by personal hardware. 

### GIF
  ![Alt Text](particle-filter.GIF)![Alt Text](particle-filter2.GIF)

### Takeaways
* We were able to achieve localization using a particle filter algorithm, which is useful whenever we have a robot whose position in an environment is unknown but we have spatial measurement data. 
* We explored some of the parameters that affect the performance of this algorithm, particularly in terms of initialization and noise when computing relative movement of particles and calculating the likelihood field. 

## Implementation Plan
## A 1-2 sentence description of how your team plans to implement each of the following components of the particle filter localization as well as a 1-2 sentence description of how you will test each component:
 
### How you will initialize your particle cloud (initialize_particle_cloud)?
We will use a densely uniform sample of the map to find the map, so we can apply Monte Carlo Localization afterwards. We can look at the distribution of particles directly from the rviz rendering of the map, and additionally change this if we find there's not enough sampling after implementing later steps. 

### How you will update the position of the particles will be updated based on the movements of the robot (update_particles_with_motion_model)?
Given a position of a particle, we can apply the sensor readings from the robot's internal odometry data to each particle. To test, we can try to track the movement of just a few particles in relation to the robot's movements to see that they match up. 

### How you will compute the importance weights of each particle after receiving the robot's laser scan data?(update_particle_weights_with_measurement_model)?
We have position of possible positions and orientations from each particle, and current empirical measurements from the robot. So we can compare the expected measurements at each particle to what we observe, and assign greater weight based on how similar the two are. We will have to see the resulting distribution of resampled particles after testing to assess the efficacy of our weighting scheme. 

### How you will normalize the particles' importance weights (normalize_particles) and resample the particles (resample_particles)?
We wil normalize the particles' importanace weights to a sum of one. We plan to drop some weights that are below a particular threshold in the population before we resample. Similarly, resampling will be tuned based on test runs of localization. 

### How you will update the estimated pose of the robot (update_estimated_robot_pose)?
We will take into account both relative odometry and LiDAR scan measurements and how much they agree to estimate the pose. We can compare this estimate to what we know about the robot's pose based on rviz, etc. 

### How you will incorporate noise into your particle filter localization?
We expect noises from robot's movement and sensor measurement, and we might consider adding some noisiness to the movement to allow for more robust resampling. If the sampling distribution collapses too quickly or catastrophically in test runs, we can add more noise inherently to the robot's movements. 

### A brief timeline sketching out when you would like to have accomplished each of the components listed above.
By 4/18, we plan to finish all of the above points. By 4/26, we should have sufficiently tested our code and calculation.
