# particle_filter_project

## The names of your team members
Timmy (Timmy) Lin and Evan Wu

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
