# particle_filter_project

## The names of your team members
Ting-Han (Timmy) Lin and Evan Wu

## Write Up

### Objectives description
In the particle filter localization project, our objective is to implement Monte Carlo localization to help a turtlebot find its position on the maze map that is made by us using Simultaneous Localization and Mapping (SLAM). At the beginning of our project, we would initialize a set of particles with randomized locations and orientations on the map. As time goes on, the particles' locations would be updated based on the robot's movement, and particles that have similar hypothetical laser scan to that of the robot would remain and converge to the position of the robot, which shows that the robot now knows its location on the map.

### High-level Description
### Code Location and Functions/Code Description in Each Step
#### Initialization of Particle Cloud
#### Movement Model
#### Measurement Model
#### Resampling
#### Incorporation of Noise
#### Updating Estimated Robot Pose
#### Optimization Of Parameters

### Challenges
During the initial stage of our project, we had difficulty visualizing the particles on rviz. It turnned out that the reason was because we didn't ensure the publishers and subscribers to be set up before we called the initialize_particle_cloud function. After we implemented sleep, we were able to see the particles shown on rviz.

### Future Work 
In the future, we can think more about how to initialize the particle cloud strictly within the boundaries of the maze. In our current implementation, some of the particles are initialized a little bit outside of the wall of the maze.

### GIF
  ![Alt Text](particle-filter.GIF)![Alt Text](particle-filter2.GIF)

### Takeaways

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
