# walking_human_simulator
Action server to simulate walking human for HRC settings.

Configuration:
```yaml
base_frame: "world" # define the reference frame for the positions

# x,y,z to reach
x: [-1.50,-1.00,-0.50]
y: [ 0.05, 0.05, 0.05]
z: [ 0.35, 0.35, 0.35]

# vector of pauses at each location
pauses_duration: [1.0,5.0,5.0]

# noises during pauses
noise_x: 0.01
noise_y: 0.01
noise_z: 0.01

# vector of timings to move between two sequential locations
motions_duration: [5.0,3.0]

# size of the human
chest_size: [0.1,0.2,0.4] # box to represent the chest
head_size: 0.08              # sphere to represent the head
arm_size: [0.35,0.06,0.06]    # box to represent the arms

poses_topic: "/poses" # topic where human poses are published
```
