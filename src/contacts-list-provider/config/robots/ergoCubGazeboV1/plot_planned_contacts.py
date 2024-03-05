import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

foot_size = 0.1

file_path = 'contact-list-provider-output.txt'
with open(file_path, 'r') as file:
    lines = file.readlines()

# Extract the relevant data
foot_data = []
current_foot = None
for line in lines:
    if line.startswith('Foot:'):
        current_foot = line.split()[1]
    elif line.startswith('Activation time:'):
        activation_time = int(line.split()[2]) / 1e9  # Convert nanoseconds to seconds
    elif line.startswith('Pose:'):
        pose = [float(x) for x in line.split()[1:]]
    elif line.startswith('Orientation:'):
        orientation = float(line.split()[1])
        foot_data.append((current_foot, activation_time, pose, orientation))

# Create the animation
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
foot_artists = {}

def init():
    artists = []
    for foot, activation_time, pose, orientation in foot_data:
        foot_artists[foot] = [ax.add_patch(plt.Rectangle((pose[0] - foot_size / 2, pose[1] - foot_size / 2), foot_size, foot_size, color=(0, 0, 1, 0.5)))]
        foot_artists[foot][0].set_visible(False)
        artists.extend(foot_artists[foot])
    return artists

def update(frame):
    artists = []
    for foot, activation_time, pose, orientation in foot_data:
        if activation_time <= frame:
            if foot not in foot_artists:
                foot_artists[foot] = [ax.add_patch(plt.Rectangle((pose[0] - foot_size / 2, pose[1] - foot_size / 2), foot_size, foot_size, color=(0, 0, 1, 0.5)))]
                foot_artists[foot][0].set_visible(False)
            else:
                new_patch = ax.add_patch(plt.Rectangle((pose[0] - foot_size / 2, pose[1] - foot_size / 2), foot_size, foot_size, color=(0, 0, 1, 0.5)))
                foot_artists[foot].append(new_patch)
                foot_artists[foot][-2].set_alpha(0.3)  # Set the alpha for the previous patch
                foot_artists[foot][-2].set_visible(True)  # Make the previous patch visible
                artists.extend(foot_artists[foot])
        else:
            if foot in foot_artists:
                foot_artists[foot][-1].set_visible(False)  # Make the latest patch invisible
                artists.extend(foot_artists[foot])
    return artists







activation_times_ms = [int(data[1] * 1) for data in foot_data]

max_frame = int(np.ceil(max(activation_times_ms)))

ani = animation.FuncAnimation(fig, update, frames=max_frame, init_func=init, interval=10, blit=True)

ani.save('foot_animation.gif', writer='pillow')

plt.show()
