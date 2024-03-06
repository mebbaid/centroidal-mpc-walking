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
foot_artists = {}

def init():
    artists = []
    for foot, activation_time, pose, orientation in foot_data:
        foot_artists[foot] = ax.add_patch(plt.Rectangle((pose[0] - foot_size / 2, pose[1] - foot_size / 2), foot_size, foot_size, color=(0, 0, 1, 0.5)))
        foot_artists[foot].set_visible(False)
        artists.append(foot_artists[foot])
    return artists

def update(frame):
    artists = []
    min_x, max_x, min_y, max_y = np.inf, -np.inf, np.inf, -np.inf

    # Track the last five steps
    recent_steps = []

    for foot, activation_time, pose, orientation in foot_data:
        if activation_time <= frame:
            new_patch = plt.Rectangle((pose[0] - foot_size / 2, pose[1] - foot_size / 2), foot_size, foot_size, color=(0, 0, 1, 0.5))
            ax.add_patch(new_patch)
            new_patch.set_alpha(0.3)
            new_patch.set_visible(True)
            artists.append(new_patch)
            # Update min and max coordinates
            min_x = min(min_x, pose[0] - foot_size / 2)
            max_x = max(max_x, pose[0] + foot_size / 2)
            min_y = min(min_y, pose[1] - foot_size / 2)
            max_y = max(max_y, pose[1] + foot_size / 2)

            # Add the position of the current step to recent_steps
            recent_steps.append((pose[0], pose[1]))

    # Set new plot limits to follow the last five steps
    if recent_steps:
        if len(recent_steps) > 5:
            recent_steps = recent_steps[-5:]  # Keep only the last five steps

        center_x = sum(x for x, _ in recent_steps) / len(recent_steps)
        center_y = sum(y for _, y in recent_steps) / len(recent_steps)
        max_range = max(max_x - min_x, max_y - min_y) / 2
        ax.set_xlim(center_x - max_range, center_x + max_range)
        ax.set_ylim(center_y - max_range, center_y + max_range)

    return artists






activation_times_ms = [int(data[1] * 1) for data in foot_data]
max_frame = int(np.ceil(max(activation_times_ms)))

ani = animation.FuncAnimation(fig, update, frames=max_frame, init_func=init, interval=10, blit=True)

ani.save('foot_animation.gif', writer='pillow')

plt.show()
