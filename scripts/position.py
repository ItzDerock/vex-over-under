import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np


def parse_pose_line(line):
    values = line.split(": ")[1].split(", ")
    print(values)
    x, y, theta = [float(v) for v in values]
    return x, y, theta * np.pi / 180  # Convert degrees to radians


def update_frame(i, ax, robot, poses):
    current_pose = poses[i]
    robot.set_data(current_pose[:2])  # Update only x and y data
    # robot.set_rotation_angle(
    #     current_pose[2] * np.pi / 180
    # )  # Update rotation angle (radians)
    ax.set_xlim(-70, 70)
    ax.set_ylim(-70, 70)
    ax.set_title(f"Frame: {i}")
    return robot, ax


def main():
    with open("./logs/path_data.txt", "r") as f:
        poses = []
        for line in f:
            if line.startswith("pose"):
                poses.append(parse_pose_line(line))

    fig, ax = plt.subplots()
    (robot,) = ax.plot([], [], marker="o", markersize=10, color="blue")

    anim = animation.FuncAnimation(
        fig,
        update_frame,
        fargs=(ax, robot, poses),
        frames=len(poses),
        interval=500,
        blit=True,
    )

    fig_manager = plt.get_current_fig_manager()
    fig_manager.full_screen_toggle()

    plt.subplots_adjust(bottom=0.2)
    play_button_ax = plt.axes([0.92, 0.1, 0.08, 0.08])
    play_button = plt.Button(play_button_ax, "Play/Pause")
    play_button.on_clicked(
        lambda b: anim.event_source.stop() if False else anim.event_source.start()
    )

    plt.show()


if __name__ == "__main__":
    main()
