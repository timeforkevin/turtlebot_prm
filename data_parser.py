import matplotlib.pyplot as plt
import numpy as np

t = np.arange(0.0, 2.0, 0.01)
s = 1 + np.sin(2*np.pi*t)
plt.plot(t, s)

plt.xlabel('time (s)')
plt.ylabel('voltage (mV)')
plt.title('About as simple as it gets, folks')
plt.grid(True)
plt.savefig("test.png")
plt.show()

x_coords = []
y_coords = []
z_coords = []

def parse_pose_data():
    with open('pose_data.txt', 'r') as op:
        lines = op.readlines()

    num_lines = len(lines)

    count = 0

    while count < num_lines:
        curr = lines[count].strip()
        if curr == 'position:':
            x_coords.append(float(lines[count+1].strip().split(':')[1].strip()))
            y_coords.append(float(lines[count+2].strip().split(':')[1].strip()))
            z_coords.append(float(lines[count+3].strip().split(':')[1].strip()))
            count += 4
        else:
            count += 1

    def write_to_file(name, data):
        with open(name, 'w') as op:
            for elem in data:
                op.write("%s\n" % elem)

    write_to_file('x_data.txt', x_coords)
    write_to_file('y_data.txt', y_coords)

    plt.plot(x_coords, y_coords)

    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Path Travlled by turtlebot')
    plt.grid(True)
    plt.savefig("path.png")
    plt.show()


if __name__ == '__main__':
    parse_pose_data()
