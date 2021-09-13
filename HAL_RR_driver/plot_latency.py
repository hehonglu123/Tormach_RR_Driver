import numpy as np

import matplotlib.pyplot as plt

joint_pos=np.load('latency_j6_pos.npy')
joint_cmd=np.load('latency_j6_pos_cmd.npy')

plt.plot(joint_pos[0],joint_pos[1])
plt.plot(joint_cmd[0],joint_cmd[1])
plt.legend(["joint_pos", "pos_cmd"], loc ="lower right")

plt.show()
