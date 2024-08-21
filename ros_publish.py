import panda_dataset as pds
import matplotlib.pyplot as plt
import numpy as np
from torch.utils.data import DataLoader

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock

if __name__ == "__main__":
    dataset_dir = "/home/workspace/src/panda_dataset/panda_dataset" # change as necessary
    panda_dataset = pds.PandaDataset(root_dir=dataset_dir, transform=pds.ToTensor())
    dataloader = DataLoader(panda_dataset, batch_size=1, num_workers=0, shuffle=True)
    sample = next(iter(dataloader))

    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    clk_pub = rospy.Publisher("/clock", Clock, queue_size=10)
    rospy.init_node("joint_state_publisher")
    rate = rospy.Rate(30)
    joint_msg = JointState()
    clock = Clock()
    joint_msg.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
    while not rospy.is_shutdown():
        try:
            for i, timestamp in enumerate(sample["steps"]):
                ros_timestamp = rospy.Time.from_sec(float(timestamp))
                step = sample["steps"][timestamp]

                joint_msg.header.stamp = ros_timestamp
                joint_msg.position = step["joint_angles"].squeeze().numpy()

                pub.publish(joint_msg)
                # clock.clock = ros_timestamp
                # clk_pub.publish(clock)

                rate.sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("Done.")