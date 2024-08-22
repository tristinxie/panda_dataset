import panda_dataset as pds
import matplotlib.pyplot as plt
import numpy as np
from torch.utils.data import DataLoader

import rospy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock

if __name__ == "__main__":
    dataset_dir = "/home/workspace/src/panda_dataset/panda_dataset" # change as necessary
    panda_dataset = pds.PandaDataset(root_dir=dataset_dir, transform=pds.ToTensor())
    dataloader = DataLoader(panda_dataset, batch_size=1, num_workers=0, shuffle=True)
    sample = next(iter(dataloader))

    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    img_pub = rospy.Publisher("/rgb/image_raw", Image, queue_size=10)
    clk_pub = rospy.Publisher("/clock", Clock, queue_size=10)

    rospy.init_node("joint_state_publisher")
    rate = rospy.Rate(30)
    joint_msg = JointState()
    image_msg = Image()
    clock = Clock()
    joint_msg.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
    while not rospy.is_shutdown():
        try:
            for i, timestamp in enumerate(sample["steps"]):
                ros_timestamp = rospy.Time.from_sec(float(timestamp))
                step = sample["steps"][timestamp]

                joint_msg.header.stamp = ros_timestamp
                joint_msg.position = step["joint_angles"].squeeze().numpy()
                joint_pub.publish(joint_msg)

                clock.clock = ros_timestamp
                clk_pub.publish(clock)

                image_msg.header.stamp = ros_timestamp
                image_msg.height = 240
                image_msg.width = 320
                image_msg.encoding = "rgb8"
                image_msg.step = 240
                image = list(step["img_data"].squeeze().permute(1,2,0).flatten().numpy())

                image_msg.data = image
                img_pub.publish(image_msg)

                rate.sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("Done.")
