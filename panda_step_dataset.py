import os
import torch
import json
import numpy as np
from PIL import Image
from torch.utils.data import Dataset

class PandaStepDataset(Dataset):
    def __init__(self, root_dir, ep_num, transform=None):
        """
        Arguments:
            root_dir (string): Directory with all the images and json file.
            ep_num (int): episode number to use as dataset
            transform (callable, optional): Optional transform to be applied
                on a sample.
        """
        self.transform = transform
        ep_num = str(ep_num).zfill(4)
        self.ep_path = os.path.join(root_dir, f"panda_{ep_num}")
        self.data = {}

        ep_json_path = os.path.join(self.ep_path, "info.json")

        with open(ep_json_path) as f:
            json_data = json.load(f)

        steps = json_data["steps"]
        self.data["steps"] = json_data["steps"]
        
        self.data["calibration_info"] = dict()
        self.data["calibration_info"]["extrinsic"] =  np.array(json_data["calibration_info"]["extrinsic"])
        self.data["calibration_info"]["intrinsic"] =  np.array(json_data["calibration_info"]["intrinsic"])

        timestamps = list(steps.keys())
        self.data["timestamps"] = timestamps

    def __len__(self):
        return len(self.data["timestamps"])

    def __getitem__(self, idx):
        steps = self.data["steps"]
        timestamp = self.data["timestamps"][idx]

        img_path = os.path.join(self.ep_path, steps[timestamp]["img_file"])
        image = np.array(Image.open(img_path))
        joint_angles = np.array(steps[timestamp]["joint_angles"])

        sample ={
                    "timestamp": timestamp,
                    "image": image, 
                    "joint_angles": joint_angles, 
                    "extrinsic": self.data["calibration_info"]["extrinsic"], 
                    "intrinsic": self.data["calibration_info"]["intrinsic"]
                }

        if self.transform:
            sample = self.transform(sample)

        return sample

class ToTensor(object):
    """Convert ndarrays in sample to Tensors."""

    def __call__(self, sample):
        image, joint_angles, intrinsic, extrinsic = sample["image"], sample["joint_angles"], sample["intrinsic"], sample["extrinsic"]
        image_t = image.transpose((2, 0, 1))
        sample["image"] = image_t
        # swap color axis because
        # numpy image: H x W x C
        # torch image: C x H x W
        sample["joint_angles"] = torch.from_numpy(joint_angles)
        sample["intrinsic"] = torch.from_numpy(intrinsic)
        sample["extrinsic"] = torch.from_numpy(extrinsic)
        return sample
