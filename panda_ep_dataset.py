import os
import torch
import json
import numpy as np
from PIL import Image
from torch.utils.data import Dataset

class PandaEpisodeDataset(Dataset):
    def __init__(self, root_dir, transform=None):
        """
        Arguments:
            root_dir (string): Directory with all the images.
            transform (callable, optional): Optional transform to be applied
                on a sample.
        """
        self.root_dir = root_dir
        self.transform = transform

    def __len__(self):
        return len(os.listdir(self.root_dir))

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()
        ep_num = str(idx+1).zfill(4)
        ep_path = os.path.join(self.root_dir, f"panda_{ep_num}")
        ep_json_path = os.path.join(ep_path, "info.json")
        with open(ep_json_path) as f:
            json_data = json.load(f)

        steps = json_data["steps"]
        timestamps = list(steps.keys())
        json_data["timestamps"] = timestamps
        for _, timestamp in enumerate(steps):
            img_path = os.path.join(ep_path, steps[timestamp]["img_file"])
            image = np.array(Image.open(img_path))
            steps[timestamp]["joint_angles"] = np.array(steps[timestamp]["joint_angles"])
            steps[timestamp]["img_data"] = image

        json_data["calibration_info"]["extrinsic"] =  np.array(json_data["calibration_info"]["extrinsic"])
        json_data["calibration_info"]["intrinsic"] =  np.array(json_data["calibration_info"]["intrinsic"])
        if self.transform:
            json_data = self.transform(json_data)

        return json_data

class ToTensor(object):
    """Convert ndarrays in sample to Tensors."""

    def __call__(self, sample):
        timestamps, intrinsic, extrinsic = sample["timestamps"], sample["calibration_info"]["intrinsic"], sample["calibration_info"]["extrinsic"]
        sample["calibration_info"]["intrinsic"] = torch.from_numpy(intrinsic)
        sample["calibration_info"]["extrinsic"] = torch.from_numpy(extrinsic)
        for timestamp in timestamps:
            step = sample["steps"][timestamp]
            image = step["img_data"]
            image_t = image.transpose((2, 0, 1))
            # swap color axis because
            # numpy image: H x W x C
            # torch image: C x H x W
            sample["steps"][timestamp]["img_data"] = image_t
            sample["steps"][timestamp]["joint_angles"] = torch.from_numpy(step["joint_angles"])
        
        return sample

