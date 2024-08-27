# panda_dataset
Dataset of Franka Emika Panda robot with ground truth camera-to-robot base, calibration parameters, joint angles, and video while it is teleoperated to do certain tasks.


## Download (7zip)
### Current
- [10 episodes full body view](https://drive.google.com/file/d/1VAqf5pfmrYJ64kDeSYfRVHPjkYAv-yAF/view)
- [20 episodes partial view](https://drive.google.com/file/d/1jbzwUwRSSRJgmMVY_rM1u_1ghXLL36ax/view)
### Previous
- [10 episodes partial dataset](https://drive.google.com/file/d/1JN1RARqZaM1HpZEr-ZnHOR5Q0k_BtRLe/view)
## Usage
`panda_ep_dataset.py` contains the custom PyTorch episode dataset. `panda_step_datastep.py` contains the custom PyTorch step dataset per episode. See `demo.ipynb` for more details and example usage.

## Dataset Schema
### PyTorch Dataloader
If you use the included PyTorch dataloader, we provide ready loaded data as additional keys you can access. You don't have to write code to read the files. Use `transform = ToTensor()` to receive data as PyTorch tensors and images in PyTorch format. 
### Panda Step Dataset format
This dataset supports batching better than the Panda Episode Dataset.
```python
PSD = {
    "timestamps": str (len(batch)),
    "intrinsic": float (batch, 3, 3),
    "extrinsic": float (batch, 1, 6), # axis angle
    "image": uint8 (batch, 3, H, W) or (batch, H, W, 3),
    "joint_angles": float (batch, 7),
}
```
### Panda Episode Dataset and `info.json` format
```python
PED = {
    "calibration_info": {
        "intrinsic": float (3,3),
        "extrinsic": float (1,6),
    },
    "steps": {
        # timestamp# = floating point unix epoch timestamp with secs and nsecs as strings
        "timestamp1": {
            "img_file": "panda_{ep_num}_f0001.png",
            "img_data": uint8 (3, H, W) or (H, W, 3), # Only available if using Dataloader.
            "joint_angles": float (7),
        },
        ...
        "timestampn": {
            "img_file": "panda_{ep_num}_f{last_frame_num}.png",
            "img_data": uint8 (3, H, W) or (H, W, 3), # Only available if using Dataloader.
            "joint_angles": float (7),
        }
    },
    "timestamps": str (len(steps)), # Only available if using Dataloader
}
```
For batching the custom collate function in `demo.ipynb` creates an extra dimension for `calibration_info` attributes. However `timestamps` and `steps` are just returned as an array since different episodes have different lengths and cannot be collated. Padding episodes to be the same length can be a solution.
