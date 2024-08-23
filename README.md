# panda_dataset
Dataset of Franka Emika Panda robot with ground truth camera-to-robot base, calibration parameters, joint angles, and video while it is teleoperated to do certain tasks.


## Download (7zip)
- [10 episodes dataset](https://drive.google.com/file/d/1JN1RARqZaM1HpZEr-ZnHOR5Q0k_BtRLe/view?usp=drive_link)

## 10 episodes visualized
### Camera angle 1
<img src="./assets/gifs/panda_0001.gif" height="120" width="160" /> <img src="./assets/gifs/panda_0002.gif" height="120" width="160" /> <img src="./assets/gifs/panda_0003.gif" height="120" width="160" /> <img src="./assets/gifs/panda_0004.gif" height="120" width="160" /> <img src="./assets/gifs/panda_0005.gif" height="120" width="160" />

### Camera angle 2
<img src="./assets/gifs/panda_0006.gif" height="120" width="160" /> <img src="./assets/gifs/panda_0007.gif" height="120" width="160" /> <img src="./assets/gifs/panda_0008.gif" height="120" width="160" /> <img src="./assets/gifs/panda_0009.gif" height="120" width="160" /> <img src="./assets/gifs/panda_0010.gif" height="120" width="160" />

## Usage
`panda_dataset.py` contains the custom PyTorch dataloader. See `demo.ipynb` for more details.

## Dataset Schema
### PyTorch Dataloader
If you use the included PyTorch dataloader, we provide ready loaded data as additional keys you can access. You don't have to write code to read the files. Use `transform = ToTensor()` to receive data as PyTorch tensors and images in PyTorch format. 

```python
PDS = {
    "calibration_info": {
        "extrinsic": float (1,6)
        "intrinsic": float (3,3)
    },
    "steps": {
        # timestamp# = floating point unix epoch timestamp with secs and nsecs as strings
        "timestamp1": {
            "img_file": "panda_{ep_num}_f0001.png",
            "img_data": uint8 (3, H, W) or (H, W, 3), # Only available if using Dataloader.
            "joint_angles": float (7)
        },
        ...
        "timestampn": {
            "img_file": "panda_{ep_num}_f{last_frame_num}.png",
            "img_data": uint8 (3, H, W) or (H, W, 3), # Only available if using Dataloader.
            "joint_angles": float (7)
        }
    },
    "timestamps": str (len(steps)) # Only available if using Dataloader
}
```
