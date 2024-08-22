# panda_dataset
Dataset of Franka Emika Panda robot with ground truth camera-to-robot base, calibration parameters, joint angles, and video while it is teleoperated to do certain tasks.


## Download (7zip)
- [10 episodes dataset](https://drive.google.com/file/d/1JN1RARqZaM1HpZEr-ZnHOR5Q0k_BtRLe/view?usp=drive_link)

## 10 episodes visualized
### Camera angle 1
<img src="./assets/gifs/panda_0001.gif" height="120" width="160" />
<img src="./assets/gifs/panda_0002.gif" height="120" width="160" />
<img src="./assets/gifs/panda_0003.gif" height="120" width="160" />
<img src="./assets/gifs/panda_0004.gif" height="120" width="160" />
<img src="./assets/gifs/panda_0005.gif" height="120" width="160" />

### Camera angle 2
<img src="./assets/gifs/panda_0006.gif" height="120" width="160" />
<img src="./assets/gifs/panda_0007.gif" height="120" width="160" />
<img src="./assets/gifs/panda_0008.gif" height="120" width="160" />
<img src="./assets/gifs/panda_0009.gif" height="120" width="160" />
<img src="./assets/gifs/panda_0010.gif" height="120" width="160" />

## Usage
`panda_dataset.py` contains the custom PyTorch dataloader. See `demo.ipynb` for more details.

## Dataset Schema
### Episode `info.json`
```

```
### PyTorch Dataloader
If you use the included PyTorch dataloader, we provide ready loaded data as additional keys you can access. You don't have to write code to read the files.

