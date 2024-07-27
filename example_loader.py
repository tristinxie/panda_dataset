import panda_dataset as pds
import matplotlib.pyplot as plt
from torch.utils.data import DataLoader

panda_dataset = pds.PandaDataset(root_dir="/home/workspace/src/ctrnet-robot-pose-estimation-ros/panda_dataset", transform=pds.ToTensor())
dataloader = DataLoader(panda_dataset, batch_size=1, num_workers=0)
for i_batch, sample_batched in enumerate(dataloader):
    timestamps = sample_batched["timestamps"]
    cam_in = sample_batched["calibration_info"]["intrinsic"][0]
    cam_ex = sample_batched["calibration_info"]["extrinsic"][0]
    print(f"Camera Intrinsics:\n{cam_in}")
    print(f"Camera Extrinsics:\n{cam_ex}")
    plt.imsave("sample_img.png", sample_batched["steps"][timestamps[200][0]]["img_data"][0].numpy().transpose((1,2,0)))