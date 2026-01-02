# Computer Vision Project Submission

<p align="center">
  <a href="https://huggingface.co/datasets/OmniInstrument/CV_project">
    <img src="https://img.shields.io/badge/HuggingFace-Dataset-yellow?logo=huggingface&style=for-the-badge">
  </a>
</p>


<div align="justify">
## Getting Started
Clone the repo

```shell
git clone --recursive git@github.com:RaglandDev/CVproj.git
```
Start the Docker environment

```shell
bash scripts/start.sh
```

> [!NOTE]
> This will automatically run a multi-stage docker container creation, pull all the datasets from Hugging Face too and build the ROS 2 workspace.
> Dataset download will occur only once.

Run the pipeline
```shell
ros2 launch tsdf_saver saver.launch.py
```

At the end, the launch file should automatically save a mesh (`.stl`) and stop/close the system after 20 seconds.

### Comparing the Mesh against the Ground Truth
```shell
/opt/venv/bin/python /home/$(whoami)/compute_metrics.py --view
```

## License
This software and dataset are released under the [MIT License](LICENSE).

## Acknowledgment
This work integrates several powerful research papers, libraries, and open-source tools:

- [**DB-TSDF**](https://robotics-upo.github.io/DB-TSDF/)
