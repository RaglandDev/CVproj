"""
=====================================================================
 * MIT License
 * 
 * Copyright (c) 2025 Omni Instrument Inc.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ===================================================================== 
"""
from huggingface_hub import snapshot_download, hf_hub_download
from pathlib import Path
import argparse


class HFDataDownloader:
    def __init__(self):
        # Constant HuggingFace repo
        self.repo_id = "OmniInstrument/CV_project"

        # Constant output directory: /home/<user>/dataset
        home = Path.home()
        self.local_dir = home / "dataset"
        self.local_dir.mkdir(parents=True, exist_ok=True)

        # Constant STL file name inside repo
        self.stl_filename = "meshes/omni_mesh.stl"

    def download_vio_stripped(self):
        print("Downloading VIO_stripped folder...")
        local_path = snapshot_download(
            repo_id=self.repo_id,
            repo_type="dataset",
            allow_patterns=["VIO_stripped/**"],
            local_dir=self.local_dir
        )
        print(f"VIO_stripped downloaded to: {local_path}")
        return Path(local_path)

    def download_stl(self):
        print(f"Downloading STL file: {self.stl_filename} ...")
        stl_path = hf_hub_download(
            repo_id=self.repo_id,
            repo_type="dataset",
            filename=self.stl_filename,
            local_dir=self.local_dir
        )
        print(f"STL downloaded to: {stl_path}")
        return Path(stl_path)

    def download_all(self):
        vio_path = self.download_vio_stripped()
        stl_path = self.download_stl()
        return vio_path, stl_path


def main():

    parser = argparse.ArgumentParser(description="Download ROS2 bag + STL from HuggingFace.")

    parser.add_argument("--download-all", action="store_true",
                        help="Download VIO_stripped and omni_mesh.stl")

    parser.add_argument("--download-vio", action="store_true",
                        help="Download only VIO_stripped")

    parser.add_argument("--download-stl", action="store_true",
                        help="Download only omni_mesh.stl")

    args = parser.parse_args()

    downloader = HFDataDownloader()

    if args.download_all:
        downloader.download_all()

    elif args.download_vio:
        downloader.download_vio_stripped()

    elif args.download_stl:
        downloader.download_stl()

    else:
        print("No action selected. Use:")
        print("   --download-all | --download-vio | --download-stl\n")
        parser.print_help()


if __name__ == "__main__":
    main()