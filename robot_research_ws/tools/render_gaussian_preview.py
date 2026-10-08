#!/usr/bin/env python3
"""Render a trained Splatfacto checkpoint at captured camera positions."""
import argparse
from pathlib import Path
import shutil

import mediapy as media
import numpy as np
import torch
from PIL import Image, ImageDraw
from nerfstudio.utils.eval_utils import eval_setup


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("config", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    _, pipeline, checkpoint, step = eval_setup(args.config, test_mode="test")
    dataset = pipeline.datamanager.train_dataset
    cameras = dataset.cameras.to(pipeline.device)
    rendered = []
    selected = set(np.linspace(0, len(cameras) - 1, 3, dtype=int).tolist())
    comparisons = []
    with torch.no_grad():
        for index in range(len(cameras)):
            outputs = pipeline.model.get_outputs_for_camera(cameras[index:index + 1])
            rgb = (outputs["rgb"].clamp(0, 1).cpu().numpy() * 255).astype(np.uint8)
            rendered.append(rgb)
            if index in selected:
                Image.fromarray(rgb).save(args.output_dir / f"render_{index:03d}.png")
                original = Image.open(dataset.image_filenames[index]).convert("RGB").resize((rgb.shape[1], rgb.shape[0]))
                panel = Image.new("RGB", (rgb.shape[1] * 2, rgb.shape[0] + 30), "white")
                panel.paste(original, (0, 30))
                panel.paste(Image.fromarray(rgb), (rgb.shape[1], 30))
                draw = ImageDraw.Draw(panel)
                draw.text((12, 8), "D435i capture", fill="black")
                draw.text((rgb.shape[1] + 12, 8), f"3D Gaussian render - step {step}", fill="black")
                comparisons.append(panel)
            print(f"Rendered {index + 1}/{len(cameras)}", flush=True)
    montage = Image.new("RGB", (comparisons[0].width, sum(p.height for p in comparisons)), "white")
    offset = 0
    for panel in comparisons:
        montage.paste(panel, (0, offset))
        offset += panel.height
    montage.save(args.output_dir / "comparison.jpg", quality=95)
    if shutil.which("ffmpeg") is None:
        import imageio_ffmpeg
        media.set_ffmpeg(imageio_ffmpeg.get_ffmpeg_exe())
    media.write_video(str(args.output_dir / "camera_path.mp4"), rendered, fps=8)
    Image.fromarray(rendered[0]).save(
        args.output_dir / "camera_path.gif", save_all=True,
        append_images=[Image.fromarray(frame) for frame in rendered[1:]], duration=125, loop=0,
    )
    print(f"Checkpoint: {checkpoint}\nPreview: {args.output_dir}", flush=True)


if __name__ == "__main__":
    main()
