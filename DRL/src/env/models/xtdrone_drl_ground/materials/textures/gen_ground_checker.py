#!/usr/bin/env python3
from PIL import Image, ImageDraw

def gen_ground_checker(outputPath: str, size: int = 1024, block: int = 32) -> None:
    light = (220, 220, 220)
    dark = (190, 190, 190)
    img = Image.new("RGB", (size, size), light)
    draw = ImageDraw.Draw(img)
    for y in range(0, size, block):
        for x in range(0, size, block):
            if ((x // block) + (y // block)) % 2 == 0:
                draw.rectangle([x, y, x + block - 1, y + block - 1], fill=dark)
    img.save(outputPath)

if __name__ == "__main__":
    gen_ground_checker("ground_checker.png")
    print("generated ground_checker.png")