from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw

def Generate_Random_Map(image_size=500, num_shapes=30, seed=None, output_path=None, show=True):

    if image_size < 2 or num_shapes < 0:
        raise ValueError("Use image_size >= 2 and num_shapes >= 0.")
    
    # A fixed seed lets me recreate a map when comparing algorithm settings.
    rng = np.random.default_rng(seed)

    # White is free space, black shapes become obstacles when the image is loaded.
    
    image = Image.new("L", (image_size, image_size), 255)
    drawing = ImageDraw.Draw(image)

    for index in range(num_shapes):
        shape = int(rng.integers(3))
        x, y = rng.integers(0, image_size, size=2)
        width, height = rng.integers(20, 51, size=2)
        # Pillow expects ordinary Python integers for these drawing coordinates.
        x, y, width, height = int(x), int(y), int(width), int(height)

        if shape == 0:
            drawing.rectangle((x, y, x + width, y + height), fill=0)
        elif shape == 1:
            radius = width // 2
            drawing.ellipse((x - radius, y - radius, x + radius, y + radius), fill=0)
        else:
            drawing.polygon([(x, y), (x + width, y), (x + width // 2, y + height)], fill=0)

    # I leave a clear route down the left edge and along the bottom so generated maps are solvable.
    corridor_width = min(10, image_size)
    drawing.rectangle((0, 0, corridor_width - 1, image_size - 1), fill=255)
    drawing.rectangle((0, image_size - corridor_width, image_size - 1, image_size - 1), fill=255)

    if output_path is None:
        output_path = Path(__file__).with_name("random_map.bmp")

    image.save(output_path)

    if show:
        plt.imshow(image, cmap="gray", vmin=0, vmax=255)
        plt.title("Random Map with Boundary Route")
        plt.show()
    # The planning code uses True for free cells and False for obstacles.
    return np.asarray(image) >= 128

if __name__ == "__main__":
    Generate_Random_Map()