#!/usr/bin/env python3
"""
Simple script to create a test sprite using PIL/Pillow
"""
try:
    from PIL import Image, ImageDraw
    
    # Create a 32x32 blue player sprite
    img = Image.new('RGBA', (32, 32), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    
    # Draw a simple character (blue rectangle with white outline)
    draw.rectangle([2, 2, 29, 29], fill=(0, 100, 255, 255), outline=(255, 255, 255, 255), width=2)
    # Add simple eyes
    draw.rectangle([8, 8, 10, 10], fill=(255, 255, 255, 255))
    draw.rectangle([21, 8, 23, 10], fill=(255, 255, 255, 255))
    # Add simple mouth
    draw.rectangle([12, 18, 19, 20], fill=(255, 255, 255, 255))
    
    img.save('/var/mnt/Projects/bevel/example/assets/test_sprite.png')
    print("Created test_sprite.png")
    
    # Create a 24x24 red enemy sprite
    img2 = Image.new('RGBA', (24, 24), (0, 0, 0, 0))
    draw2 = ImageDraw.Draw(img2)
    
    # Draw a simple enemy (red triangle-like shape)
    draw2.polygon([(12, 2), (22, 20), (2, 20)], fill=(255, 50, 50, 255), outline=(150, 0, 0, 255))
    # Add simple eyes
    draw2.rectangle([8, 10, 9, 11], fill=(255, 255, 255, 255))
    draw2.rectangle([14, 10, 15, 11], fill=(255, 255, 255, 255))
    
    img2.save('/var/mnt/Projects/bevel/example/assets/enemy_sprite.png')
    print("Created enemy_sprite.png")
    
except ImportError:
    print("PIL/Pillow not available. Creating placeholder files...")
    # Create empty files as placeholders
    with open('/var/mnt/Projects/bevel/example/assets/test_sprite.png', 'w') as f:
        f.write("# Placeholder sprite file - install PIL to generate actual sprites\n")
    with open('/var/mnt/Projects/bevel/example/assets/enemy_sprite.png', 'w') as f:
        f.write("# Placeholder sprite file - install PIL to generate actual sprites\n")
