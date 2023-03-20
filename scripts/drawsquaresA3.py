import drawsvg as draw

# size in mm
width = 297
height = 420

square_size_x = float(width/14)
square_size_y = square_size_x

nsquaresx = int(width/square_size_x)
nsquaresy = int(height/square_size_y)

# draw.Rectangle()
d = draw.Drawing(width=width, height=height,

                )

for i in range(0, (nsquaresx)):
    for j in range(0, (nsquaresy)):
        fill = 'none' if (j + i) % 2 == 0 else 'black'
        d.append(
            draw.Rectangle(
                    x=(i * square_size_x),
                    y=(j * square_size_y),
                    width=square_size_x,
                    height=square_size_y,
                    fill=fill,
            )
        )

d.save_svg('chessboard.svg')
