
import os


central = os.path.abspath("input.txt")
baricades = os.path.abspath("walls.txt")
destination = os.path.abspath("output.txt")
indexFile = os.path.abspath("indexOfHole.txt")

try:
    centrals = open(central, "r")
    walls = open(baricades, 'r')
    _index = open(indexFile, 'r')
    dest = open(destination, 'w')

    n = 0
    isNotFounded = True

    index = int(_index.readline())

    while centrals.readable() & isNotFounded:
        line = centrals.readline()
        if line == '\n':
            continue
        label, number, *coord = line.split()
        coord = list(map(float, coord))
        if index < len(coord) // 3 + n:
            isNotFounded = False
            dest.write(f"{coord[(index - n) * 3]} {coord[(index - n) * 3 + 1]} {coord[(index - n) * 3 + 2]}\n")
            for i in range(n + 1):
                wall = walls.readline()
            label, number, name, *coord_Holes = wall.split()
            coord_hol = list(map(float, coord_Holes))

            kof_hol = len(coord_hol)//4
            barriers = [coord_hol[i * 4: i * 4 + 4] for i in range(kof_hol)]
            x0, y0 = coord[-2], coord[-1]
            for barrier in barriers:
                x, y, w, h = barrier
                left = [x0+x - w / 2, y0+y]
                right = [x0+x + w / 2, y0+y]
                top = [x0+x, y0+y + h / 2]
                bottom = [x0+x, y0+y - h / 2]
                dest.write(f"{left[0]} {left[1]} {right[0]} {right[1]} {top[0]} {top[1]} {bottom[0]} {bottom[1]}\n")
        else:
            n += 1
finally:
    centrals.close()
    walls.close()
    _index.close()
    dest.close()