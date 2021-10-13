#!/usr/bin/env python
import sys
import random

grids = []
start = []
target = []

def fill(x0, y0, x1, y1, c):
    for x in range(x0, x1+1):
        for y in range(y0, y1+1):
            grids[y][x] = c

def print_grids():
    print ("type octile")
    print ("height %d" % l)
    print ("width %d" % l)
    print ("map")
    for i in range(l):
        print (''.join(grids[i]))

def gen_empty(l):
    global grids
    grids = [['.'] * l for _ in range(l)]
    fill(l-2, 0, l-2, l-1, 'T')
    print_grids()

def gen_maxscan(l):
    global grids
    grids = [['.'] * l for _ in range(l)]

    # add obstacles on border
    fill(0, 0, l-1, 0, 'T')
    fill(0, 0, 0, l-1, 'T')
    fill(l-1, 0, l-1, l-1, 'T')
    fill(0, l-1, l-1, l-1, 'T')

    #  fill(1, 1, 1, l-1, 'T')
    #  fill(1, 1, 1, l//4, '.')

    for x in range(1, l // 8, 2):
        for y in range(1, l-1):
            if x % 4 == 1:
                grids[y][x] = 'T' if y % 2 == 0 else '.'
            elif x % 4 == 2:
                grids[y][x] = '.'
            elif x % 4 == 3:
                grids[y][x] = '.' if y % 2 == 0 else 'T'

    for y in range(2, l-1):
        if y % 4:
            fill(l // 8 + 1, y, l // 2, y, 'T')

    fill(l-3, 0, l-3, l-1, 'T')
    grids[1][1] = '.'
    grids[l-1][l-1] = '.'

    print_grids()


def gen_square_grid(l, r):
    global grids
    grids = [['.'] * l for _ in range(l)]

    # add obstacles on border
    fill(0, 0, l-1, 0, 'T')
    fill(0, 0, 0, l-1, 'T')
    fill(l-1, 0, l-1, l-1, 'T')
    fill(0, l-1, l-1, l-1, 'T')

    # mid line obstacles
    for x in range(l // 8, l - l // 8):
        grids[l // 2][x] = 'T'

    # mid random obstacles
    for y in range(l // 4, l - l // 4):
        for x in range(1, l):
            if random.random() < r:
                grids[y][x] = 'T'

    print_grids()

def gen_square_query(l, num=100):
    with open(mapfile, 'r') as f:
        grids = f.readlines()[4:]
    h = len(grids)
    w = len(grids[0])
    global start, target
    start, target = [], []

    while len(start) < num:
        x = random.randint(1, w-1)
        y = random.randint(1, h // 8)
        if (grids[y][x] == '.'):
            start.append((x, y))

    while len(target) < num:
        x = random.randint(1, w-1)
        y = h - random.randint(1, h // 8)
        if (grids[y][x] == '.'):
            target.append((x, y))


    print (mapfile)
    print (min(len(start), len(target)))
    for i in range(min(len(start), len(target))):
        print (start[i][0], start[i][1], target[i][0], target[i][1])


def gen_diag_grid(l, r):
    """
    l * l map

    x = 0, x = l-1 and y = 0, y = l-1 are obstacle

    random obstacle in the middle

    start: above obstacles
    target: below obstacles
    """
    global grids
    grids = [['.'] * l for _ in range(l)]

    h = int(float(l) * r)

    # add obstacles on border
    fill(0, 0, l-1, 0, 'T')
    fill(0, 0, 0, l-1, 'T')
    fill(l-1, 0, l-1, l-1, 'T')
    fill(0, l-1, l-1, l-1, 'T')

    for x in range(l // 8, l - l // 8):
        y = l - x
        grids[y][x] = 'T'

    for d in range(1, h // 2):
        for x in range(l - d):
            y = l - d - x
            if not random.randint(0, 31):
                grids[y][x] = 'T'

    for d in range(1, h // 2):
        for x in range(d, l):
            y = l-1 + d - x
            if not random.randint(0, 31):
                grids[y][x] = 'T'

    print_grids()


def gen_diag_query(mapfile, num):
    with open(mapfile, 'r') as f:
        grids = f.readlines()[4:]
    h = len(grids)
    w = len(grids[0])
    global start, target
    start, target = [], []

    while len(start) < num:
        x = random.randint(1, w // 8)
        y = random.randint(1, h // 8)
        if (grids[y][x] == '.'):
            start.append((x, y))

    while len(target) < num:
        x = w - random.randint(1, w // 8)
        y = h - random.randint(1, h // 8)
        if (grids[y][x] == '.'):
            target.append((x, y))


    print (mapfile)
    print (min(len(start), len(target)))
    for i in range(min(len(start), len(target))):
        print (start[i][0], start[i][1], target[i][0], target[i][1])

def gen_query(mapfile, num):
    with open(mapfile, 'r') as f:
        grids = [list(i.strip()) for i in f.readlines()[4:]]
    h = len(grids)
    w = len(grids[0])
    global start, target
    start, target = [], []

    while len(start) < num:
        x = random.randint(0, w-1)
        y = random.randint(0, h-1)
        if (grids[y][x] == '.'):
            start.append((x, y))

    while len(target) < num:
        x = random.randint(0, w-1)
        y = random.randint(0, h-1)
        if (grids[y][x] == '.'):
            target.append((x, y))

    print (mapfile)
    print (min(len(start), len(target)))
    for i in range(min(len(start), len(target))):
        print (start[i][0], start[i][1], target[i][0], target[i][1])


def randomlize(mapfile, ratio):
    with open(mapfile, "r") as f:
        grids = [list(i.strip()) for i in f.readlines()[4:]]
    h = len(grids)
    w = len(grids[0])

    cnt = 0
    for i in range(h):
        for j in range(w):
            if grids[i][j] == '.':
                cnt += 1

    num = cnt * ratio

    while (num > 0):
        row = random.randint(0, h-1)
        col = random.randint(0, w-1)
        if (grids[row][col] == '.'):
            grids[row][col] = '@'
            num -= 1

    print ("type octile\nheight %d\nwidth %d\nmap" % (h, w))
    for i in range(h):
        print(''.join(grids[i]))


def scale_up(mapfile: str, r: int):
    with open(mapfile, "r") as f:
        grids = [list(i.strip()) for i in f.readlines()[4:]]
    h = len(grids)
    w = len(grids[0])

    newh = h * r
    neww = w * r
    newg = [['.']*neww for i in range(newh)]

    assert(neww * newh == h*w*r**2)

    for y in range(newh):
        for x in range(neww):
            newg[y][x] = grids[y // r][x // r]

    print ("type octile\nheight %d\nwidth %d\nmap" % (newh, neww))
    for i in range(newh):
        print(''.join(newg[i]))

def scale_up_scen(sfile: str, r: int):
    header = ["bucket_id", "map", "h", "w", "sx", "sy", "tx", "ty", "ref_dist"]
    import pandas as pd
    df: pd.DataFrame = pd.read_csv(sfile, skiprows=1, sep='\t', header=0, names=header)
    df['bucket_id'] *= r
    df['h'] *= r
    df['w'] *= r
    df['sx'] *= r
    df['sy'] *= r
    df['tx'] *= r
    df['ty'] *= r
    df['ref_dist'] *= r
    print('version 1')
    print(df.to_csv(header=False, sep='\t', index=False))

if __name__ == "__main__":
    """
    ./gen map <l> <r>
    ./gen query <mapfile> <num>
    """
    if (sys.argv[1] == "diag-map"):
        l = int(sys.argv[2])
        r = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.2
        gen_diag_grid(l, r)
    elif (sys.argv[1] == 'square-map'):
        l = int(sys.argv[2])
        r = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.2
        gen_square_grid(l, r)
    elif (sys.argv[1] == "maxscan"):
        l = int(sys.argv[2])
        gen_maxscan(l)
    elif (sys.argv[1] == "diag-query"):
        mapfile = sys.argv[2]
        num = int(sys.argv[3]) if len(sys.argv) >= 4 else 100
        gen_diag_query(mapfile, num)
    elif (sys.argv[1] == 'square-query'):
        mapfile = sys.argv[2]
        num = int(sys.argv[3]) if len(sys.argv) >= 4 else 100
        gen_square_query(mapfile, num)
    elif (sys.argv[1] == "query"):
        mapfile = sys.argv[2]
        num = int(sys.argv[3]) if len(sys.argv) >= 4 else 100
        gen_query(mapfile, num)
    elif (sys.argv[1] == 'empty'):
        l = int(sys.argv[2])
        gen_empty(l)
    elif (sys.argv[1] == "rand"):
        mapfile = sys.argv[2]
        ratio = float(sys.argv[3])
        randomlize(mapfile, ratio)
    elif (sys.argv[1] == "scale"):
        mapfile = sys.argv[2]
        f = int(sys.argv[3])
        scale_up(mapfile, f)
    elif (sys.argv[1] == "scale-scen"):
        sfile = sys.argv[2]
        f = int(sys.argv[3])
        scale_up_scen(sfile, f)
