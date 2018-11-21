def bresenhamLineAlg(x0, y0, x1, y1):

    is_steep = abs(y1-y0) > abs(x1-x0)

    #Rotating
    if is_steep:
        x0, y0 = y0, x0
        x1, y1 = y1, x1

    #Swapping start and end
    swapped=False
    if x0 > x1:
        x0, x1 = x1, x0
        y0, y1 = y1, y0
        swapped=True

    dx = x1 - x0
    dy = y1 - y0

    if (dy == 0):
        ystep=0
    elif (dy < 0):
        ystep=-1
    else:
        ystep=1

    error = int(dx /2)
    y = y0
    points = []

    for x in range(x0, x1 + 1):
        if is_steep:
            points.append((y, x))
        else:
            points.append((x, y))

        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    print (points)


bresenhamLineAlg(1,1,8,5)