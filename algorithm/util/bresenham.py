def bresenham(x0, y0, x1, y1):
    dx = x1 - x0
    dy = y1 - y0

    ux = 1 if dx > 0 else -1
    uy = 1 if dy > 0 else -1

    dx2 = abs(dx * 2)
    dy2 = abs(dy * 2)

    result = []

    if abs(dx) > abs(dy):
        e = -dx
        x = x0
        y = y0
        while x != x1 + ux:
            result.append((x, y))
            e += dy2
            if e > 0:
                y += uy
                e -= dx2
            x += ux
    else:
        e = -dy
        x = x0
        y = y0
        while y != y1 + uy:
            result.append((x, y))
            e += dx2
            if e > 0:
                x += ux
                e -= dy2
            y += uy

    return result
