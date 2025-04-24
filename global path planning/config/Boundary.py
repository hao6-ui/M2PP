def getBoundary(env):
    bottom = 1000
    top = -1000

    for circle in env.obs_circle:
        bottom = min(bottom, circle[1] - circle[2])
        top = max(top, circle[1] + circle[2])

    for rectangle in env.obs_rectangle:
        bottom = min(bottom, rectangle[1])
        top = max(top, rectangle[1] + rectangle[3])
    top += 3
    bottom -=3
    return top, bottom
