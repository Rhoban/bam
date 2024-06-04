from sympy import *


def compute_dynamic(m, l1, l2, l3, beta, g=9.81):
    g = 9.81

    t = symbols("t")

    l = Function("l")(t)

    alpha = acos((l1**2 + l3**2 - l**2) / (2 * l1 * l3))
    x = cos(alpha + beta) * l2
    y = -sin(alpha + beta) * l2

    dx = diff(x, t)
    dy = diff(y, t)
    K = m * (dx**2 + dy**2) / 2
    P = m * g * y
    L = K - P

    dL_dl = diff(L, l)
    dL_dl_dot = diff(L, diff(l, t))
    dL_dl_dot_dt = diff(dL_dl_dot, t)
    f = dL_dl_dot_dt - dL_dl

    f = f.simplify().expand()

    M = 0
    B = 0

    for term in f.args:
        if term.func == Mul:
            if term.has(diff(l, (t, 2))):
                M += term
            else:
                B += term
        else:
            print("A term is not a Mul")
            exit()

    M = (M.simplify() / diff(l, (t, 2))).simplify()
    B = B.simplify()

    l_, dl_ = symbols("l_ dl_")
    M = M.subs(diff(l, t), dl_).subs(l, l_)
    B = B.subs(diff(l, t), dl_).subs(l, l_)

    if M.has(diff(l, (t, 2))):
        print("M/diff(l, (t,2)) still has acceleration term")
        exit()

    print(f"M: {M}")
    print(f"B: {B}")

    Mf = lambdify([l_, dl_], M)
    Bf = lambdify([l_, dl_], B)

    return Mf, Bf


if __name__ == "__main__":
    import pygame
    import time
    import numpy as np

    A = [-0.5, 0.15]
    beta = np.arctan2(A[1], -A[0])
    l1 = 0.2
    l2 = 1.0
    l3 = np.sqrt(A[0] ** 2 + A[1] ** 2)
    m = 10.0

    print(f"l1: {l1}, l2: {l2}, l3: {l3}, beta: {beta}")

    # dyn = compute_dynamic(m, l1, l2, l3, beta)

    pygame.init()

    def xy_to_screen(xy):
        return (400 + int(xy[0] * 200), 300 - int(xy[1] * 200))

    def draw_point(xy, size=5):
        pygame.draw.circle(screen, (255, 255, 255), xy_to_screen(xy), size)

    def draw_line(xy1, xy2, color):
        pygame.draw.line(screen, color, xy_to_screen(xy1), xy_to_screen(xy2), 2)

    l = 0.5
    Mf, Bf = compute_dynamic(m, l1, l2, l3, beta)
    dl = 0.0
    t = 0
    force = 0
    screen = pygame.display.set_mode((800, 600))

    while True:
        M = Mf(l, dl)
        B = Bf(l, dl)

        events = pygame.event.get()
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:
            force = -50
        elif keys[pygame.K_RIGHT]:
            force = 50
        else:
            force = 0

        ddl = (force - 50.0 * dl - B) / M
        dl += ddl * 0.01
        l += dl * 0.01

        screen.fill((0, 0, 0))

        alpha = np.arccos((l1**2 + l3**2 - l**2) / (2 * l1 * l3))
        B = [-cos(alpha + beta) * l1, sin(alpha + beta) * l1]
        C = [cos(alpha + beta) * l2, -sin(alpha + beta) * l2]

        draw_line(A, B, (255, 0, 0))
        draw_line(B, C, (255, 255, 255))
        draw_point(A)
        draw_point(B)
        draw_point(C, 20)
        draw_point([0, 0])

        pygame.display.flip()
        time.sleep(0.01)
        t += 0.01
