def estimate_10hz(x_curr, y_curr, x_next, y_next):
    delta_x = x_next - x_curr
    delta_y = y_next - y_curr
    vx = delta_x/0.1
    vy = delta_y/0.1

    x_pred = x_curr
    y_pred = y_curr
    for i in range(10):
        x_pred = x_pred + vx*0.01
        y_pred = y_pred + vy*0.01
        print(f"x: {x_pred}, y: {y_pred}")
        i = i+1

x_values = [0.0000, 10]
y_values = [0.0000, -0.0000]

estimate_10hz(x_values[0], y_values[0], x_values[1], y_values[1])