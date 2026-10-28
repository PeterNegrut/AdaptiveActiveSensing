import math

EST_STEP = 0.01  # fixed 100 Hz estimation rate

def estimate_10hz(t_curr, x_prev, y_prev, x_curr, y_curr, hz):
    r = 1/hz
    vx = (x_curr - x_prev)/r
    vy = (y_curr - y_prev)/r

    t_preds = []
    x_preds = []
    y_preds = []

    # Only predict the 0.01 s grid times between this reading and the next one
    # (the small 1e-4 guards against float times like 0.09999999)
    k_first = math.floor(t_curr/EST_STEP + 1e-4) + 1
    k_last = math.floor((t_curr + r)/EST_STEP + 1e-4)
    for k in range(k_first, k_last + 1):
        t_pred = k*EST_STEP
        t_preds.append(t_pred)
        x_preds.append(x_curr + vx*(t_pred - t_curr))
        y_preds.append(y_curr + vy*(t_pred - t_curr))

    return t_preds, x_preds, y_preds

if __name__ == "__main__":
    # 60 Hz example: reading at t=0 (at rest), then at t=1/60
    t_preds, x_preds, y_preds = estimate_10hz(0.0, 0.0, 0.0, 0.0, 0.0, 60)
    for t, x, y in zip(t_preds, x_preds, y_preds):
        print(f"t: {t:.2f}, X: {x:.4f}, Y: {y:.4f}")

    t_preds, x_preds, y_preds = estimate_10hz(1/60, 0.0, 0.0, 1.0, 1.0, 60)
    for t, x, y in zip(t_preds, x_preds, y_preds):
        print(f"t: {t:.2f}, X: {x:.4f}, Y: {y:.4f}")
