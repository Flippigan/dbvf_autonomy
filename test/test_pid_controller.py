from dbvf_autonomy.precision_landing_node import PIDController


def test_zero_error():
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0, output_limit=1.0)
    assert pid.update(0.0, 0.05) == 0.0


def test_proportional():
    pid = PIDController(kp=2.0, ki=0.0, kd=0.0, output_limit=10.0)
    output = pid.update(3.0, 0.05)
    assert abs(output - 6.0) < 1e-9


def test_integral_accumulation():
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, output_limit=100.0)
    # First update: integral = 1.0 * 0.1 = 0.1
    out1 = pid.update(1.0, 0.1)
    assert abs(out1 - 0.1) < 1e-9
    # Second update: integral = 0.1 + 1.0 * 0.1 = 0.2
    out2 = pid.update(1.0, 0.1)
    assert abs(out2 - 0.2) < 1e-9


def test_integral_windup_clamp():
    pid = PIDController(kp=0.0, ki=10.0, kd=0.0, output_limit=0.5)
    # Large integral accumulation should be clamped at output_limit
    for _ in range(100):
        out = pid.update(1.0, 0.1)
    assert abs(out) <= 0.5 + 1e-9


def test_derivative():
    pid = PIDController(kp=0.0, ki=0.0, kd=1.0, output_limit=10.0)
    # First update: derivative = (1.0 - 0.0) / 0.1 = 10.0 → clamped to 10.0
    pid.update(0.0, 0.1)  # Set prev_error = 0
    out = pid.update(1.0, 0.1)
    # derivative = (1.0 - 0.0) / 0.1 = 10.0
    assert abs(out - 10.0) < 1e-9


def test_output_clamping():
    pid = PIDController(kp=10.0, ki=0.0, kd=0.0, output_limit=0.5)
    out = pid.update(5.0, 0.05)
    assert abs(out) <= 0.5 + 1e-9
    assert out == 0.5

    out_neg = pid.update(-5.0, 0.05)
    assert out_neg == -0.5


def test_reset():
    pid = PIDController(kp=0.0, ki=1.0, kd=1.0, output_limit=100.0)
    pid.update(5.0, 0.1)  # Accumulate integral and prev_error
    pid.reset()
    # After reset, integral=0 and prev_error=0
    out = pid.update(0.0, 0.1)
    assert abs(out) < 1e-9
