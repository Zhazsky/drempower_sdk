uint8[] motor_ids
uint8 type        # 0: Position, 1: Speed, 2: Torque, 3: Adaptive Position, 4: Step Position, 5: Impedance, 6: Motion Aid
uint8 mode        # Context-dependent mode
float32[] values  # Context-dependent values (angle, speed, kp, kd, etc.)
