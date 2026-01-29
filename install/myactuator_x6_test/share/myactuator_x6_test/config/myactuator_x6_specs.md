# MyActuator X6-60 Specifications

## Product Parameters

| Parameter | Unit | X6-60 |
|-----------|------|-------|
| Gear Ratio | - | 19.612 |
| Input Voltage | V | 48 |
| No Load Speed | RPM | 176 |
| No-Load Input Current | A | 0.9 |
| Rated Speed | RPM | 153 |
| Rated Torque | N.m | 20 |
| Rated Output Power | W | 320 |
| Rated Phase Current | A(rms) | 9.5 |
| Peak Torque | N.m | 60 |
| Peak Phase Current | A(rms) | 29.1 |
| Efficiency | % | 72.7 |
| Motor Back-EMF Constant | Vdc/Krpm | 16 |
| Module Torque Constant | N.m/A | 2.1 |
| Motor Phase Resistance | Ohm | 0.41 |
| Motor Phase Inductance | mH | 0.51 |
| Pole Pair | - | 10 |
| 3 Phase Connection | - | Y |
| Back Drive Torque | N.m | 1.6 |
| Backlash | Arcmin | 10 |
| Output Bearing Type | - | Crossed Roller Bearings |
| Axial Load (Suffer) | KN | 1.8 |
| Axial Load (Stress) | KN | 0.8 |
| Radial Load | KN | 2 |
| Inertia | Kg.cm2 | 0.66 |
| Encoder Type & Interface | - | Dual Encoder ABS-17BIT(Input) / 17BIT(Output) |
| Control Accuracy | Degree | <0.01 |
| Communication | - | EtherCAT & CAN BUS |
| Weight | Kg | 0.82 |
| Insulation Grade | - | F |

## Derived Conversion Factors

### Encoder
- Resolution: 131072 counts/rev (17-bit)
- Dual encoder: Input (motor) and Output (after gearbox)

### Position/Velocity Conversion
```
# Using OUTPUT encoder (post-gearbox) - gear ratio NOT in calculation
counts_per_output_rev = 131072
factor_cmd (rad -> counts) = 131072 / (2*pi) = 20,861.0
factor_state (counts -> rad) = (2*pi) / 131072 = 4.794e-5
```

### Effort/Torque Conversion
```
rated_torque = 20 Nm
peak_torque = 60 Nm (3x rated, short duration)
factor_cmd (Nm -> permille) = 1000 / 20 = 50.0
factor_state (permille -> Nm) = 20 / 1000 = 0.02
```

### Speed Limits
```
max_output_speed = 176 RPM (no load) = 18.43 rad/s
rated_output_speed = 153 RPM = 16.02 rad/s
```

## CiA402 Operation Modes
- Mode 8: CSP (Cyclic Sync Position)
- Mode 9: CSV (Cyclic Sync Velocity)
- Mode 10: CST (Cyclic Sync Torque)

## CiA402 Limit Objects (SDO)

| Object | Name | Type | Unit | Default |
|--------|------|------|------|---------|
| 0x6072 | Max Torque | uint16 | permille of rated | 1000 (20Nm) |
| 0x6080 | Max Motor Speed | uint32 | RPM (motor side) | 3500 |
| 0x60E0 | Positive Torque Limit | uint16 | permille | 1000 |
| 0x60E1 | Negative Torque Limit | uint16 | permille | 1000 |

To allow peak torque (60Nm), set 0x6072 to 3000 (300% of rated).
