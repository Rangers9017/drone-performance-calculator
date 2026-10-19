# DroneCalc Pro

A dark themed desktop calculator for estimating multirotor performance. The interface includes animated component views and a summary dashboard.

## Features

- Motor RPM and power estimates
- Propeller thrust and power estimates
- Battery voltage, capacity and flight-time estimates
- Weight and thrust-to-weight calculations
- ESC current recommendations and temperature estimates
- Performance charts and a combined summary

## Requirements

- Python 3.10+
- Tkinter
- Packages in `requirements.txt`

Install packages:

```bash
python -m pip install -r requirements.txt
```

Start on Windows with `drone_calc_calistir.bat`, or run:

```bash
python drone_calc.py
```

## Türkçe

Motor, pervane, batarya, ağırlık ve ESC değerleriyle çok rotorlu drone performansını yaklaşık olarak inceleyen animasyonlu bir masaüstü uygulamasıdır.

## Accuracy note

Results are estimates based on simplified and empirical models. Real performance depends on the motor, propeller, battery, ESC, frame and operating conditions. Use manufacturer data and safe bench measurements before choosing flight hardware; this tool is not flight certification.
