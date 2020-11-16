# Run Mlflow to See Fast-Planner Dev Tests

In order to view and examine the dev tests we ran on Fast-Planner you should follow the following steps

## Installation

Install MLflow from PyPI via ``pip install mlflow``

MLflow requires ``conda`` to be on the ``PATH`` for the projects feature.

If it's not:
```bash
export PATH=<path/to/anaconda>/bin:$PATH
```

## Usage

```bash
cd <path/to/repo>/dev_tests

mlflow ui
```



Launching the Tracking UI
-------------------------
The MLflow Tracking UI will show runs logged in ``./mlruns`` at <http://localhost:5000>.
Start it with:
  ```bash
cd <path/to/repo>/dev_tests

mlflow ui
```

Enter: <http://localhost:5000>
