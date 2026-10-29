# This week's lab: preprocessing and KNN

Open [preprocessing_lab.ipynb](preprocessing_lab.ipynb). Implement five-sample averaging, baseline-relative features, standardized distances, and a three-neighbor vote. Checks and plots are supplied. The notebook ends with the KNN visualization.

The lab includes **40 synthetic recordings**. No hardware, personal calibration profile, or real recordings are needed. Synthetic results are teaching examples, not evidence of wearer accuracy.

## Install and open

Download or clone the **whole repository**, then run these commands from its root. Keep the notebook in its current folder; it imports the project's calibration code.

On macOS/Linux:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r notebooks/signals_basics/preprocessing/requirements.txt
python -m jupyter lab notebooks/signals_basics/preprocessing/preprocessing_lab.ipynb
```

On Windows PowerShell:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r notebooks/signals_basics/preprocessing/requirements.txt
.\.venv\Scripts\python.exe -m jupyter lab notebooks/signals_basics/preprocessing/preprocessing_lab.ipynb
```

Open the local URL printed in Terminal if a browser does not open automatically. In Jupyter, run cells in order with **Shift+Enter**. Unfinished functions report **PENDING** until you implement them. After changing plot settings, rerun that cell.

## Included files

- [Student notebook](preprocessing_lab.ipynb): four coding exercises and visual feedback.
- `lab_support.py`: supplied loading, checks and plots. You do not need to edit it.
- [Synthetic recordings](synthetic/README.md): dataset description and reproducible generator.
- [Slides PDF](../slides/guaqband-signals.pdf) and [editable slides](../slides/slides.md).

Leave `USE_SYNTHETIC = True`. Real recording instructions are an optional later extension in [recordings/README.md](recordings/README.md).
